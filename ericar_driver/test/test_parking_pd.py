"""카메라 경계선 투영·주차 PD·폐루프 합성 시험. 실제 차체 진입 성공 증명 아님"""
import math
import copy
import numpy as np
import cv2
import pytest
from test_parking_2 import config,laser,picture,Parking
from ericar_driver.parking_control import ParkingPD,boundary_observation,ground_points,image_point
from ericar_driver.parking_white import parking_straight_lines


def scene(c,x=0.,y=0.,yaw=0.,one=None,center_bias=0.,gap_width=.5):
    image=np.zeros((480,640,3),np.uint8)
    rotation=np.array([[math.cos(yaw),-math.sin(yaw)],[math.sin(yaw),math.cos(yaw)]])
    for side in (-1,1):
        if one is not None and side!=one:continue
        world=np.column_stack((np.linspace(x+.3,x+2.,40),np.full(40,side*.25)))
        pixels=[image_point(q,image.shape,c['robot']['camera']) for q in (world-[x,y])@rotation]
        pixels=[q for q in pixels if q is not None]
        if len(pixels)>1:cv2.polylines(image,[np.array(pixels,np.int32)],False,(255,255,255),6)
    normal=np.array([1.,0.])@rotation;tangent=np.array([0.,1.])@rotation
    entrance=(np.array([x+.4,center_bias])-[x,y])@rotation
    target=dict(entrance=entrance.tolist(),normal=normal.tolist(),tangent=tangent.tolist(),width=gap_width,
                polygon=np.array([entrance-gap_width/2*tangent,entrance+gap_width/2*tangent,
                                  entrance+gap_width/2*tangent+.59*normal,entrance-gap_width/2*tangent+.59*normal]).tolist())
    return image,target


def observe(c,image,target):
    _,lines=parking_straight_lines(image,c['answer']['parking'])
    return boundary_observation(lines,image.shape,target,c['robot'],c['answer']['parking']['pd'])


def test_camera_inverse_matches_existing_main_model():
    c=config();expected=np.array([[.5,-.1],[.8,.2],[1.2,-.3]])
    pixels=[image_point(q,(480,640,3),c['robot']['camera']) for q in expected]
    actual,valid=ground_points(pixels,(480,640,3),c['robot']['camera'])
    assert valid.all() and np.allclose(actual,expected,atol=.012)
    _,valid=ground_points([[320,0]],(480,640,3),c['robot']['camera'])
    assert not valid[0]


@pytest.mark.parametrize('offset',[-.05,0.,.05])
def test_white_pair_recovers_center_and_direction(offset):
    c=config();image,target=scene(c,y=offset)
    observation,reason=observe(c,image,target)
    assert reason=='BOUNDARY_PAIR'
    assert observation['lateral_m']==pytest.approx(-offset,abs=.015)
    assert abs(observation['heading_rad'])<.04


@pytest.mark.parametrize('side',[-1,1])
def test_partial_single_boundary_is_usable_without_four_corners(side):
    c=config();image,target=scene(c,y=.03,one=side)
    observation,reason=observe(c,image,target)
    assert reason=='SINGLE_BOUNDARY'
    assert observation['source']==('single_left' if side==1 else 'single_right')
    assert observation['lateral_m']==pytest.approx(-.03,abs=.015)


@pytest.mark.parametrize('gap_width,bias',[(.5,0.),(.7,.06),(.8,-.06)])
def test_wider_obstacles_do_not_move_painted_center(gap_width,bias):
    c=config();image,target=scene(c,y=.04,gap_width=gap_width,center_bias=bias)
    observation,reason=observe(c,image,target)
    assert reason=='BOUNDARY_PAIR'
    assert observation['lateral_m']==pytest.approx(-.04,abs=.015)
    assert observation['width_m']==pytest.approx(.5,abs=.025)


def test_initial_turn_and_no_associated_line_keep_existing_turn():
    c=config();pd=ParkingPD(c['robot'],c['answer']['parking'])
    image,target=scene(c);target['normal']=[0.,-1.]
    _,lines=parking_straight_lines(image,c['answer']['parking'])
    pd.update(lines,image.shape,target,1.)
    assert pd.debug['reason']=='INITIAL_FIXED_RIGHT_TURN'
    assert pd.command(1.,.05)==(-.35,True)
    target['normal']=[1.,0.];pd.update([],image.shape,target,1.1)
    assert pd.command(1.1,.05)==(-.35,True)


@pytest.mark.parametrize('offset,sign',[(.05,-1),(-.05,1)])
def test_pd_corrects_toward_center_without_shared_lane_pid(offset,sign):
    c=config();pd=ParkingPD(c['robot'],c['answer']['parking']);image,target=scene(c,y=offset)
    _,lines=parking_straight_lines(image,c['answer']['parking'])
    pd.update(lines,image.shape,target,1.)
    for i in range(12):steering,ready=pd.command(1.+i*.01,.05)
    assert ready and steering*sign>0
    assert pd.pid.terms['i']==0
    assert abs(steering)<=c['robot']['max_steering']


def test_centered_pose_unwinds_and_limits_steering_rate():
    c=config();pd=ParkingPD(c['robot'],c['answer']['parking']);image,target=scene(c)
    _,lines=parking_straight_lines(image,c['answer']['parking']);pd.update(lines,image.shape,target,1.)
    before=pd.steering
    for i in range(12):
        value,ready=pd.command(1.+i*.01,.05)
        assert ready and abs(value-before)<=.8*.05+1e-9
        before=value
    assert abs(value)<.03


def test_used_feedback_timeout_stops_and_new_observation_recovers():
    c=config();pd=ParkingPD(c['robot'],c['answer']['parking']);image,target=scene(c)
    _,lines=parking_straight_lines(image,c['answer']['parking']);pd.update(lines,image.shape,target,1.)
    assert pd.command(1.31,.05)==(0.,False)
    pd.update(lines,image.shape,target,1.4)
    assert pd.command(1.4,.05)[1]
    pd.pause();assert pd.command(1.5,.05)==(0.,False)
    pd.reset();assert not pd.engaged


def simulate(y0,yaw0,enabled=True):
    c=config();c['answer']['parking']['pd']['enabled']=enabled
    pd=ParkingPD(c['robot'],c['answer']['parking']);x=0.;y=y0;yaw=yaw0;trace=[]
    for i in range(120):
        now=1.+i*.05;image,target=scene(c,x,y,yaw)
        _,lines=parking_straight_lines(image,c['answer']['parking']);pd.update(lines,image.shape,target,now)
        steering,ready=pd.command(now,.05)
        assert ready
        speed=0. if pd.debug.get('steering_transition') else .12
        x+=speed*math.cos(yaw)*.05;y+=speed*math.sin(yaw)*.05
        yaw+=speed*math.tan(steering)/c['robot']['wheelbase']*.05
        trace.append(dict(t=round(i*.05,3),x=x,y=y,yaw=yaw,steering=steering,speed=speed))
    return trace


@pytest.mark.parametrize('y0,yaw0',[(.05,.3),(-.05,-.3),(.07,0.),(-.07,0.)])
def test_camera_bicycle_closed_loop_improves_over_fixed_turn(y0,yaw0):
    corrected=simulate(y0,yaw0);baseline=simulate(y0,yaw0,False)
    robot=config()['robot']
    corners=[(a,b) for a in (robot['front_bumper_x']-robot['length'],robot['front_bumper_x']) for b in (-robot['width']/2,robot['width']/2)]
    # 합성 직선 칸에서 회전한 직사각형 차체의 양옆 선 침범까지 검사
    maximum=max(abs(t['y']+a*math.sin(t['yaw'])+b*math.cos(t['yaw'])) for t in corrected for a,b in corners)
    assert maximum<.25
    assert abs(corrected[-1]['yaw'])<.12
    assert abs(corrected[-1]['y'])<.09
    assert abs(corrected[-1]['y'])<abs(baseline[-1]['y'])
    assert abs(corrected[-1]['yaw'])<abs(baseline[-1]['yaw'])


def test_parking_step_applies_pd_then_retains_white_loss_stop_and_obstacle_check():
    c=config();p=Parking(c);p.start(0.)
    image,target=scene(c);p.state='parking_turn';p.turn_started=p.last_tick=1.;p.selected=target
    data=laser(c,boxes=[]);last=None
    for i in range(1,8):
        now=1.+i*.1
        command=p.step(now,image,now,data,now,None,now,[],.12,dict(valid=True))
        assert command[0]>.0 and command[2]=='parking_turn';last=command
    assert abs(last[1])<.04 and p.debug['feedback']['active']
    now+=.1;blocked=laser(c,[(.29,.32,-.03,.03)])
    assert p.step(now,image,now,blocked,now,None,now,[],0.,dict(valid=True))[0]==0
    assert p.reason=='OBSTACLE_IN_NEXT_BODY_PATH'
    now+=.1
    assert p.step(now,picture(False),now,data,now,None,now,[],0.,dict(valid=True))==(0.,0.,'parked')


def test_nonparallel_aisle_lines_are_not_parking_feedback():
    c=config();image,target=scene(c)
    target['normal']=[math.cos(.65),math.sin(.65)]
    observation,_=observe(c,image,target)
    assert observation is None


def test_main_ros_publisher_uses_pd_output_and_white_stop():
    import fake_ros
    import importlib.util
    import sys
    from unittest.mock import patch
    from pathlib import Path
    c=config()
    with patch.dict(sys.modules):
        fake_ros.install(lambda node:None)
        path=Path(__file__).resolve().parents[1]/'ericar_driver/node.py'
        spec=importlib.util.spec_from_file_location('ericar_driver._pd_node',path)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        node=module.DriverNode(c);image,target=scene(c)
        p=node.loop.driver.parking;p.start(0.);p.state='parking_turn';p.turn_started=p.last_tick=1.;p.selected=target
        node.loop.driver.state='parking_turn';node.scan=laser(c,boxes=[]);node.velocity=.12
        node.loop.lane_cache=dict(valid=True,error=0.,heading=0.,stop_distance=None,forward=.6,abnormal=False,solid={},lane_mask=np.zeros((8,8),np.uint8),stamp=1.)
        node.loop.lane_stamp=1.
        for i in range(1,8):
            now=round(1.+i*.1,6);fake_ros.Clock.now_s=now
            node.image=image;node.image_stamp=node.scan_stamp=node.odom_stamp=node.detection_stamp=now
            node.control()
        commands=[m for topic,m in node.published if topic==c['topics']['drive']]
        assert commands[-1].speed>0 and abs(commands[-1].steering_angle)<.06
        now=round(now+.1,6);fake_ros.Clock.now_s=now
        node.image=picture(False);node.image_stamp=node.scan_stamp=node.odom_stamp=node.detection_stamp=now
        node.control()
        assert node.loop.driver.state=='parked'
        assert [m for topic,m in node.published if topic==c['topics']['drive']][-1].speed==0


def test_pd_debug_view_renders_used_boundary_and_target():
    import importlib.util
    from pathlib import Path
    c=config();image,target=scene(c);pd=ParkingPD(c['robot'],c['answer']['parking'])
    _,lines=parking_straight_lines(image,c['answer']['parking']);pd.update(lines,image.shape,target,1.);pd.command(1.,.05)
    path=Path(__file__).resolve().parents[1]/'tools/debug_view.py'
    spec=importlib.util.spec_from_file_location('pd_view',path);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    module.ROBOT=c['robot'];module.ANSWER=c['answer']
    park=dict(state='parking_turn',feedback=pd.debug,white_line=dict(segments=lines))
    frame=module.compose(image,None,{'control':dict(parking=park,controller_state='parking_turn',section=4)})
    assert frame.shape==(596,1291,3)
    assert 'PARK PD' in module.draw_parking(np.zeros((360,400,3),np.uint8),{'parking':park})


def test_opposite_correction_sets_steering_before_resuming_motion():
    c=config();p=Parking(c);p.start(0.);image,target=scene(c,y=-.07)
    p.state='parking_turn';p.turn_started=p.last_tick=1.;p.selected=target;data=laser(c,boxes=[])
    outputs=[]
    for i in range(1,9):
        now=1.+i*.1
        command=p.step(now,image,now,data,now,None,now,[],0.,dict(valid=True))
        outputs.append(command)
    assert outputs[0][0]==0 and outputs[0][1]<0
    assert any(speed>0 and steering>0 for speed,steering,state in outputs)
    assert p.state=='parking_turn'
