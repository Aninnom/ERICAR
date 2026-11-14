"""주차 탐색 속도 분리·원호 유지·정렬 전환. 이상적 합성 장면, 실차 검증과 구분"""
import math
import copy
import importlib.util
import sys
from pathlib import Path
from unittest.mock import patch
import cv2
import numpy as np
import pytest
from test_parking_2 import config,laser,picture,Parking,Driver
from test_parking_auto1 import scan_at
from test_parking_pd import scene
from ericar_driver.parking_sign import SignPassage,slow_distance
from ericar_driver.parking_control import image_point


def cruise_config():
    c=config();c['answer']['parking']['cruise_search_enabled']=True
    return c


def tick(p,c,t=1.,objects=(),stamp=None,data=None,v=.3,command=(.35,0.),w=0.,image=None,image_stamp=None,measured_rate=None):
    return p.step(t,picture() if image is None else image,t if image_stamp is None else image_stamp,
                  laser(c,boxes=[]) if data is None else data,t,None,t,[],v,dict(valid=True),
                  search_command=command,angular_velocity=w,detections=objects,
                  detection_stamp=t if stamp is None else stamp,measured_turn_rate=measured_rate)


def sign(area=.003,center=.7):
    return [dict(name='parking',area=area,center=[center,.4],confidence=.9)]


def test_far_sign_drops_do_not_force_slow_and_lidar_starts_immediately():
    c=cruise_config();p=Parking(c);p.start(1.)
    result=tick(p,c,objects=sign(),data=scan_at(c,.3),v=.3)
    assert result==(.35,0.,'parking_cruise') and p.gap_debug['scan_stamp']==1.
    assert p.gap_debug['evaluations']  # UNKNOWN 후보도 실제 탐색 진행
    for i in range(1,8):assert tick(p,c,1.+.2*i)==(.35,0.,'parking_cruise')
    assert not p.sign.near_seen


def test_near_sign_requires_distinct_inference_results_then_latches_slow():
    c=cruise_config();p=Parking(c);p.start(1.)
    assert tick(p,c,objects=sign(.02))[2]=='parking_cruise'
    assert tick(p,c,1.2)[2]=='parking_cruise'
    # 동일 추론 결과의 재조회는 미검출 횟수에서 제외
    for t in (1.21,1.22,1.23):assert tick(p,c,t,stamp=1.2)[2]=='parking_cruise'
    assert p.sign.misses==1
    assert tick(p,c,1.4)[2]=='parking_cruise'
    assert tick(p,c,1.6)==(.12,0.,'parking_scan')
    assert p.slow_reason=='NEAR_SIGN_PASSED'
    assert tick(p,c,1.8,objects=sign(.02))[2]=='parking_scan'


def test_stale_detector_and_interrupted_loss_are_not_passage():
    c=cruise_config();s=SignPassage(c['answer']['parking'],c['answer']['detector'])
    s.observe(sign(.02),1.,1.)
    for t in (1.2,1.4,1.6):s.observe([],1.,t)
    assert s.misses==0 and not s.passed
    s.observe([],1.8,1.8);s.observe(sign(.02),2.,2.);s.observe([],2.2,2.2)
    assert s.misses==1 and not s.passed
    s.observe([],2.3,10.)
    assert s.misses==1 and not s.passed


def test_right_edge_sign_can_establish_near_passage():
    c=cruise_config();p=Parking(c);p.start(1.);tick(p,c,objects=sign(.007,.96))
    assert p.sign.near_seen
    for t in (1.2,1.4,1.6):result=tick(p,c,t)
    assert result[2]=='parking_scan'


def test_unknown_target_inside_braking_distance_slows_before_free():
    c=cruise_config();p=Parking(c);p.start(1.)
    result=tick(p,c,objects=sign(),data=scan_at(c,.3),v=.4)
    assert p.gap_debug['evaluations'][0]['state']=='UNKNOWN'
    assert p.selected is None and result==(.12,0.,'parking_scan')
    assert p.slow_reason=='TARGET_BRAKING_DISTANCE'
    assert slow_distance(.4,c['answer']['parking'])>slow_distance(.2,c['answer']['parking'])


def test_free_can_be_latched_while_still_cruising_then_approached():
    c=cruise_config();p=Parking(c);p.start(1.)
    # 넓은 빈틈의 실제 raycast. 반사 근거가 충분하면 먼 위치에서도 FREE 가능
    data=laser(c,[(.15,.4,-.84,-.25),(1.2,1.45,-.84,-.25)])
    result=tick(p,c,data=data,v=.12)
    assert p.selected is not None
    assert result[2]=='parking_cruise'
    saved=np.array(p.selected['entrance'])
    result=tick(p,c,1.1,v=.4)  # 다음 프레임에 후보가 안 보여도 저장 목표 유지
    assert result[2]=='parking_approach' and p.slow_reason=='TARGET_BRAKING_DISTANCE'
    assert np.allclose(p.selected['entrance'],saved-[.04,0.])


def test_actual_speed_must_drop_before_turn_entry():
    c=config();p=Parking(c);p.start(1.)
    result=tick(p,c,data=laser(c),v=.3)
    assert result[2]=='parking_approach' and p.approach_debug['waiting_for_slow_speed']
    assert result[0]==.12 and result[1]==0
    assert tick(p,c,1.1,data=laser(c),v=.12)[2]=='parking_turn'


def test_cruise_obstacle_and_main_stop_are_not_overridden():
    c=cruise_config();p=Parking(c);p.start(1.)
    assert tick(p,c,data=laser(c,[(.29,.32,-.03,.03)]))[0]==0
    assert p.reason=='OBSTACLE_IN_NEXT_BODY_PATH'
    assert tick(p,c,1.1,command=(0.,.1),v=0.)[:2]==(0.,.1)
    p.cancel();assert tick(p,c,1.2)[2]=='parking_cancelled'


def test_trial_cruise_is_capped_separately_from_full_course():
    c=cruise_config();c['answer']['parking']['test_drive_until_sign']=True;p=Parking(c);p.start(1.)
    assert tick(p,c,command=(.4,0.))[0]==.2


def test_main_driver_after_traffic_arms_cruise_and_keeps_lane_command():
    c=cruise_config();c['answer']['missions']['start_stage']='traffic';d=Driver(c);d.done.add('traffic');d.pid.integral=.04
    d.main_lane_command=lambda *args,**kwargs:(.33,.02)
    lane=dict(valid=True,error=0.,heading=0.,stop_distance=None,forward=.6,abnormal=False,solid={})
    for i in range(4):
        now=1.+i*.1
        data=dict(image=picture(),image_stamp=now,scan_stamp=now,pose=None,pose_stamp=now,history=[])
        result=d.update(lane,laser(c,boxes=[]),sign(),now,.3,now,parking_input=data)
    assert d.state=='parking_cruise' and result[:2]==(.33,.02)
    assert d.parking.gap_debug['scan_stamp']==pytest.approx(now)
    assert d.pid.integral==.04  # 일반 주행 PID 상태를 표지판 확인 시 초기화하지 않음


def test_yolo_remains_enabled_during_cruise_only():
    import fake_ros
    with patch.dict(sys.modules):
        fake_ros.install(lambda node:None)
        path=Path(__file__).resolve().parents[1]/'ericar_driver/node.py'
        spec=importlib.util.spec_from_file_location('ericar_driver._cruise_node',path)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        assert not module.DriverNode.detector_paused('parking_cruise')
        assert not module.DriverNode.detector_paused('lane')
        for state in ('parking_scan','parking_approach','parking_turn','parked'):
            assert module.DriverNode.detector_paused(state)


def turn_fixture(angle=.5):
    c=config();p=Parking(c);p.start(1.);image,target=scene(c)
    target['entrance']=[-.15,0.]  # 후단이 입구 안쪽인 가정. 별도로 회전각 검사
    p.selected=target;p.state='parking_turn';p.turn_started=p.last_tick=1.;p.turn_angle=angle
    return c,p,image


def test_false_aligned_visual_cue_cannot_unwind_arc_early():
    c,p,image=turn_fixture(.5)
    for i in range(1,10):
        result=tick(p,c,1.+i*.1,v=.12,image=image)
        assert result[0]>0 and result[1]<=-p.c['pd']['arc_min_steering'] and p.turn_phase=='arc'
    assert abs(p.pd.target)<.04  # 시각 PD만 사용했다면 거의 직진할 장면


def test_align_requires_distinct_frames_and_release_requires_enough_rotation():
    c,p,image=turn_fixture(1.3)
    tick(p,c,1.1,image=image)
    for t in (1.11,1.12,1.13):tick(p,c,t,image=image,image_stamp=1.1)
    assert p.turn_phase=='arc' and p.align_frames==1
    tick(p,c,1.2,image=image);result=tick(p,c,1.3,image=image)
    assert p.turn_phase=='align' and not p.arc_released and result[1]<=-.2
    p.turn_angle=1.5
    for i in range(4,12):result=tick(p,c,1.+i*.1,image=image)
    assert p.arc_released and abs(result[1])<.04


def test_arc_does_not_complete_on_early_white_loss_or_turn_the_wrong_way():
    c,p,image=turn_fixture(.5);tick(p,c,1.1,image=image)
    result=tick(p,c,1.2,image=picture(False))
    assert result[2]=='parking_turn' and result[1]<=-p.c['pd']['arc_min_steering']
    p.turn_angle=1.5;p.selected['entrance']=[.3,0.]  # 회전했어도 입구 밖이면 완료 금지
    assert tick(p,c,1.3,image=picture(False))[2]=='parking_turn'


def render_bay(c,pose,center,paint_end=-.84):
    x,y,yaw=pose;rotation=np.array([[math.cos(yaw),-math.sin(yaw)],[math.sin(yaw),math.cos(yaw)]])
    frame=np.zeros((480,640,3),np.uint8)
    world_lines=[[(center-.25,-.25),(center-.25,paint_end)],[(center+.25,-.25),(center+.25,paint_end)],[(center+.25,-.25),(3.,-.25)]]
    for start,end in world_lines:
        world=np.linspace(start,end,80)
        pixels=[image_point(q,frame.shape,c['robot']['camera']) for q in (world-[x,y])@rotation]
        pixels=[q for q in pixels if q is not None and max(abs(q[0]),abs(q[1]))<100000]
        if len(pixels)>1:cv2.polylines(frame,[np.array(pixels,np.int32)],False,(255,255,255),6)
    return frame


def simulate_arc(bias=0.):
    c=config();p=Parking(c);p.start(1.);center=p._turn_lead()
    x=bias;y=yaw=0.;speed=.12;steer=-.35;trace=[]
    p.selected=dict(entrance=[center,-.25],normal=[0.,-1.],tangent=[1.,0.],width=.5,
                    polygon=[[center-.25,-.25],[center+.25,-.25],[center+.25,-.84],[center-.25,-.84]])
    p.state='parking_turn';p.turn_started=p.last_tick=1.
    for i in range(1,241):
        dt=.05;now=1.+i*dt;v=speed;rate=v*math.tan(steer)/c['robot']['wheelbase']
        if abs(rate)<1e-8:x+=v*math.cos(yaw)*dt;y+=v*math.sin(yaw)*dt
        else:
            x+=v/rate*(math.sin(yaw+rate*dt)-math.sin(yaw))
            y+=v/rate*(-math.cos(yaw+rate*dt)+math.cos(yaw))
        yaw+=rate*dt
        image=render_bay(c,(x,y,yaw),center)
        speed,steer,state=tick(p,c,now,v=v,w=rate,image=image)
        trace.append(dict(t=now,x=x,y=y,yaw=yaw,speed=speed,steering=steer,phase=p.turn_phase,state=state,angle=p.turn_angle))
        if state in ('parked','parking_failed'):break
    return c,p,center,trace


def body_within_side_lines(c,center,trace):
    r=c['robot'];body=np.array([[r['front_bumper_x']-r['length'],-r['width']/2],
                              [r['front_bumper_x'],-r['width']/2],[r['front_bumper_x'],r['width']/2],
                              [r['front_bumper_x']-r['length'],r['width']/2]])
    minimum=1.
    for item in trace:
        yaw=item['yaw'];rotation=np.array([[math.cos(yaw),-math.sin(yaw)],[math.sin(yaw),math.cos(yaw)]])
        points=body@rotation.T+[item['x'],item['y']]
        checked=[q for q in points if -.84<=q[1]<=-.25]
        for a,b in zip(points,np.roll(points,-1,axis=0)):
            for row in (-.84,-.25):
                if abs(b[1]-a[1])>1e-8:
                    fraction=(row-a[1])/(b[1]-a[1])
                    if 0<=fraction<=1:checked.append(a+fraction*(b-a))
        if checked:minimum=min(minimum,*[.25-abs(q[0]-center) for q in checked])
    return minimum


@pytest.mark.parametrize('bias',[-.04,-.02,0.,.02,.04])
def test_full_camera_bicycle_arc_keeps_turn_and_finishes_after_85_degrees(bias):
    c,p,center,trace=simulate_arc(bias)
    assert p.state=='parked'
    assert math.degrees(p.turn_angle)>=85
    assert trace[-1]['speed']==0
    assert all(t['steering']<=-p.c['pd']['arc_min_steering'] for t in trace if t['state']=='parking_turn' and t['phase']=='arc')
    assert body_within_side_lines(c,center,trace)>=0


def test_high_entry_speed_that_passes_target_does_not_force_a_late_turn():
    c=config();p=Parking(c);p.start(1.)
    assert tick(p,c,data=laser(c),v=.4)[2]=='parking_approach'
    result=tick(p,c,1.2,data=laser(c),v=.4)
    assert result==(.12,0.,'parking_scan') and p.selected is None


def test_ros_main_motor_cruises_then_slows_after_sign_passage():
    import fake_ros
    c=cruise_config()
    with patch.dict(sys.modules):
        fake_ros.install(lambda node:None)
        path=Path(__file__).resolve().parents[1]/'ericar_driver/node.py'
        spec=importlib.util.spec_from_file_location('ericar_driver._cruise_publish',path)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        node=module.DriverNode(c);node.loop.driver.main_lane_command=lambda *a,**kw:(.33,.02)
        lane=dict(valid=True,error=0.,heading=0.,stop_distance=None,forward=.6,abnormal=False,solid={},lane_mask=np.zeros((8,8),np.uint8))
        node.loop.vision.analyze=lambda *args:dict(lane)
        def frame(t,objects):
            fake_ros.Clock.now_s=t;node.image=picture()
            node.image_stamp=node.scan_stamp=node.odom_stamp=node.detection_stamp=t
            node.scan=laser(c,boxes=[]);node.velocity=.3;node.detections=objects
            node.control()
            return [m for topic,m in node.published if topic==c['topics']['drive']][-1]
        for t in (1.,1.1,1.2):command=frame(t,sign())
        assert node.loop.driver.state=='parking_cruise' and command.speed==.33
        frame(1.4,sign(.02))
        for t in (1.6,1.8,2.):command=frame(t,[])
        assert node.loop.driver.state=='parking_scan' and command.speed==.12
