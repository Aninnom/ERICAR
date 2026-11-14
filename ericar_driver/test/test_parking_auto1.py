"""회전 시작점 접근 시뮬레이션. 실측 주차 성공률·차선 미침범 증명은 아님"""
import math
from unittest.mock import patch
import numpy as np
import pytest
from test_parking_2 import config,laser,picture,Parking,Driver,completed_turn_pose


BOXES=[(.15,.4,-.84,-.25),(.95,1.2,-.84,-.25)]


def scan_at(c,offset=0.,travel=0.):
    return laser(c,[(a+offset-travel,b+offset-travel,y,z) for a,b,y,z in BOXES])


def tick(p,data,t=1.,v=.12,w=0.,present=True,scan_stamp=None,lane_valid=True):
    return p.step(t,picture(present),t,data,t if scan_stamp is None else scan_stamp,
                  None,-100.,[],v,dict(valid=lane_valid),search_command=(.12,0.),angular_velocity=w)


@pytest.mark.parametrize('offset',[-.1,0.,.15,.30])
def test_different_obstacle_positions_reach_same_relative_turn_start(offset):
    c=config();p=Parking(c);p.start(0.)
    lead=p._turn_lead();seen_approach=False;turn_travel=None
    for i in range(80):
        travel=i*.012;t=1.+i*.1
        command=tick(p,scan_at(c,offset,travel),t)
        if p.state=='parking_approach':
            seen_approach=True
            assert command==(.12,0.,'parking_approach')
        elif p.state=='parking_turn':
            turn_travel=travel
            assert command[0]>0 and command[1]==c['answer']['parking']['turn_steering']
            assert abs(p.selected['entrance'][0]-lead)<=c['answer']['parking']['approach_start_tolerance']+.012
            break
        else:
            # 먼 후보는 가림으로 UNKNOWN. FREE 확인 전까지 서행 유지
            assert command==(.12,0.,'parking_scan')
    assert seen_approach and turn_travel is not None
    assert abs(turn_travel-(.675+offset-lead))<=.032
    completed_turn_pose(p)
    assert tick(p,laser(c,boxes=[]),t+.1,present=False)==(0.,0.,'parked')


def test_single_free_is_latched_through_unknown_and_competing_gap():
    c=config();p=Parking(c);p.start(0.)
    tick(p,scan_at(c));initial=np.array(p.selected['entrance'])
    for i in range(1,5):
        data=laser(c,boxes=[]) if i%2 else scan_at(c,.3)
        command=tick(p,data,1.+.1*i,present=False)
        assert command[2]=='parking_approach'
        assert np.allclose(p.selected['entrance'],initial-[.012*i,0.])
    # 접근 중 흰 선 소실은 완료 조건 아님
    assert not p.line_seen


def test_passed_free_gap_is_skipped_instead_of_late_turn():
    c=config();p=Parking(c);p.start(0.)
    command=tick(p,scan_at(c,-.25))
    assert command==(.12,0.,'parking_scan')
    assert p.selected is None and p.reason=='TURN_START_PASSED_LOOKING_FOR_NEXT_GAP'
    assert any(e['rejection']=='TURN_START_ALREADY_PASSED' for e in p.gap_debug['evaluations'])
    assert tick(p,scan_at(c),1.1)[2]=='parking_approach'


def test_signed_measured_velocity_not_elapsed_time_advances_target():
    c=config();p=Parking(c);p.start(0.);tick(p,scan_at(c))
    remaining=p.approach_debug['remaining_m']
    tick(p,laser(c,boxes=[]),1.2,v=0.)
    assert p.approach_debug['remaining_m']==pytest.approx(remaining)
    tick(p,laser(c,boxes=[]),1.3,v=-.12)
    assert p.approach_debug['remaining_m']==pytest.approx(remaining+.012)
    assert tick(p,laser(c,boxes=[]),5.5,v=0.)[0]==0
    assert p.reason=='NO_ODOMETRY_MOTION'


def test_relative_target_accounts_for_approach_heading_change():
    c=config();p=Parking(c);p.start(0.);tick(p,scan_at(c))
    point=np.array(p.selected['entrance']);v=.12;w=.1;dt=.2;yaw=w*dt
    command=tick(p,laser(c,boxes=[]),1.2,v=v,w=w)
    rotation=np.array([[math.cos(yaw),-math.sin(yaw)],[math.sin(yaw),math.cos(yaw)]])
    translation=np.array([v/w*math.sin(yaw),v/w*(1-math.cos(yaw))])
    assert np.allclose(p.selected['entrance'],(point-translation)@rotation)
    assert command[2]=='parking_approach'


@pytest.mark.parametrize('limit,reason',[('approach_timeout','APPROACH_TIMEOUT'),('approach_distance_limit','APPROACH_DISTANCE_LIMIT')])
def test_approach_has_time_and_distance_limits(limit,reason):
    c=config();c['answer']['parking'][limit]=.05;p=Parking(c);p.start(0.);tick(p,scan_at(c))
    command=tick(p,laser(c,boxes=[]),1.5,v=.12)
    assert command==(0.,0.,'parking_failed') and p.reason==reason


def test_obstacle_stale_input_and_cancel_remain_stops_during_approach():
    c=config();p=Parking(c);p.start(0.);tick(p,scan_at(c))
    assert tick(p,laser(c,[(.29,.32,-.03,.03)]),1.1,v=0.)[0]==0
    assert p.state=='parking_approach' and p.reason=='OBSTACLE_IN_NEXT_BODY_PATH'
    assert tick(p,laser(c,boxes=[]),2.,scan_stamp=1.1)[0]==0
    assert p.state=='parking_approach' and p.reason=='WAITING_FOR_FRESH_SENSORS'
    p.cancel();assert tick(p,laser(c),2.1)==(0.,0.,'parking_cancelled')


def test_main_motor_publishes_approach_then_turn_then_stop():
    import fake_ros
    import importlib.util
    import sys
    from pathlib import Path
    c=config()
    with patch.dict(sys.modules):
        fake_ros.install(lambda node:None)
        path=Path(__file__).resolve().parents[1]/'ericar_driver/node.py'
        spec=importlib.util.spec_from_file_location('ericar_driver._auto1_node',path)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        node=module.DriverNode(c)
        lane=dict(valid=True,error=0.,heading=0.,stop_distance=None,forward=.6,abnormal=False,solid={},lane_mask=np.zeros((8,8),np.uint8))
        node.loop.vision.analyze=lambda *args:dict(lane)
        travel=0.;seen_approach=False;seen_turn=False
        for i in range(40):
            now=round(1.+i*.1,6);fake_ros.Clock.now_s=now
            node.image=picture();node.image_stamp=node.scan_stamp=node.odom_stamp=node.detection_stamp=now
            node.velocity=.12;node.angular_velocity=0.;node.scan=scan_at(c,travel=travel)
            node.detections=[dict(name='parking',area=.1)]
            node.control()
            command=[m for topic,m in node.published if topic==c['topics']['drive']][-1]
            if node.loop.driver.state=='parking_approach':
                seen_approach=True;assert command.speed>0
                assert abs(command.steering_angle-c['robot']['steering_offset'])<.03
            if node.loop.driver.state=='parking_turn':
                seen_turn=True;assert command.speed>0 and command.steering_angle<-.3
                break
            travel+=.012
        assert seen_approach and seen_turn
        now=round(now+.1,6);fake_ros.Clock.now_s=now
        completed_turn_pose(node.loop.driver.parking)
        node.image=picture(False);node.image_stamp=node.scan_stamp=node.odom_stamp=now
        node.control()
        assert node.loop.driver.state=='parked'
        assert [m for topic,m in node.published if topic==c['topics']['drive']][-1].speed==0


def test_scan_age_is_compensated_without_pose_sync_gate():
    c=config();p=Parking(c);p.start(0.)
    result=tick(p,scan_at(c),1.,v=.12,scan_stamp=.7)
    assert result[2]=='parking_approach'
    assert p.selected['entrance'][0]==pytest.approx(.674-.036,abs=.003)


def test_debug_view_shows_latched_target_and_turn_distance():
    import importlib.util
    from pathlib import Path
    c=config();p=Parking(c);p.start(0.);tick(p,scan_at(c))
    path=Path(__file__).resolve().parents[1]/'tools/debug_view.py'
    spec=importlib.util.spec_from_file_location('auto1_view',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    module.ROBOT=c['robot'];module.ANSWER=c['answer']
    panel=np.zeros((module.PANEL_H,module.LIDAR_W,3),np.uint8)
    text=module.draw_parking(panel,{'parking':p.debug})
    assert 'APPROACH turn in' in text
    assert np.count_nonzero(panel)>0
    frame=module.compose(picture(),None,{'control':{'parking':p.debug,'controller_state':p.state,'section':4}})
    assert frame.shape==(596,1291,3)
