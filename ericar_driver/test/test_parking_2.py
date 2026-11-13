"""단순 주차 상태·실제 명령·흰 선 정지. 실차 성공 증명 아님"""
import copy
import importlib.util
import math
from pathlib import Path
import sys
from unittest.mock import patch
import cv2
import numpy as np
import pytest
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from ericar_driver.params import ANSWER_CONFIG
from ericar_driver.parking import Parking
from ericar_driver.parking_gap import gap_candidates,classify_gap
from ericar_driver.parking_white import parking_straight_lines
from ericar_driver.lidar import scan_sectors
from ericar_driver.driver import Driver


def config():
    c=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/robot.yaml').read_text())
    c['answer']=copy.deepcopy(ANSWER_CONFIG);c['answer']['missions']['start_stage']='parking'
    c['answer']['lidar']['zero_is_no_return']=True
    return c


def laser(c,boxes=None):
    if boxes is None:boxes=[(.15,.4,-.84,-.25),(.95,1.2,-.84,-.25)]
    boxes=list(boxes)+[(-4.1,-4.,-4.,4.),(4.,4.1,-4.,4.),(-4.,4.,4.,4.1),(-4.,4.,-4.1,-4.)]
    angles=np.linspace(-math.radians(110),math.radians(110),881)
    directions=np.column_stack((np.cos(angles),np.sin(angles)));origin=np.array([c['robot']['lidar']['x'],0.])
    ranges=np.full(len(angles),np.inf)
    for x0,x1,y0,y1 in boxes:
        with np.errstate(divide='ignore',invalid='ignore'):
            lo=(np.array([x0,y0])-origin)/directions;hi=(np.array([x1,y1])-origin)/directions
        enter=np.minimum(lo,hi).max(axis=1);leave=np.maximum(lo,hi).min(axis=1)
        hit=(leave>=np.maximum(enter,0.))&(leave>0.)
        ranges=np.minimum(ranges,np.where(hit,np.maximum(enter,0.),np.inf))
    return scan_sectors(ranges,angles[0],angles[1]-angles[0],.01,12.,c['answer']['lidar'])


def picture(present=True):
    frame=np.zeros((480,640,3),np.uint8)
    if present:cv2.line(frame,(350,470),(280,270),(240,240,240),8)
    return frame


def step(p,data,now=1.,present=True,velocity=.12,image_stamp=None):
    return p.step(now,picture(present),now if image_stamp is None else image_stamp,data,now,
                  None,now,[],velocity,dict(valid=True),search_command=(.12,0.))


def test_first_real_free_scan_immediately_commands_right_turn_without_pose():
    c=config();p=Parking(c);p.start(0.)
    data=laser(c);b=gap_candidates(data,c['robot'],p.c)[0][0]
    assert classify_gap(b,data,c['robot'],p.c)[0]=='FREE'
    speed,steering,state=step(p,data)
    assert state=='parking_turn' and speed>0 and steering==p.c['turn_steering']<0
    assert p.line_seen and p.selected
    assert not hasattr(p,'future')


def test_search_rolls_even_when_lane_detector_is_invalid():
    c=config();p=Parking(c);p.start(0.)
    data=laser(c,boxes=[])
    result=p.step(1.,picture(),1.,data,1.,None,1.,[],.12,dict(valid=False),search_command=(0.,0.))
    assert result==(.12,0.,'parking_scan')


def test_unknown_after_free_does_not_cancel_turn_and_white_loss_stops_once():
    c=config();p=Parking(c);p.start(0.);step(p,laser(c))
    assert step(p,laser(c,boxes=[]),1.1)[2]=='parking_turn'
    speed,steering,state=step(p,laser(c,boxes=[]),1.2,present=False)
    assert speed==0 and steering==0 and state=='parked'
    assert step(p,laser(c),1.3)[0]==0


def test_white_never_seen_stops_as_failure_not_completion():
    c=config();p=Parking(c);p.start(0.);step(p,laser(c),present=False)
    assert not p.line_seen
    assert step(p,laser(c,boxes=[]),2.6,present=False)[0]==0
    assert p.state=='parking_failed' and p.reason=='WHITE_LINE_NEVER_SEEN'


def test_same_or_stale_camera_frame_is_not_a_disappearance():
    c=config();p=Parking(c);p.start(0.);step(p,laser(c))
    assert step(p,laser(c,boxes=[]),1.1,False,image_stamp=1.)[2]=='parking_turn'
    assert step(p,laser(c,boxes=[]),1.8,False,image_stamp=1.)[0]==0
    assert p.state=='parking_turn' and p.reason=='WAITING_FOR_FRESH_SENSORS'


def test_actual_obstacle_stops_without_erasing_latched_turn():
    c=config();p=Parking(c);p.start(0.);step(p,laser(c))
    blocked=laser(c,boxes=[(.29,.32,-.03,.03)])
    result=step(p,blocked,1.1)
    assert result[0]==0 and p.state=='parking_turn' and p.reason=='OBSTACLE_IN_NEXT_BODY_PATH'


def test_turn_timeout_cancel_and_invalid_steering():
    c=config();p=Parking(c);p.start(0.);step(p,laser(c))
    assert step(p,laser(c),9.1)[0]==0 and p.reason=='TURN_TIMEOUT'
    p=Parking(c);p.start(0.);p.cancel()
    assert step(p,laser(c))[2]=='parking_cancelled'
    c['answer']['parking']['turn_steering']=.2
    with pytest.raises(ValueError):Parking(c)


def test_white_detector_excludes_yellow_and_horizontal_lines():
    c=config();image=picture(False)
    cv2.line(image,(100,450),(250,270),(0,255,255),8)
    cv2.line(image,(100,350),(500,350),(255,255,255),8)
    assert not parking_straight_lines(image,c['answer']['parking'])[1]
    assert parking_straight_lines(picture(),c['answer']['parking'])[1]


def test_main_driver_sign_to_right_turn_to_white_loss():
    c=config();d=Driver(c);lane=dict(valid=True,error=0.,heading=0.,stop_distance=None,forward=.6,abnormal=False,solid={})
    for i in range(6):
        now=1.+i*.1
        data=dict(image=picture(),image_stamp=now,scan_stamp=now,pose=None,pose_stamp=now,history=[])
        result=d.update(lane,laser(c),[dict(name='parking',area=.1)],now,.12,now,parking_input=data)
        if d.state=='parking_turn':break
    assert d.state=='parking_turn' and result[0]>0 and result[1]<0
    now+=.1;data.update(image=picture(False),image_stamp=now,scan_stamp=now,pose_stamp=now)
    result=d.update(lane,laser(c),[],now,.12,now,parking_input=data)
    assert d.state=='parked' and result[:2]==(0.,0.)


def test_ros_main_publisher_receives_turn_and_stop_commands():
    import fake_ros
    c=config()
    with patch.dict(sys.modules):
        fake_ros.install(lambda node:None)
        path=Path(__file__).resolve().parents[1]/'ericar_driver/node.py'
        spec=importlib.util.spec_from_file_location('ericar_driver._p2_node',path)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        node=module.DriverNode(c)
        lane=dict(valid=True,error=0.,heading=0.,stop_distance=None,forward=.6,abnormal=False,solid={},lane_mask=np.zeros((8,8),np.uint8))
        node.loop.vision.analyze=lambda *args:dict(lane)
        for i in range(6):
            now=round(1.+i*.1,6);fake_ros.Clock.now_s=now
            node.image=picture();node.image_stamp=node.scan_stamp=node.odom_stamp=node.detection_stamp=now
            node.scan=laser(c);node.velocity=.12;node.detections=[dict(name='parking',area=.1)]
            node.control()
            if node.loop.driver.state=='parking_turn':break
        commands=[m for topic,m in node.published if topic==c['topics']['drive']]
        assert commands[-1].speed>0 and commands[-1].steering_angle<0
        fake_ros.Clock.now_s=round(fake_ros.Clock.now_s+.1,6);now=fake_ros.Clock.now_s
        node.image=picture(False);node.image_stamp=node.scan_stamp=node.odom_stamp=now
        node.control()
        assert node.loop.driver.state=='parked'
        assert [m for topic,m in node.published if topic==c['topics']['drive']][-1].speed==0


def test_viewer_and_controller_use_same_white_and_gap_decisions(monkeypatch):
    from ericar_driver.parking_white import straight_overlay
    c=config();mask,lines=parking_straight_lines(picture(),c['answer']['parking'])
    shown,viewer_mask,viewer_lines=straight_overlay(picture(),c['answer']['parking'])
    assert lines==viewer_lines and np.array_equal(mask,viewer_mask)
    monkeypatch.setattr(sys,'argv',['debug_view.py'])
    path=Path(__file__).resolve().parents[1]/'tools/debug_view.py'
    spec=importlib.util.spec_from_file_location('p2_view',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    module.VIEW_CONFIG=(c['robot'],c['answer']['parking'])
    import time
    data=laser(c);r=data['parking_rays']
    frame=module.compose(picture(),None,None,(r['distances'],r['angles'],.01,12.,time.monotonic()))
    assert frame.shape==(478,1320,3)


def test_line_acquisition_timeout_and_turn_limit_are_stops():
    c=config();p=Parking(c);p.start(0.);step(p,laser(c))
    p.turn_distance=c['answer']['parking']['turn_distance_limit']
    command=step(p,laser(c),1.1)
    assert command[0]==0 and p.reason=='TURN_DISTANCE_LIMIT'


def test_motor_deadline_and_start_gate_do_not_depend_on_odom_pose():
    c=config();p=Parking(c);p.start(0.)
    result=p.step(1.,picture(),1.,laser(c),1.,None,-100.,[],.12,dict(valid=False),search_command=None)
    assert result[0]>0 and result[1]<0 and result[2]=='parking_turn'
    # pose 없이 동작해도 scan 미갱신은 정지
    result=p.step(2.,picture(),2.,laser(c),1.,None,-100.,[],.12,dict(valid=False))
    assert result[0]==0 and p.state=='parking_turn'


def test_unknown_gap_does_not_trigger_turn_and_sensor_invalid_stops():
    c=config();p=Parking(c);p.start(0.)
    data=laser(c)
    data['parking_rays']['measured']=np.zeros_like(data['parking_rays']['measured'])
    command=step(p,data)
    assert command[2]=='parking_scan'
    assert not p.selected
    data['valid_fraction']=.2
    assert step(p,data,1.1)[0]==0


def test_cancel_reaches_main_motor_publisher():
    import fake_ros
    c=config()
    with patch.dict(sys.modules):
        fake_ros.install(lambda node:None)
        path=Path(__file__).resolve().parents[1]/'ericar_driver/node.py'
        spec=importlib.util.spec_from_file_location('ericar_driver._p2_cancel',path)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        node=module.DriverNode(c);node.loop.driver.state='parking_turn'
        node.on_parking_cancel(fake_ros.Msg(data=True))
        assert node.loop.driver.state=='parking_cancelled'
        assert [m for topic,m in node.published if topic==c['topics']['drive']][-1].speed==0


def test_empty_or_all_unmeasured_scan_cannot_command_motion():
    c=config();p=Parking(c);p.start(0.);data=laser(c,boxes=[])
    data['parking_rays']['measured']=np.zeros_like(data['parking_rays']['measured'])
    assert step(p,data)[0]==0 and p.reason=='INVALID_SCAN_HEALTH'
    data['valid_fraction']=float('nan')
    assert step(p,data,1.1)[0]==0
