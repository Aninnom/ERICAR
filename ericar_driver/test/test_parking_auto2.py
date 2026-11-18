"""auto2: 실측 반경 보존·선 소실 최우선 정지·이동 중 조향 풀림 방지"""
import copy
import importlib.util
import math
import sys
from pathlib import Path
from unittest.mock import patch
import numpy as np
import pytest
from test_parking_turn_done import config
from test_parking_2 import laser,picture
from test_parking_pd import scene
from ericar_driver.parking import Parking


def turning(c=None,angle=0.,phase='arc'):
    c=config() if c is None else c;p=Parking(c);p.start(1.)
    lead=p._turn_lead()
    p.selected=dict(entrance=[lead,-.25],normal=[0.,-1.],tangent=[1.,0.],width=.5,
                    polygon=[[lead-.25,-.25],[lead+.25,-.25],[lead+.25,-.84],[lead-.25,-.84]])
    p.state='parking_turn';p.turn_started=p.last_tick=1.;p.turn_angle=angle;p.turn_phase=phase
    return c,p


def frame(p,c,t=1.,present=True,image_stamp=None,image=None,rate=0.):
    return p.step(t,picture(present) if image is None else image,t if image_stamp is None else image_stamp,
                  laser(c,boxes=[]),t,None,-100.,[],.12,dict(valid=True),angular_velocity=rate)


@pytest.mark.parametrize('angle',[0.,.4,1.0])
def test_first_missing_line_below_gate_keeps_turning(angle):
    c,p=turning(angle=angle)
    command=frame(p,c)
    assert command[0]>0 and p.line_seen
    assert p.debug['feedback']['steering_rad']==command[1]
    assert frame(p,c,1.1,False)[2]=='parking_turn'
    assert not p.debug['white_line']['stop_armed']


def test_first_missing_line_after_gate_stops():
    c,p=turning(angle=1.4);frame(p,c)
    assert frame(p,c,1.1,False)==(0.,0.,'parked')
    assert p.reason=='WHITE_STRAIGHT_LINE_DISAPPEARED_STOP'


def test_line_loss_precedes_timeout_distance_orientation_and_pd():
    c,p=turning();frame(p,c)
    p.turn_angle=p.c['white_loss_min_angle']
    p.turn_distance=p.c['turn_distance_limit']+1.
    p.selected['normal']=[1.,0.]  # 방향 보조 정지보다도 선 소실 사유가 우선
    def forbidden(*args,**kwargs):raise AssertionError('controller must not run after white loss')
    p.pd.update=p.pd.command=p._clear=forbidden
    result=frame(p,c,1.+p.c['turn_timeout']+.1,False)
    assert result==(0.,0.,'parked') and p.reason=='WHITE_STRAIGHT_LINE_DISAPPEARED_STOP'


def test_same_camera_frame_is_not_falsely_treated_as_line_loss():
    c,p=turning();frame(p,c)
    p.turn_angle=p.c['white_loss_min_angle']
    assert frame(p,c,1.1,False,image_stamp=1.)[2]=='parking_turn'
    assert frame(p,c,1.2,False)==(0.,0.,'parked')


def test_never_acquired_line_stops_as_failure_not_completion():
    c,p=turning()
    assert frame(p,c,1.,False)[2]=='parking_turn'
    assert frame(p,c,2.6,False)==(0.,0.,'parking_failed')
    assert p.reason=='WHITE_LINE_NEVER_SEEN'


@pytest.mark.parametrize('phase',['arc','align'])
@pytest.mark.parametrize('offset',[-.06,0.,.06])
def test_visual_pd_cannot_weaken_nominal_turn_even_when_enabled(phase,offset):
    c=config();c['answer']['parking']['pd']['enabled']=True;c['answer']['parking']['turn_done_heading']=None
    c,p=turning(c,phase=phase);image,target=scene(c,y=offset)
    p.selected=target;p.arc_released=True;p.turn_angle=math.pi/2
    commands=[frame(p,c,1.+i*.1,image=image) for i in range(1,8)]
    assert p.pd.debug['active']
    assert all(speed>0 and -c['robot']['max_steering']<=steer<=p.c['turn_steering'] for speed,steer,state in commands)


def test_latest_main_calibration_and_defaults_are_preserved():
    c=config();p=Parking(c)
    assert p._turn_lead()==pytest.approx(.615)
    assert p.c['pd']['enabled'] is False
    assert p.c['turn_done_heading']==.05
    assert p.c['cruise_search_enabled'] is True


def test_orientation_backup_stops_when_white_remains_visible():
    c,p=turning();p.selected['normal']=[math.cos(-.1),math.sin(-.1)]
    assert frame(p,c)[0]>0
    assert frame(p,c,1.1,rate=-.6)==(0.,0.,'parked')
    assert p.reason=='TURN_COMPLETE_STOP'


def test_main_motor_publisher_stops_on_loss_and_never_restarts():
    import fake_ros
    c=config()
    with patch.dict(sys.modules):
        fake_ros.install(lambda node:None)
        path=Path(__file__).resolve().parents[1]/'ericar_driver/node.py'
        spec=importlib.util.spec_from_file_location('ericar_driver._auto2_node',path)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        node=module.DriverNode(c);_,p=turning(c)
        node.loop.driver.parking=p;node.loop.driver.state='parking_turn'
        node.loop.lane_cache=dict(valid=True,error=0.,heading=0.,stop_distance=None,forward=.6,abnormal=False,solid={},lane_mask=np.zeros((8,8),np.uint8),stamp=1.)
        node.loop.lane_stamp=1.;node.scan=laser(c,boxes=[]);node.velocity=.12
        commands=[]
        for t,present in ((1.,True),(1.1,False),(1.2,True)):
            if not present:p.turn_angle=p.c['white_loss_min_angle']
            fake_ros.Clock.now_s=t;node.image=picture(present)
            node.image_stamp=node.scan_stamp=node.odom_stamp=node.detection_stamp=t
            node.control()
            commands.append([m for topic,m in node.published if topic==c['topics']['drive']][-1])
        assert commands[0].speed>0 and commands[0].steering_angle<=-.35+c['robot']['steering_offset']
        assert commands[1].speed==commands[2].speed==0
        assert node.loop.driver.state=='parked'
