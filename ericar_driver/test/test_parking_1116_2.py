"""가로선 조기 정지 없이 회전 진입 속도 안정화 검증"""
import pytest
from test_fast_parking1 import setup
from test_parking_cruise_arc import tick
from test_parking_auto2 import turning,frame
from test_parking_2 import picture


def test_one_low_speed_sample_does_not_start_turn_and_keeps_rolling():
    c,p=setup(0.)
    assert tick(p,c,1.,v=.14,command=(.4,0.))[2]=='parking_approach'
    command=tick(p,c,1.1,v=.12,command=(.4,0.))
    assert command[0]==.12 and command[2]=='parking_approach'
    assert not p.approach_debug['entry_speed_ready']
    assert tick(p,c,1.36,v=.12,command=(.4,0.))[2]=='parking_turn'


def test_speed_spike_and_sensor_pause_reset_stable_window():
    c,p=setup(.2)
    tick(p,c,1.,v=.12)
    tick(p,c,1.1,v=.15)
    assert p.entry_stable_since is None
    tick(p,c,1.2,v=.12)
    assert p.entry_stable_since==1.2
    p.pause(1.3)
    assert p.entry_stable_since is None


def test_settling_buffer_lowers_approach_cap_without_zero_command():
    c,p=setup(.3)
    first=tick(p,c,1.,v=.3,command=(.4,0.))
    assert first[2]=='parking_approach'
    assert .12<first[0]<.4
    p.approach_debug['remaining_m']=.14
    assert p._approach_speed(.4)==pytest.approx(.12)


def test_close_horizontal_line_does_not_add_a_new_stop_rule():
    import cv2
    c,p=turning()
    a=picture();cv2.line(a,(50,450),(590,450),(255,255,255),8)
    assert frame(p,c,image=a)[0]==.12
    assert p.state=='parking_turn'
    p.turn_angle=p.c['white_loss_min_angle']
    assert frame(p,c,1.1,present=False)==(0.,0.,'parked')


def test_turn_geometry_and_stop_reference_remain_unchanged():
    c,p=setup()
    assert p._turn_lead()==.615
    assert p.c['approach_start_offset']==0.
    assert p.c['turn_speed']==.12 and p.c['turn_steering']==-.35
    assert p.c['white_loss_frames']==1 and not p.c['pd']['enabled']
