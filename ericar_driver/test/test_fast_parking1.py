"""빠른 접근의 상태 전이·감속 거리·목표 누락 복귀 합성 검증"""
import pytest
from test_parking_turn_done import config
from test_parking_cruise_arc import tick,sign
from ericar_driver.parking import Parking
from ericar_driver.parking_sign import slow_distance


def setup(remaining=None):
    c=config();p=Parking(c);p.start(1.)
    if remaining is not None:
        x=p._turn_lead()+remaining
        p.selected=dict(entrance=[x,-.25],normal=[0.,-1.],tangent=[1.,0.],width=.5,
                        polygon=[[x-.25,-.25],[x+.25,-.25],[x+.25,-.84],[x-.25,-.84]])
    return c,p


def test_sign_passage_alone_keeps_lane_speed_then_bounds_missing_target():
    c,p=setup()
    tick(p,c,objects=sign(.02),command=(.4,0.))
    for t in (1.2,1.4,1.6):
        assert tick(p,c,t,command=(.4,0.))==(.4,0.,'parking_cruise')
    assert p.sign.passed
    assert tick(p,c,2.0,command=(.4,0.))[2]=='parking_cruise'
    assert tick(p,c,2.61,command=(.4,0.))[2]=='parking_scan'
    assert p.slow_reason=='NO_TARGET_AFTER_SIGN_PASSAGE'


def test_distance_limit_also_bounds_no_target_grace():
    c,p=setup();p.sign.passed=True
    c['answer']['parking']['no_target_cruise_seconds']=10.
    for t in (1.,1.3,1.6):assert tick(p,c,t,v=.4,command=(.4,0.))[2]=='parking_cruise'
    assert tick(p,c,1.9,v=.4,command=(.4,0.))[2]=='parking_scan'


def test_latched_far_target_keeps_cruise_after_sign_passage():
    c,p=setup(2.);p.sign.passed=True
    for i in range(15):
        assert tick(p,c,1.+i*.1,v=.3,command=(.4,0.))[2]=='parking_cruise'
    assert p.no_target_since is None


def test_demanded_speed_triggers_braking_before_acceleration():
    c,p=setup(.3)
    assert tick(p,c,v=0.,command=(.4,0.))[2]=='parking_approach'
    assert p.slow_reason=='TARGET_BRAKING_DISTANCE'
    assert p.braking_distance==pytest.approx(slow_distance(.4,p.c))


@pytest.mark.parametrize('speed',[.2,.3,.4])
def test_ideal_braking_reaches_turn_at_low_speed(speed):
    c,p=setup(1.)
    v=speed;t=1.;slowed=False
    for i in range(500):
        command=tick(p,c,t,v=v,command=(speed,0.))
        if p.state=='parking_approach':slowed=True
        if p.state=='parking_turn':
            assert slowed and v<=p.c['turn_entry_max_speed']
            assert command[0]==.12 and command[1]==-.35
            return
        assert p.state in ('parking_cruise','parking_approach'),p.reason
        v=max(command[0],v-.30*.02) if v>command[0] else min(command[0],v+.10*.02)
        t+=.02
    pytest.fail('no turn entry')


def test_trial_cap_and_main_stop_remain_effective():
    c,p=setup(2.);p.c['test_drive_until_sign']=True
    assert tick(p,c,command=(.4,.1))[0]==.2
    assert tick(p,c,1.1,command=(0.,0.))[0]==0.


def test_stale_sensor_never_keeps_cruise_command():
    c,p=setup(2.)
    assert tick(p,c,2.,image_stamp=1.)[0]==0.
