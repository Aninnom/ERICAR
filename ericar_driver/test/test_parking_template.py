"""White start/side anchor renders fixed cells; LiDAR only marks occupancy."""
from unittest.mock import patch

import numpy as np

from test_parking_turn_done import config
from ericar_driver.parking import Parking
from ericar_driver.parking_template import slots_from_anchor


def landmarks(start=True,anchor=True):
    anchors=[] if not anchor else [dict(anchor=[.50,-.20],tangent=[1.,0.],normal=[0.,-1.],
                                          confidence='COLOR_TRANSITION',length=.8,pixel=[0,0,1,1],
                                          support_pixel=[0,0,1,1])]
    return dict(start_seen=start,anchors=anchors,start_segments=[],white_longitudinal=[],
                yellow_longitudinal=[],reason='ANCHOR_FOUND' if anchors else 'WAITING_FOR_WHITE_SIDE')


def test_known_anchor_renders_three_metric_cells():
    cells=slots_from_anchor([.50,-.20],[1.,0.],[0.,-1.],.50,.59,3)
    assert [cell['slot_index'] for cell in cells]==[0,1,2]
    assert np.allclose([cell['entrance'] for cell in cells],[[.75,-.20],[1.25,-.20],[1.75,-.20]])
    assert all(cell['width']==.50 and cell['depth']==.59 for cell in cells)


def test_start_then_stable_anchor_selects_first_free_template_cell():
    c=config();c['answer']['parking']['template_enabled']=True;p=Parking(c);p.start(0.)
    image=np.zeros((480,640,3),np.uint8)
    def occupancy(slot,*_):
        return ('OCCUPIED' if slot['slot_index']==0 else 'FREE'),dict(reason='TEST')
    with patch('ericar_driver.parking.parking_template_landmarks',return_value=landmarks()), \
         patch('ericar_driver.parking.classify_camera_slot',side_effect=occupancy):
        assert not p._observe_template(image,1.,{},1.,1.,0.,0.)
        assert not p._observe_template(image,1.1,{},1.1,1.1,0.,0.)
        assert p._observe_template(image,1.2,{},1.2,1.2,0.,0.)
    assert p.template_armed and len(p.template_slots)==3
    assert p.selected['source']=='parking_template' and p.selected['slot_index']==1
    assert p.template_debug['evaluations'][0]['state']=='OCCUPIED'
    assert p.template_debug['evaluations'][1]['control_applied']


def test_template_waits_when_no_side_anchor_exists():
    c=config();c['answer']['parking']['template_enabled']=True;p=Parking(c);p.start(0.)
    image=np.zeros((480,640,3),np.uint8)
    with patch('ericar_driver.parking.parking_template_landmarks',return_value=landmarks(anchor=False)):
        p._observe_template(image,1.,{},1.,1.,0.,0.)
        p._observe_template(image,1.1,{},1.1,1.1,0.,0.)
    assert p.template_armed and not p.template_slots and p.selected is None
    assert p.template_debug['reason']=='WAITING_FOR_SIDE_ANCHOR'


def test_template_mode_never_uses_obstacle_pair_gap_as_target():
    c=config();c['answer']['parking']['template_enabled']=True;p=Parking(c);p.start(0.)
    image=np.zeros((480,640,3),np.uint8)
    scan=dict(valid_fraction=1.,parking_rays=dict(measured=np.ones(8,bool)),parking_points=np.empty((0,2)),points=np.empty((0,2)))
    with patch.object(p,'_observe_template',return_value=False), \
         patch.object(p,'_find_gap',side_effect=AssertionError('gap path must stay disabled')), \
         patch.object(p,'_clear',return_value=True):
        result=p.step(1.,image,1.,scan,1.,None,1.,[],0.,dict(valid=True),search_command=(.12,0.))
    assert result[2] in ('parking_cruise','parking_scan') and p.selected is None


def test_selected_template_cell_is_rejected_after_two_occupied_frames():
    c=config();c['answer']['parking']['template_enabled']=True;p=Parking(c);p.start(0.)
    image=np.zeros((480,640,3),np.uint8);current=['FREE']
    def occupancy(slot,*_):
        state=current[0] if slot['slot_index']==0 else 'UNKNOWN'
        return state,dict(reason='TEST')
    with patch('ericar_driver.parking.parking_template_landmarks',return_value=landmarks()), \
         patch('ericar_driver.parking.classify_camera_slot',side_effect=occupancy):
        p._observe_template(image,1.,{},1.,1.,0.,0.)
        p._observe_template(image,1.1,{},1.1,1.1,0.,0.)
        assert p._observe_template(image,1.2,{},1.2,1.2,0.,0.) and p.selected['slot_index']==0
        current[0]='OCCUPIED';p._observe_template(image,1.3,{},1.3,1.3,0.,0.)
        assert p.selected is not None
        p._observe_template(image,1.4,{},1.4,1.4,0.,0.)
    assert p.selected is None and 0 in p.rejected_template_slots


def test_planned_turn_path_is_reported_for_three_cell_debug_view():
    c=config();p=Parking(c);p.selected=slots_from_anchor([.5,-.2],[1.,0.],[0.,-1.],.5,.59,3)[0]
    path=p._planned_turn_path(.1)
    assert path.shape==(25,3) and np.allclose(path[0,:2],[.1,0.])


def test_obstacle_on_planned_right_arc_blocks_commit():
    c=config();p=Parking(c);p.selected=slots_from_anchor([.5,-.2],[1.,0.],[0.,-1.],.5,.59,3)[0]
    lidar=np.array([c['robot']['lidar']['x'],c['robot']['lidar'].get('y',0.)])
    scan=dict(points=np.array([[.6,-.3]])-lidar)
    assert not p._turn_path_clear(scan)
    assert not p.path_debug['clear'] and p.path_debug['clearance_m']<p.path_debug['required_m']


def test_far_free_cell_waits_until_within_reach():
    """멀리 있는 빈 칸은 회전 시작까지 template_select_max_remaining 안에 들어올 때 선택 (11/18 redecide_1118_06)"""
    c=config();c['answer']['parking']['template_enabled']=True
    c['answer']['parking']['template_select_max_remaining']=1.0;p=Parking(c);p.start(0.)
    image=np.zeros((480,640,3),np.uint8)
    def occupancy(slot,*_):
        return ('FREE' if slot['slot_index']==2 else 'OCCUPIED'),dict(reason='TEST')
    with patch('ericar_driver.parking.parking_template_landmarks',return_value=landmarks()), \
         patch('ericar_driver.parking.classify_camera_slot',side_effect=occupancy):
        for step in range(3):
            assert not p._observe_template(image,1.+.1*step,{},1.+.1*step,1.+.1*step,0.,0.)
        assert p.selected is None and p.template_debug['evaluations'][2]['state']=='FREE'
        for slot in p.template_slots:p._move_target(slot,1.,.2,0.)     # 0.2 m 전진: 셋째 칸 회전 시작까지 1.135 → 0.935 m
        assert p._observe_template(image,1.3,{},1.3,1.3,0.,0.)
    assert p.selected['slot_index']==2


def _approach_until(p,c,point,steps=40):
    """선택한 틀 칸으로 0.12 m/s 접근. point(차체 좌표, 첫 주기 기준)가 있으면 차가 가는 만큼 뒤로 밀려 보이게 함. 반환: 주기별 상태"""
    lidar=np.array([c['robot']['lidar']['x'],c['robot']['lidar'].get('y',0.)]);states=[];now=10.
    p.state='parking_approach';p.approach_started=now;p.approach_distance=0.;p.last_tick=now
    for step in range(steps):
        now+=.05
        points=np.zeros((0,2)) if point is None else np.array([[point[0]-.006*(step+1),point[1]]])-lidar
        p._approach(now,.05,.12,0.,dict(points=points))
        states.append(p.state)
        if p.state!='parking_approach':break
    return states


def _selected_first_cell(c,ahead):
    """회전 시작까지 ahead(m) 남은 첫 칸"""
    p=Parking(c);lead=p._turn_lead()
    p.selected=slots_from_anchor([lead+ahead-.25,-.38],[1.,0.],[0.,-1.],.5,.59,3)[0]
    return p


def test_sign_base_at_near_corner_does_not_block_measured_radius_arc():
    """첫 칸 가까운 모서리의 주차 표지판 받침 (11/18 redecide_1118_07 146.5 s 라이다 점): 실측 반경으로 검사하면 통과"""
    c=config();p=_selected_first_cell(c,0.)
    lidar=np.array([c['robot']['lidar']['x'],c['robot']['lidar'].get('y',0.)])
    assert p._turn_path_clear(dict(points=np.array([[.37,-.39]])-lidar)),p.path_debug
    assert not p._turn_path_clear(dict(points=np.array([[.60,-.30]])-lidar))      # 경로 한가운데 물체는 그대로 막힘


def test_blocked_turn_waits_then_turns_when_near_corner_object_clears():
    """0.1 s 전 스캔의 받침 점(0.43, -0.38)으로 막혀도 칸을 버리지 않고 조금 더 가서 회전 시작 (redecide_1118_08 22.8 s)"""
    c=config();c['answer']['parking']['template_enabled']=True;p=_selected_first_cell(c,.03)
    states=_approach_until(p,c,(.45,-.38))
    assert states[-1]=='parking_turn' and 0 not in p.rejected_template_slots,(states,p.path_debug)


def test_blocked_turn_rejects_cell_when_obstacle_stays_on_arc():
    c=config();c['answer']['parking']['template_enabled']=True;p=_selected_first_cell(c,.03)
    states=_approach_until(p,c,(.75,-.45))
    assert states[-1]=='parking_scan' and 0 in p.rejected_template_slots and p.reason=='SELECTED_SLOT_TURN_PATH_BLOCKED',(states,p.reason)
