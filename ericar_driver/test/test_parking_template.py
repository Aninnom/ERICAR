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


def _landmarks_at(anchor_x,start_x,slope=0.):
    norm=(1+slope*slope)**.5
    anchors=[dict(anchor=[anchor_x,-.25],tangent=[1/norm,slope/norm],normal=[slope/norm,-1/norm],
                  confidence='COLOR_TRANSITION',length=.5,pixel=[0,0,1,1],support_pixel=[0,0,1,1])]
    return dict(start_seen=True,anchors=anchors,start_x=start_x,start_segments=[],white_longitudinal=[],
                yellow_longitudinal=[],reason='ANCHOR_FOUND')


def _feed(p,items):
    image=np.zeros((480,640,3),np.uint8)
    def unknown(slot,*_):return 'UNKNOWN',dict(reason='TEST')
    for index,item in enumerate(items):
        with patch('ericar_driver.parking.parking_template_landmarks',return_value=item), \
             patch('ericar_driver.parking.classify_camera_slot',side_effect=unknown):
            p._observe_template(image,1.+.1*index,{},1.+.1*index,1.+.1*index,0.,0.)


def test_anchor_before_start_line_is_ignored():
    """옅은 노란 차로 선이 흰색으로 읽혀 시작선보다 앞(가까운 쪽)에 생긴 기준점은 버림 (11/18 redecide_1118_12 17.3 s)"""
    c=config();c['answer']['parking']['template_enabled']=True;c['answer']['parking']['template_anchor_from_start']=False;p=Parking(c);p.start(0.)
    _feed(p,[_landmarks_at(.88,1.73)]*4)
    assert p.template_armed and not p.template_slots and p.template_debug['anchors_rejected']==1
    assert abs(p.template_debug['start_x']-1.73)<1e-6
    _feed(p,[_landmarks_at(2.15,1.73)]*2)           # 시작선 0.42 m 뒤의 진짜 기준점
    assert len(p.template_slots)==3 and abs(p.template_slots[0]['entrance'][0]-2.40)<.02


def test_start_line_track_ignores_later_transverse_lines():
    """시작선을 지나친 뒤 더 앞에 보이는 다른 가로 흰 선이 시작선 자리를 차지하지 않음"""
    c=config();c['answer']['parking']['template_enabled']=True;c['answer']['parking']['template_anchor_from_start']=False;p=Parking(c);p.start(0.)
    _feed(p,[_landmarks_at(5.,.70,slope=9.)]*3)     # 기준 후보는 방향 조건으로 제외, 시작선만 기록 (3번 봐야 굳음)
    assert abs(p.template_debug['start_x']-.70)<1e-6
    _feed(p,[_landmarks_at(.95,1.80)]*2)            # 1.1 m 더 앞의 가로선은 무시 → 시작선 0.70 기준으로 0.95는 인정
    assert abs(p.template_debug['start_x']-.70)<1e-6 and len(p.template_slots)==3


def test_skewed_far_anchor_is_ignored():
    c=config();c['answer']['parking']['template_enabled']=True;c['answer']['parking']['template_anchor_from_start']=False;p=Parking(c);p.start(0.)
    _feed(p,[_landmarks_at(2.08,1.66,slope=.45)]*4)
    assert not p.template_slots and p.template_debug['anchors_rejected']==1


def test_row_start_comes_from_start_line_when_close():
    """줄 시작 = 시작선 + 0.45 m. 시작선이 0.65 m 안에 들어와 3번 보인 뒤 확정, 색 전환 기준점은 쓰지 않음 (11/18 redecide_1118_15)"""
    c=config();c['answer']['parking']['template_enabled']=True;p=Parking(c);p.start(0.)
    far=dict(_landmarks_at(.95,1.10),right_ground=[[[.4,-.24],[1.2,-.24]]])
    _feed(p,[far]*4)                                 # 시작선이 아직 멂: 색 전환 후보(0.95)가 있어도 확정 안 함
    assert not p.template_slots
    p=Parking(c);p.start(0.)
    near=dict(_landmarks_at(.74,.60),right_ground=[[[.4,-.24],[1.2,-.24]]])
    _feed(p,[near]*2)
    assert not p.template_slots                      # 시작선을 3번 보기 전
    _feed(p,[near]*3)
    assert len(p.template_slots)==3
    assert np.allclose(p.template_slots[0]['entrance'],[.60+.45+.25,-.24],atol=.02),p.template_slots[0]['entrance']
    assert p.template_debug['anchors'][0]['confidence']=='START_LINE_OFFSET' if p.template_debug['anchors'] else True


def test_selected_cell_is_kept_during_turn_even_if_it_reads_occupied():
    """회전 중에는 선택 칸을 버리지 않음 (redecide_1118_15: 회전 중 selected가 None이 되어 주행 노드 종료)"""
    c=config();c['answer']['parking']['template_enabled']=True;p=Parking(c);p.start(0.)
    image=np.zeros((480,640,3),np.uint8)
    state=dict(value='FREE')
    def occupancy(slot,*_):return (state['value'] if slot['slot_index']==0 else 'OCCUPIED'),dict(reason='TEST')
    with patch('ericar_driver.parking.parking_template_landmarks',return_value=landmarks()), \
         patch('ericar_driver.parking.classify_camera_slot',side_effect=occupancy):
        for step in range(3):p._observe_template(image,1.+.1*step,{},1.+.1*step,1.+.1*step,0.,0.)
        assert p.selected is not None and p.selected['slot_index']==0
        p.state='parking_turn';state['value']='OCCUPIED'
        for step in range(3,7):p._observe_template(image,1.+.1*step,{},1.+.1*step,1.+.1*step,0.,0.)
    assert p.selected is not None and 0 not in p.rejected_template_slots

