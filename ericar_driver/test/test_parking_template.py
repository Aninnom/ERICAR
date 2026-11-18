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
    # 전환점(0.50)은 차로 가로 시작선(0.10) 뒤 0.40 m → 시작선 관문 통과
    return dict(start_seen=start,start_x=.10 if start else None,anchors=anchors,start_segments=[],white_longitudinal=[],
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
    # 회전 시작점까지 직진 구간 + 실측 반경 원호 25점
    assert path.shape[1]==3 and np.allclose(path[0,:2],[0.,0.]) and np.allclose(path[-25,:2],[.1,0.])
    radius=c['answer']['parking']['turn_radius']
    assert np.allclose(np.hypot(path[-25:,0]-.1,path[-25:,1]+radius),radius)


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
    row=c['answer']['parking']['template_start_to_row']  # 11/19 경기장 0.54 (표지판 기둥 대조)
    assert np.allclose(p.template_slots[0]['entrance'],[.60+row+.25,-.24],atol=.02),p.template_slots[0]['entrance']
    assert p.template_debug['anchors'][0]['confidence']=='START_LINE_OFFSET' if p.template_debug['anchors'] else True


def test_selected_cell_is_kept_during_turn_even_if_it_reads_occupied():
    """회전 중에는 선택 칸을 버리지 않음 (redecide_1118_15: 회전 중 selected가 None이 되어 주행 노드 종료)"""
    c=config();c['answer']['parking']['template_enabled']=True;p=Parking(c);p.start(0.)
    image=np.zeros((480,640,3),np.uint8)
    state=dict(value='FREE')


def test_points_inside_current_body_do_not_cancel_turn():
    """11/18: 칸 주변 점은 0.38 m 이상 떨어졌는데 clearance 0.0으로 회전 취소 → 자기 반사 점 제외"""
    c=config();p=Parking(c);p.selected=slots_from_anchor([.5,-.2],[1.,0.],[0.,-1.],.5,.59,3)[0]
    lidar=np.array([c['robot']['lidar']['x'],c['robot']['lidar'].get('y',0.)])
    own=np.array([[.15,.05],[.0,-.08]])
    far=np.array([[1.1,y] for y in np.arange(-.34,-.76,-.02)])
    scan=dict(points=np.vstack((own,far))-lidar)
    assert p._turn_path_clear(scan)
    assert p.path_debug['ignored_own_points']==2 and p.path_debug['clearance_m']>.3


def test_blocked_turn_path_waits_then_turns_when_it_clears():
    """한 번 막힘으로 칸을 버리지 않음: 정지 후 다시 확인, 비면 바로 회전"""
    c=config();p=Parking(c);p.start(0.)
    slot=slots_from_anchor([.5,-.2],[1.,0.],[0.,-1.],.5,.59,3)[0]
    slot['source']='parking_template';slot['entrance'][0]=p._turn_lead()
    p.selected=slot;p.state='parking_approach';p.approach_started=0.;p.approach_distance=0.
    lidar=np.array([c['robot']['lidar']['x'],c['robot']['lidar'].get('y',0.)])
    blocked=dict(points=np.array([[.5,-.35]])-lidar);clear=dict(points=np.array([[3.,2.]])-lidar)
    turn_speed=c['answer']['parking']['turn_speed']
    p.entry_stable_since=-10.
    p.scan_stamp=1.;p._approach(1.,0.,turn_speed,0.,blocked)
    assert p.state=='parking_approach' and p.reason=='TURN_PATH_BLOCKED_WAITING' and p.selected is not None
    p.scan_stamp=1.1;p._approach(1.1,0.,0.,0.,blocked)
    assert p.state=='parking_approach' and p.path_block_since is not None
    p.scan_stamp=1.2;p._approach(1.2,0.,0.,0.,clear)
    assert p.state=='parking_turn' and p.path_block_since is None


def test_persistently_blocked_turn_path_rejects_slot():
    c=config();p=Parking(c);p.start(0.)
    slot=slots_from_anchor([.5,-.2],[1.,0.],[0.,-1.],.5,.59,3)[0]
    slot['source']='parking_template';slot['entrance'][0]=p._turn_lead()
    p.selected=slot;p.state='parking_approach';p.approach_started=0.;p.approach_distance=0.
    p.entry_stable_since=-10.
    lidar=np.array([c['robot']['lidar']['x'],c['robot']['lidar'].get('y',0.)])
    blocked=dict(points=np.array([[.5,-.35]])-lidar)
    t=1.
    p.scan_stamp=t;p._approach(t,0.,c['answer']['parking']['turn_speed'],0.,blocked)
    while p.state=='parking_approach' and t<3.:
        t+=.1;p.scan_stamp=t;p._approach(t,0.,0.,0.,blocked)
    assert p.state=='parking_scan' and p.reason=='SELECTED_SLOT_TURN_PATH_BLOCKED' and 0 in p.rejected_template_slots
    assert t>=1.+c['answer']['parking']['turn_path_block_seconds']-1e-9
    assert p.path_debug['blocker']['point'] is not None


def test_near_transition_refines_far_latched_template():
    """1.3 m에서 짧게 래치된 템플릿을 0.70~0.95 m 전환점으로 진행 방향만 보정"""
    c=config();c['answer']['parking']['template_enabled']=True;p=Parking(c);p.start(0.)
    p.template_slots=slots_from_anchor([.70,-.28],[1.,0.],[0.,-1.],.5,.59,3,.513)
    near=dict(anchor=[.85,-.29],tangent=[1.,0.],normal=[0.,-1.],confidence='COLOR_TRANSITION')
    for _ in range(6):p._refine_template([near],0.,0.,0.)
    assert abs(p.template_slots[0]['polygon'][0][0]-.85)<.01
    assert abs(p.template_slots[0]['polygon'][0][1]+.28)<1e-9  # 깊이 방향은 유지
    far=dict(near,anchor=[1.3,-.29]);before=p.template_slots[0]['polygon'][0][0]
    p._refine_template([far],0.,0.,0.)
    assert p.template_slots[0]['polygon'][0][0]==before
    white=dict(near,anchor=[.8,-.29],confidence='WHITE_START_ONLY')
    p._refine_template([white],0.,0.,0.)
    assert p.template_slots[0]['polygon'][0][0]==before


def test_transition_before_start_line_is_rejected():
    """11/18 redecide_1118_14: 반사된 노란 차선 위 가짜 전환점(시작선보다 앞)으로 주차장 전에 회전"""
    c=config();c['answer']['parking']['template_enabled']=True;p=Parking(c);p.start(0.)
    image=np.zeros((480,640,3),np.uint8)
    fake=landmarks();fake['start_x']=1.60  # 시작선 1.6 m, 전환점 0.5 m → 시작선 앞
    with patch('ericar_driver.parking.parking_template_landmarks',return_value=fake):
        for stamp in (1.,1.1,1.2,1.3):p._observe_template(image,stamp,{},stamp,stamp,0.,0.)
    assert p.template_armed and not p.template_slots
    assert p.start_debug['rejected']==1 and p.start_debug['accepted']==0


def test_transition_far_right_or_skewed_is_rejected():
    c=config();c['answer']['parking']['template_enabled']=True;p=Parking(c);p.start(0.)
    image=np.zeros((480,640,3),np.uint8)
    far=landmarks();far['anchors'][0]=dict(far['anchors'][0],anchor=[.5,-.95])
    skew=landmarks();skew['anchors'][0]=dict(skew['anchors'][0],tangent=[.8,-.6],normal=[-.6,-.8])
    for fake in (far,skew):
        with patch('ericar_driver.parking.parking_template_landmarks',return_value=fake):
            for stamp in (1.,1.1,1.2,1.3):p._observe_template(image,stamp,{},stamp,stamp,0.,0.)
        assert not p.template_slots


def test_start_line_track_keeps_passed_line_after_latch():
    """래치 뒤 시작선이 카메라 아래로 지나가면 오도메트리로 이동, 칸 안쪽 다른 가로선으로 교체하지 않음"""
    c=config();c['answer']['parking']['template_enabled']=True;p=Parking(c);p.start(0.)
    image=np.zeros((480,640,3),np.uint8)
    with patch('ericar_driver.parking.parking_template_landmarks',return_value=landmarks()):
        for stamp in (1.,1.1,1.2):p._observe_template(image,stamp,{},stamp,stamp,0.,0.)
    assert p.template_slots
    other=landmarks();other['start_x']=1.2
    with patch('ericar_driver.parking.parking_template_landmarks',return_value=other):
        p._observe_template(image,1.3,{},1.3,1.3,1.,0.)  # 0.1 s × 1 m/s 전진
    assert abs(p.start_debug['start_x']-0.)<1e-6


def test_glare_yellow_is_not_white_for_template():
    """반사된 노란 선(채도 약 60)은 흰 선분이 아님, 진짜 흰 선(채도 0)은 유지"""
    from ericar_driver.parking_template import _mask
    import cv2
    c=config();cfg=c['answer']['parking']
    image=np.zeros((480,640,3),np.uint8)
    glare=cv2.cvtColor(np.uint8([[[20,60,240]]]),cv2.COLOR_HSV2BGR)[0,0].tolist()
    cv2.line(image,(100,470),(200,250),glare,8);cv2.line(image,(400,470),(500,250),(255,255,255),8)
    white=_mask(image,cfg['grid_white_min'],cfg['grid_white_max'],.4)
    exclude=_mask(image,cfg['template_white_exclude_min'],cfg['template_white_exclude_max'],.4)
    kept=cv2.bitwise_and(white,cv2.bitwise_not(exclude))
    assert white[400,:300].any()  # 기존 흰색 범위는 반사 노란 선을 흰색으로 봄
    assert not kept[400,:300].any() and kept[400,300:].any()


def _cruise(p,armed=True,evaluations=(),stamp=1.,velocity=.3,command=.4):
    """_observe_template 결과를 고정하고 step 한 번 (scan·image 정상)"""
    image=np.zeros((480,640,3),np.uint8)
    scan=dict(valid_fraction=1.,parking_rays=dict(measured=np.ones(8,bool)),
              parking_points=np.empty((0,2)),points=np.empty((0,2)))
    def observe(*_):
        p.template_armed=armed;p.template_debug=dict(evaluations=list(evaluations));return False
    with patch.object(p,'_observe_template',side_effect=observe),patch.object(p,'_clear',return_value=True):
        return p.step(stamp,image,stamp,scan,stamp,None,stamp,[],velocity,dict(valid=True),
                      search_command=(command,0.))


def test_start_line_alone_keeps_lane_speed_with_parking_cap():
    """11/18 redecide_1118_33: 시작선만 보고 0.1 s 만에 0.12 m/s 서행 → 회전까지 22 s"""
    c=config();c['answer']['parking']['template_enabled']=True;p=Parking(c);p.start(0.)
    speed,_,state=_cruise(p)
    assert state=='parking_cruise' and speed==c['answer']['parking']['template_cruise_max_speed']


def test_template_slot_inside_braking_distance_slows():
    c=config();c['answer']['parking']['template_enabled']=True;p=Parking(c);p.start(0.)
    near=[dict(slot_index=0,state='UNKNOWN',turn_start_remaining_m=.2)]
    _cruise(p,evaluations=near)
    assert p.state=='parking_scan' and p.slow_reason=='TARGET_BRAKING_DISTANCE'


def test_occupied_or_passed_slots_do_not_trigger_braking():
    c=config();c['answer']['parking']['template_enabled']=True;p=Parking(c);p.start(0.)
    evaluations=[dict(slot_index=0,state='OCCUPIED',turn_start_remaining_m=.1),
                 dict(slot_index=1,state='FREE',turn_start_remaining_m=-.3),
                 dict(slot_index=2,state='FREE',turn_start_remaining_m=1.5)]
    _cruise(p,evaluations=evaluations)
    assert p.state=='parking_cruise' and abs(p.nearest_start-1.5)<1e-9


def test_sign_leaving_view_slows_in_template_mode():
    c=config();c['answer']['parking']['template_enabled']=True;p=Parking(c);p.start(0.)
    p.sign.passed=True
    with patch.object(p.sign,'observe'):_cruise(p)
    assert p.state=='parking_scan' and p.slow_reason=='NEAR_SIGN_PASSED'


def test_armed_distance_fallback_slows_without_other_evidence():
    c=config();c['answer']['parking']['template_enabled']=True;p=Parking(c);p.start(0.)
    limit=c['answer']['parking']['template_cruise_armed_distance']
    t=1.
    while p.state=='parking_cruise' and t<40.:
        t+=.5;_cruise(p,stamp=t,velocity=.3)  # 0.5 s × 0.3 m/s = 0.15 m/step
    assert p.state=='parking_scan' and p.slow_reason=='START_LINE_DISTANCE_FALLBACK'
    assert limit<=p.armed_distance<=limit+.16


def test_selected_cell_is_kept_during_turn_even_if_it_reads_occupied():
    """회전 중에는 선택 칸을 버리지 않음 (혁준 72ea089, redecide_1118_15: 회전 중 selected None → 주행 노드 종료)"""
    c=config();c['answer']['parking']['template_enabled']=True;p=Parking(c);p.start(0.)
    image=np.zeros((480,640,3),np.uint8);state=dict(value='FREE')
    def occupancy(slot,*_):return (state['value'] if slot['slot_index']==0 else 'OCCUPIED'),dict(reason='TEST')
    with patch('ericar_driver.parking.parking_template_landmarks',return_value=landmarks()), \
         patch('ericar_driver.parking.classify_camera_slot',side_effect=occupancy):
        for step in range(3):p._observe_template(image,1.+.1*step,{},1.+.1*step,1.+.1*step,0.,0.)
        assert p.selected is not None and p.selected['slot_index']==0
        p.state='parking_turn';state['value']='OCCUPIED'
        for step in range(3,7):p._observe_template(image,1.+.1*step,{},1.+.1*step,1.+.1*step,0.,0.)
    assert p.selected is not None and 0 not in p.rejected_template_slots




def test_white_loss_before_minimum_angle_does_not_stop_turn():
    """11/18 redecide_1118_35: 28°에 선 소실 → 65° 도달 즉시 정지(덜 들어감). 이제 방향 조건까지 진행"""
    c=config();c['answer']['parking']['template_enabled']=True;p=Parking(c);p.start(0.)
    p.line_seen=True;p.line_seen_armed=False;p.loss_frames=30;p.turn_angle=1.2
    assert not (p.line_seen_armed and p.loss_frames>=1)
    lines=[[0,0,1,1]]
    with patch('ericar_driver.parking.parking_straight_lines',return_value=(None,lines)):
        p._white(np.zeros((4,4,3),np.uint8),1.)
        assert not p.line_seen_armed  # 69°: 템플릿 모드 정지 각도(80°) 전에 보인 선은 정지 근거 아님 (track_test_1118_02 71° 정지)
        p.turn_angle=1.45;p._white(np.zeros((4,4,3),np.uint8),1.1)
    assert p.line_seen_armed and p.loss_frames==0  # 80° 뒤에 다시 보임 → 이후 소실은 정지 근거


def _lot_frames(p,stamps,shift=0.):
    """틀을 직접 넣고(시작선·기준점 검출 생략) 칸 0만 FREE로 판정하며 _observe_template 반복"""
    image=np.zeros((480,640,3),np.uint8)
    empty=dict(start_seen=False,start_x=None,anchors=[],start_segments=[],white_longitudinal=[],
               yellow_longitudinal=[],right_ground=[],reason='TEST')
    def occupancy(slot,*_):return ('FREE' if slot['slot_index']==0 else 'OCCUPIED'),dict(reason='TEST')
    for stamp in stamps:
        if shift:
            for slot in p.template_slots:
                slot['entrance']=[slot['entrance'][0]+shift,slot['entrance'][1]]
                slot['polygon']=[[x+shift,y] for x,y in slot['polygon']]
        with patch('ericar_driver.parking.parking_template_landmarks',return_value=empty), \
             patch('ericar_driver.parking.classify_camera_slot',side_effect=occupancy):
            p._observe_template(image,stamp,{},stamp,stamp,0.,0.)


def _lot_parking(entrance_x):
    c=config();c['answer']['parking']['template_enabled']=True;p=Parking(c);p.start(0.)
    p.template_armed=True
    p.template_slots=slots_from_anchor([entrance_x-.25,-.25],[1.,0.],[0.,-1.],.5,.59,3,.513)
    return c,p


def test_first_slot_shift_survives_resync():
    """11/19: 첫 칸 미루기가 선택 순간에만 적용되고 다음 프레임 보정 틀로 덮여 사라지던 문제"""
    c,p=_lot_parking(1.2);shift=c['answer']['parking']['template_first_slot_shift']
    _lot_frames(p,[1.,1.1,1.2])
    assert p.selected['slot_index']==0 and p.selected.get('first_slot_shift_m')==shift
    fused=p.template_slots[0]['entrance'][0]+(p.template_alignment[0] if p.template_alignment is not None else 0.)
    assert abs(p.selected['entrance'][0]-(fused+shift))<1e-6


def test_target_is_frozen_near_turn_start():
    """track_testv2_1118_09: 회전 시작 프레임에 목표가 0.126 m 튐 → 회전점 0.25 m 안에서는 보정 틀로 다시 맞추지 않음"""
    c,p=_lot_parking(Parking(config())._turn_lead()+.10)
    _lot_frames(p,[1.,1.1])
    before=list(p.selected['entrance'])
    _lot_frames(p,[1.2],shift=-.12)
    assert p.selected['entrance']==before and p.selected.get('target_frozen')


def test_target_jump_is_rate_limited_far_from_turn():
    c,p=_lot_parking(Parking(config())._turn_lead()+.80)
    _lot_frames(p,[1.,1.1])
    before=p.selected['entrance'][0]
    _lot_frames(p,[1.2],shift=-.12)
    limit=c['answer']['parking']['template_target_max_jump']
    assert abs(abs(p.selected['entrance'][0]-before)-limit)<1e-6


def test_parked_check_reports_side_gaps():
    c=config();p=Parking(c)
    lidar=np.array([c['robot']['lidar']['x'],0.]);half=c['robot']['width']/2
    scan=dict(points=np.array([[.1,.40],[.2,.42],[.0,-.30],[1.0,0.]])-lidar)
    check=p._parked_check(scan)
    assert abs(check['left_gap_m']-(.40-half))<1e-9 and abs(check['right_gap_m']-(.30-half))<1e-9
