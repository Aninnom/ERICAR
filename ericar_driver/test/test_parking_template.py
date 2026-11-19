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
    row=c['answer']['parking']['template_start_to_row']  # 11/19 경기장 0.54 (A/B 결정)
    assert np.allclose(p.template_slots[0]['entrance'],[.60+row+.25,-.24],atol=.02),p.template_slots[0]['entrance']
    assert p.template_debug['anchors'][0]['confidence']=='START_LINE_OFFSET' if p.template_debug['anchors'] else True


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


def test_slot_cap_uses_selected_slot_index():
    """11/19 parking_1119_2: 칸을 고르면 시작선 뒤 상한을 칸 번호별로, 못 골랐으면 기존 0.30"""
    c=config();cfg=c['answer']['parking'];cfg['template_enabled']=True;p=Parking(c);p.start(0.)
    p.template_armed=True
    assert p._cruise_cap()==cfg['template_cruise_max_speed']
    for slot,cap in enumerate(cfg['template_cruise_max_speed_by_slot']):
        p.selected=dict(slot_index=slot)
        assert p._cruise_cap()==min(cap,cfg['cruise_max_speed'])
    cfg['template_cruise_max_speed_by_slot']=None
    assert p._cruise_cap()==cfg['template_cruise_max_speed']


def test_sign_passed_slow_skipped_once_slot_selected():
    """11/19: 칸을 골랐으면 표지판 통과 서행 대신 감속 거리 감속 (2·3번 칸 0.12 m/s 약 8 s)"""
    c=config();cfg=c['answer']['parking'];cfg['template_enabled']=True;p=Parking(c);p.start(0.)
    p.sign.passed=True
    assert p._sign_slow()
    p.selected=dict(slot_index=1)
    assert not p._sign_slow()
    cfg['template_sign_slow_without_target_only']=False
    assert p._sign_slow()


def test_sign_slow_deferred_while_unoccupied_slot_ahead():
    """11/19 parking_1119_3: 칸을 못 골랐어도 비점유 칸 회전점이 남아 있으면 표지판 통과 서행 대신 감속 거리로"""
    c=config();cfg=c['answer']['parking'];cfg['template_enabled']=True;p=Parking(c);p.start(0.)
    p.sign.passed=True
    p.nearest_start=1.4
    assert not p._sign_slow()
    p.nearest_start=None
    assert p._sign_slow()
    cfg['template_sign_slow_defer_to_nearest']=False;p.nearest_start=1.4
    assert p._sign_slow()


def test_sign_passed_with_far_unknown_slot_keeps_cruise():
    """표지판 통과 + 칸 2 UNKNOWN 1.5 m 앞: 서행하지 않고 차선 주행(상한 0.30) 유지"""
    c=config();cfg=c['answer']['parking'];cfg['template_enabled']=True;p=Parking(c);p.start(0.)
    p.sign.passed=True
    ev=[dict(slot_index=0,state='OCCUPIED',turn_start_remaining_m=.5),
        dict(slot_index=1,state='OCCUPIED',turn_start_remaining_m=1.0),
        dict(slot_index=2,state='UNKNOWN',turn_start_remaining_m=1.5)]
    speed,_,state=_cruise(p,evaluations=ev)
    assert state=='parking_cruise' and speed==cfg['template_cruise_max_speed']


def test_smooth_start_cap_never_above_current_speed():
    """거리 비례 감속 시작 상한 = 그때 속도 (다시 빨라지지 않음)"""
    c=config();cfg=c['answer']['parking'];cfg['template_enabled']=True;p=Parking(c);p.start(0.)
    p.template_armed=True;p.selected=dict(slot_index=1,entrance=[1.5,-.26],normal=[0.,-1.],tangent=[1.,0.])
    p.last_velocity=.30
    p._slow(0.,'NEAR_SIGN_PASSED')
    assert p.smooth_approach and p.approach_speed_cap==.30
    p.approach_debug=dict(remaining_m=.95)
    assert p._approach_speed(.6)<=.30+1e-9
