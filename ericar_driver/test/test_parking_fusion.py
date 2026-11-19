"""Camera cells and LiDAR objects share one bounded, inspectable frame."""
import json
import numpy as np

from test_parking_turn_done import config
from ericar_driver.parking_fusion import fuse_template_slots
from ericar_driver.parking_template import slots_from_anchor


def scene(shift_y=.0):
    c=config();slots=slots_from_anchor([.5,-.2],[1.,0.],[0.,-1.],.5,.59,3)
    points=[]
    for x,y in ((1.25,-.25+shift_y),(1.75,-.25+shift_y)):
        points.extend([[x,y],[x+.01,y+.005],[x+.02,y-.005],[x+.03,y]])
    origin=np.array([c['robot']['lidar']['x'],c['robot']['lidar'].get('y',0.)])
    local=np.asarray(points)-origin
    return c,slots,dict(points=local,parking_points=local)


def test_bounded_alignment_brings_shifted_lidar_clusters_onto_cells():
    c,slots,scan=scene(.15)
    fused,assigned,debug,offset=fuse_template_slots(slots,scan,c['robot'],c['answer']['parking'])
    # 물체가 입구선 밖(차로 쪽)이면 그만큼만 깊이 방향 이동, 상한 template_fusion_max_normal_offset
    bound=c['answer']['parking']['template_fusion_max_normal_offset']
    assert abs(offset[0])<=c['answer']['parking']['template_fusion_max_offset'] and .10<=offset[1]<=bound+1e-9
    assert [len(part) for part in assigned]==[0,4,4]
    assert [cluster['slot_index'] for cluster in debug['clusters']]==[1,2]
    json.dumps(debug)  # telemetry must not contain NumPy scalar objects


def test_boundary_return_is_assigned_to_only_one_cell():
    c,slots,scan=scene(0.)
    # Add a compact cluster exactly around the S1/S2 longitudinal boundary.
    origin=np.array([c['robot']['lidar']['x'],c['robot']['lidar'].get('y',0.)])
    boundary=np.array([[1.,-.4],[1.01,-.4],[1.02,-.4]])-origin
    scan['points']=scan['parking_points']=np.vstack((scan['parking_points'],boundary))
    _,assigned,debug,_=fuse_template_slots(slots,scan,c['robot'],c['answer']['parking'])
    assert sum(len(part) for part in assigned)==len(scan['parking_points'])
    assert all(cluster['slot_index'] is None or 0<=cluster['slot_index']<3 for cluster in debug['clusters'])


def test_empty_scan_keeps_nominal_camera_template():
    c,slots,_=scene();scan=dict(points=np.empty((0,2)),parking_points=np.empty((0,2)))
    fused,assigned,debug,offset=fuse_template_slots(slots,scan,c['robot'],c['answer']['parking'])
    assert np.allclose(offset,[0.,0.]) and np.allclose(fused[0]['polygon'],slots[0]['polygon'])
    assert debug['reason']=='NO_LIDAR_CLUSTER_KEEP_ALIGNMENT' and not any(map(len,assigned))


def side_face(x,y0=-.34,y1=-.76,step=.02):
    """LIMO 옆면: 차 쪽(-x) 면이 깊이 방향으로 이어진 점열 (11/18 redecide_1118_08 23.04 s)"""
    return [[x,y] for y in np.arange(y0,y1,-step)]


def test_side_faces_pull_short_template_forward_without_depth_drift():
    """실차: 템플릿이 실제보다 약 0.21 m 앞(차 쪽). 옛 상한 0.15·앞면 가정은 깊이로 -0.15 끌고 감
    (융합 모델 시험이라 진행 방향 상한은 예전 0.20으로 둠. 11/19 경기장 기본값은 0.05)"""
    c=config();p=c['answer']['parking'];p['template_fusion_max_offset']=.20
    slots=slots_from_anchor([.20,-.28],[1.,0.],[0.,-1.],.5,.59,3,p['template_slot_pitch'])
    origin=np.array([c['robot']['lidar']['x'],c['robot']['lidar'].get('y',0.)])
    # 실제 칸 0 왼쪽 경계 0.41 → 칸 1·2 LIMO 옆면 = 경계 + (0.513-0.225)/2
    faces=[.41+.513+.144,.41+2*.513+.144]
    points=np.asarray(side_face(faces[0])+side_face(faces[1]))-origin
    scan=dict(points=points,parking_points=points)
    offset=None
    for _ in range(12):
        fused,assigned,debug,offset=fuse_template_slots(slots,scan,c['robot'],p,offset)
    assert .17<=offset[0]<=.25
    assert abs(offset[1])<=.02  # 옛 모델은 -0.15까지 깊게 끌려감
    assert [cluster['slot_index'] for cluster in debug['clusters']]==[1,2]
    assert len(assigned[0])==0


def test_second_row_object_behind_empty_middle_cell_does_not_pull_lot_deeper():
    """track_test_1118_04·05: 빈 가운데 칸 너머 둘째 줄 앞면(깊이 0.6~0.73 m)을 담으려 칸을 0.14 m 깊게 밀어 OCCUPIED"""
    c=config();p=c['answer']['parking']
    slots=slots_from_anchor([.20,-.36],[1.,0.],[0.,-1.],.5,.59,3,p['template_slot_pitch'])
    origin=np.array([c['robot']['lidar']['x'],c['robot']['lidar'].get('y',0.)])
    first=side_face(.20+.144,-.37,-.70)                         # 첫 칸 LIMO 옆면
    second_row=[[.20+.513+x,y] for x in (.20,.22,.24,.26) for y in (-.97,-1.0)]  # 가운데 칸 뒤 둘째 줄
    points=np.asarray(first+second_row)-origin
    scan=dict(points=points,parking_points=points)
    offset=None
    for _ in range(12):
        fused,assigned,debug,offset=fuse_template_slots(slots,scan,c['robot'],p,offset)
    assert offset[1]>=-p['template_fusion_max_deeper_offset']-1e-9 and offset[1]>-.03  # 칸 안쪽(-y)으로 거의 안 밀림
    middle=fused[1];back=middle['entrance'][1]-p['template_occupancy_depth']
    assert all(y<back for _,y in second_row)                    # 빈 칸 판정 범위(앞쪽 0.50 m) 밖


def _reference_offset(slots,clusters,config):
    """예전 반복문 정렬 (같은 규칙)으로 오프셋 한 번 계산"""
    from ericar_driver.parking_fusion import _shift_slots,_assignment,_slot_coordinates
    t=np.asarray(slots[0]['tangent']);n=np.asarray(slots[0]['normal'])
    bound=config['template_fusion_max_offset'];nb=config['template_fusion_max_normal_offset'];step=config['template_fusion_step']
    deeper=config['template_fusion_max_deeper_offset'];best=None
    for du in np.arange(-bound,bound+step*.5,step):
        for dv in np.arange(-nb,deeper+step*.5,step):
            off=du*t+dv*n;cand=_shift_slots(slots,off);assigned=[];fit=0.
            for cluster in clusters:
                idx=_assignment(cluster.mean(axis=0),cand,.06)
                if idx is None:continue
                coords=np.array([_slot_coordinates(q,cand[idx]) for q in cluster])
                if coords[:,1].min()>config['template_fusion_max_face_depth']:continue
                assigned.append(idx)
                fit-=abs(coords[:,0].min()+config['template_fusion_obstacle_half_width'])*config['template_fusion_face_weight']
                fit-=max(0.,-coords[:,1].min())*config['template_fusion_face_weight']
            score=10.*len(assigned)+2.*len(set(assigned))+fit-np.linalg.norm(off)*config['template_fusion_offset_penalty']
            if best is None or score>best[0]:best=(score,off,len(assigned))
    return best[1] if best[2] else np.zeros(2)


def test_vectorized_alignment_matches_loop_reference():
    rng=np.random.default_rng(7);c=config();p=c['answer']['parking']
    origin=np.array([c['robot']['lidar']['x'],c['robot']['lidar'].get('y',0.)])
    for _ in range(20):
        slots=slots_from_anchor([rng.uniform(.1,.6),-.3],[1.,0.],[0.,-1.],.5,.59,3,.513)
        clusters=[np.column_stack((np.full(6,rng.uniform(.3,1.9)),np.linspace(-.35,rng.uniform(-1.,-.5),6)))
                  for _ in range(rng.integers(1,4))]
        points=np.vstack(clusters)-origin
        _,_,_,offset=fuse_template_slots(slots,dict(points=points,parking_points=points),c['robot'],p,None)
        assert np.allclose(offset,_reference_offset(slots,[q for q in clusters],p),atol=1e-9)
