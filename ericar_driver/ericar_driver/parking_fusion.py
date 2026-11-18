"""Bounded camera-template/LiDAR fusion for the three parking cells."""
import numpy as np


def _body_points(scan,robot):
    points=np.asarray(scan.get('parking_points',scan.get('points',[])),float).reshape(-1,2).copy()
    if len(points):points+=np.array([robot['lidar']['x'],robot['lidar'].get('y',0.)])
    return points[np.isfinite(points).all(axis=1)]


def _shift_slots(slots,offset):
    shift=np.asarray(offset,float);result=[]
    for source in slots:
        slot=dict(source)
        slot['entrance']=(np.asarray(source['entrance'])+shift).tolist()
        slot['polygon']=(np.asarray(source['polygon'])+shift).tolist()
        slot['center']=(np.asarray(source.get('center',source['entrance']))+shift).tolist()
        result.append(slot)
    return result


def _clusters(points,distance,min_points):
    """Preserve LaserScan angular order and split at spatial discontinuities."""
    if not len(points):return []
    breaks=np.flatnonzero(np.linalg.norm(np.diff(points,axis=0),axis=1)>distance)+1
    return [part for part in np.split(points,breaks) if len(part)>=min_points]


def _slot_coordinates(point,slot):
    delta=np.asarray(point)-np.asarray(slot['entrance'])
    return float(delta@slot['tangent']),float(delta@slot['normal'])


def _assignment(point,slots,padding):
    choices=[]
    for index,slot in enumerate(slots):
        u,v=_slot_coordinates(point,slot)
        if abs(u)<=slot['width']/2+padding and -padding<=v<=slot['depth']+padding:
            # One return belongs to one cell only. Prefer the cell whose centre is closest.
            score=(u/(slot['width']/2+padding))**2+((v-slot['depth']/2)/(slot['depth']/2+padding))**2
            choices.append((score,index))
    return None if not choices else min(choices)[1]


def _membership(U,V,width,depth,padding):
    """Vectorized _assignment: U,V (...,S) cell coordinates -> chosen cell index or -1."""
    inside=(np.abs(U)<=width/2+padding)&(V>=-padding)&(V<=depth+padding)
    score=(U/(width/2+padding))**2+((V-depth/2)/(depth/2+padding))**2
    score=np.where(inside,score,np.inf)
    best=np.argmin(score,axis=-1)
    return np.where(np.isfinite(np.take_along_axis(score,best[...,None],-1)[...,0]),best,-1)


def fuse_template_slots(slots,scan,robot,config,previous=None):
    """Align a camera lot to nearby LiDAR objects, then assign every return once.

    Alignment is translation-only and bounded. Camera geometry and the known
    cell pitch remain authoritative; LiDAR cannot stretch or rotate the lot.

    11/18 redecide_1118_08: 카메라 원거리 투영이 약 0.2 m 짧아 템플릿이 실제 칸보다 앞에 생성.
    LiDAR는 칸 안 LIMO의 옆면(차 쪽 면)을 봄 → 진행 방향(tangent)은 옆면 u_min이
    -obstacle_half_width에 오도록 맞춤. 깊이 방향(normal)은 물체가 입구선 밖으로 나올 때만 이동.
    11/18 track_test_1118_04·05: 빈 가운데 칸 너머 둘째 줄 물체(칸 깊이 0.6~0.75 m)를 담으려고 칸을
    0.14 m 깊게 밀어 가운데 칸 OCCUPIED → 칸 포기. 차 쪽 면이 template_fusion_max_face_depth보다 깊은
    군집은 정렬 점수에서 제외. 모든 후보 오프셋을 numpy로 한 번에 평가 (점별 파이썬 반복이 Jetson에서
    한 주기 100~155 ms → 영상 나이 속도 상한 0.12~0.15 m/s의 원인).
    """
    points=_body_points(scan,robot)
    if not slots:return [],[],dict(reason='NO_TEMPLATE_SLOTS'),np.zeros(2)
    nominal=np.vstack([slot['polygon'] for slot in slots]);lo=nominal.min(axis=0);hi=nominal.max(axis=0)
    roi=float(config.get('template_fusion_roi_padding',.25))
    nearby=points[(points[:,0]>=lo[0]-roi)&(points[:,0]<=hi[0]+roi)&
                  (points[:,1]>=lo[1]-roi)&(points[:,1]<=hi[1]+roi)]
    clusters=_clusters(nearby,float(config.get('template_fusion_cluster_distance',.12)),
                       int(config.get('template_fusion_cluster_min_points',3)))
    tangent=np.asarray(slots[0]['tangent'],float);normal=np.asarray(slots[0]['normal'],float)
    width=float(slots[0]['width']);depth=float(slots[0]['depth'])
    entrances=np.asarray([slot['entrance'] for slot in slots],float)
    et=entrances@tangent;en=entrances@normal
    bound=float(config.get('template_fusion_max_offset',.15))
    normal_bound=float(config.get('template_fusion_max_normal_offset',bound))
    step=float(config.get('template_fusion_step',.03))
    half=float(config.get('template_fusion_obstacle_half_width',.1125))
    face_weight=float(config.get('template_fusion_face_weight',4.))
    face_depth=float(config.get('template_fusion_max_face_depth',float('inf')))
    base=np.asarray(config.get('template_lidar_offset',[0.,0.]),float)
    reference=base if previous is None else np.asarray(previous,float)
    along=np.arange(-bound,bound+step*.5,step)
    # 깊이 방향은 비대칭: 차로 쪽(-normal)은 입구선 밖으로 나온 물체를 담도록 normal_bound까지,
    # 칸 안쪽(+normal)은 둘째 줄을 따라 깊어지지 않게 template_fusion_max_deeper_offset까지만
    deeper=float(config.get('template_fusion_max_deeper_offset',normal_bound))
    across=np.arange(-normal_bound,deeper+step*.5,step) if normal_bound>0 or deeper>0 else np.zeros(1)
    # 예전 반복 순서(along 바깥, across 안쪽)를 유지해 동점일 때 같은 후보를 고름
    du_grid,dv_grid=np.meshgrid(along,across,indexing='ij')
    offsets=base+du_grid.reshape(-1,1)*tangent+dv_grid.reshape(-1,1)*normal
    du=offsets@tangent;dv=offsets@normal
    count=np.zeros(len(offsets));unique=np.zeros(len(offsets));fit=np.zeros(len(offsets))
    if clusters:
        centres=np.asarray([cluster.mean(axis=0) for cluster in clusters])
        umin0=np.asarray([float((cluster@tangent).min()) for cluster in clusters])[:,None]-et[None,:]
        vmin0=np.asarray([float((cluster@normal).min()) for cluster in clusters])[:,None]-en[None,:]
        U=(centres@tangent)[None,:,None]-et[None,None,:]-du[:,None,None]
        V=(centres@normal)[None,:,None]-en[None,None,:]-dv[:,None,None]
        index=_membership(U,V,width,depth,.06)                       # (K,C)
        safe=np.maximum(index,0)
        umin=np.take_along_axis(np.broadcast_to(umin0,(len(offsets),)+umin0.shape),safe[...,None],-1)[...,0]-du[:,None]
        vmin=np.take_along_axis(np.broadcast_to(vmin0,(len(offsets),)+vmin0.shape),safe[...,None],-1)[...,0]-dv[:,None]
        counted=(index>=0)&(vmin<=face_depth)
        # 차가 -u 쪽에서 접근 → 보이는 면은 물체의 u 최소 모서리. 칸 중앙 주차 가정
        # 깊이 방향은 한쪽만: 물체의 차로 쪽 끝이 입구선 밖이면 벌점, 안쪽 깊이는 무관
        penalty=np.abs(umin+half)*face_weight+np.maximum(0.,-vmin)*face_weight
        fit=-np.where(counted,penalty,0.).sum(axis=1)
        count=counted.sum(axis=1).astype(float)
        onehot=np.zeros((len(offsets),len(slots)),bool)
        rows=np.repeat(np.arange(len(offsets)),index.shape[1])
        flat=np.where(counted,index,-1).reshape(-1);keep=flat>=0
        onehot[rows[keep],flat[keep]]=True;unique=onehot.sum(axis=1).astype(float)
    regularization=np.linalg.norm(offsets-reference,axis=1)
    score=10.*count+2.*unique+fit-regularization*float(config.get('template_fusion_offset_penalty',2.))
    best=int(np.argmax(score))
    observed=base if not clusters or count[best]==0 else offsets[best]
    alpha=float(config.get('template_fusion_smoothing',.35))
    alignment=observed if previous is None else reference*(1-alpha)+observed*alpha
    aligned=_shift_slots(slots,alignment);padding=float(config.get('grid_occupancy_padding',.03))
    at=et+float(alignment@tangent);an=en+float(alignment@normal)
    if len(nearby):
        owner=_membership((nearby@tangent)[:,None]-at[None,:],(nearby@normal)[:,None]-an[None,:],width,depth,padding)
    else:owner=np.zeros(0,int)
    assigned=[nearby[owner==index].reshape(-1,2) for index in range(len(aligned))]
    boxes=[]
    for index,cluster in enumerate(clusters):
        lo_c=cluster.min(axis=0);hi_c=cluster.max(axis=0);centre=cluster.mean(axis=0)
        owner_c=int(_membership(np.array([centre@tangent])-at,np.array([centre@normal])-an,width,depth,.06))
        box=np.array([[lo_c[0],lo_c[1]],[hi_c[0],lo_c[1]],[hi_c[0],hi_c[1]],[lo_c[0],hi_c[1]]])
        boxes.append(dict(id=index,slot_index=None if owner_c<0 else owner_c,hits=len(cluster),
                          centroid=centre.tolist(),points=box.tolist()))
    debug=dict(reason='FUSED' if clusters else 'NO_LIDAR_CLUSTER_KEEP_ALIGNMENT',
               offset=np.asarray(alignment).tolist(),observed_offset=np.asarray(observed).tolist(),
               nearby_points=nearby[:200].tolist(),clusters=boxes,
               assigned_counts=[len(part) for part in assigned])
    return aligned,assigned,debug,np.asarray(alignment,float)
