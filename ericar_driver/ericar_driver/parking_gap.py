"""LiDAR의 오른쪽 두 물체 사이 입구 추정. 미측정 광선은 빈칸 증거에서 제외"""
import math
import numpy as np


def gap_candidates(scan,robot,config,road_angle=0.):
    rays=scan.get('parking_rays',{})
    angles=np.asarray(rays.get('angles',[]),float)
    distances=np.asarray(rays.get('distances',[]),float)
    measured=np.asarray(rays.get('measured',np.zeros(len(angles),bool)),bool)
    if not(len(angles)==len(distances)==len(measured)):
        return [],dict(reason='INVALID_RAY_ARRAYS',clusters=[],gaps=[])
    local=np.column_stack((distances*np.cos(angles),distances*np.sin(angles)))
    local+=np.array([robot['lidar']['x'],robot['lidar'].get('y',0.)])
    c,s=math.cos(road_angle),math.sin(road_angle)
    rotation=np.array([[c,-s],[s,c]])
    road=local@rotation
    keep=measured&(road[:,0]>-.4)&(road[:,0]<config['gap_view_far'])&(road[:,1]<-.08)&(road[:,1]>-config['gap_view_right'])
    indices=np.flatnonzero(keep)
    clusters=[];rejections=[]
    if len(indices):
        points=road[indices]
        breaks=np.flatnonzero((np.diff(indices)>2)|(np.linalg.norm(np.diff(points,axis=0),axis=1)>config['gap_cluster_distance']))+1
        for cluster in np.split(points,breaks):
            lo=cluster.min(axis=0);hi=cluster.max(axis=0)
            extent=float(np.linalg.norm(hi-lo))
            reason='TOO_FEW_POINTS' if len(cluster)<config['gap_cluster_min_points'] else (
                'OBSERVED_EXTENT_REJECTED' if not config['gap_object_min_width']<=extent<=config['gap_object_max_width'] else None)
            if reason:
                q=np.array([[lo[0],lo[1]],[hi[0],lo[1]],[hi[0],hi[1]],[lo[0],hi[1]]])@rotation.T
                rejections.append(dict(reason=reason,points=q.tolist(),hits=len(cluster),extent_m=extent))
                continue
            clusters.append(dict(lo=lo,hi=hi,count=len(cluster)))
    clusters.sort(key=lambda b:float(b['lo'][0]))
    tangent=np.array([c,s]);normal=np.array([s,-c])
    candidates=[];gaps=[]
    padding=config['gap_edge_padding']
    # 앞줄 물체 쌍 검사. 사이에 낀 군집이 앞줄보다 깊으면(빈틈 너머로 보이는 벽 조각 등) 건너뜀
    # 칸 안쪽에 있는 군집은 classify_gap의 내부 반사점 검사가 OCCUPIED로 처리
    for i,first in enumerate(clusters):
        for j in range(i+1,len(clusters)):
            second=clusters[j]
            front=max(first['hi'][1],second['hi'][1])-config['gap_row_tolerance']
            if any(between['hi'][1]>front for between in clusters[i+1:j]):break
            left=float(first['hi'][0]+padding);right=float(second['lo'][0]-padding)
            width=right-left
            level=float(max(first['hi'][1],second['hi'][1])+config['gap_front_padding'])
            reason='CANDIDATE'
            if abs(first['hi'][1]-second['hi'][1])>config['gap_row_tolerance']:reason='FRONTS_NOT_ALIGNED'
            elif not robot['width']+2*config['body_margin']+config['gap_extra_width']<=width<=config['gap_max_width']:reason='GAP_WIDTH_REJECTED'
            elif (left+right)/2<config['view_near']:reason='GAP_BEHIND_APPROACH'
            gaps.append(dict(width=width,reason=reason,center_road=[(left+right)/2,level]))
            if reason!='CANDIDATE':continue
            entrance=np.array([(left+right)/2,level])@rotation.T
            depth=config['slot_depth']
            polygon=np.array([entrance-width/2*tangent,entrance+width/2*tangent,
                              entrance+width/2*tangent+depth*normal,entrance-width/2*tangent+depth*normal])
            candidates.append(dict(entrance=entrance.tolist(),tangent=tangent.tolist(),normal=normal.tolist(),
                width=width,depth=depth,polygon=polygon.tolist(),source='lidar_obstacle_gap',requires_measured_rays=True))
    boxes=[]
    for i,b in enumerate(clusters):
        lo,hi=b['lo'],b['hi']
        q=np.array([[lo[0],lo[1]],[hi[0],lo[1]],[hi[0],hi[1]],[lo[0],hi[1]]])@rotation.T
        boxes.append(dict(id=i,points=q.tolist(),hits=b['count']))
    return candidates,dict(cluster_rejections=rejections,road_angle_rad=float(road_angle),reason='GAPS_FOUND' if candidates else 'WAITING_FOR_TWO_RIGHT_OBSTACLES',
                           clusters=boxes,gaps=gaps,candidates=[dict(polygon=b['polygon'],entrance=b['entrance']) for b in candidates])


def classify_gap(bay,scan,robot,config):
    """현재 차체 좌표의 후보를 공통 기준으로 판정. FREE 1회가 트리거"""
    entrance=np.asarray(bay['entrance'],float);tangent=np.asarray(bay['tangent'],float);normal=np.asarray(bay['normal'],float)
    points=np.asarray(scan.get('parking_points',scan.get('points',[])),float).reshape(-1,2)+[robot['lidar']['x'],robot['lidar'].get('y',0.)]
    delta=points-entrance;u=delta@tangent;v=delta@normal;edge=config['occupancy_edge_margin']
    hits=int(np.count_nonzero((np.abs(u)<bay['width']/2-edge)&(v>edge)&(v<bay['depth']-edge)))
    details=dict(interior_hits=hits,total_samples=12,covered_samples=0,required_samples=math.ceil(12*config['free_ray_fraction']))
    if hits>=config['occupancy_min_hits']:return 'OCCUPIED',dict(details,reason='INTERIOR_RETURNS')
    rays=scan.get('parking_rays',{});angles=np.asarray(rays.get('angles',[]));inc=rays.get('increment',0.)
    measured=np.asarray(rays.get('measured',[]),bool);distances=np.asarray(rays.get('distances',[]))
    if not len(angles) or len(measured)!=len(angles) or len(distances)!=len(angles) or not np.isfinite(inc) or inc==0:
        return 'UNKNOWN',dict(details,reason='NO_USABLE_RAYS')
    samples=np.array([entrance+x*tangent+y*normal for x in np.linspace(-bay['width']*.35,bay['width']*.35,3)
                      for y in np.linspace(.08,bay['depth']-.08,4)])
    samples-=np.array([robot['lidar']['x'],robot['lidar'].get('y',0.)])
    bearing=np.arctan2(samples[:,1],samples[:,0]);need=np.linalg.norm(samples,axis=1)
    covered=np.zeros(12,bool);inside=np.zeros(12,bool);usable=np.zeros(12,bool)
    for wrap in (-2*math.pi,0.,2*math.pi):
        indices=np.rint((bearing+wrap-angles[0])/inc).astype(int);valid=(indices>=0)&(indices<len(angles))
        at=np.flatnonzero(valid);ix=indices[valid];inside[at]=True;usable[at]|=measured[ix]
        covered[at]|=measured[ix]&(distances[ix]>need[at]+.02)
    details.update(covered_samples=int(covered.sum()),occluded_or_short_samples=int(np.count_nonzero(inside&usable&~covered)),
                   no_return_or_invalid_samples=int(np.count_nonzero(inside&~usable)))
    free=hits==0 and covered.sum()>=details['required_samples']
    return ('FREE' if free else 'UNKNOWN'),dict(details,reason='COVERAGE_CONFIRMED' if free else 'INSUFFICIENT_VISIBLE_DEPTH')


def classify_camera_slot(bay,scan,robot,config):
    """Classify a camera-defined cell without requiring obstacle clustering.

    Raw measured returns inside an expanded cell are occupancy evidence.  A
    valid no-return ray (YDLidar 0 normalized to range_max, or +inf) is useful
    FREE evidence because it passed through the requested sample depth.  Truly
    invalid NaN/negative rays remain UNKNOWN.
    """
    entrance=np.asarray(bay['entrance'],float)
    tangent=np.asarray(bay['tangent'],float);normal=np.asarray(bay['normal'],float)
    points=np.asarray(scan.get('parking_points',scan.get('points',[])),float).reshape(-1,2)
    points+=np.array([robot['lidar']['x'],robot['lidar'].get('y',0.)])
    delta=points-entrance;u=delta@tangent;v=delta@normal
    padding=float(config.get('grid_occupancy_padding',.03))
    inside=(np.abs(u)<bay['width']/2+padding)&(v>-padding)&(v<bay['depth']+padding)
    hits=int(np.count_nonzero(inside))
    hit_points=points[inside]
    total_samples=12;required=math.ceil(total_samples*config['free_ray_fraction'])
    details=dict(interior_hits=hits,interior_points=hit_points[:24].tolist(),
                 total_samples=total_samples,covered_samples=0,required_samples=required)
    if hits>=config['occupancy_min_hits']:
        return 'OCCUPIED',dict(details,reason='RAW_INTERIOR_RETURNS')
    # Even one real return inside the cell contradicts FREE.  More frames may
    # turn it into OCCUPIED, but it must never trigger the one-shot latch now.
    if hits:
        return 'UNKNOWN',dict(details,reason='SPARSE_INTERIOR_RETURNS')
    rays=scan.get('parking_rays',{});angles=np.asarray(rays.get('angles',[]),float)
    distances=np.asarray(rays.get('distances',[]),float);valid_rays=np.asarray(rays.get('valid',[]),bool)
    increment=float(rays.get('increment',0.))
    if (not len(angles) or len(distances)!=len(angles) or len(valid_rays)!=len(angles) or
            not math.isfinite(increment) or increment==0):
        return 'UNKNOWN',dict(details,reason='NO_USABLE_RAYS')
    samples=np.array([entrance+x*tangent+y*normal
                      for x in np.linspace(-bay['width']*.35,bay['width']*.35,3)
                      for y in np.linspace(.08,bay['depth']-.08,4)])
    samples-=np.array([robot['lidar']['x'],robot['lidar'].get('y',0.)])
    bearing=np.arctan2(samples[:,1],samples[:,0]);needed=np.linalg.norm(samples,axis=1)
    covered=np.zeros(total_samples,bool);in_scan=np.zeros(total_samples,bool);usable=np.zeros(total_samples,bool)
    clearance=float(config.get('grid_free_clearance',.02))
    for wrap in (-2*math.pi,0.,2*math.pi):
        indices=np.rint((bearing+wrap-angles[0])/increment).astype(int)
        valid_index=(indices>=0)&(indices<len(angles));at=np.flatnonzero(valid_index);ix=indices[valid_index]
        in_scan[at]=True;usable[at]|=valid_rays[ix]
        covered[at]|=valid_rays[ix]&(distances[ix]>needed[at]+clearance)
    details.update(covered_samples=int(covered.sum()),
                   invalid_samples=int(np.count_nonzero(in_scan&~usable)),
                   blocked_or_short_samples=int(np.count_nonzero(in_scan&usable&~covered)))
    free=covered.sum()>=required
    return ('FREE' if free else 'UNKNOWN'),dict(details,reason='VALID_CLEAR_RAYS' if free else 'INSUFFICIENT_CLEAR_RAYS')
