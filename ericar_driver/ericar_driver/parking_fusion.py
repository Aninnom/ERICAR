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


def fuse_template_slots(slots,scan,robot,config,previous=None):
    """Align a camera lot to nearby LiDAR objects, then assign every return once.

    Alignment is deliberately translation-only and bounded. Camera geometry and
    the known 0.50 m cell pitch remain authoritative; LiDAR cannot stretch or
    rotate the lot to manufacture a free cell.
    """
    points=_body_points(scan,robot)
    if not slots:return [],[],dict(reason='NO_TEMPLATE_SLOTS'),np.zeros(2)
    nominal=np.vstack([slot['polygon'] for slot in slots]);lo=nominal.min(axis=0);hi=nominal.max(axis=0)
    roi=float(config.get('template_fusion_roi_padding',.25))
    nearby=points[(points[:,0]>=lo[0]-roi)&(points[:,0]<=hi[0]+roi)&
                  (points[:,1]>=lo[1]-roi)&(points[:,1]<=hi[1]+roi)]
    clusters=_clusters(nearby,float(config.get('template_fusion_cluster_distance',.12)),
                       int(config.get('template_fusion_cluster_min_points',3)))
    bound=float(config.get('template_fusion_max_offset',.15));step=float(config.get('template_fusion_step',.03))
    base=np.asarray(config.get('template_lidar_offset',[0.,0.]),float)
    reference=base if previous is None else np.asarray(previous,float)
    offsets=np.arange(-bound,bound+step*.5,step)
    best=None
    for dx in offsets:
        for dy in offsets:
            offset=base+[dx,dy];candidate=_shift_slots(slots,offset)
            assigned=[];fit=0.;front_depth=float(config.get('template_fusion_obstacle_front_depth',.08))
            for cluster in clusters:
                centre=cluster.mean(axis=0);index=_assignment(centre,candidate,.06)
                if index is None:continue
                assigned.append(index)
                u,v=_slot_coordinates(centre,candidate[index]);slot=candidate[index]
                # LiDAR observes the nearest face of a parked object. Keep that
                # face just inside the entrance instead of pulling the lot all
                # the way until every object is centred in depth.
                fit-=abs(v-front_depth)*2.+abs(u)/(slot['width']+.01)*.1
            unique=len(set(assigned));count=len(assigned)
            regularization=float(np.linalg.norm(offset-reference))
            score=10.*count+2.*unique+fit-regularization*float(config.get('template_fusion_offset_penalty',2.))
            if best is None or score>best[0]:best=(score,np.asarray(offset),count,unique)
    observed=base if not clusters or best[2]==0 else best[1]
    alpha=float(config.get('template_fusion_smoothing',.35))
    alignment=observed if previous is None else reference*(1-alpha)+observed*alpha
    aligned=_shift_slots(slots,alignment);padding=float(config.get('grid_occupancy_padding',.03))
    assigned=[[] for _ in aligned]
    for point in nearby:
        index=_assignment(point,aligned,padding)
        if index is not None:assigned[index].append(point)
    assigned=[np.asarray(part,float).reshape(-1,2) for part in assigned]
    boxes=[]
    for index,cluster in enumerate(clusters):
        lo_c=cluster.min(axis=0);hi_c=cluster.max(axis=0);slot=_assignment(cluster.mean(axis=0),aligned,.06)
        box=np.array([[lo_c[0],lo_c[1]],[hi_c[0],lo_c[1]],[hi_c[0],hi_c[1]],[lo_c[0],hi_c[1]]])
        boxes.append(dict(id=index,slot_index=slot,hits=len(cluster),centroid=cluster.mean(axis=0).tolist(),
                          points=box.tolist()))
    debug=dict(reason='FUSED' if clusters else 'NO_LIDAR_CLUSTER_KEEP_ALIGNMENT',
               offset=alignment.tolist(),observed_offset=np.asarray(observed).tolist(),
               nearby_points=nearby[:200].tolist(),clusters=boxes,
               assigned_counts=[len(part) for part in assigned])
    return aligned,assigned,debug,alignment
