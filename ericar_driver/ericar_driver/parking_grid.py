"""Experimental painted-grid parking bay detector.

This module only proposes metric bay polygons.  It does not publish commands,
change the existing one-shot FREE latch, or participate in white-line stopping.
The existing camera model is reused so the experiment can start before another
data collection session; real camera/LiDAR bags are still required for tuning.
"""
import math
import cv2
import numpy as np

from .parking_control import ground_points
def _angle_difference(first, second):
    difference=abs(first-second)%math.pi
    return min(difference,math.pi-difference)


def _cluster_lines(records, tolerance):
    groups=[]
    for record in sorted(records,key=lambda item:item['offset']):
        if not groups or record['offset']-groups[-1][-1]['offset']>tolerance:
            groups.append([record])
        else:
            groups[-1].append(record)
    merged=[]
    for group in groups:
        weights=np.array([item['length'] for item in group])
        offset=float(np.average([item['offset'] for item in group],weights=weights))
        merged.append(dict(offset=offset,
                           projection_min=min(item['projection_min'] for item in group),
                           projection_max=max(item['projection_max'] for item in group),
                           observations=len(group)))
    return merged


def _covers(line, low, high, extrapolation):
    return (line['projection_min']<=low+extrapolation and
            line['projection_max']>=high-extrapolation)


def grid_slots_from_ground_segments(segments, slot_width=.50, slot_depth=.59,
                                    size_tolerance=.12, angle_tolerance=.18,
                                    merge_distance=.045, extrapolation=.10):
    """Build rectangular cells from finite metric ground-line segments.

    Segments are ``[x0,y0,x1,y1]`` in the rear-axle/body frame.  The two grid
    axes are estimated rather than assumed parallel to the vehicle.  Returned
    candidates have the same entrance/tangent/normal/polygon contract as the
    current LiDAR-gap detector, but no FREE claim is made here.
    """
    segments=np.asarray(segments,float).reshape(-1,4)
    if not len(segments):return []
    starts=segments[:,:2];ends=segments[:,2:];delta=ends-starts
    lengths=np.linalg.norm(delta,axis=1)
    valid=np.isfinite(segments).all(axis=1)&(lengths>.08)
    if np.count_nonzero(valid)<4:return []
    starts,ends,delta,lengths=starts[valid],ends[valid],delta[valid],lengths[valid]
    angles=np.arctan2(delta[:,1],delta[:,0])
    # Fourfold circular mean treats perpendicular lines as the same grid axes.
    axis_angle=math.atan2(float(np.sum(lengths*np.sin(4*angles))),
                          float(np.sum(lengths*np.cos(4*angles))))/4
    tangent0=np.array([math.cos(axis_angle),math.sin(axis_angle)])
    normal0=np.array([-tangent0[1],tangent0[0]])
    families=([],[])
    for start,end,length,angle in zip(starts,ends,lengths,angles):
        error0=_angle_difference(angle,axis_angle)
        error1=_angle_difference(angle,axis_angle+math.pi/2)
        family=0 if error0<=error1 else 1
        if min(error0,error1)>angle_tolerance:continue
        direction=tangent0 if family==0 else normal0
        perpendicular=normal0 if family==0 else tangent0
        projection=sorted((float(start@direction),float(end@direction)))
        families[family].append(dict(offset=float(((start+end)/2)@perpendicular),
                                             projection_min=projection[0],projection_max=projection[1],
                                             length=float(length)))
    first=_cluster_lines(families[0],merge_distance)
    second=_cluster_lines(families[1],merge_distance)
    if len(first)<2 or len(second)<2:return []
    candidates=[]
    for a0,a1 in zip(first,first[1:]):
        gap_normal=a1['offset']-a0['offset']
        for b0,b1 in zip(second,second[1:]):
            gap_tangent=b1['offset']-b0['offset']
            # first-family lines span tangent0 and are separated along normal0.
            layouts=[]
            if abs(gap_tangent-slot_width)<=size_tolerance and abs(gap_normal-slot_depth)<=size_tolerance:
                layouts.append((tangent0,normal0,gap_tangent,gap_normal))
            if abs(gap_normal-slot_width)<=size_tolerance and abs(gap_tangent-slot_depth)<=size_tolerance:
                layouts.append((normal0,tangent0,gap_normal,gap_tangent))
            for tangent,normal,width,depth in layouts:
                # All four finite paint lines must reach their intersections.
                if tangent is tangent0:
                    covered=(_covers(a0,b0['offset'],b1['offset'],extrapolation) and
                             _covers(a1,b0['offset'],b1['offset'],extrapolation) and
                             _covers(b0,a0['offset'],a1['offset'],extrapolation) and
                             _covers(b1,a0['offset'],a1['offset'],extrapolation))
                    center=tangent0*(b0['offset']+b1['offset'])/2+normal0*(a0['offset']+a1['offset'])/2
                else:
                    covered=(_covers(a0,b0['offset'],b1['offset'],extrapolation) and
                             _covers(a1,b0['offset'],b1['offset'],extrapolation) and
                             _covers(b0,a0['offset'],a1['offset'],extrapolation) and
                             _covers(b1,a0['offset'],a1['offset'],extrapolation))
                    center=normal0*(a0['offset']+a1['offset'])/2+tangent0*(b0['offset']+b1['offset'])/2
                if not covered:continue
                # Choose the side of the cell closest to the vehicle as entrance.
                option_a=center-normal*depth/2;option_b=center+normal*depth/2
                if np.linalg.norm(option_b)<np.linalg.norm(option_a):
                    normal=-normal;entrance=option_b
                else:entrance=option_a
                corners=np.array([entrance-width/2*tangent,entrance+width/2*tangent,
                                  entrance+width/2*tangent+depth*normal,
                                  entrance-width/2*tangent+depth*normal])
                candidates.append(dict(entrance=entrance.tolist(),tangent=tangent.tolist(),
                    normal=normal.tolist(),width=float(width),depth=float(depth),polygon=corners.tolist(),
                    center=center.tolist(),source='painted_grid',requires_measured_rays=True,
                    geometry_score=float(max(0.,1-(abs(width-slot_width)+abs(depth-slot_depth))/(2*size_tolerance)))))
    # Suppress the alternate axis interpretation and duplicate paint edges.
    kept=[]
    for candidate in sorted(candidates,key=lambda item:-item['geometry_score']):
        center=np.asarray(candidate['center'])
        if any(np.linalg.norm(center-np.asarray(other['center']))<merge_distance*2 for other in kept):continue
        kept.append(candidate)
    return kept


def partial_grid_slots_from_ground_segments(segments,slot_width=.50,slot_depth=.59,
                                            size_tolerance=.12,angle_tolerance=.18,
                                            merge_distance=.045,extrapolation=.14):
    """Infer a known-size cell from two side lines and one entrance line."""
    segments=np.asarray(segments,float).reshape(-1,4)
    if not len(segments):return []
    starts=segments[:,:2];ends=segments[:,2:];delta=ends-starts
    lengths=np.linalg.norm(delta,axis=1)
    valid=np.isfinite(segments).all(axis=1)&(lengths>.10)
    if np.count_nonzero(valid)<3:return []
    starts,ends,delta,lengths=starts[valid],ends[valid],delta[valid],lengths[valid]
    angles=np.arctan2(delta[:,1],delta[:,0])
    axis_angle=math.atan2(float(np.sum(lengths*np.sin(4*angles))),
                          float(np.sum(lengths*np.cos(4*angles))))/4
    axes=(np.array([math.cos(axis_angle),math.sin(axis_angle)]),
          np.array([-math.sin(axis_angle),math.cos(axis_angle)]))
    families=([],[])
    for start,end,length,angle in zip(starts,ends,lengths,angles):
        errors=(_angle_difference(angle,axis_angle),_angle_difference(angle,axis_angle+math.pi/2))
        family=0 if errors[0]<=errors[1] else 1
        if errors[family]>angle_tolerance:continue
        direction=axes[family];perpendicular=axes[1-family]
        projection=sorted((float(start@direction),float(end@direction)))
        families[family].append(dict(offset=float(((start+end)/2)@perpendicular),
                                             projection_min=projection[0],projection_max=projection[1],
                                             length=float(length)))
    merged=(_cluster_lines(families[0],merge_distance),_cluster_lines(families[1],merge_distance))
    candidates=[]
    for side_index in (0,1):
        direction=axes[side_index];across=axes[1-side_index]
        side_lines=merged[side_index];cross_lines=merged[1-side_index]
        for left,right in zip(side_lines,side_lines[1:]):
            width=right['offset']-left['offset']
            if abs(width-slot_width)>size_tolerance:continue
            for entrance_line in cross_lines:
                if not _covers(entrance_line,left['offset'],right['offset'],extrapolation):continue
                entrance=direction*entrance_line['offset']+across*(left['offset']+right['offset'])/2
                # The cell interior is the side farther from the vehicle.
                options=(direction,-direction)
                normal=max(options,key=lambda value:np.linalg.norm(entrance+value*slot_depth))
                far=float(entrance_line['offset']+np.dot(normal,direction)*slot_depth)
                if not (_covers(left,min(entrance_line['offset'],far),max(entrance_line['offset'],far),extrapolation) and
                        _covers(right,min(entrance_line['offset'],far),max(entrance_line['offset'],far),extrapolation)):
                    continue
                tangent=across
                corners=np.array([entrance-width/2*tangent,entrance+width/2*tangent,
                                  entrance+width/2*tangent+slot_depth*normal,
                                  entrance-width/2*tangent+slot_depth*normal])
                center=entrance+normal*slot_depth/2
                candidates.append(dict(entrance=entrance.tolist(),tangent=tangent.tolist(),normal=normal.tolist(),
                    width=float(width),depth=float(slot_depth),polygon=corners.tolist(),center=center.tolist(),
                    source='painted_grid_partial',requires_measured_rays=True,geometry_score=.75))
    kept=[]
    for candidate in candidates:
        if any(np.linalg.norm(np.asarray(candidate['center'])-np.asarray(other['center']))<merge_distance*2
               for other in kept):continue
        kept.append(candidate)
    return kept


def reachable_right_slots(candidates, minimum_forward=.30, right_normal_y=-.55,
                          entrance_right_min=-.85,entrance_right_max=-.05):
    """Keep the nearest right-facing row reachable by the existing right turn."""
    usable=[candidate for candidate in candidates
            if (candidate['entrance'][0]>=minimum_forward and candidate['normal'][1]<=right_normal_y and
                entrance_right_min<=candidate['entrance'][1]<=entrance_right_max)]
    result=[]
    for candidate in usable:
        tangent=np.asarray(candidate['tangent']);center=np.asarray(candidate['center'])
        column=float(center@tangent)
        normal=np.asarray(candidate['normal'])
        entrance_depth=abs(float(np.asarray(candidate['entrance'])@normal))
        blocked_by_nearer=any(
            other is not candidate and abs(float(np.asarray(other['center'])@tangent)-column)<candidate['width']*.45
            and abs(float(np.asarray(other['entrance'])@normal))+candidate['depth']*.5<entrance_depth
            for other in usable)
        if not blocked_by_nearer:result.append(candidate)
    return sorted(result,key=lambda item:item['entrance'][0])


def move_ground_segments(segments,dt,velocity,angular_velocity):
    """Move stored body-frame line segments to the current vehicle frame."""
    values=np.asarray(segments,float).reshape(-1,4)
    if not len(values):return []
    yaw=angular_velocity*dt;travel=velocity*dt
    x=travel if abs(yaw)<1e-8 else travel*math.sin(yaw)/yaw
    y=0. if abs(yaw)<1e-8 else travel*(1-math.cos(yaw))/yaw
    c,s=math.cos(yaw),math.sin(yaw);rotation=np.array([[c,-s],[s,c]])
    points=(values.reshape(-1,2)-[x,y])@rotation
    return points.reshape(-1,4).tolist()


def _paint_mask(image,low,high,roi_top):
    hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    mask=cv2.inRange(hsv,np.asarray(low,np.uint8),np.asarray(high,np.uint8))
    mask[:int(image.shape[0]*roi_top)]=0
    return cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((2,2),np.uint8))


def _ground_segments(mask,image,robot,parking):
    h,w=mask.shape
    raw=cv2.HoughLinesP(mask,1,np.pi/180,parking.get('grid_hough_votes',18),
                        minLineLength=int(w*parking.get('grid_min_length_fraction',.08)),maxLineGap=18)
    ground=[];pixels=[]
    if raw is not None:
        for line in np.asarray(raw).reshape(-1,4):
            points,valid=ground_points(line.reshape(2,2),image.shape,robot['camera'])
            if not valid.all():continue
            length=float(np.linalg.norm(points[1]-points[0]))
            if length<.10 or points[:,0].min()<.15 or points[:,0].max()>3.:continue
            if np.max(np.abs(points[:,1]))>1.8:continue
            # Parking is on the right.  Reject left-side lane/background lines
            # before they can dominate the orthogonal grid-axis estimate.
            if float(points[:,1].mean())>parking.get('grid_right_midpoint_max',.15):continue
            ground.append([*points[0],*points[1]]);pixels.append(line.tolist())
    return ground,pixels


def parking_grid_candidates(image, robot, parking, prior_segments=None):
    """Detect yellow/white metric cells; occupancy remains LiDAR's job."""
    if image is None:return [],dict(reason='NO_IMAGE',segments=[])
    top=parking.get('grid_roi_top',.42)
    yellow=_paint_mask(image,parking.get('grid_yellow_min',[18,40,50]),
                       parking.get('grid_yellow_max',[40,255,255]),top)
    white=_paint_mask(image,parking.get('grid_white_min',[0,0,95]),
                      parking.get('grid_white_max',[179,85,255]),top)
    current={};pixels={}
    current['yellow'],pixels['yellow']=_ground_segments(yellow,image,robot,parking)
    current['white'],pixels['white']=_ground_segments(white,image,robot,parking)
    prior=prior_segments or {}
    sources={color:list(prior.get(color,[]))+current[color] for color in ('yellow','white')}
    sources['combined']=sources['yellow']+sources['white']
    all_cells=[];chosen_color=None;chosen_segments=[]
    # Actual 11/18 course cells are yellow.  White and combined remain fallback
    # for the earlier official layout without letting stop lines dominate first.
    for color in ('yellow','white','combined'):
        cells=grid_slots_from_ground_segments(sources[color],parking.get('grid_slot_width',.50),
            parking.get('grid_slot_depth',.59),parking.get('grid_size_tolerance',.12),
            extrapolation=parking.get('grid_maximum_extrapolation',.14))
        if not cells:
            cells=partial_grid_slots_from_ground_segments(sources[color],parking.get('grid_slot_width',.50),
                parking.get('grid_slot_depth',.59),parking.get('grid_size_tolerance',.12),
                extrapolation=parking.get('grid_maximum_extrapolation',.14))
        if not cells and parking.get('grid_relaxed',False):
            # 11/18 혁준: 가로선이 일부만 보여도 칸으로 추정 (parking_grid_relaxed.py). 끄기: grid_relaxed False
            from .parking_grid_relaxed import relaxed_partial_slots
            cells=relaxed_partial_slots(sources[color],parking.get('grid_slot_width',.50),
                parking.get('grid_slot_depth',.59),parking.get('grid_size_tolerance',.12),
                visible_min=parking.get('grid_relaxed_visible_min',.15),
                single=parking.get('grid_relaxed_single',False),
                extrapolation=parking.get('grid_maximum_extrapolation',.14))
        reachable=reachable_right_slots(cells,parking.get('view_near',.30),
            entrance_right_min=parking.get('grid_entrance_right_min',-.85),
            entrance_right_max=parking.get('grid_entrance_right_max',-.05))
        all_cells.extend(cells)
        if reachable:
            chosen_color=color;chosen_segments=sources[color];break
    flat_pixels=(pixels['yellow']+pixels['white'])[:160]
    flat_colors=(['yellow']*len(pixels['yellow'])+['white']*len(pixels['white']))[:160]
    debug_segments=(chosen_segments or sources['combined'])[-120:]
    return reachable,dict(reason='GRID_SLOTS_FOUND' if reachable else 'NO_REACHABLE_GRID_SLOT',
                          color=chosen_color,segments_px=flat_pixels,
                          segment_colors=flat_colors,
                          segments_ground=debug_segments,all_cells=all_cells[:24],
                          current_segments_by_color=current)
