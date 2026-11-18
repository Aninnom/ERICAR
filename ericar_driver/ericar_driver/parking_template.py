"""Parking-lot template anchored by the white start line and side transition."""
import cv2
import numpy as np

from .parking_control import ground_points


def _mask(image,low,high,top):
    hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    result=cv2.inRange(hsv,np.asarray(low,np.uint8),np.asarray(high,np.uint8))
    result[:int(image.shape[0]*top)]=0
    return cv2.morphologyEx(result,cv2.MORPH_OPEN,np.ones((2,2),np.uint8))


def _segments(mask,image,robot,config):
    raw=cv2.HoughLinesP(mask,1,np.pi/180,config.get('template_hough_votes',18),
                        minLineLength=int(image.shape[1]*config.get('template_min_length_fraction',.07)),
                        maxLineGap=config.get('template_max_line_gap_px',18))
    values=[]
    if raw is None:return values
    for pixel in np.asarray(raw).reshape(-1,4):
        points,valid=ground_points(pixel.reshape(2,2),image.shape,robot['camera'])
        if not valid.all():continue
        delta=points[1]-points[0];length=float(np.linalg.norm(delta))
        if length<config.get('template_min_ground_length',.18):continue
        if points[:,0].min()<.10 or points[:,0].max()>config.get('template_ground_far',2.5):continue
        values.append(dict(points=points,pixel=pixel.tolist(),length=length,delta=delta))
    return values


def parking_template_landmarks(image,robot,config):
    """Return the transverse start cue and yellow-to-white side anchors."""
    if image is None:return dict(start_seen=False,anchors=[],reason='NO_IMAGE')
    top=config.get('template_roi_top',.40)
    white=_mask(image,config.get('grid_white_min',[0,0,95]),config.get('grid_white_max',[179,85,255]),top)
    yellow=_mask(image,config.get('grid_yellow_min',[18,40,50]),config.get('grid_yellow_max',[40,255,255]),top)
    # 11/18 redecide_1118_14: 밝은 조명 반사로 노란 차선 일부가 채도 48~85 → 흰색 범위에 들어가
    # 노란 차선 중간에 가짜 황색→흰색 전환점 생성. 노란 색상 + 채도 일정 이상은 흰색에서 제외
    # (실제 흰 선 채도 1~5, 반사된 노란 선 48 이상)
    exclude=_mask(image,config.get('template_white_exclude_min',[10,30,60]),
                  config.get('template_white_exclude_max',[45,255,255]),top)
    white=cv2.bitwise_and(white,cv2.bitwise_not(exclude))
    white_lines=_segments(white,image,robot,config);yellow_lines=_segments(yellow,image,robot,config)
    ratio=config.get('template_orientation_ratio',1.8)
    transverse=[];white_long=[];yellow_long=[];start_positions=[]
    for record in white_lines:
        dx,dy=np.abs(record['delta'])
        midpoint=record['points'].mean(axis=0)
        if dy>=dx*ratio and record['length']>=config.get('template_start_min_length',.35):
            transverse.append(record)
            # 시작선 위치(차 앞 거리): 차로 안에 놓인 가로 흰 선 중 가장 가까운 것 (11/18 혁준, 기준점 위치 검사용)
            half=config.get('template_start_lane_half',.45)
            if record['points'][:,1].min()>=-half and record['points'][:,1].max()<=half:
                start_positions.append(float(midpoint[0]))
        if dx>=dy*ratio and config.get('template_right_min',-1.1)<=midpoint[1]<=config.get('template_right_max',-.05):
            white_long.append(record)
    for record in yellow_lines:
        dx,dy=np.abs(record['delta']);midpoint=record['points'].mean(axis=0)
        if dx>=dy*ratio and config.get('template_right_min',-1.1)<=midpoint[1]<=config.get('template_right_max',-.05):
            yellow_long.append(record)
    anchors=[];tolerance=config.get('template_transition_tolerance',.24)
    for white_line in white_long:
        wp=white_line['points'];white_start=wp[np.argmin(wp[:,0])]
        direction=wp[np.argmax(wp[:,0])]-white_start
        direction=direction/max(np.linalg.norm(direction),1e-9)
        if direction[0]<0:direction=-direction
        best=None
        for yellow_line in yellow_long:
            yp=yellow_line['points'];yellow_end=yp[np.argmax(yp[:,0])]
            distance=float(np.linalg.norm(yellow_end-white_start))
            if distance<=tolerance and (best is None or distance<best[0]):best=(distance,yellow_end,yellow_line)
        if best is not None:
            anchor=(white_start+best[1])/2;confidence='COLOR_TRANSITION';support=best[2]
            # 11/18 재생: 먼 흰 선분은 짧아 방향 오차 큼 → 칸이 차로 쪽으로 돌아감.
            # 같은 차로 경계인 노란 선분(길고 가까움)이 더 길면 그 방향 사용
            if support['length']>white_line['length']:
                yp=support['points'];direction=yp[np.argmax(yp[:,0])]-yp[np.argmin(yp[:,0])]
                direction=direction/max(np.linalg.norm(direction),1e-9)
        else:
            anchor=white_start;confidence='WHITE_START_ONLY';support=None
        normal=np.array([direction[1],-direction[0]])
        if normal[1]>0:normal=-normal
        anchors.append(dict(anchor=anchor.tolist(),tangent=direction.tolist(),normal=normal.tolist(),
                            confidence=confidence,length=white_line['length'],pixel=white_line['pixel'],
                            support_pixel=None if support is None else support['pixel']))
    anchors.sort(key=lambda item:(item['confidence']!='COLOR_TRANSITION',item['anchor'][0],-item['length']))
    return dict(start_seen=bool(transverse),anchors=anchors,start_x=min(start_positions) if start_positions else None,
                start_segments=[item['pixel'] for item in transverse[:12]],
                white_longitudinal=[item['pixel'] for item in white_long[:24]],
                yellow_longitudinal=[item['pixel'] for item in yellow_long[:24]],
                right_ground=[item['points'].tolist() for item in (white_long+yellow_long)[:40]],
                reason='ANCHOR_FOUND' if anchors else 'WAITING_FOR_WHITE_SIDE')


def slots_from_anchor(anchor,tangent,normal,width=.50,depth=.59,count=3,pitch=None):
    """Render known-size consecutive cells in the vehicle ground frame.

    pitch: 칸 중심 간격. 도면 3칸 x 6110~7650 = 1.54 m → 0.513 (선 두께 포함). None이면 width
    """
    anchor=np.asarray(anchor,float);tangent=np.asarray(tangent,float);normal=np.asarray(normal,float)
    pitch=width if pitch is None else float(pitch)
    result=[]
    for index in range(count):
        entrance=anchor+(index*pitch+width/2)*tangent
        corners=np.array([entrance-width/2*tangent,entrance+width/2*tangent,
                          entrance+width/2*tangent+depth*normal,
                          entrance-width/2*tangent+depth*normal])
        result.append(dict(entrance=entrance.tolist(),tangent=tangent.tolist(),normal=normal.tolist(),
                           width=float(width),depth=float(depth),polygon=corners.tolist(),
                           center=(entrance+normal*depth/2).tolist(),source='parking_template',
                           slot_index=index,requires_measured_rays=True))
    return result
