"""fast_parking1: 목표 감속 거리까지 차선 주행 → 저속 우회전 → 흰 선 소실 즉시 정지"""
import math
import numpy as np
from .common import clip
from .lidar import body_points,path_clearance
from .parking_gap import gap_candidates,classify_gap,classify_camera_slot
from .parking_grid import parking_grid_candidates,move_ground_segments
from .parking_template import parking_template_landmarks,slots_from_anchor
from .parking_fusion import fuse_template_slots
from .parking_white import parking_straight_lines
from .parking_control import ParkingPD
from .parking_sign import SignPassage,slow_distance


class Parking:
    def __init__(self,config):
        self.c=config['answer']['parking'];self.robot=config['robot']
        self.pd=ParkingPD(self.robot,self.c)
        self.sign=SignPassage(self.c,config['answer']['detector'])
        self._reset_progress()
        if not -self.robot['max_steering']<=self.c['turn_steering']<0:raise ValueError('turn_steering must be a bounded right turn')
        if not math.isfinite(self._turn_lead()) or self._turn_lead()<=0:raise ValueError('positive turn lead required')
        if not 0<=self.c['approach_start_tolerance']<=self.c['approach_passed_tolerance']:raise ValueError('invalid approach tolerances')
        self.state='idle';self.reason='WAITING_FOR_PARKING_SIGN';self.debug={};self.stable_since=None
        self.selected=None;self.gap_debug={};self.line_debug={};self.last_scan=None
        self.grid_debug={};self.grid_tracks=[];self.grid_segment_frames=[];self.last_grid_image=None
        self.template_debug={};self.template_armed=False;self.template_start_frames=0
        self.template_anchor_track=None;self.template_slots=[];self.last_template_image=None
        self.template_occupied_frames={};self.rejected_template_slots=set();self.path_debug={}
        self.template_alignment=None;self.fusion_debug={};self.template_start_track=None
        self.approach_distance=0.;self.approach_debug={}
        self.last_image=None;self.line_seen=False;self.line_present=False
        self.loss_frames=0;self.search_distance=0.;self.turn_distance=0.;self.motion_since=None

    def _reset_progress(self):
        self.cruise_distance=0.;self.nearest_start=None;self.slow_started=0.;self.slow_reason=None
        self.turn_angle=0.;self.turn_phase='arc';self.align_frames=0;self.arc_released=False
        self.no_target_since=None;self.no_target_distance=0.;self.braking_distance=0.
        self.smooth_approach=False;self.approach_speed_cap=None;self.entry_stable_since=None

    def start(self,now):
        self.pd.reset();self.sign.reset();self._reset_progress()
        self.state='parking_cruise' if self.c['cruise_search_enabled'] else 'parking_scan'
        self.reason='MAIN_LANE_WITH_LIDAR_SEARCH' if self.state=='parking_cruise' else 'SLOW_SEARCH_UNTIL_FIRST_FREE'
        self.slow_started=now
        self.started=self.last_tick=now;self.turn_started=None;self.selected=None
        self.last_scan=self.last_image=None;self.line_seen=False;self.line_present=False
        self.loss_frames=0;self.search_distance=self.turn_distance=0.;self.motion_since=None
        self.gap_debug={};self.line_debug={}
        self.grid_debug={};self.grid_tracks=[];self.grid_segment_frames=[];self.last_grid_image=None
        self.template_debug={};self.template_armed=False;self.template_start_frames=0
        self.template_anchor_track=None;self.template_slots=[];self.last_template_image=None
        self.template_occupied_frames={};self.rejected_template_slots=set();self.path_debug={}
        self.template_alignment=None;self.fusion_debug={};self.template_start_track=None
        self.approach_distance=0.;self.approach_debug={}

    def close(self):pass

    def fail(self,reason):
        self.state='parking_failed';self.reason=reason
        return self._output()

    def cancel(self):
        self.state='parking_cancelled';self.reason='EXTERNAL_CANCEL'

    def pause(self,now):
        self.pd.pause();self.align_frames=0;self.entry_stable_since=None
        self.last_tick=now;self.motion_since=None;self.reason='WAITING_FOR_FRESH_SENSORS'

    def _output(self,speed=0.,steering=0.):
        feedback=dict(self.pd.debug)
        if self.state in ('parked','parking_failed','parking_cancelled'):
            feedback.update(active=False,reason=self.reason,pixel_segments=[],reference_pixel=None,steering_rad=0.)
        self.debug=dict(mode='parking_real2_fusion1',state=self.state,reason=self.reason,gap=self.gap_debug,
                        parking_grid=self.grid_debug,
                        parking_template=self.template_debug,
                        parking_fusion=self.fusion_debug,
                        selected_gap=self.selected,approach=self.approach_debug,feedback=feedback,sign=self.sign.debug(),
                        turn_path=self.path_debug,
                        cruise_distance_m=self.cruise_distance,slow_reason=self.slow_reason,
                        braking_distance_m=self.braking_distance,no_target_distance_m=self.no_target_distance,
                        approach_speed_cap_m_s=self.approach_speed_cap,
                        no_target_age_s=0. if self.no_target_since is None else max(0.,self.last_tick-self.no_target_since),
                        turn_phase=self.turn_phase,turn_angle_deg=math.degrees(self.turn_angle),arc_released=self.arc_released,white_line=self.line_debug,line_seen=self.line_seen,
                        search_distance_m=self.search_distance,turn_distance_m=self.turn_distance,
                        published_speed=float(speed),steering=float(steering))
        return float(speed),float(steering),self.state

    def _clear(self,scan,steering,speed):
        self.clear_reason='OBSTACLE_IN_NEXT_BODY_PATH'
        fraction=0. if scan is None else scan.get('valid_fraction',0.)
        rays={} if scan is None else scan.get('parking_rays',{})
        if not math.isfinite(fraction) or fraction<.5 or np.count_nonzero(rays.get('measured',[]))<3:
            self.clear_reason='INVALID_SCAN_HEALTH';return False
        points=body_points(scan,self.robot)
        if not np.isfinite(points).all():return False
        front=self.robot['front_bumper_x'];rear=self.robot['length']-front;half=self.robot['width']/2
        horizon=max(.05,speed*.25+speed*speed/(2*.30))
        curvature=math.tan(steering)/self.robot['wheelbase']
        for distance in np.linspace(0.,horizon,12):
            yaw=distance*curvature
            x=distance if abs(curvature)<1e-8 else math.sin(yaw)/curvature
            y=0. if abs(curvature)<1e-8 else (1-math.cos(yaw))/curvature
            delta=points-[x,y];c,s=math.cos(yaw),math.sin(yaw)
            xx=delta[:,0]*c+delta[:,1]*s;yy=-delta[:,0]*s+delta[:,1]*c
            dx=np.maximum(np.maximum(xx-front,-rear-xx),0.);dy=np.maximum(np.abs(yy)-half,0.)
            if len(points) and np.min(np.hypot(dx,dy))<=self.c['body_margin']:return False
        return True

    def _white(self,image,stamp):
        if stamp==self.last_image:return False
        self.last_image=stamp
        _,lines=parking_straight_lines(image,self.c)
        self.line_present=bool(lines)
        if lines:self.line_seen=True;self.loss_frames=0
        elif self.line_seen:self.loss_frames+=1
        self.line_debug=dict(stamp=float(stamp),present=self.line_present,segments=lines[:24],
                             seen_during_turn=self.line_seen,loss_frames=self.loss_frames)
        if not lines and self.line_seen:return True
        if self.selected is not None and lines:
            self.pd.update(lines,image.shape,self.selected,stamp,phase=self.turn_phase)
            feedback=self.pd.debug
            entry_progress=-float(np.dot(self.selected['entrance'],self.selected['normal']))
            qualified=(feedback.get('active') and self.turn_angle>=self.c['arc_min_angle'] and
                       abs(feedback.get('heading_rad',99.))<=self.c['arc_align_heading'] and
                       abs(feedback.get('lateral_m',99.))<=self.c['arc_align_lateral'] and
                       entry_progress>=-self.c['arc_entry_tolerance'])
            if self.turn_phase=='arc':
                self.align_frames=self.align_frames+1 if qualified else 0
                if self.align_frames>=self.c['arc_align_frames']:
                    self.turn_phase='align';self.pd.pid.reset()
            expected=math.atan2(self.selected['normal'][1],self.selected['normal'][0])
            if (self.turn_phase=='align' and self.turn_angle>=self.c['arc_release_angle'] and
                    abs(expected)<=self.c['arc_release_heading'] and feedback.get('active') and
                    abs(feedback.get('heading_rad',99.))<=self.c['arc_release_heading']):
                self.arc_released=True
        return True

    def _turn_lead(self):
        # 뒷축의 90도 원호 전진량 R. 영점 보정은 직진 보정이므로 물리 조향각에 중복 합산 제외
        # turn_radius: 실측 회전 반경(m). 11/14 lap4 IMU·odom 실측 0.60~0.63 (계산값 0.548 → 입구를 0.07 m 지나쳐 칸 왼쪽 선 위)
        radius=self.c.get('turn_radius') or self.robot['wheelbase']/abs(math.tan(self.c['turn_steering']))
        return radius+self.c['approach_start_offset']

    def _planned_turn_path(self,remaining=0.,samples=25):
        """Rear-axle points for the fixed right arc, in the current body frame."""
        radius=self.c.get('turn_radius') or self.robot['wheelbase']/abs(math.tan(self.c['turn_steering']))
        distance=self.c.get('turn_path_check_distance',radius*math.pi/2)
        travel=np.linspace(0.,distance,samples);angle=-travel/radius
        return np.column_stack((remaining+radius*np.sin(-angle),-radius*(1-np.cos(angle)),angle))

    def _turn_path_clear(self,scan,remaining=0.):
        """Check the complete entry arc before committing to a latched cell."""
        points=body_points(scan,self.robot)
        distance=self.c.get('turn_path_check_distance',self._turn_lead())
        clearance=path_clearance(points,self.c['turn_steering'],self.robot,distance)
        required=self.c.get('turn_path_margin',.03)
        path=self._planned_turn_path(remaining)
        self.path_debug=dict(clear=bool(clearance>=required),clearance_m=clearance,
                             required_m=required,points=path.tolist(),
                             candidate_slot=None if self.selected is None else self.selected.get('slot_index'),
                             rejected_slot=None)
        return clearance>=required

    @staticmethod
    def _move_target(target,dt,velocity,angular_velocity):
        yaw=angular_velocity*dt;travel=velocity*dt
        x=travel if abs(yaw)<1e-8 else travel*math.sin(yaw)/yaw
        y=0. if abs(yaw)<1e-8 else travel*(1-math.cos(yaw))/yaw
        c,s=math.cos(yaw),math.sin(yaw);rotation=np.array([[c,-s],[s,c]])
        for key in ('entrance','polygon'):
            target[key]=((np.asarray(target[key])-[x,y])@rotation).tolist()
        for key in ('tangent','normal'):
            target[key]=(np.asarray(target[key])@rotation).tolist()

    def _approach(self,now,dt,velocity,angular_velocity,scan):
        self.approach_distance+=abs(velocity)*dt
        self.search_distance+=abs(velocity)*dt
        if now-self.approach_started>self.c['approach_timeout']:return self.fail('APPROACH_TIMEOUT')
        if self.approach_distance>self.c['approach_distance_limit']:return self.fail('APPROACH_DISTANCE_LIMIT')
        # 절대 pose·TF·시간 동기화 관문 없이 /odom 속도로 상대 목표 갱신
        self._move_target(self.selected,dt,velocity,angular_velocity)
        remaining=float(self.selected['entrance'][0]-self._turn_lead())
        self.approach_debug=dict(target_entrance_body=self.selected['entrance'],turn_lead_m=self._turn_lead(),
                                 remaining_m=remaining,travel_m=self.approach_distance,source='relative_odom_twist')
        self.approach_debug['turn_path']=self._planned_turn_path(max(0.,remaining)).tolist()
        if remaining < -self.c['approach_passed_tolerance']:
            self.selected=None;self.state='parking_scan'
            self.reason='TURN_START_PASSED_LOOKING_FOR_NEXT_GAP'
            return None
        settle=self.c['turn_entry_settle_seconds']
        in_band=(abs(velocity-self.c['turn_speed'])<=self.c['turn_entry_speed_tolerance'] and
                 abs(velocity)<=self.c['turn_entry_max_speed'])
        if in_band:
            if self.entry_stable_since is None:self.entry_stable_since=now
        else:self.entry_stable_since=None
        stable_age=0. if self.entry_stable_since is None else now-self.entry_stable_since
        ready=(abs(velocity)<=self.c['turn_entry_max_speed'] if settle<=0 else in_band and stable_age>=settle)
        self.approach_debug.update(entry_speed_m_s=velocity,entry_stable_seconds=stable_age,
                                   entry_speed_ready=ready,waiting_for_slow_speed=remaining<=self.c['approach_start_tolerance'] and not ready)
        if remaining<=self.c['approach_start_tolerance'] and ready:
            if self.selected.get('source')=='parking_template' and not self._turn_path_clear(scan,max(0.,remaining)):
                slot=self.selected.get('slot_index')
                if slot is not None:self.rejected_template_slots.add(slot)
                self.path_debug['rejected_slot']=slot
                self.selected=None;self.state='parking_scan';self.entry_stable_since=None
                self.reason='SELECTED_SLOT_TURN_PATH_BLOCKED'
                return None
            self.turn_angle=0.;self.turn_phase='arc';self.align_frames=0;self.arc_released=False
            self.state='parking_turn';self.turn_started=now;self.turn_distance=0.
            self.line_seen=False;self.last_image=None;self.pd.reset();self.reason='ALIGNED_START_FIXED_RIGHT_TURN'
        return None

    def _slow(self,now,reason):
        self.slow_started=now;self.slow_reason=reason;self.entry_stable_since=None
        self.state='parking_approach' if self.selected is not None else 'parking_scan'
        # A latched FREE target reached from cruise keeps moving while its speed
        # cap falls with remaining distance.  UNKNOWN/no-target fallback still
        # uses the original fixed scan speed and never invents a parking target.
        self.smooth_approach=bool(self.selected is not None and reason=='TARGET_BRAKING_DISTANCE')
        self.approach_speed_cap=None
        if self.selected is not None:self.approach_started=now;self.approach_distance=0.

    def _approach_speed(self,requested):
        """Monotonic distance-based speed cap for a latched FREE target."""
        if not self.smooth_approach:
            return self.c['scan_speed']
        remaining=max(0.,float(self.approach_debug.get('remaining_m',0.)))
        usable=max(0.,remaining-self.c['cruise_slow_buffer']-self.c['turn_entry_settle_distance'])
        allowed=math.sqrt(self.c['scan_speed']**2+
                          2*self.c['cruise_braking_acceleration']*usable)
        requested=max(0.,float(requested))
        candidate=min(requested,self.c['cruise_max_speed'],allowed)
        if self.approach_speed_cap is None:
            self.approach_speed_cap=candidate
        else:
            # Once braking begins, noisy relative odometry cannot command a
            # re-acceleration that would cause another visible speed surge.
            self.approach_speed_cap=min(self.approach_speed_cap,candidate)
        return max(self.c['scan_speed'],self.approach_speed_cap)

    def _find_gap(self,scan,scan_stamp,now,velocity,rate):
        if scan_stamp==self.last_scan:return False
        self.last_scan=scan_stamp;self.nearest_start=None
        candidates,self.gap_debug=gap_candidates(scan,self.robot,self.c)
        self.gap_debug['scan_stamp']=float(scan_stamp);free=[];evaluations=[];late_seen=False
        for candidate in candidates:
            status,evidence=classify_gap(candidate,scan,self.robot,self.c)
            candidate=dict(candidate);self._move_target(candidate,now-scan_stamp,velocity,rate)
            remaining=float(candidate['entrance'][0]-self._turn_lead())
            late=remaining < -self.c['approach_passed_tolerance']
            late_seen|=status=='FREE' and late
            # UNKNOWN 후보도 진입점이 가까우면 감속 근거. FREE 판정 기준은 그대로 유지
            if status!='OCCUPIED' and not late:
                self.nearest_start=remaining if self.nearest_start is None else min(self.nearest_start,remaining)
            evaluations.append(dict(state=status,evidence=evidence,entrance=candidate['entrance'],
                                    turn_start_remaining_m=remaining,rejection='TURN_START_ALREADY_PASSED' if late and status=='FREE' else None))
            if status=='FREE' and not late:free.append(candidate)
        self.gap_debug['evaluations']=evaluations
        if free:self.selected=min(free,key=lambda b:b['entrance'][0])
        return late_seen

    def _observe_grid(self,image,image_stamp,scan,scan_stamp,now,velocity,rate):
        """Fuse painted cells with LiDAR occupancy and optionally latch a target.

        Camera geometry proposes cells; the existing ray-coverage classifier
        decides FREE/OCCUPIED/UNKNOWN. Geometry must remain stable across
        frames, while the first qualified FREE result keeps the original
        one-shot latch behavior.
        """
        if (not self.c.get('grid_enabled',False) or image_stamp==self.last_grid_image or
                self.state not in ('parking_cruise','parking_scan','parking_approach')):return False
        previous_stamp=self.last_grid_image
        self.last_grid_image=image_stamp
        elapsed=0. if previous_stamp is None else max(0.,min(.5,image_stamp-previous_stamp))
        for frame in self.grid_segment_frames:
            for color in ('yellow','white'):
                frame[color]=move_ground_segments(frame.get(color,[]),elapsed,velocity,rate)
        prior={color:[segment for frame in self.grid_segment_frames for segment in frame.get(color,[])]
               for color in ('yellow','white')}
        try:
            candidates,details=parking_grid_candidates(image,self.robot,self.c,prior)
        # An experimental observer must never interrupt the proven controller.
        except Exception as error:
            self.grid_tracks=[]
            self.grid_segment_frames=[]
            observe_only=bool(self.c.get('grid_observe_only',True))
            enabled=bool(self.c.get('grid_enable_actuation',False))
            self.grid_debug=dict(stamp=float(image_stamp),reason='GRID_DETECTOR_ERROR',error=type(error).__name__,
                                 observe_only=observe_only,enable_actuation=enabled,
                                 control_applied=False,evaluations=[])
            return False
        current=details.pop('current_segments_by_color',{})
        if any(current.get(color) for color in ('yellow','white')):
            self.grid_segment_frames.append(current)
            self.grid_segment_frames=self.grid_segment_frames[-self.c.get('grid_history_frames',6):]
        for track in self.grid_tracks:self._move_target(track['target'],elapsed,velocity,rate)
        evaluations=[];new_tracks=[]
        for source in candidates:
            candidate=dict(source)
            # Express the camera candidate at the LiDAR timestamp for occupancy,
            # then at now for diagnostics.  This remains relative-odometry only.
            self._move_target(candidate,scan_stamp-image_stamp,velocity,rate)
            status,evidence=classify_camera_slot(candidate,scan,self.robot,self.c)
            displayed=dict(candidate);self._move_target(displayed,now-scan_stamp,velocity,rate)
            match=None
            for track in self.grid_tracks:
                distance=float(np.linalg.norm(np.asarray(track['target']['entrance'])-np.asarray(displayed['entrance'])))
                alignment=float(np.dot(track['target']['normal'],displayed['normal']))
                if distance<=self.c.get('grid_track_distance',.15) and alignment>=self.c.get('grid_track_alignment',.9):
                    match=track;break
            count=1 if match is None else match['count']+1
            track=dict(target=displayed,count=count,state=status)
            new_tracks.append(track)
            stable=count>=self.c.get('grid_stable_frames',3)
            remaining=float(displayed['entrance'][0]-self._turn_lead())
            late=remaining < -self.c['approach_passed_tolerance']
            evaluations.append(dict(state=status,evidence=evidence,stable=stable,observations=count,
                                    entrance=displayed['entrance'],polygon=displayed['polygon'],
                                    turn_start_remaining_m=remaining,
                                    rejection='TURN_START_ALREADY_PASSED' if late and status=='FREE' else None,
                                    source='painted_grid',control_applied=False))
        self.grid_tracks=new_tracks
        observe_only=bool(self.c.get('grid_observe_only',True))
        enabled=bool(self.c.get('grid_enable_actuation',False))
        selected_now=False
        if enabled and not observe_only and self.selected is None:
            usable=[index for index,evaluation in enumerate(evaluations)
                    if evaluation['stable'] and evaluation['state']=='FREE' and evaluation['rejection'] is None]
            if usable:
                chosen=min(usable,key=lambda index:new_tracks[index]['target']['entrance'][0])
                self.selected=dict(new_tracks[chosen]['target'])
                evaluations[chosen]['control_applied']=True
                selected_now=True
        self.grid_debug=dict(stamp=float(image_stamp),scan_stamp=float(scan_stamp),reason=details.get('reason'),
                             observe_only=observe_only,enable_actuation=enabled,
                             control_applied=selected_now,
                             history_frames=len(self.grid_segment_frames),color=details.get('color'),
                             segments_px=details.get('segments_px',[]),
                             segments_ground=details.get('segments_ground',[]),evaluations=evaluations)
        return selected_now

    def _observe_template(self,image,image_stamp,scan,scan_stamp,now,velocity,rate):
        """Latch a three-cell metric lot from the start and side white lines."""
        if image_stamp==self.last_template_image:return False
        previous=self.last_template_image;self.last_template_image=image_stamp
        elapsed=0. if previous is None else max(0.,min(.5,image_stamp-previous))
        if self.template_anchor_track is not None:
            self._move_target(self.template_anchor_track['target'],elapsed,velocity,rate)
        for slot in self.template_slots:self._move_target(slot,elapsed,velocity,rate)
        if self.template_start_track is not None:self._move_target(self.template_start_track,elapsed,velocity,rate)
        try:landmarks=parking_template_landmarks(image,self.robot,self.c)
        except Exception as error:
            self.template_debug=dict(reason='TEMPLATE_DETECTOR_ERROR',error=type(error).__name__,
                                     armed=self.template_armed,evaluations=[])
            return False
        if not self.template_armed:
            self.template_start_frames=self.template_start_frames+1 if landmarks['start_seen'] else 0
            if self.template_start_frames>=self.c.get('template_start_frames',2):self.template_armed=True
        # 시작선 위치 추적 (11/18 혁준): 처음 본 값으로 잡고 주행 거리만큼 당김. 이후 관측은 추적 위치 근처일 때만 반영
        # (주차 줄 너머의 다른 가로 흰 선이 시작선을 대신하지 않게)
        seen_start=landmarks.get('start_x')
        if seen_start is not None:
            tracked=None if self.template_start_track is None else self.template_start_track['entrance'][0]
            if tracked is None or abs(seen_start-tracked)<=self.c.get('template_start_track_distance',.25):
                self.template_start_track=dict(entrance=[seen_start,0.],polygon=[[seen_start,0.]],
                                               tangent=[1.,0.],normal=[0.,-1.])
        start_x=None if self.template_start_track is None else float(self.template_start_track['entrance'][0])
        # 기준점은 시작선보다 template_anchor_after_start(m) 넘게 앞쪽일 때만 인정
        # (redecide_1118_12: 옅은 노란 차로 선이 흰색으로 읽혀 시작선 0.85 m 전에 기준점이 잡힘 → 주차 구역 전에 우회전.
        #  정상 기준점은 시작선 뒤 0.22~0.45 m, 03·04·06·07·08 녹화)
        after=self.c.get('template_anchor_after_start')
        slope=self.c.get('template_anchor_max_slope')
        candidates=[item for item in landmarks['anchors']
                    if (after is None or start_x is None or item['anchor'][0]>=start_x+after) and
                    (slope is None or abs(item['tangent'][1])<=slope)]
        anchor_observations=[]
        if self.template_armed and not self.template_slots and candidates:
            anchor=candidates[0]
            observed=slots_from_anchor(anchor['anchor'],anchor['tangent'],anchor['normal'],
                self.c.get('grid_slot_width',.50),self.c.get('grid_slot_depth',.59),
                self.c.get('template_slot_count',3))
            for slot in observed:self._move_target(slot,now-image_stamp,velocity,rate)
            first=observed[0];count=1
            if self.template_anchor_track is not None:
                old=self.template_anchor_track['target']
                distance=float(np.linalg.norm(np.asarray(old['entrance'])-np.asarray(first['entrance'])))
                alignment=float(np.dot(old['tangent'],first['tangent']))
                if (distance<=self.c.get('template_anchor_distance',.18) and
                        alignment>=self.c.get('template_anchor_alignment',.94)):
                    count=self.template_anchor_track['count']+1
            self.template_anchor_track=dict(target=dict(first),count=count)
            anchor_observations.append(dict(confidence=anchor['confidence'],entrance=first['entrance'],
                                            observations=count))
            if count>=self.c.get('template_anchor_frames',2):self.template_slots=observed
        evaluations=[];selected_now=False
        if self.template_slots:
            fused_slots,assigned,self.fusion_debug,self.template_alignment=fuse_template_slots(
                self.template_slots,scan,self.robot,self.c,self.template_alignment)
            lidar_origin=np.array([self.robot['lidar']['x'],self.robot['lidar'].get('y',0.)])
            for slot,slot_points in zip(fused_slots,assigned):
                at_scan=dict(slot);self._move_target(at_scan,scan_stamp-now,velocity,rate)
                slot_scan=dict(scan);slot_scan['parking_points']=slot_points-lidar_origin
                status,evidence=classify_camera_slot(at_scan,slot_scan,self.robot,self.c)
                evidence=dict(evidence,fused_assigned_points=len(slot_points),
                              fusion_offset=self.template_alignment.tolist())
                remaining=float(slot['entrance'][0]-self._turn_lead())
                late=remaining < -self.c['approach_passed_tolerance']
                evaluations.append(dict(slot_index=slot['slot_index'],state=status,evidence=evidence,
                    entrance=slot['entrance'],polygon=slot['polygon'],turn_start_remaining_m=remaining,
                    rejection='TURN_START_ALREADY_PASSED' if late and status=='FREE' else None,
                    control_applied=False))
            selected_index=None if self.selected is None else self.selected.get('slot_index')
            if self.selected is not None and self.selected.get('source')=='parking_template':
                selected_evaluation=next((item for item in evaluations
                                          if item['slot_index']==selected_index),None)
                occupied=selected_evaluation is not None and selected_evaluation['state']=='OCCUPIED'
                count=self.template_occupied_frames.get(selected_index,0)+1 if occupied else 0
                self.template_occupied_frames[selected_index]=count
                if selected_evaluation is not None:selected_evaluation['selected_occupied_frames']=count
                if count>=self.c.get('template_selected_occupied_frames',2):
                    self.rejected_template_slots.add(selected_index)
                    if selected_evaluation is not None:
                        selected_evaluation['rejection']='SELECTED_SLOT_BECAME_OCCUPIED'
                    self.selected=None;selected_index=None
                    if self.state=='parking_approach':
                        self.state='parking_scan';self.entry_stable_since=None
            if self.selected is None:
                reach=self.c.get('template_select_max_remaining')
                usable=[index for index,item in enumerate(evaluations)
                        if item['state']=='FREE' and item['rejection'] is None and
                        item['slot_index'] not in self.rejected_template_slots and
                        (reach is None or item['turn_start_remaining_m']<=reach)]
                if usable:
                    chosen=min(usable,key=lambda index:fused_slots[index]['entrance'][0])
                    self.selected=dict(fused_slots[chosen]);evaluations[chosen]['control_applied']=True
                    selected_now=True
        self.template_debug=dict(reason=('TEMPLATE_ACTIVE' if self.template_slots else
                                         'WAITING_FOR_SIDE_ANCHOR' if self.template_armed else
                                         'WAITING_FOR_START_LINE'),
                                 armed=self.template_armed,start_seen=landmarks['start_seen'],
                                 start_frames=self.template_start_frames,anchors=anchor_observations,
                                 anchor_candidates=landmarks['anchors'][:6],slot_count=len(self.template_slots),
                                 start_x=start_x,anchors_rejected=len(landmarks['anchors'])-len(candidates),
                                 slots=self.template_slots,rejected_slots=sorted(self.rejected_template_slots),
                                 evaluations=evaluations,start_segments=landmarks.get('start_segments',[]),
                                 white_longitudinal=landmarks.get('white_longitudinal',[]),
                                 yellow_longitudinal=landmarks.get('yellow_longitudinal',[]),
                                 control_applied=selected_now)
        return selected_now

    def step(self,now,image,image_stamp,scan,scan_stamp,pose,pose_stamp,history,velocity,lane,
             camera_info=None,search_command=None,angular_velocity=0.,detections=(),detection_stamp=-1e9,
             measured_turn_rate=None):
        if self.state in ('parked','parking_failed','parking_cancelled'):return self._output()
        if now<self.last_tick:return self.fail('CLOCK_RESET')
        dt=min(max(now-self.last_tick,0.),.5);self.last_tick=now
        rate=angular_velocity if measured_turn_rate is None else measured_turn_rate
        if not all(math.isfinite(v) for v in (velocity,angular_velocity,rate)) or scan is None or not 0<=now-scan_stamp<self.c['sensor_timeout']:
            self.pause(now);return self._output()
        if image is None or not 0<=now-image_stamp<self.c['sensor_timeout']:
            self.pause(now);return self._output()
        self.sign.observe(detections,detection_stamp,now)
        if self.c.get('template_enabled',False):
            target_selected=self._observe_template(image,image_stamp,scan,scan_stamp,now,velocity,rate)
        else:target_selected=self._observe_grid(image,image_stamp,scan,scan_stamp,now,velocity,rate)
        late=False
        if self.state=='parking_cruise':
            self.cruise_distance+=abs(velocity)*dt
            if now-self.started>self.c['cruise_timeout']:return self.fail('CRUISE_SEARCH_TIMEOUT')
            if self.cruise_distance>self.c['cruise_distance_limit']:return self.fail('CRUISE_SEARCH_DISTANCE_LIMIT')
            if self.selected is not None:
                # A freshly fused grid target is already expressed at ``now``.
                # Moving it again here would advance the turn point twice.
                if not target_selected:self._move_target(self.selected,dt,velocity,rate)
                self.nearest_start=self.selected['entrance'][0]-self._turn_lead()
            elif self.c.get('template_enabled',False):
                if self.template_armed:self.nearest_start=0.
            else:late=self._find_gap(scan,scan_stamp,now,velocity,rate)
            cap=self.c['trial_cruise_max_speed'] if self.c.get('test_drive_until_sign') else self.c['cruise_max_speed']
            requested=0. if search_command is None else min(cap,max(0.,search_command[0]))
            # 가속 직전에도 요구 속도 기준 감속 거리 확보
            self.braking_distance=slow_distance(max(abs(velocity),requested),self.c)
            close=self.nearest_start is not None and self.nearest_start<=self.braking_distance
            missing=self.sign.passed and self.selected is None and self.nearest_start is None
            if missing:
                if self.no_target_since is None:self.no_target_since=now;self.no_target_distance=0.
                self.no_target_distance+=abs(velocity)*dt
            else:self.no_target_since=None;self.no_target_distance=0.
            fallback=missing and (now-self.no_target_since>=self.c['no_target_cruise_seconds'] or
                                  self.no_target_distance>=self.c['no_target_cruise_distance'])
            sign_slow=self.sign.passed and self.c['slow_on_sign_passed']
            if close or sign_slow or fallback:
                self._slow(now,'TARGET_BRAKING_DISTANCE' if close else
                           'NEAR_SIGN_PASSED' if sign_slow else 'NO_TARGET_AFTER_SIGN_PASSAGE')
                dt=0.  # cruise에서 이미 반영한 목표 이동·주행 거리를 중복 적분하지 않음
        if self.state=='parking_scan':
            self.search_distance+=abs(velocity)*dt
            if now-self.slow_started>self.c['search_timeout']:return self.fail('NO_FREE_GAP_BEFORE_TIMEOUT')
            if self.search_distance>self.c['search_distance_limit']:return self.fail('SEARCH_DISTANCE_LIMIT')
            if self.selected is None and not self.c.get('template_enabled',False):
                late|=self._find_gap(scan,scan_stamp,now,velocity,rate)
            if self.selected is not None:
                self.state='parking_approach';self.approach_started=now;self.approach_distance=0.;dt=0.
        if self.state=='parking_approach':
            stopped=self._approach(now,dt,velocity,rate,scan)
            if stopped is not None:return stopped
            if self.state=='parking_turn':self._white(image,image_stamp);dt=0.
        if self.state=='parking_cruise':
            speed=0. if search_command is None else max(0.,search_command[0])
            cap=self.c['trial_cruise_max_speed'] if self.c.get('test_drive_until_sign') else self.c['cruise_max_speed']
            speed=min(speed,cap);steering=0. if search_command is None else search_command[1]
            self.reason='MAIN_LANE_WITH_LIDAR_SEARCH'
        elif self.state in ('parking_scan','parking_approach'):
            steering=search_command[1] if lane and lane.get('valid') and search_command else 0.
            requested=self.c['scan_speed'] if search_command is None else search_command[0]
            speed=(self._approach_speed(requested) if self.state=='parking_approach'
                   else self.c['scan_speed'])
            if self.state=='parking_approach':
                self.reason='DECELERATING_BEFORE_TURN' if self.approach_debug.get('waiting_for_slow_speed') else 'APPROACHING_LATCHED_TURN_START'
            elif self.reason=='SELECTED_SLOT_TURN_PATH_BLOCKED':pass
            elif late:self.reason='TURN_START_PASSED_LOOKING_FOR_NEXT_GAP'
            elif self.reason!='TURN_START_PASSED_LOOKING_FOR_NEXT_GAP':self.reason='SLOW_SEARCH_UNTIL_FIRST_FREE'
        else:
            self._move_target(self.selected,dt,velocity,rate)
            self.turn_distance+=abs(velocity)*dt;self.turn_angle=max(0.,self.turn_angle-rate*dt)
            new_image=self._white(image,image_stamp)
            loss_armed=self.turn_angle>=self.c.get('white_loss_min_angle',0.)
            self.line_debug['stop_armed']=loss_armed
            if new_image and loss_armed and self.line_seen and self.loss_frames>=self.c['white_loss_frames']:
                self.state='parked';self.reason='WHITE_STRAIGHT_LINE_DISAPPEARED_STOP';return self._output()
            # 흰 선 소실은 최소 회전각 이후에만 아래 종료 조건보다 우선
            slot_heading=math.atan2(self.selected['normal'][1],self.selected['normal'][0])
            done=self.c.get('turn_done_heading')
            if done is not None and slot_heading>=-done:
                self.state='parked';self.reason='TURN_COMPLETE_STOP';return self._output()
            if now-self.turn_started>self.c['turn_timeout']:return self.fail('TURN_TIMEOUT')
            if self.turn_distance>self.c['turn_distance_limit']:return self.fail('TURN_DISTANCE_LIMIT')
            if not self.line_seen and now-self.turn_started>self.c['white_acquire_timeout']:return self.fail('WHITE_LINE_NEVER_SEEN')
            speed=self.c['turn_speed']
            command_phase=self.turn_phase
            steering,ready=self.pd.command(now,dt,phase=command_phase,hold_turn=not self.arc_released,preserve_arc=True)
            if not ready:
                self.reason='WAITING_FOR_PARKING_LINE_FEEDBACK';self.motion_since=None;return self._output()
            self.reason='GUARDED_PD_RIGHT_TURN' if self.c['pd']['enabled'] else 'CALIBRATED_RIGHT_TURN'
        steering=clip(steering,-self.robot['max_steering'],self.robot['max_steering'])
        if not self._clear(scan,steering,max(speed,abs(velocity))):
            self.reason=self.clear_reason;self.motion_since=None;return self._output()
        if self.state=='parking_turn' and self.pd.debug.get('steering_transition'):
            self.reason='PARKING_PD_STEERING_TRANSITION';self.motion_since=None;return self._output(0.,steering)
        if speed<=.001:self.motion_since=None;return self._output(0.,steering)
        if abs(velocity)>.005:self.motion_since=now
        elif self.motion_since is None:self.motion_since=now
        elif now-self.motion_since>self.c['motion_timeout']:return self.fail('NO_ODOMETRY_MOTION')
        return self._output(speed,steering)
