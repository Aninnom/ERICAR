"""fast_parking1: 목표 감속 거리까지 차선 주행 → 저속 우회전 → 흰 선 소실 즉시 정지"""
import math
import numpy as np
from .common import clip
from .lidar import body_points,footprint_clearance
from .parking_gap import gap_candidates,classify_gap,classify_camera_slot
from .parking_grid import parking_grid_candidates,move_ground_segments
from .parking_template import parking_template_landmarks,slots_from_anchor
from .parking_fusion import fuse_template_slots
from .parking_start_anchor import anchor_from_start_line
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
        self.template_alignment=None;self.fusion_debug={}
        self.refine_debug={};self.refine_count=0;self.selected_fresh=False;self.refine_last=None
        self.template_start_track=None;self.start_debug={}
        self.armed_distance=0.;self.line_seen_armed=False
        self.path_block_since=None;self.path_block_scans=0;self.path_block_stamp=None;self.scan_stamp=None
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
        self.template_alignment=None;self.fusion_debug={}
        self.refine_debug={};self.refine_count=0;self.selected_fresh=False;self.refine_last=None
        self.template_start_track=None;self.start_debug={}
        self.armed_distance=0.;self.line_seen_armed=False
        self.path_block_since=None;self.path_block_scans=0;self.path_block_stamp=None;self.scan_stamp=None
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
        if lines:
            self.line_seen=True;self.loss_frames=0
            if self.turn_angle>=self._white_loss_angle():self.line_seen_armed=True
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

    def _turn_radius(self):
        return self.c.get('turn_radius') or self.robot['wheelbase']/abs(math.tan(self.c['turn_steering']))

    def _planned_turn_path(self,remaining=0.,samples=25):
        """Rear-axle poses: straight to the turn start, then the measured right arc."""
        radius=self._turn_radius()
        distance=self.c.get('turn_path_check_distance',radius*math.pi/2)
        straight=np.linspace(0.,max(0.,remaining),4)[:-1] if remaining>.01 else np.empty(0)
        travel=np.linspace(0.,distance,samples);angle=-travel/radius
        arc=np.column_stack((max(0.,remaining)+radius*np.sin(-angle),-radius*(1-np.cos(angle)),angle))
        return np.vstack((np.column_stack((straight,np.zeros_like(straight),np.zeros_like(straight))),arc))

    def _turn_path_clear(self,scan,remaining=0.):
        """Check the complete entry arc before committing to a latched cell.

        11/18 redecide_1118_08: 칸 주변 LiDAR 점과는 0.38 m 이상 떨어졌는데 clearance 0.0으로 회전 취소.
        - 원호는 실측 반경(turn_radius)·회전 시작점 기준, 차체는 앞 범퍼·길이 실측값 사용
        - 지금 차체 안(+self_margin)에 있는 점은 자기 반사·잡음으로 보고 제외 (실제 접촉이면 _clear가 정지)
        - 막은 점·위치를 debug에 기록 → 다음 주행에서 원인 확인
        """
        points=body_points(scan,self.robot)
        points=points[np.isfinite(points).all(axis=1)] if len(points) else points
        path=self._planned_turn_path(remaining)
        centre=self.robot.get('front_bumper_x',self.robot['length']/2+self.robot['wheelbase']/2)-self.robot['length']/2
        poses=np.column_stack((path[:,0]+centre*np.cos(path[:,2]),path[:,1]+centre*np.sin(path[:,2]),path[:,2]))
        ignored=0
        if len(points):
            here=np.array([[centre,0.,0.]])
            own=np.array([footprint_clearance(point[None],here,self.robot) for point in points])
            keep=own>self.c.get('turn_path_self_margin',.01);ignored=int(np.count_nonzero(~keep));points=points[keep]
        required=self.c.get('turn_path_margin',.03)
        blocker=None;clearance=float('inf')
        if len(points):
            dx=points[None,:,0]-poses[:,None,0];dy=points[None,:,1]-poses[:,None,1]
            c=np.cos(poses[:,2,None]);s=np.sin(poses[:,2,None])
            longitudinal=np.maximum(np.abs(dx*c+dy*s)-self.robot['length']/2,0)
            lateral=np.maximum(np.abs(-dx*s+dy*c)-self.robot['width']/2,0)
            distance=np.hypot(longitudinal,lateral);pose,index=np.unravel_index(np.argmin(distance),distance.shape)
            clearance=float(distance[pose,index])
            blocker=dict(point=points[index].tolist(),pose=path[pose].tolist(),pose_index=int(pose))
        self.path_debug=dict(clear=bool(clearance>=required),clearance_m=clearance,
                             required_m=required,points=path.tolist(),blocker=blocker,ignored_own_points=ignored,
                             candidate_slot=None if self.selected is None else self.selected.get('slot_index'),
                             rejected_slot=None)
        return clearance>=required

    @staticmethod
    def _move_target(target,dt,velocity,angular_velocity):
        yaw=angular_velocity*dt;travel=velocity*dt
        x=travel if abs(yaw)<1e-8 else travel*math.sin(yaw)/yaw
        y=0. if abs(yaw)<1e-8 else travel*(1-math.cos(yaw))/yaw
        c,s=math.cos(yaw),math.sin(yaw);rotation=np.array([[c,-s],[s,c]])
        for key in ('entrance','polygon','center'):
            if key in target:target[key]=((np.asarray(target[key])-[x,y])@rotation).tolist()
        for key in ('tangent','normal'):
            target[key]=(np.asarray(target[key])@rotation).tolist()

    def _approach(self,now,dt,velocity,angular_velocity,scan):
        self.approach_distance+=abs(velocity)*dt
        self.search_distance+=abs(velocity)*dt
        if now-self.approach_started>self.c['approach_timeout']:return self.fail('APPROACH_TIMEOUT')
        if self.approach_distance>self.c['approach_distance_limit']:return self.fail('APPROACH_DISTANCE_LIMIT')
        # 절대 pose·TF·시간 동기화 관문 없이 /odom 속도로 상대 목표 갱신
        if not self.selected_fresh:self._move_target(self.selected,dt,velocity,angular_velocity)
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
        holding=self.path_block_since is not None
        if remaining<=self.c['approach_start_tolerance'] and (ready or holding):
            if self.selected.get('source')=='parking_template' and not self._turn_path_clear(scan,max(0.,remaining)):
                # 11/18: 한 번 막힘으로 바로 칸을 버리면 그대로 지나쳐 조향이 안 들어감.
                # 회전 시작점에서 정지 후 새 스캔으로 다시 확인, 계속 막힐 때만 다른 칸 탐색
                if self.path_block_since is None:
                    self.path_block_since=now;self.path_block_scans=0;self.path_block_stamp=None
                if self.scan_stamp!=self.path_block_stamp:
                    self.path_block_stamp=self.scan_stamp;self.path_block_scans+=1
                waited=now-self.path_block_since
                self.path_debug.update(block_scans=self.path_block_scans,block_seconds=waited)
                if (self.path_block_scans<self.c.get('turn_path_block_scans',4) or
                        waited<self.c.get('turn_path_block_seconds',1.)):
                    self.reason='TURN_PATH_BLOCKED_WAITING';return None
                slot=self.selected.get('slot_index')
                if slot is not None:self.rejected_template_slots.add(slot)
                self.path_debug['rejected_slot']=slot
                self.selected=None;self.state='parking_scan';self.entry_stable_since=None
                self.path_block_since=None
                self.reason='SELECTED_SLOT_TURN_PATH_BLOCKED'
                return None
            self.path_block_since=None
            self.turn_angle=0.;self.turn_phase='arc';self.align_frames=0;self.arc_released=False
            self.state='parking_turn';self.turn_started=now;self.turn_distance=0.
            self.line_seen=False;self.line_seen_armed=False;self.last_image=None;self.pd.reset();self.reason='ALIGNED_START_FIXED_RIGHT_TURN'
        return None

    def _white_loss_angle(self):
        """흰 선 소실 정지를 켜는 회전각. 템플릿 모드는 template_white_loss_min_angle (track_test_1118_02: 71°에서 덜 들어감)"""
        angle=self.c.get('white_loss_min_angle',0.)
        if self.c.get('template_enabled',False):angle=self.c.get('template_white_loss_min_angle',angle)
        return angle

    def _cruise_cap(self):
        if self.c.get('test_drive_until_sign'):return self.c['trial_cruise_max_speed']
        cap=self.c['cruise_max_speed']
        # 시작선 이후 주차 구간: 첫 칸 회전점까지 감속 거리 확보용 상한 (0.4 → 0.49 m 필요, 0.3 → 0.35 m)
        if self.c.get('template_enabled',False) and self.template_armed:
            cap=min(cap,self.c.get('template_cruise_max_speed',cap))
        return cap

    def _template_nearest_start(self):
        evaluations=self.template_debug.get('evaluations') or []
        values=[item['turn_start_remaining_m'] for item in evaluations
                if item.get('state')!='OCCUPIED' and item.get('slot_index') not in self.rejected_template_slots and
                item['turn_start_remaining_m']>=-self.c['approach_passed_tolerance']]
        return min(values) if values else None

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

    def _gate_anchors(self,landmarks,elapsed,age,velocity,rate):
        """Accept only yellow→white transitions beyond the transverse start line.

        11/18 redecide_1118_11/12/14: 시작선보다 1 m 이상 앞(노란 차선 중간)에서 가짜 전환점 래치
        → 주차장 전에 우회전. 실제 전환점(주차장 앞 경계)은 시작선 뒤 약 0.5 m (도면 5.61 → 6.11,
        사진 0.47). 시작선은 보일 때 갱신, 카메라 아래로 지나간 뒤에는 오도메트리로 이동
        """
        track=self.template_start_track
        if track is not None:self._move_target(track,elapsed,velocity,rate)
        observed=landmarks.get('start_x')
        # 래치 전: 보이는 가장 가까운 가로선(신호등 정지선 등을 지나면 다음 선으로 교체)
        # 래치 뒤: 시작선을 지나친 다음 칸 안쪽 가로선으로 바뀌지 않게 추적값과 가까운 관측만 갱신
        if (observed is not None and track is not None and self.template_slots and
                abs(observed-float(track['entrance'][0]))>self.c.get('template_start_track_gate',.30)):
            observed=None
        if observed is not None:
            landmarks=dict(landmarks,start_x=observed)
            track=dict(entrance=[landmarks['start_x'],0.],polygon=[[landmarks['start_x'],0.]],
                       tangent=[1.,0.],normal=[0.,-1.])
            self._move_target(track,age,velocity,rate);self.template_start_track=track
        start=None if self.template_start_track is None else float(self.template_start_track['entrance'][0])
        low=self.c.get('template_start_to_lot_min',.25);high=self.c.get('template_start_to_lot_max',.90)
        lateral=self.c.get('template_anchor_lateral',[-.50,-.12])
        max_angle=self.c.get('template_anchor_max_angle',.26)
        accepted=[];rejected=0
        for anchor in landmarks.get('anchors',[]):
            gap=None if start is None else float(anchor['anchor'][0])-start
            # 오른쪽 차로 경계 근처·차로와 거의 평행한 전환점만 (먼 곳 다른 흰 선 y≈-0.95 제외)
            y=float(anchor['anchor'][1]);angle=abs(math.atan2(anchor['tangent'][1],anchor['tangent'][0]))
            if (gap is not None and low<=gap<=high and lateral[0]<=y<=lateral[1] and angle<=max_angle):
                accepted.append(anchor)
            else:rejected+=1
        self.start_debug=dict(start_x=start,accepted=len(accepted),rejected=rejected,
                              gap_range=[low,high])
        return accepted

    def _refine_template(self,anchors,age,velocity,rate):
        """Re-anchor the latched lot from a near yellow→white transition.

        11/18 redecide_1118_08: 1.3 m에서 래치한 템플릿이 LiDAR 대비 약 0.2 m 짧음
        (카메라 원거리 투영 오차, 멀수록 큼). 가까운 전환점(template_refine_range, 기본 0.70~0.95 m)으로 진행 방향만 다시 맞춤.
        크기·간격·방향은 고정. 회전 시작 뒤에는 변경 없음
        """
        self.refine_debug=dict(applied=False)
        if self.state=='parking_turn' or not self.c.get('template_refine_enabled',True):return
        found=False
        low,high=self.c.get('template_refine_range',[.70,.95])
        first=self.template_slots[0];tangent=np.asarray(first['tangent'],float)
        normal=np.asarray(first['normal'],float)
        current=np.asarray(first['polygon'][0],float)
        for anchor in anchors:
            if anchor['confidence']!='COLOR_TRANSITION':continue
            point=np.asarray(anchor['anchor'],float)
            if not low<=point[0]<=high:continue
            found=True
            if float(np.dot(anchor['tangent'],tangent))<self.c.get('template_anchor_alignment',.94):continue
            target=dict(entrance=point.tolist(),polygon=[point.tolist()],tangent=list(anchor['tangent']),
                        normal=list(anchor['normal']))
            self._move_target(target,age,velocity,rate)
            delta=np.asarray(target['entrance'])-current
            du=float(delta@tangent);dv=float(delta@normal)
            self.refine_debug=dict(applied=False,du=du,dv=dv,anchor=target['entrance'])
            if abs(du)>self.c.get('template_refine_max_jump',.35) or abs(dv)>self.c.get('template_refine_max_normal',.08):
                self.refine_debug['reason']='JUMP_TOO_LARGE';self.refine_last=None;return
            # 한 프레임 잡음(11/18 14.15 s, 0.81 m 단발) 제외: 직전 관측과 템플릿 기준 차이가 일치할 때만 반영
            last=self.refine_last;self.refine_last=du
            if last is None or abs(du-last)>self.c.get('template_refine_consistency',.06):
                self.refine_debug['reason']='WAITING_FOR_CONSISTENT_OBSERVATION';return
            shift=self.c.get('template_refine_gain',.5)*(du+last)/2*tangent
            self.refine_last=du-float(shift@tangent)
            for slot in self.template_slots:
                for key in ('entrance','center'):
                    if key in slot:slot[key]=(np.asarray(slot[key])+shift).tolist()
                slot['polygon']=(np.asarray(slot['polygon'])+shift).tolist()
            # 융합 오프셋에서 같은 양을 빼서 출력 연속 유지. 이후 LiDAR가 새 기준에서 다시 맞춤
            if self.template_alignment is not None:self.template_alignment=self.template_alignment-shift
            self.refine_count+=1
            self.refine_debug.update(applied=True,shift=shift.tolist(),count=self.refine_count)
            return
        if not found:self.refine_last=None

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
            near=tracked is not None and abs(seen_start-tracked)<=self.c.get('template_start_track_distance',.25)
            count=self.template_start_track['count']+1 if near else 1
            # 추적 위치 근처의 관측만 반영. 단 아직 3번 못 본 추적은 잘못 잡은 것일 수 있어 새 관측으로 다시 시작
            if tracked is None or near or self.template_start_track['count']<3:
                self.template_start_track=dict(entrance=[seen_start,0.],polygon=[[seen_start,0.]],
                                               tangent=[1.,0.],normal=[0.,-1.],count=count)
        start_x=None if self.template_start_track is None else float(self.template_start_track['entrance'][0])
        # 기준점은 시작선보다 template_anchor_after_start(m) 넘게 앞쪽일 때만 인정
        # (redecide_1118_12: 옅은 노란 차로 선이 흰색으로 읽혀 시작선 0.85 m 전에 기준점이 잡힘 → 주차 구역 전에 우회전.
        #  정상 기준점은 시작선 뒤 0.22~0.45 m, 03·04·06·07·08 녹화)
        after=self.c.get('template_anchor_after_start')
        slope=self.c.get('template_anchor_max_slope')
        candidates=[item for item in landmarks['anchors']
                    if (after is None or start_x is None or item['anchor'][0]>=start_x+after) and
                    (slope is None or abs(item['tangent'][1])<=slope)]
        # 줄 시작 = 시작선 + 고정 거리 (template_anchor_from_start, parking_start_anchor.py). 색 전환 기준점은 조명에 따라 어긋남
        # 시작선이 template_start_fix_distance 안에 들어와 3번 넘게 확인됐을 때부터 사용. 시작선을 한 번도 못 봤으면 색 전환 기준으로
        if self.c.get('template_anchor_from_start',False) and start_x is not None:
            ready=(start_x<=self.c.get('template_start_fix_distance',1.2) and
                   self.template_start_track.get('count',0)>=self.c.get('template_start_frames_min',3))
            candidates=[anchor_from_start_line(start_x,landmarks.get('right_ground',[]),self.c)] if ready else []
        anchor_observations=[]
        if self.template_armed and not self.template_slots and candidates:
            anchor=candidates[0]
            observed=slots_from_anchor(anchor['anchor'],anchor['tangent'],anchor['normal'],
                self.c.get('grid_slot_width',.50),self.c.get('grid_slot_depth',.59),
                self.c.get('template_slot_count',3),self.c.get('template_slot_pitch'))
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
                # 빈 칸 판정은 칸 앞쪽만 (track_test_1118_04·05: 둘째 줄 물체 앞면이 칸 뒤 경계에 걸려 OCCUPIED).
                # LIMO 길이 0.33 m라 앞쪽 0.50 m로도 주차된 차는 잡힘
                at_scan['depth']=min(float(at_scan['depth']),self.c.get('template_occupancy_depth',at_scan['depth']))
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
            # 회전을 시작한 뒤에는 선택 칸을 다시 판정하지 않음 (11/18 혁준, redecide_1118_15: 회전 중 2프레임 OCCUPIED로
            # selected가 None이 됐는데 상태는 parking_turn에 남아 다음 주기 _move_target(None) 예외 → 주행 노드 종료)
            if (self.selected is not None and self.selected.get('source')=='parking_template' and
                    self.state!='parking_turn'):
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
                    self.selected=None;selected_index=None;self.path_block_since=None
                    if self.state=='parking_approach':
                        self.state='parking_scan';self.entry_stable_since=None
            if self.selected is None and self.state!='parking_turn':
                reach=self.c.get('template_select_max_remaining')
                usable=[index for index,item in enumerate(evaluations)
                        if item['state']=='FREE' and item['rejection'] is None and
                        item['slot_index'] not in self.rejected_template_slots and
                        (reach is None or item['turn_start_remaining_m']<=reach)]
                if usable:
                    chosen=min(usable,key=lambda index:fused_slots[index]['entrance'][0])
                    self.selected=dict(fused_slots[chosen]);evaluations[chosen]['control_applied']=True
                    # 첫 칸 목표 이동 (11/18 혁준, redecide_1118_22·24): 주차 표지판 받침이 첫 칸 시작 모서리(우회전 안쪽)에 있어
                    # 회전 경로 여유 0.00~0.03 m로 SELECTED_SLOT_TURN_PATH_BLOCKED → 첫 칸을 버리고 지나감.
                    # 선택한 칸이 0번이면 조향 목표만 줄 방향(다음 칸 쪽)으로 template_first_slot_shift(m) 옮김. 칸 평가용 틀은 그대로
                    shift=self.c.get('template_first_slot_shift',0.)
                    if shift and self.selected.get('slot_index')==0:
                        move=shift*np.asarray(self.selected['tangent'],float)
                        for key in ('entrance','center'):
                            self.selected[key]=(np.asarray(self.selected[key],float)+move).tolist()
                        self.selected['polygon']=(np.asarray(self.selected['polygon'],float)+move).tolist()
                        self.selected['first_slot_shift_m']=float(shift)
                    selected_now=True
            elif (self.selected is not None and self.selected.get('source')=='parking_template' and
                  self.state in ('parking_cruise','parking_scan','parking_approach')):
                # 래치한 칸도 회전 전까지는 보정된 템플릿·LiDAR 정렬을 따라감 (오도메트리 누적 대신)
                fresh=next((slot for slot in fused_slots if slot['slot_index']==selected_index),None)
                if fresh is not None:
                    self.selected=dict(fresh);self.selected_fresh=True;selected_now=True
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
                                 control_applied=selected_now,refine=self.refine_debug,
                                 start_line=self.start_debug)
        return selected_now

    def step(self,now,image,image_stamp,scan,scan_stamp,pose,pose_stamp,history,velocity,lane,
             camera_info=None,search_command=None,angular_velocity=0.,detections=(),detection_stamp=-1e9,
             measured_turn_rate=None):
        if self.state in ('parked','parking_failed','parking_cancelled'):return self._output()
        if now<self.last_tick:return self.fail('CLOCK_RESET')
        dt=min(max(now-self.last_tick,0.),.5);self.last_tick=now;self.selected_fresh=False
        self.scan_stamp=scan_stamp
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
                # 11/18 redecide_1118_33: 시작선을 2프레임 보자마자 nearest_start=0 → 0.1 s 만에 서행,
                # 회전까지 2.4 m를 0.12 m/s로 22 s. 이제 차선 주행 속도를 유지하고 YOLO도 계속 돎.
                # 감속 근거: 아직 지나지 않은 비점유 칸의 회전 시작점까지 거리 (기존 감속 거리 비교 그대로)
                self.nearest_start=self._template_nearest_start()
                if self.template_armed:self.armed_distance+=abs(velocity)*dt
            else:late=self._find_gap(scan,scan_stamp,now,velocity,rate)
            cap=self._cruise_cap()
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
            sign_slow=self.sign.passed and (self.c['slow_on_sign_passed'] or
                (self.c.get('template_enabled',False) and self.c.get('template_slow_on_sign_passed',False)))
            # 안전장치: 시작선을 본 뒤 template_cruise_armed_distance(m) 지나도 표지판 소실·칸 감속 근거가 없으면 서행
            armed_slow=(self.c.get('template_enabled',False) and self.template_armed and
                        self.armed_distance>=self.c.get('template_cruise_armed_distance',float('inf')))
            if close or sign_slow or fallback or armed_slow:
                self._slow(now,'TARGET_BRAKING_DISTANCE' if close else
                           'NEAR_SIGN_PASSED' if sign_slow else
                           'START_LINE_DISTANCE_FALLBACK' if armed_slow else 'NO_TARGET_AFTER_SIGN_PASSAGE')
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
            speed=min(speed,self._cruise_cap());steering=0. if search_command is None else search_command[1]
            self.reason='MAIN_LANE_WITH_LIDAR_SEARCH'
        elif self.state in ('parking_scan','parking_approach'):
            steering=search_command[1] if lane and lane.get('valid') and search_command else 0.
            requested=self.c['scan_speed'] if search_command is None else search_command[0]
            speed=(self._approach_speed(requested) if self.state=='parking_approach'
                   else self.c.get('search_speed',self.c['scan_speed']) if self.c.get('template_enabled',False)
                   else self.c['scan_speed'])
            if self.state=='parking_approach' and self.path_block_since is not None:
                speed=0.;self.reason='TURN_PATH_BLOCKED_WAITING'
            elif self.state=='parking_approach':
                self.reason='DECELERATING_BEFORE_TURN' if self.approach_debug.get('waiting_for_slow_speed') else 'APPROACHING_LATCHED_TURN_START'
            elif self.reason=='SELECTED_SLOT_TURN_PATH_BLOCKED':pass
            elif late:self.reason='TURN_START_PASSED_LOOKING_FOR_NEXT_GAP'
            elif self.reason!='TURN_START_PASSED_LOOKING_FOR_NEXT_GAP':self.reason='SLOW_SEARCH_UNTIL_FIRST_FREE'
        else:
            if self.selected is None:return self.fail('TURN_TARGET_LOST')
            self._move_target(self.selected,dt,velocity,rate)
            self.turn_distance+=abs(velocity)*dt;self.turn_angle=max(0.,self.turn_angle-rate*dt)
            new_image=self._white(image,image_stamp)
            loss_armed=self.turn_angle>=self._white_loss_angle()
            self.line_debug['stop_armed']=loss_armed
            # 11/18 redecide_1118_35: 셋째 칸은 28°부터 선이 안 보임 → 65° 도달 즉시 정지(덜 들어감).
            # 최소 회전각 이후에 보였던 선이 사라질 때만 정지, 아니면 방향 조건(TURN_COMPLETE_STOP)으로 종료
            seen=(self.line_seen_armed if self.c.get('template_enabled',False) and
                  self.c.get('white_loss_requires_seen_after_arm',True) else self.line_seen)
            self.line_debug['seen_after_arm']=self.line_seen_armed
            if new_image and loss_armed and seen and self.loss_frames>=self.c['white_loss_frames']:
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
