"""parking_auto2: 일반속도 탐색 → 보정 반경 접근 → 기본 우회전 유지 → 흰 선 소실 즉시 정지"""
import math
import numpy as np
from .common import clip
from .lidar import body_points
from .parking_gap import gap_candidates,classify_gap
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
        self.approach_distance=0.;self.approach_debug={}
        self.last_image=None;self.line_seen=False;self.line_present=False
        self.loss_frames=0;self.search_distance=0.;self.turn_distance=0.;self.motion_since=None

    def _reset_progress(self):
        self.cruise_distance=0.;self.nearest_start=None;self.slow_started=0.;self.slow_reason=None
        self.turn_angle=0.;self.turn_phase='arc';self.align_frames=0;self.arc_released=False

    def start(self,now):
        self.pd.reset();self.sign.reset();self._reset_progress()
        self.state='parking_cruise' if self.c['cruise_search_enabled'] else 'parking_scan'
        self.reason='MAIN_LANE_WITH_LIDAR_SEARCH' if self.state=='parking_cruise' else 'SLOW_SEARCH_UNTIL_FIRST_FREE'
        self.slow_started=now
        self.started=self.last_tick=now;self.turn_started=None;self.selected=None
        self.last_scan=self.last_image=None;self.line_seen=False;self.line_present=False
        self.loss_frames=0;self.search_distance=self.turn_distance=0.;self.motion_since=None
        self.gap_debug={};self.line_debug={}
        self.approach_distance=0.;self.approach_debug={}

    def close(self):pass

    def fail(self,reason):
        self.state='parking_failed';self.reason=reason
        return self._output()

    def cancel(self):
        self.state='parking_cancelled';self.reason='EXTERNAL_CANCEL'

    def pause(self,now):
        self.pd.pause();self.align_frames=0
        self.last_tick=now;self.motion_since=None;self.reason='WAITING_FOR_FRESH_SENSORS'

    def _output(self,speed=0.,steering=0.):
        feedback=dict(self.pd.debug)
        if self.state in ('parked','parking_failed','parking_cancelled'):
            feedback.update(active=False,reason=self.reason,pixel_segments=[],reference_pixel=None,steering_rad=0.)
        self.debug=dict(mode='parking_auto2',state=self.state,reason=self.reason,gap=self.gap_debug,
                        selected_gap=self.selected,approach=self.approach_debug,feedback=feedback,sign=self.sign.debug(),
                        cruise_distance_m=self.cruise_distance,slow_reason=self.slow_reason,
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

    def _approach(self,now,dt,velocity,angular_velocity):
        self.approach_distance+=abs(velocity)*dt
        self.search_distance+=abs(velocity)*dt
        if now-self.approach_started>self.c['approach_timeout']:return self.fail('APPROACH_TIMEOUT')
        if self.approach_distance>self.c['approach_distance_limit']:return self.fail('APPROACH_DISTANCE_LIMIT')
        # 절대 pose·TF·시간 동기화 관문 없이 /odom 속도로 상대 목표 갱신
        self._move_target(self.selected,dt,velocity,angular_velocity)
        remaining=float(self.selected['entrance'][0]-self._turn_lead())
        self.approach_debug=dict(target_entrance_body=self.selected['entrance'],turn_lead_m=self._turn_lead(),
                                 remaining_m=remaining,travel_m=self.approach_distance,source='relative_odom_twist')
        if remaining < -self.c['approach_passed_tolerance']:
            self.selected=None;self.state='parking_scan'
            self.reason='TURN_START_PASSED_LOOKING_FOR_NEXT_GAP'
            return None
        self.approach_debug['waiting_for_slow_speed']=remaining<=self.c['approach_start_tolerance'] and abs(velocity)>self.c['turn_entry_max_speed']
        if remaining<=self.c['approach_start_tolerance'] and abs(velocity)<=self.c['turn_entry_max_speed']:
            self.turn_angle=0.;self.turn_phase='arc';self.align_frames=0;self.arc_released=False
            self.state='parking_turn';self.turn_started=now;self.turn_distance=0.
            self.line_seen=False;self.last_image=None;self.pd.reset();self.reason='ALIGNED_START_FIXED_RIGHT_TURN'
        return None

    def _slow(self,now,reason):
        self.slow_started=now;self.slow_reason=reason
        self.state='parking_approach' if self.selected is not None else 'parking_scan'
        if self.selected is not None:self.approach_started=now;self.approach_distance=0.

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
        late=False
        if self.state=='parking_cruise':
            self.cruise_distance+=abs(velocity)*dt
            if now-self.started>self.c['cruise_timeout']:return self.fail('CRUISE_SEARCH_TIMEOUT')
            if self.cruise_distance>self.c['cruise_distance_limit']:return self.fail('CRUISE_SEARCH_DISTANCE_LIMIT')
            if self.selected is not None:
                self._move_target(self.selected,dt,velocity,rate)
                self.nearest_start=self.selected['entrance'][0]-self._turn_lead()
            else:late=self._find_gap(scan,scan_stamp,now,velocity,rate)
            close=self.nearest_start is not None and self.nearest_start<=slow_distance(velocity,self.c)
            if self.sign.passed or close:
                self._slow(now,'TARGET_BRAKING_DISTANCE' if close else 'NEAR_SIGN_PASSED')
                dt=0.  # cruise에서 이미 반영한 목표 이동·주행 거리를 중복 적분하지 않음
        if self.state=='parking_scan':
            self.search_distance+=abs(velocity)*dt
            if now-self.slow_started>self.c['search_timeout']:return self.fail('NO_FREE_GAP_BEFORE_TIMEOUT')
            if self.search_distance>self.c['search_distance_limit']:return self.fail('SEARCH_DISTANCE_LIMIT')
            late|=self._find_gap(scan,scan_stamp,now,velocity,rate)
            if self.selected is not None:
                self.state='parking_approach';self.approach_started=now;self.approach_distance=0.;dt=0.
        if self.state=='parking_approach':
            stopped=self._approach(now,dt,velocity,rate)
            if stopped is not None:return stopped
            if self.state=='parking_turn':self._white(image,image_stamp);dt=0.
        if self.state=='parking_cruise':
            speed=0. if search_command is None else max(0.,search_command[0])
            cap=self.c['trial_cruise_max_speed'] if self.c.get('test_drive_until_sign') else self.c['cruise_max_speed']
            speed=min(speed,cap);steering=0. if search_command is None else search_command[1]
            self.reason='MAIN_LANE_WITH_LIDAR_SEARCH'
        elif self.state in ('parking_scan','parking_approach'):
            steering=search_command[1] if lane and lane.get('valid') and search_command else 0.
            speed=self.c['scan_speed']
            if self.state=='parking_approach':
                self.reason='DECELERATING_BEFORE_TURN' if self.approach_debug.get('waiting_for_slow_speed') else 'APPROACHING_LATCHED_TURN_START'
            elif late:self.reason='TURN_START_PASSED_LOOKING_FOR_NEXT_GAP'
            elif self.reason!='TURN_START_PASSED_LOOKING_FOR_NEXT_GAP':self.reason='SLOW_SEARCH_UNTIL_FIRST_FREE'
        else:
            self._move_target(self.selected,dt,velocity,rate)
            self.turn_distance+=abs(velocity)*dt;self.turn_angle=max(0.,self.turn_angle-rate*dt)
            new_image=self._white(image,image_stamp)
            if new_image and self.line_seen and self.loss_frames>=self.c['white_loss_frames']:
                self.state='parked';self.reason='WHITE_STRAIGHT_LINE_DISAPPEARED_STOP';return self._output()
            # 흰 선 소실은 아래 방향·시간·거리·조향 조건보다 우선
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
