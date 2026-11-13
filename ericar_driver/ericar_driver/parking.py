"""parking_auto1: FREE 목표 저장 → 회전 시작점 접근 → 우회전 → 흰 직선 소실 정지"""
import math
import numpy as np
from .common import clip
from .lidar import body_points
from .parking_gap import gap_candidates,classify_gap
from .parking_white import parking_straight_lines


class Parking:
    def __init__(self,config):
        self.c=config['answer']['parking'];self.robot=config['robot']
        if not -self.robot['max_steering']<=self.c['turn_steering']<0:raise ValueError('turn_steering must be a bounded right turn')
        if not math.isfinite(self._turn_lead()) or self._turn_lead()<=0:raise ValueError('positive turn lead required')
        if not 0<=self.c['approach_start_tolerance']<=self.c['approach_passed_tolerance']:raise ValueError('invalid approach tolerances')
        self.state='idle';self.reason='WAITING_FOR_PARKING_SIGN';self.debug={};self.stable_since=None
        self.selected=None;self.gap_debug={};self.line_debug={};self.last_scan=None
        self.approach_distance=0.;self.approach_debug={}
        self.last_image=None;self.line_seen=False;self.line_present=False
        self.loss_frames=0;self.search_distance=0.;self.turn_distance=0.;self.motion_since=None

    def start(self,now):
        self.state='parking_scan';self.reason='SLOW_SEARCH_UNTIL_FIRST_FREE'
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
        self.last_tick=now;self.motion_since=None;self.reason='WAITING_FOR_FRESH_SENSORS'

    def _output(self,speed=0.,steering=0.):
        self.debug=dict(mode='parking_auto1',state=self.state,reason=self.reason,gap=self.gap_debug,
                        selected_gap=self.selected,approach=self.approach_debug,white_line=self.line_debug,line_seen=self.line_seen,
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
        return True

    def _turn_lead(self):
        # 뒷축의 90도 원호 전진량 R. 영점 보정은 직진 보정이므로 물리 조향각에 중복 합산 제외
        radius=self.robot['wheelbase']/abs(math.tan(self.c['turn_steering']))
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
        if remaining<=self.c['approach_start_tolerance']:
            self.state='parking_turn';self.turn_started=now;self.turn_distance=0.
            self.line_seen=False;self.last_image=None;self.reason='ALIGNED_START_FIXED_RIGHT_TURN'
        return None

    def step(self,now,image,image_stamp,scan,scan_stamp,pose,pose_stamp,history,velocity,lane,camera_info=None,search_command=None,angular_velocity=0.):
        if self.state in ('parked','parking_failed','parking_cancelled'):return self._output()
        if now<self.last_tick:return self.fail('CLOCK_RESET')
        dt=min(max(now-self.last_tick,0.),.5);self.last_tick=now
        if not math.isfinite(velocity) or not math.isfinite(angular_velocity) or scan is None or not 0<=now-scan_stamp<self.c['sensor_timeout']:
            self.pause(now);return self._output()
        if image is None or not 0<=now-image_stamp<self.c['sensor_timeout']:
            self.pause(now);return self._output()
        rejected_late=False
        if self.state=='parking_scan':
            self.search_distance+=abs(velocity)*dt
            if now-self.started>self.c['search_timeout']:return self.fail('NO_FREE_GAP_BEFORE_TIMEOUT')
            if self.search_distance>self.c['search_distance_limit']:return self.fail('SEARCH_DISTANCE_LIMIT')
            if scan_stamp!=self.last_scan:
                self.last_scan=scan_stamp
                candidates,self.gap_debug=gap_candidates(scan,self.robot,self.c)
                self.gap_debug['scan_stamp']=float(scan_stamp)
                free=[];evaluations=[]
                for candidate in candidates:
                    state,evidence=classify_gap(candidate,scan,self.robot,self.c)
                    candidate=dict(candidate)
                    # 스캔 시각 이후 이동량의 짧은 등속 근사. 엄격한 동기화 기록 미사용
                    self._move_target(candidate,now-scan_stamp,velocity,angular_velocity)
                    remaining=float(candidate['entrance'][0]-self._turn_lead())
                    late=state=='FREE' and remaining < -self.c['approach_passed_tolerance']
                    rejected_late|=late
                    evaluations.append(dict(state=state,evidence=evidence,entrance=candidate['entrance'],
                                            turn_start_remaining_m=remaining,rejection='TURN_START_ALREADY_PASSED' if late else None))
                    if state=='FREE' and not late:free.append(candidate)
                self.gap_debug['evaluations']=evaluations
                if free:
                    self.selected=min(free,key=lambda b:b['entrance'][0])
                    self.state='parking_approach';self.approach_started=now;self.approach_distance=0.
                    dt=0.  # FREE를 잡기 전 이동량을 새 목표에 중복 차감하지 않음
        if self.state=='parking_approach':
            stopped=self._approach(now,dt,velocity,angular_velocity)
            if stopped is not None:return stopped
            if self.state=='parking_turn':
                self._white(image,image_stamp)
                dt=0.  # 접근 중 이동량을 회전 거리로 중복 집계하지 않음
        if self.state in ('parking_scan','parking_approach'):
            steering=search_command[1] if lane and lane.get('valid') and search_command else 0.
            speed=self.c['scan_speed']
            if self.state=='parking_approach':self.reason='APPROACHING_LATCHED_TURN_START'
            elif rejected_late:self.reason='TURN_START_PASSED_LOOKING_FOR_NEXT_GAP'
            elif self.reason!='TURN_START_PASSED_LOOKING_FOR_NEXT_GAP':self.reason='SLOW_SEARCH_UNTIL_FIRST_FREE'
        else:
            self.turn_distance+=abs(velocity)*dt
            if now-self.turn_started>self.c['turn_timeout']:return self.fail('TURN_TIMEOUT')
            if self.turn_distance>self.c['turn_distance_limit']:return self.fail('TURN_DISTANCE_LIMIT')
            new_image=self._white(image,image_stamp)
            if new_image and self.line_seen and self.loss_frames>=self.c['white_loss_frames']:
                self.state='parked';self.reason='WHITE_STRAIGHT_LINE_DISAPPEARED_STOP'
                return self._output()
            if not self.line_seen and now-self.turn_started>self.c['white_acquire_timeout']:
                return self.fail('WHITE_LINE_NEVER_SEEN')
            speed=self.c['turn_speed'];steering=self.c['turn_steering']
            self.reason='RIGHT_TURN_UNTIL_WHITE_LINE_DISAPPEARS'
        steering=clip(steering,-self.robot['max_steering'],self.robot['max_steering'])
        if not self._clear(scan,steering,speed):
            self.reason=self.clear_reason;self.motion_since=None
            return self._output()
        if abs(velocity)>.005:self.motion_since=now
        elif self.motion_since is None:self.motion_since=now
        elif now-self.motion_since>self.c['motion_timeout']:return self.fail('NO_ODOMETRY_MOTION')
        return self._output(speed,steering)
