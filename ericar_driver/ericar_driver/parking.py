"""parking_2: 표지판 → 서행 FREE 검색 → 고정 우회전 → 흰 직선 소실 즉시 정지"""
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
        self.state='idle';self.reason='WAITING_FOR_PARKING_SIGN';self.debug={};self.stable_since=None
        self.selected=None;self.gap_debug={};self.line_debug={};self.last_scan=None
        self.last_image=None;self.line_seen=False;self.line_present=False
        self.loss_frames=0;self.search_distance=0.;self.turn_distance=0.;self.motion_since=None

    def start(self,now):
        self.state='parking_scan';self.reason='SLOW_SEARCH_UNTIL_FIRST_FREE'
        self.started=self.last_tick=now;self.turn_started=None;self.selected=None
        self.last_scan=self.last_image=None;self.line_seen=False;self.line_present=False
        self.loss_frames=0;self.search_distance=self.turn_distance=0.;self.motion_since=None
        self.gap_debug={};self.line_debug={}

    def close(self):pass

    def fail(self,reason):
        self.state='parking_failed';self.reason=reason
        return self._output()

    def cancel(self):
        self.state='parking_cancelled';self.reason='EXTERNAL_CANCEL'

    def pause(self,now):
        self.last_tick=now;self.motion_since=None;self.reason='WAITING_FOR_FRESH_SENSORS'

    def _output(self,speed=0.,steering=0.):
        self.debug=dict(mode='parking_2',state=self.state,reason=self.reason,gap=self.gap_debug,
                        selected_gap=self.selected,white_line=self.line_debug,line_seen=self.line_seen,
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

    def step(self,now,image,image_stamp,scan,scan_stamp,pose,pose_stamp,history,velocity,lane,camera_info=None,search_command=None):
        if self.state in ('parked','parking_failed','parking_cancelled'):return self._output()
        if now<self.last_tick:return self.fail('CLOCK_RESET')
        dt=min(max(now-self.last_tick,0.),.5);self.last_tick=now
        if not math.isfinite(velocity) or scan is None or not 0<=now-scan_stamp<self.c['sensor_timeout']:
            self.pause(now);return self._output()
        if image is None or not 0<=now-image_stamp<self.c['sensor_timeout']:
            self.pause(now);return self._output()
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
                    evaluations.append(dict(state=state,evidence=evidence,entrance=candidate['entrance']))
                    if state=='FREE':free.append(candidate)
                self.gap_debug['evaluations']=evaluations
                if free:
                    self.selected=min(free,key=lambda b:b['entrance'][0])
                    self.state='parking_turn';self.turn_started=now;self.turn_distance=0.
                    self.line_seen=False;self.last_image=None;self._white(image,image_stamp)
                    self.reason='FIRST_FREE_FIXED_RIGHT_TURN'
            if self.state=='parking_scan':
                steering=search_command[1] if lane and lane.get('valid') and search_command else 0.
                speed=self.c['scan_speed'];self.reason='SLOW_SEARCH_UNTIL_FIRST_FREE'
            else:speed=self.c['turn_speed'];steering=self.c['turn_steering']
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
