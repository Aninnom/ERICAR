"""주차 전용 흰 경계선 PD. main 차선 PID와 상태 분리, 부분 측면선 사용"""
import math
import numpy as np
from .common import PID,clip


def ground_points(pixels,shape,camera):
    """lane_vision 카메라 모델의 역변환. 뒷축 기준 앞 x·왼쪽 y (m)"""
    h,w=shape[:2];pixels=np.asarray(pixels,float)
    f=w/(2*math.tan(math.radians(camera['hfov_deg'])/2))
    down=(pixels[:,1]-h/2)/f;left=(w/2-pixels[:,0])/f
    c,s=math.cos(camera['pitch']),math.sin(camera['pitch'])
    denominator=s+down*c
    with np.errstate(divide='ignore',invalid='ignore'):
        depth=camera['z']/denominator
        forward=camera['x']+depth*(c-down*s)
        lateral=camera.get('y',0.)+depth*left
    points=np.column_stack((forward,lateral))
    valid=(denominator>1e-4)&np.isfinite(points).all(axis=1)&(depth>0)
    return points,valid


def image_point(point,shape,camera):
    h,w=shape[:2];dx=point[0]-camera['x'];c,s=math.cos(camera['pitch']),math.sin(camera['pitch'])
    depth=c*dx+s*camera['z'];up=s*dx-c*camera['z']
    if depth<=.03:return None
    f=w/(2*math.tan(math.radians(camera['hfov_deg'])/2))
    return [int(w/2-f*(point[1]-camera.get('y',0.))/depth),int(h/2-f*up/depth)]


def boundary_observation(lines,shape,target,robot,config):
    """평행한 두 선 우선, 한 선이면 저장한 칸 방향·폭으로 좌우 식별. 가로선·이웃 칸 제외"""
    expected=np.asarray(target['normal'],float)
    expected_heading=math.atan2(expected[1],expected[0])
    if expected[0]<=.1:return None,'BAY_NOT_FACING_CAMERA'
    reference=config['reference_distance'];entrance=target['entrance']
    expected_center=entrance[1]+expected[1]/expected[0]*(reference-entrance[0])
    fits=[]
    for line in lines:
        points,valid=ground_points(np.asarray(line).reshape(2,2),shape,robot['camera'])
        if not valid.all():continue
        points=points[np.argsort(points[:,0])];delta=points[1]-points[0]
        length=float(np.linalg.norm(delta))
        if length<config['minimum_ground_length'] or delta[0]<.03:continue
        if points[0,0]<config['ground_near'] or points[1,0]>config['ground_far']:continue
        if reference<points[0,0]-config['maximum_extrapolation'] or reference>points[1,0]+config['maximum_extrapolation']:continue
        heading=math.atan2(delta[1],delta[0])
        if abs(heading-expected_heading)>config['direction_tolerance']:continue
        lateral=float(points[0,1]+(reference-points[0,0])*delta[1]/delta[0])
        if abs(lateral)>config['maximum_lateral']:continue
        fits.append(dict(lateral=lateral,heading=heading,length=length,pixel=list(map(int,line))))
    # Hough 중복 도색 가장자리 묶음. 실제 서로 다른 경계만 폭 검사
    groups=[]
    for fit in sorted(fits,key=lambda f:-f['length']):
        if any(abs(fit['lateral']-g['lateral'])<config['merge_distance'] and
               abs(fit['heading']-g['heading'])<config['parallel_tolerance'] for g in groups):continue
        groups.append(fit)
    options=[];width=config['slot_width']
    for i,a in enumerate(groups):
        for b in groups[i+1:]:
            if abs(a['heading']-b['heading'])>config['parallel_tolerance']:continue
            heading=(a['heading']+b['heading'])/2
            observed_width=abs(a['lateral']-b['lateral'])*math.cos(heading)
            center=(a['lateral']+b['lateral'])/2
            if abs(observed_width-width)>config['width_tolerance']:continue
            if abs(center-expected_center)>config['center_association']:continue
            options.append((abs(center-expected_center)+abs(observed_width-width),dict(
                source='pair',lateral_m=center,heading_rad=heading,width_m=observed_width,
                pixel_segments=[a['pixel'],b['pixel']])))
    if options:return min(options,key=lambda p:p[0])[1],'BOUNDARY_PAIR'
    for fit in groups:
        half=width/(2*math.cos(fit['heading']))
        for side in (-1,1):
            center=fit['lateral']-side*half
            mismatch=abs(center-expected_center)
            if mismatch<=config['single_association']:
                options.append((mismatch,dict(source='single_left' if side>0 else 'single_right',
                    lateral_m=center,heading_rad=fit['heading'],width_m=None,pixel_segments=[fit['pixel']])))
    if options:return min(options,key=lambda p:p[0])[1],'SINGLE_BOUNDARY'
    return None,'NO_ASSOCIATED_PARKING_BOUNDARY'


class ParkingPD:
    def __init__(self,robot,parking):
        self.robot=robot;self.parking=parking;self.c=parking['pd'];self.pid=PID(self.c)
        if not 0<=self.c['full_heading']<self.c['activate_heading']<math.pi/2:raise ValueError('invalid parking PD heading range')
        if self.c['steering_rate']<=0 or self.c['feedback_timeout']<=0 or self.c['slot_width']<=robot['width']:raise ValueError('invalid parking PD limits')
        self.reset()

    def reset(self):
        self.pid.reset();self.last_stamp=None;self.valid_stamp=None;self.engaged=False
        self.steering=self.target=self.parking['turn_steering'];self.debug=dict(enabled=self.c['enabled'],active=False,reason='WAITING_FOR_TURN')

    def pause(self):
        self.pid.reset();self.last_stamp=None;self.valid_stamp=None
        self.debug=dict(self.debug,active=False,reason='WAITING_FOR_FRESH_FEEDBACK')

    def update(self,lines,shape,target,stamp):
        if not self.c['enabled']:return
        expected_heading=math.atan2(target['normal'][1],target['normal'][0])
        if abs(expected_heading)>self.c['activate_heading']:
            self.debug=dict(enabled=True,active=False,reason='INITIAL_FIXED_RIGHT_TURN',expected_heading_rad=expected_heading)
            return
        observation,reason=boundary_observation(lines,shape,target,self.robot,self.c)
        if observation is None:
            self.debug=dict(self.debug,active=False,reason=reason,pixel_segments=[],reference_pixel=None)
            return
        dt=.05 if self.last_stamp is None else stamp-self.last_stamp
        if dt<=0:return
        if dt>self.c['feedback_timeout']:self.pid.reset()
        self.last_stamp=self.valid_stamp=stamp;self.engaged=True
        error=observation['lateral_m']+self.c['heading_weight']*observation['heading_rad']
        desired=clip(self.pid.update(error,dt),-self.robot['max_steering'],self.robot['max_steering'])
        blend=clip((self.c['activate_heading']-abs(observation['heading_rad']))/
                   (self.c['activate_heading']-self.c['full_heading']),0.,1.)
        self.target=(1-blend)*self.parking['turn_steering']+blend*desired
        self.debug=dict(observation,enabled=True,active=True,reason=reason,error=error,
                        expected_heading_rad=expected_heading,blend=blend,target_steering_rad=self.target,
                        terms=dict(self.pid.terms),reference_pixel=image_point(
                            [self.c['reference_distance'],observation['lateral_m']],shape,self.robot['camera']))

    def command(self,now,dt):
        self.debug['age_s']=None if self.valid_stamp is None else max(0.,now-self.valid_stamp)
        if not self.c['enabled']:return self.parking['turn_steering'],True
        if self.engaged and (self.valid_stamp is None or now-self.valid_stamp>self.c['feedback_timeout']):
            self.debug=dict(self.debug,active=False,reason='FEEDBACK_TIMEOUT')
            return 0.,False
        maximum=self.c['steering_rate']*max(0.,dt)
        self.steering+=clip(self.target-self.steering,-maximum,maximum)
        self.debug['steering_rad']=self.steering
        # 반대 방향 보정으로 넘기는 동안 전진 유보. 조향 변화율 제한으로 생기는 반대쪽 이동 억제
        self.debug['steering_transition']=self.engaged and self.steering*self.target<0 and abs(self.target)>.04 and abs(self.target-self.steering)>.15
        return self.steering,True
