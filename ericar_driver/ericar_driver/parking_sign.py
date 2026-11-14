"""주차 표지판 통과 관측. 새 YOLO 추론 결과만 누적, 검출 정지는 통과 증거에서 제외"""
import math


class SignPassage:
    def __init__(self,parking,detector):
        self.c=parking;self.timeout=detector['timeout'];self.minimum=detector['minimum_area_by_class']['parking']
        self.reset()

    def reset(self):
        self.last_result=None;self.near_seen=False;self.missing_since=None
        self.misses=0;self.passed=False;self.area=0.

    def observe(self,objects,stamp,now):
        if not math.isfinite(stamp) or not 0<=now-stamp<self.timeout:return
        if self.last_result is not None and stamp<=self.last_result:return
        if self.last_result is not None and stamp-self.last_result>self.timeout:
            self.misses=0;self.missing_since=None
        self.last_result=stamp
        signs=[o for o in objects if o.get('name')=='parking' and o.get('area',0)>=self.minimum]
        if signs:
            self.misses=0;self.missing_since=None
            for sign in signs:
                self.area=float(sign.get('area',0.))
                center=sign.get('center')
                if center is None and sign.get('bbox'):
                    center=[(sign['bbox'][0]+sign['bbox'][2])/2,(sign['bbox'][1]+sign['bbox'][3])/2]
                edge=center is not None and center[0]>=self.c['sign_exit_center'] and self.area>=self.c['sign_exit_min_area']
                self.near_seen|=self.area>=self.c['sign_near_area'] or edge
        elif self.near_seen:
            if self.missing_since is None:self.missing_since=stamp
            self.misses+=1
            if self.misses>=self.c['sign_gone_frames'] and stamp-self.missing_since>=self.c['sign_gone_seconds']:
                self.passed=True

    def debug(self):
        return dict(near_seen=self.near_seen,passed=self.passed,missing_results=self.misses,
                    last_area=self.area,last_result_stamp=self.last_result)


def slow_distance(speed,config):
    """현재 속도에서 탐색 속도까지의 감속·반응·여유 거리 (m)"""
    speed=abs(speed);slow=config['scan_speed']
    return speed*config['cruise_reaction_time']+max(0.,speed*speed-slow*slow)/(2*config['cruise_braking_acceleration'])+config['cruise_slow_buffer']
