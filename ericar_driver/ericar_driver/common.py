"""공통 유틸: 값 제한, PID"""
# 출처: hello-osy/2026-orda-mentoring src/orda_sim/orda_sim/answer.py (613c9b4)


def clip(value, low, high):
    return float(max(low, min(high, value)))



class PID:
    def __init__(self, config):
        self.config = config
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.previous = None
        self.derivative = 0.0
        self.terms = dict(p=0.0, i=0.0, d=0.0, total=0.0)

    def update(self, error, dt):
        c = self.config
        dt = clip(dt, 0.001, 0.2)
        self.integral = clip(self.integral + error * dt,
                             -c['integral_limit'], c['integral_limit'])
        derivative = 0.0 if self.previous is None else (error - self.previous) / dt
        alpha = c['derivative_alpha']
        self.derivative = alpha * derivative + (1 - alpha) * self.derivative
        self.previous = error
        p, i, d = c['kp'] * error, c['ki'] * self.integral, c['kd'] * self.derivative
        self.terms = dict(p=p, i=i, d=d, total=p+i+d)
        return p+i+d
