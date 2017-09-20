
MIN_NUM = float('-inf')
MAX_NUM = float('inf')


class PID(object):
    def __init__(self, kp, ki, kd, mn=MIN_NUM, mx=MAX_NUM):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min = mn
        self.max = mx
        self.pterm = 0.0
        self.iterm = 0.0
        self.dterm = 0.0

        self.int_val = self.last_int_val = self.last_error = 0.

    def reset(self):
        self.int_val = 0.0
        self.last_int_val = 0.0

    def step(self, error, sample_time):
        self.last_int_val = self.int_val

        integral = self.int_val + error * sample_time;
        derivative = (error - self.last_error) / sample_time;

        # make individual terms available for tuning
        self.pterm = self.kp * error
        self.iterm = self.ki * self.int_val
        self.dterm = self.kd * derivative

        y = self.pterm + self.iterm + self.dterm;
        val = max(self.min, min(y, self.max))

        if val > self.max:
            val = self.max
        elif val < self.min:
            val = self.min
        else:
            self.int_val = integral
        self.last_error = error

        return val
