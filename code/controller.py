

class PID:
    # modified controller from the udacity Self-Driving car project 3
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kp = Kp
        self.set_point = 0.
        self.error = 0.
        self.integral = 0.
        self.old_error = 0.

    def set_desired(self, desired):
        self.set_point = desired

    def update(self, measurement):
        # proportional error
        self.error = self.set_point - measurement

        # integral error
        self.integral += self.error

        # derivative error
        delta_error = self.old_error - measurement
        self.old_error = measurement

        return self.Kp * self.error + self.Ki * self.integral + self.Kp * delta_error
