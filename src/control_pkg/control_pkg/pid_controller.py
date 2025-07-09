import numpy as np

class PIDController():
    def __init__(self, Kp, Ki, Kd, min_saturation, max_saturation, Ts):
        # Atributes
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.Ts = Ts
        
        self.control_prev = 0
        self.error_prev = 0
        self.error_prev_prev = 0

        self.min_saturation = min_saturation
        self.max_saturation = max_saturation


    def pid(self, setpoint, feedback):
        error = setpoint - feedback

        K0 = self.Kp + self.Ts * self.Ki + self.Kd / self.Ts
        K1 = -self.Kp - 2 * self.Kd / self.Ts
        K2 = self.Kd / self.Ts

        control_effort = self.control_prev + K0 * error + K1 * self.error_prev + K2 * self.error_prev_prev

        if control_effort > 0:
            control_effort = np.clip(control_effort, self.min_saturation, self.max_saturation)
        else:
            control_effort = np.clip(control_effort, -self.max_saturation, -self.min_saturation)

        # Shift states for next iteration
        self.error_prev_prev = self.error_prev
        self.error_prev = error
        self.control_prev = control_effort

        return control_effort