import numpy as np

class PIDController():
    def __init__(self, Kp, Ki, Kd, saturation, Ts):
        # Atributes
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.Ts = Ts
        
        self.control_prev = 0
        self.error_prev = 0
        self.error_prev_prev = 0

        self.saturation = saturation


    def pid(self, setpoint, feedback):
        error = setpoint - feedback

        K0 = self.kp + self.Ts * self.ki + self.kd / self.Ts
        K1 = -self.kp - 2 * self.kd / self.Ts
        K2 = self.kd / self.Ts

        control_effort = self.control_prev + K0 * error + K1 * self.error_prev + K2 * self.error_prev_prev

        control_effort = np.clip(control_effort, -self.saturation, self.saturation)

        # Shift states for next iteration
        self.error_prev_prev = self.error_prev
        self.error_prev = error
        self.control_prev = control_effort

        return control_effort