#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from chungungo_interfaces.msg import ThrustersVelocity
from tf_transformations import euler_from_quaternion
import numpy as np


ANGLE_KP = 1.0
ANGLE_KI = 0.0
ANGLE_KD = 0.0

VEL_KP = 1.0
VEL_KI = 0.0
VEL_KD = 0.0

class PIDError:
    def __init__(self, kp, ki, kd, lim_top, lim_low):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.error_prev = 0.0
        self.error_prev_prev = 0.0
        self.control_prev = 0.0
        self.Ts = 0.01  # Sample time
        self.lim_top = lim_top # 2.0
        self.lim_low = lim_low # -2.0


    def compute_control(self, setpoint, current_value):

        error = setpoint - current_value # Assuming z is the yaw angle 

        K0 = self.kp + self.Ts * self.ki + self.kd / self.Ts
        K1 = -self.kp - 2 * self.kd / self.Ts
        K2 = self.kd / self.Ts

        control_effort = self.control_prev + K0 * error + K1 * self.error_prev + K2 * self.error_prev_prev

        control_effort = np.clip(control_effort, self.lim_low, self.lim_top)

        # Shift states for next iteration
        self.error_prev_prev = self.error_prev
        self.error_prev = error
        self.control_prev = control_effort

        return control_effort

class IMUctrl(Node):
    def __init__(self):
        super().__init__("IMUctrl")

        # -------- Publishers and Subscribers --------
        self.imu_sub = self.create_subscription(Imu, "/imu/data", self.imu_cb, 1)
        self.vel_pub = self.create_publisher(ThrustersVelocity, '/imu_velocity', 10)

        # -------- Atributes --------
        self.vel_msg = ThrustersVelocity()
        
        timer_period = 0.01 
        self.timer = self.create_timer(timer_period, self.control_cb)

        # PID Controller
        self.Ts = 0.01
        self.pid_vel = PIDError(VEL_KP, VEL_KI, VEL_KD, -2.0, 2.0)
        self.pid_angle = PIDError(ANGLE_KP, ANGLE_KI, ANGLE_KD, -2.0, 2.0)

        self.desired_vel = 0.0
        self.desired_angle = 0.0

        self.linear_vel = 0.0
        self.angular_vel = 0.0

        # Imu data
        self.orientation = Vector3()
        self.linear_acceleration = Vector3()

    def imu_cb(self, imu_msg):
        orientation_list = [imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.orientation.x = roll
        self.orientation.y = pitch
        self.orientation.z = yaw
        self.linear_acceleration = imu_msg.linear_acceleration


    def por_nombrar(self):
        # Controlador de velocidad
        pass

    def angle_controler(self):
        # Controlador de angulo
        
        pass

def main(args=None):
    rclpy.init(args=args)
    node = IMUctrl()
    rclpy.spin(node)
    
    node.serial.close()
    rclpy.shutdown()


if __name__ == "__main__":
    main()