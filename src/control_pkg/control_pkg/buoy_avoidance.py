#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from chungungo_interfaces.msg import CloseBuoysCentroids, ThrustersVelocity
import numpy as np

ANGLE_KP = 1.0
ANGLE_KI = 0.0
ANGLE_KD = 0.0

DIST_KP = 1.0
DIST_KI = 0.0
DIST_KD = 0.0

ANGLE_TH = 1.5(np.pi/180)
BASE_VELOCITY = 50.0

class BuoyAvoidance(Node): 
    def __init__(self):
        super().__init__("BuoyAvoidance")

        # -------- Publishers and Subscribers --------
        self.centroids_sub = self.create_subscription(CloseBuoysCentroids, "/centroids", self.centroids_cb, 1)
        self.buoy_distance_vel_pub = self.create_publisher(ThrustersVelocity, '/buoy_distance_velocity', 10)

        # -------- Atributes --------
        # Detection
        self.buoy_1 = None
        self.buoy_2 = None
        
        timer_period = 0.01 
        self.timer = self.create_timer(timer_period, self.control_cb)

        # PID Controller
        self.Ts = 0.01
        self.error_prev = 0.0 
        self.error_prev_prev = 0.0 
        self.control_prev = 0.0

        self.distance_to_buoy = 0.0
        self.angle_to_buoy = 0.0

        self.linear_vel = 0.0
        self.angular_vel = 0.0


    def centroids_cb(self, msg):
        self.buoy_1 = msg.centroid_1
        self.buoy_2 = msg.centroid_2

        if self.buoy_1 is not None:
            self.distance_to_buoy = self.buoy_1.range
            self.angle_to_buoy = self.buoy_1.theta
            self.get_logger().info(f"Closest buoy is at {self.buoy_1.range:.3f} meters")

    def control_cb(self):
        pass

    def pid(self, setpoint, kp, ki, kd):

        error = setpoint - self.distance_to_buoy 

        K0 = kp + self.Ts * ki + kd / self.Ts
        K1 = -kp - 2 * kd / self.Ts
        K2 = kd / self.Ts

        control_effort = self.control_prev + K0 * error + K1 * self.error_prev + K2 * self.error_prev_prev

        control_effort = np.clip(control_effort, -1050, 1050)

        # Shift states for next iteration
        self.error.prev_prev = self.error_prev
        self.error_prev = error
        self.control_prev = control_effort

        return control_effort

    def publish_velocity(self, linear_velocity, angular_velocity):
        self.ve


def main(args=None):
    rclpy.init(args=args)
    node = BuoyAvoidance()
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()

        