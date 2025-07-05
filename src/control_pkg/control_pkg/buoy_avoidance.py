#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from chungungo_interfaces.msg import CloseBuoysCentroids, ThrustersVelocity
from control_pkg.pid_controller import PIDController
import numpy as np

ANGLE_KP = 10.0
ANGLE_KI = 0.0
ANGLE_KD = 0.0

DIST_KP = 1.0
DIST_KI = 0.0
DIST_KD = 0.0

SATURATION = 1050

ANGLE_TH = 1.5*(np.pi/180)
BASE_VELOCITY = 0.0

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
        self.angle_controller = PIDController(Kp=ANGLE_KP, Ki=ANGLE_KI, Kd=ANGLE_KD, saturation=SATURATION, Ts=0.01)

        self.distance_to_buoy = 0.0
        self.angle_to_buoy = 0.0

        self.linear_vel = 0.0
        self.angular_vel = 0.0

        self.angle_control = False


    def centroids_cb(self, msg):
        self.buoy_1 = msg.centroid_1 
        self.buoy_2 = msg.centroid_2

        invalid_buoy_1 = (np.isnan(self.buoy_1.range).any()) or (np.isnan(self.buoy_1.theta).any())

        if (self.buoy_1 is None) or invalid_buoy_1:
            self.angle_control = False
            self.distance_to_buoy = 0.0
            self.angle_to_buoy = 0.0

        else:
            self.angle_control = True
            self.distance_to_buoy = self.buoy_1.range
            self.angle_to_buoy = self.buoy_1.theta
            # self.get_logger().info(f"Closest buoy is at {self.buoy_1.range:.3f} meters with angle {self.buoy_1.theta:.3f}")

    def control_cb(self):
        if self.angle_control:
            diff_velocity = self.angle_controller.pid(setpoint=0, feedback=self.angle_to_buoy)
            self.publish_velocity(linear_velocity=BASE_VELOCITY, diff_velocity=diff_velocity)
        else:
            self.publish_velocity(linear_velocity=0, diff_velocity=0)
    

    def publish_velocity(self, linear_velocity, diff_velocity):
        left_velocity = linear_velocity + diff_velocity
        right_velocity = linear_velocity - diff_velocity

        self.get_logger().info(f"Diff Velocity is: L = {left_velocity:.3f} | R = {right_velocity:.3f}")

        msg = ThrustersVelocity()
        msg.left_velocity = float(left_velocity)
        msg.right_velocity = float(right_velocity)
        self.buoy_distance_vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BuoyAvoidance()
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()

        