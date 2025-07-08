#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from chungungo_interfaces.msg import GoalCentroid, ThrustersVelocity
from control_pkg.pid_controller import PIDController
import numpy as np


class BuoyAvoidance(Node): 
    def __init__(self):
        super().__init__("BuoyAvoidance")

        # -------- Parameters --------
        self.parameters_setup()

        # -------- Publishers and Subscribers --------
        self.centroids_sub = self.create_subscription(GoalCentroid, "/goal_centroid", self.centroids_cb, 1)
        self.operation_mode_sub = self.create_subscription(String, "/operation_mode", self.mode_cb, 10)
        self.buoy_distance_vel_pub = self.create_publisher(ThrustersVelocity, '/buoy_distance_velocity', 10)

        # -------- Atributes --------
        # Detection
        self.goal_buoy = None
        
        timer_period = 0.01 
        self.timer = self.create_timer(timer_period, self.control_cb)

        # PID Controller
        self.angle_controller = PIDController(Kp=self.angle_kp, Ki=self.angle_ki, Kd=self.angle_kd, min_saturation=100, max_saturation=300, Ts=0.01)
        self.distance_controller = PIDController(Kp=self.dist_kp, Ki=self.dist_ki, Kd=self.dist_ki, min_saturation=100, max_saturation=1050, Ts=0.01)

        self.distance_to_buoy = 0.0
        self.angle_to_buoy = 0.0

        self.linear_vel = 0.0
        self.angular_vel = 0.0

        self.angular_setpoint = 90
        self.linear_setpoint = 1.0

        self.valid_control = False
        self.angle_control = False
        self.distance_control = False
        self.operation_mode = None

    def parameters_setup(self):
        self.declare_parameter("angle_kp", 5.0)
        self.declare_parameter("angle_ki", 1.0)
        self.declare_parameter("angle_kd", 0.0)

        self.declare_parameter("dist_kp", 25.0)
        self.declare_parameter("dist_ki", 1.0)
        self.declare_parameter("dist_kd", 0.0)

        self.declare_parameter("angle_th", 7)
        self.declare_parameter("dist_th", 0.3)
        self.declare_parameter("base_velocity", 0.0)

        # Load parameters
        self.angle_kp = self.get_parameter("angle_kp").value
        self.angle_ki = self.get_parameter("angle_ki").value
        self.angle_kd = self.get_parameter("angle_kd").value

        self.dist_kp = self.get_parameter("dist_kp").value
        self.dist_ki = self.get_parameter("dist_ki").value
        self.dist_kd = self.get_parameter("dist_kd").value

        self.angle_th = self.get_parameter("angle_th").value
        self.dist_th = self.get_parameter("dist_th").value

        self.base_velocity = self.get_parameter("base_velocity").value

    def mode_cb(self, mode_msg):
        self.operation_mode = mode_msg.data

        if self.operation_mode == "buoy" or "nav_channel":
            self.reset_errors(self.angle_controller)
            self.reset_errors(self.distance_controller)

    def centroids_cb(self, msg):
        self.goal_buoy = msg.centroid_1 

        invalid_goal_buoy = (np.isnan(self.goal_buoy.range).any()) or (np.isnan(self.goal_buoy.theta).any())

        if (self.goal_buoy is None) or invalid_goal_buoy:
            self.valid_control = False
            self.distance_to_buoy = 0.0
            self.angle_to_buoy = 0.0

        else:
            self.valid_control = True
            
            self.distance_to_buoy = self.goal_buoy.range
            self.angle_to_buoy = self.goal_buoy.theta

            if np.abs(self.angle_to_buoy) > self.angle_th:
                self.angle_control = True
            else:
                self.angle_control = False
                self.reset_errors(self.angle_controller)

            if np.abs(self.distance_to_buoy) > self.dist_th:
                self.distance_control = True
            else:
                self.distance_control = False
                self.reset_errors(self.distance_controller)


            # self.get_logger().info(f"Closest buoy is at {self.buoy_1.range:.3f} meters with angle {self.buoy_1.theta:.3f}")

    def control_cb(self):
        
        if self.valid_control:
            if self.angle_control:
                angular_velocity = self.angle_controller.pid(setpoint=self.angular_setpoint, feedback=self.angle_to_buoy)
                linear_velocity = 0.0
            
            elif not self.angle_control:
                if self.distance_control:
                    angular_velocity = 0.0
                    linear_velocity = -1 * self.distance_controller.pid(setpoint=self.linear_setpoint, feedback=self.distance_to_buoy)
                elif not self.distance_control:
                    angular_velocity = 0.0
                    linear_velocity = 0.0

            self.publish_velocity(linear_velocity=linear_velocity, angular_velocity=angular_velocity)

        else:
            self.publish_velocity(linear_velocity=0, angular_velocity=0)

    def reset_errors(self, controller):
        controller.control_prev = 0.0
        controller.error_prev = 0.0
        controller.error_prev_prev = 0.0
    

    def publish_velocity(self, linear_velocity, angular_velocity):
        left_velocity = linear_velocity + angular_velocity
        right_velocity = linear_velocity - angular_velocity

        self.get_logger().info(f"Linear Vel = {linear_velocity:.3f} | Angular Vel = {angular_velocity:.3f}\nL_Vel = {left_velocity:.3f} | R_Vel = {right_velocity:.3f}")

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

        