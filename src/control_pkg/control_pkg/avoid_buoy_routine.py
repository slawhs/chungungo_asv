#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from chungungo_interfaces.msg import BuoysDetected, CloseBuoysCentroids, GoalCentroid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int8, String

import numpy as np


class AvoidBuoyRoutine(Node): 
    def __init__(self):
        super().__init__("AvoidBuoyRoutine")

        # -------- Parameters --------
        self.parameters_setup()

        # -------- Publishers and Subscribers --------
        self.interfaces_setup()

        # -------- Atributes --------
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.avoid_buoy_cb)

        self.closest_buoy = None
        self.controller_state = "inactive"  # Possible values: inactive or active to indicate if the controller is running or not
        self.current_avoid_step = 0  # Indicator of current step in the avoidance routine

    def parameters_setup(self):
        self.declare_parameter("safety_distance", 0.3)
        self.safety_distance = self.get_parameter("safety_distance").get_parameter_value().value

    def interfaces_setup(self):
        self.centroids_sub = self.create_subscription(CloseBuoysCentroids, "/centroids", self.centroids_cb, 10)  # Recieves the two closest obstacles centroids from the LiDAR 
        self.controller_state_sub = self.create_subscription(String, "/controller_state", self.controller_state_cb, 10)  # Recieves the current controller state
        self.goals_completed_sub = self.create_subscription(Int8, "/goals_completed", self.avoid_steps_cb, 1)  # Recieves the number of steps during the avoidance routine
        self.operation_mode_sub = self.create_subscription(String, "/operation_mode", self.mode_cb, 10)

        self.goal_pub = self.create_publisher(GoalCentroid, "/goal_centroid", 10)

    def avoid_steps_cb(self, msg):    
        self.current_avoid_step = msg.data

        if self.current_avoid_step == 1:  # First step: going backwards
            self.get_logger().info("Completed Step 1: Going backwards.")
            self.get_logger().info("Starting Step 2: Turning 30° away from the buoy.")
        elif self.current_avoid_step == 2:
            self.get_logger().info("Completed Step 2: Turning 30° away from the buoy.")
            self.get_logger().info("¡Buoy Avoided!")
            self.closest_buoy = None
            
    def mode_cb(self, msg):
        self.operation_mode = msg.data

    def controller_state_cb(self, msg):
        last_controller_state = self.controller_state
        self.controller_state = msg.data

        if last_controller_state == "inactive" and self.controller_state == "active":
            self.get_logger().info("Controller Active.")
        if last_controller_state == "active" and self.controller_state == "inactive":
            self.get_logger().info("Controller Inactive. Waiting for new buoys to detect.")

    def centroids_cb(self, msg):
        if msg.centroid_1 is not None:
            self.closest_buoy = msg.centroid_1

    def avoid_buoy_cb(self):
        if self.closest_buoy is not None:
            if self.closest_buoy.range < self.safety_distance:
                if self.current_avoid_step == 0 and self.controller_state == "inactive":
                    self.get_logger().info(f"Buoy too close. Avoiding...")
                    self.get_logger().info(f"Starting Step 1: Going backwards.")
                    # Set a goal centroid to avoid the buoy
                    goal_distance = self.closest_buoy.range + 0.5
                    goal_angle = self.closest_buoy.theta
                    self.publish_goal(goal_distance, goal_angle)
                    self.publish_goal_laserscan(goal_distance, goal_angle)
                
            if self.current_avoid_step == 1 and self.controller_state== "inactive":
                goal_distance = self.closest_buoy.range

                if self.closest_buoy.theta > 0:  # if closest buoy is on the left side
                    goal_angle = self.closest_buoy.theta - 30  # Turn right 30° from the buoy
                else:  # if closest buoy is on the right side
                    goal_angle = self.closest_buoy.theta + 30  # Turn left 30° from the buoy
                
                self.publish_goal(goal_distance, goal_angle)
                self.publish_goal_laserscan(goal_distance, goal_angle)

  

    def publish_goal(self, goal_distance, goal_angle):
        msg = GoalCentroid()
        msg.goal_centroid.range = goal_distance
        msg.goal_centroid.theta = goal_angle

        self.goal_pub.publish(msg)
        # self.get_logger().info(f"Goal Centroid: {msg.goal_centroid:.3f}, {msg.centroid_1.y:.3f}")

    def publish_goal_laserscan(self, goal_distance, goal_angle):
        msg = LaserScan()
        msg.header.frame_id = "laser"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.angle_min = -3.1241390705108643
        msg.angle_max = 3.1415927410125732
        msg.angle_increment = 0.008714509196579456  #? radians
        msg.range_min = 0.15
        msg.range_max = 12.0

        # Create a range array with the goal distance at the goal angle
        ranges = [float('inf')] * 720

        goal_angle = goal_angle if goal_angle >= 0 else goal_angle + 2*np.pi
        goal_angle = np.radians(goal_angle)  # Convert goal angle to radians

        goal_index = int((goal_angle)/msg.angle_increment)
        
        self.get_logger().info(f"Setting goal distance at index {goal_index} with value {goal_distance:.3f} m")

        if 0 <= goal_index < len(ranges):
            ranges[goal_index] = float(goal_distance)

        msg.ranges = ranges

        self.goal_laserscan_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AvoidBuoyRoutine()
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()

        