#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from chungungo_interfaces.msg import BuoysDetected, CloseBuoysCentroids, GoalCentroid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32, String

import numpy as np


class NavChannelRoutine(Node): 
    def __init__(self):
        super().__init__("NavChannelRoutine")

        # -------- Publishers and Subscribers --------
        self.buoys_sub = self.create_subscription(BuoysDetected, "/buoys_detected", self.buoys_cb, 10)  # Recieves a list with maximum 4 segmented buoys sorted from left to right
        self.centroids_sub = self.create_subscription(CloseBuoysCentroids, "/centroids", self.centroids_cb, 10)  # Recieves the two closest obstacles centroids from the LiDAR 
        self.goals_completed_sub = self.create_subscription(Int32, "/goals_completed", self.passed_portals_cb, 10)  # Recieves the number of compĺeted goals
        self.controller_state_sub = self.create_subscription(String, "/controller_state", self.controller_state_cb, 10)  # Recieves the current controller state
        self.operation_mode_sub = self.create_subscription(String, "/operation_mode", self.mode_cb, 10)

        self.goal_pub = self.create_publisher(GoalCentroid, "/goal_centroid", 10)
        self.goal_laserscan_pub = self.create_publisher(LaserScan, "/goal_laserscan", 1)  # Optional, for visualization purposes

        # -------- Atributes --------
        self.routine_active = False
        self.operation_mode = None

        self.buoys_array = [None] * 4  # Array to store the four detected buoys
        self.centroids_array = [None] * 2  # Array to store the two closest centroids}
        
        self.portal = [None] * 2
        self.portal_colors = [None] * 2
        
        self.passed_portals = 0
        self.controller_state = "inactive" # Possible values: inactive or active to indicate if the controller is running or not

        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.goal_centroid_cb)

    def mode_cb(self, msg):
        last_operation_mode = self.operation_mode
        
        self.operation_mode = msg.data
        
        if last_operation_mode != self.operation_mode and self.operation_mode == "nav_channel":
            self.get_logger().info("Nav Channel Routine started.")
            self.routine_active = True

    def controller_state_cb(self, msg):
        if self.routine_active:
            last_controller_state = self.controller_state
            self.controller_state = msg.data

            if last_controller_state == "inactive" and self.controller_state == "active":
                self.get_logger().info("Controller Active.")
            if last_controller_state == "active" and self.controller_state == "inactive":
                self.get_logger().info("Controller Inactive. Waiting for new buoys to detect.")

    def passed_portals_cb(self, msg):
        if self.routine_active:
            self.passed_portals = msg.data

            if self.passed_portals >= 2:
                self.get_logger().info("Two portals passed. ¡Routine completed!")
                self.routine_active = False
                self.destroy_node()

    def buoys_cb(self, buoys_msg):
        if self.routine_active:
            # Buoy_msg contains four Buoy messages, each with: id, color, centroid_x and angle
            self.buoys_array = [None] * 4  # Reset the array to None

            if buoys_msg.buoy_1.id != None:
                self.buoys_array[0] = buoys_msg.buoy_1
            if buoys_msg.buoy_2 != None:
                self.buoys_array[1] = buoys_msg.buoy_2
            if buoys_msg.buoy_3 != None:
                self.buoys_array[2] = buoys_msg.buoy_3
            if buoys_msg.buoy_4 != None:
                self.buoys_array[3] = buoys_msg.buoy_4        

    def centroids_cb(self, msg):
        if self.routine_active:
            if msg.centroid_1.theta > msg.centroid_2.theta:
                self.centroids_array[0] = msg.centroid_1
                self.centroids_array[1] = msg.centroid_2
            else:
                self.centroids_array[0] = msg.centroid_2
                self.centroids_array[1] = msg.centroid_1

    def goal_centroid_cb(self):
        if self.routine_active:
            # self.get_logger().info("Checking for Portal")
            if self.controller_state == "inactive":
                camera_detected_two_buoys = self.buoys_array[0] is not None and self.buoys_array[1] is not None
                lidar_detected_two_centroids = self.centroids_array[0] is not None and self.centroids_array[1] is not None

                if camera_detected_two_buoys and lidar_detected_two_centroids:
                    # self.get_logger().info("Camera and LiDAR detected two buoys.")
                    # If both camera and LiDAR detected two buoys and two centroids, we can proceed to match them
                    self.portal_colors = self.match_buoys_to_centroids()

                    valid_portal = self.check_portal_colors(self.portal_colors)
                    # self.get_logger().info(f"valid_portal = {valid_portal}")

                    if valid_portal:
                        # self.get_logger().info("Valid portal detected.")
                        goal_distance, goal_angle = self.set_goal(self.centroids_array)
                        self.publish_goal(goal_distance, goal_angle)
                        self.publish_goal_laserscan(goal_distance, goal_angle)

    def match_buoys_to_centroids(self):
        # For the each centroid, find the closest buoy (by angle) to that centroid, storing the centroids
        closest_buoy_1 = min(self.buoys_array, key=lambda buoy: abs(buoy.angle - self.centroids_array[0].theta))
        closest_buoy_2 = min(self.buoys_array, key=lambda buoy: abs(buoy.angle - self.centroids_array[1].theta))

        portal_colors = [closest_buoy_1.color, closest_buoy_2.color]

        return portal_colors

    def check_portal_colors(self, portal_colors):
        expected_colors = ["red", "green"]

        return portal_colors == expected_colors

    # def set_goal(self, portal_centroids):
    #     # Calculate the angle to the center of the portal
    #     left_buoy = portal_centroids[0]
    #     right_buoy = portal_centroids[1]

    #     half_to_left_angle_num = -(left_buoy.range**2) + (right_buoy.range**2)
    #     half_to_left_angle_den = np.sqrt(left_buoy.range**4 + right_buoy.range**4 - 2 * (left_buoy.range**2) * (right_buoy.range**2) * np.cos(2*(left_buoy.theta - right_buoy.theta)))
 
    #     half_to_left_angle = np.arccos(half_to_left_angle_num / half_to_left_angle_den)
    #     alpha = left_buoy.theta - right_buoy.theta
    #     half_to_right_angle = alpha - half_to_left_angle

    #     if abs(left_buoy.theta) > abs(right_buoy.theta):
    #         goal_angle = left_buoy.theta - half_to_left_angle
    #     else:
    #         goal_angle = right_buoy.theta - half_to_right_angle

    #     goal_distance = 12.0

    #     self.get_logger().info(f"Valid Portal {self.portal_colors}. Goal set = {goal_distance:.3f} m, {goal_angle:.3f}°.")

    #     return goal_distance, goal_angle
    
    def set_goal(self, portal_centroids):
        # Calculate the angle to the center of the portal
        left_buoy_distance = portal_centroids[0].range
        left_buoy_angle = portal_centroids[0].theta
       
        right_buoy_distance = portal_centroids[1].range
        right_buoy_angle = portal_centroids[1].theta

        goal_distance = (left_buoy_distance + right_buoy_distance) / 2.0
        goal_angle = self.calculate_goal_angle(left_buoy_distance, right_buoy_distance, left_buoy_angle, right_buoy_angle)

        self.get_logger().info(f"Valid Portal {self.portal_colors}. Goal set = {goal_distance:.3f} m, {goal_angle:.3f}°.")

        return goal_distance, goal_angle


    def calculate_goal_angle(self, left_buoy_distance, right_buoy_distance, left_buoy_angle, right_buoy_angle):
        # Convert to radians if in degrees
        left_angle_radians = np.radians(left_buoy_angle)
        right_angle_radians = np.radians(right_buoy_angle)

        # Cartesian coordinates
        x1, y1 = left_buoy_distance * np.cos(left_angle_radians), left_buoy_distance * np.sin(left_angle_radians)
        x2, y2 = right_buoy_distance * np.cos(right_angle_radians), right_buoy_distance * np.sin(right_angle_radians)

        # Midpoint
        xm, ym = (x1 + x2) / 2, (y1 + y2) / 2

        # Angle of midpoint
        angle_rad = np.arctan2(ym, xm)

        return np.degrees(angle_rad)



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
        
        # self.get_logger().info(f"Setting goal distance at index {goal_index} with value {goal_distance:.3f} m")

        if 0 <= goal_index < len(ranges):
            ranges[goal_index] = float(goal_distance)

        msg.ranges = ranges

        self.goal_laserscan_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = NavChannelRoutine()
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()

        