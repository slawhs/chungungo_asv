#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from chungungo_interfaces.msg import BuoysDetected, CloseBuoysCentroids, GoalCentroid
import numpy as np


class NavChannelRoutine(Node): 
    def __init__(self):
        super().__init__("NavChannelRoutine")

        # -------- Parameters --------
        self.parameters_setup()

        # -------- Publishers and Subscribers --------
        self.buoys_sub = self.create_subscription(BuoysDetected, "/buoys_detected", self.buoys_cb, 1)  # Recieves a list with maximum 4 segmented buoys sorted from left to right
        self.centroids_sub = self.create_subscription(CloseBuoysCentroids, "/centroids", self.centroids_cb, 1)  # Recieves the two closest obstacles centroids from the LiDAR   

        self.goal_pub = self.create_publisher(GoalCentroid, "/goal_centroid", 1)

        # -------- Atributes --------
        self.buoys_array = [None] * 4  # Array to store the four detected buoys
        self.centroids_array = [None] * 2  # Array to store the two closest centroids}
        self.portal = [None] * 2
        self.portal_colors = [None] * 2

        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.goal_centroid_cb)

    def parameters_setup(self):
       pass

    def buoys_cb(self, buoys_msg):
        # Buoy_msg contains four Buoy messages, each with: id, color, centroid_x and angle
        self.buoys_array = [None] * 4  # Reset the array to None

        if buoys_msg[0].id != None:
            self.buoys_array[0] = buoys_msg.buoy_1
        if buoys_msg[1] != None:
            self.buoys_array[1] = buoys_msg.buoy_2
        if buoys_msg[2] != None:
            self.buoys_array[2] = buoys_msg.buoy_3
        if buoys_msg[3] != None:
            self.buoys_array[3] = buoys_msg.buoy_4        

    def centroids_cb(self, msg):
        if msg.centroid_1.theta > msg.centroid_2.theta:
            self.centroids_array[0] = msg.centroid_1
            self.centroids_array[1] = msg.centroid_2
        else:
            self.centroids_array[0] = msg.centroid_2
            self.centroids_array[1] = msg.centroid_1

    def goal_centroid_cb(self):
        
        camera_detected_two_buoys = self.buoys_array[0] is not None and self.buoys_array[1] is not None
        lidar_detected_two_centroids = self.centroids_array[0] is not None and self.centroids_array[1] is not None

        if camera_detected_two_buoys and lidar_detected_two_centroids:
            # If both camera and LiDAR detected two buoys and two centroids, we can proceed to match them
            self.portal_colors = self.match_buoys_to_centroids()

            valid_portal = self.check_portal_colors(self.portal)

            if valid_portal:
                goal_distance, goal_angle = self.set_goal(self.centroids_array)
                self.publish_centroids(goal_distance, goal_angle)

        if self.buoys_array[0] is not None and self.buoys_array[1] is not None:
    
            # Calculate the centroid of the two closest buoys
            centroid_x = (self.buoys_array[0].centroid_x + self.buoys_array[1].centroid_x) / 2.0
            centroid_y = (self.buoys_array[0].angle + self.buoys_array[1].angle) / 2.0

            # Create a new CloseBuoysCentroids message
            msg = CloseBuoysCentroids()
            msg.centroid_1.x = centroid_x
            msg.centroid_1.y = centroid_y
            msg.centroid_2.x = 0.0

    def match_buoys_to_centroids(self):
        # For the each centroid, find the closest buoy (by angle) to that centroid, storing the centroids
        closest_buoy_1 = min(self.buoys_array, key=lambda buoy: abs(buoy.angle - self.centroids_array[0].theta))
        closest_buoy_2 = min(self.buoys_array, key=lambda buoy: abs(buoy.angle - self.centroids_array[1].theta))

        portal_colors = [closest_buoy_1.color, closest_buoy_2.color]

        return portal_colors

    def check_portal_colors(self, portal_colors):
        expected_colors = ["red", "green"]

        return portal_colors == expected_colors

    def set_goal(self, portal_centroids):
        # Calculate the angle to the center of the portal
        left_buoy = portal_centroids[0]
        right_buoy = portal_centroids[1]

        half_to_left_angle_num = -(left_buoy.range**2) + (right_buoy.range**2)
        half_to_left_angle_den = np.sqrt(left_buoy.range**4 + right_buoy.range**4 - 2 * (left_buoy.range**2) * (right_buoy.range**2) * np.cos(2*(left_buoy.theta - right_buoy.theta)))
 
        half_to_left_angle = np.arccos(half_to_left_angle_num / half_to_left_angle_den)
        alpha = left_buoy.theta - right_buoy.theta
        half_to_right_angle = alpha - half_to_left_angle

        if abs(left_buoy.angle) > abs(right_buoy.angle):
            goal_angle = left_buoy.theta - half_to_left_angle
        else:
            goal_angle = right_buoy.theta - half_to_right_angle

        goal_distance = 12.0

        return goal_distance, goal_angle

    def publish_centroids(self, goal_distance, goal_angle):
        msg = GoalCentroid()
        msg.goal_centroid.range = goal_distance
        msg.goal_centroid.theta = goal_angle

        self.goal_pub.publish(msg)
        self.get_logger().info(f"Goal Centroid: {msg.centroid_1.x:.3f}, {msg.centroid_1.y:.3f}")


def main(args=None):
    rclpy.init(args=args)
    node = NavChannelRoutine()
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()

        