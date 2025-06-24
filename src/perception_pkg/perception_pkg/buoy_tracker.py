#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from chungungo_interfaces.msg import CloseBuoysCentroids 
from std_msgs.msg import String
from sklearn.cluster import DBSCAN, OPTICS

import numpy as np

BUOY_RADIUS = 0.15

class BuoyTracker(Node): 
    def __init__(self):
        super().__init__("BuoyTracker")

        # -------- Publishers and Subscribers --------
        self.centroids_pub = self.create_subscription(CloseBuoysCentroids, "/centroids", self.centroids_cb, 1)
        # -------- Atributes --------
        self.centroid_1 = None
        self.centroid_2 = None

        self.centroid_1_distance = 12.0
        self.centroid_2_distance = 12.0
        
        timer_period = 0.1 
        self.timer = self.create_timer(timer_period, self.detect_cb)

    def centroids_cb(self, msg):
        self.centroid_1 = msg.centroid_1
        self.centroid_2 = msg.centroid_2

    def detect_cb(self):
        if self.centroid_1 is not None:
            self.get_logger().info(f"Closest buoy is at {self.centroid_1.range:.3f} meters")
        if self.centroid_2 is not None:    
            self.get_logger().info(f"Second closest buoy is at {self.centroid_2.range:.3f} meters")


def main(args=None):
    rclpy.init(args=args)
    node = BuoyTracker()
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()

        