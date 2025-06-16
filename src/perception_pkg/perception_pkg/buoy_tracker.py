#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from chungungo_interfaces.msg import CloseBuoysCentroids 
from std_msgs.msg import String
from sklearn.cluster import DBSCAN, OPTICS

import numpy as np

# ------ LiDAR Parameters ------
TIME_INCREMENT = 0.00011
ANGLE_INCREMENT = 0.008714509196579456  #? radians
ANGLE_MIN = -3.1241390705108643
ANGLE_MAX = 3.1415927410125732
RANGE_MIN = 0.15
RANGE_MAX = 12.0
N_SAMPLES = 720

# ------ Clustering parameters ------
EPS = 0.06  #? Distance (meters) between two points to be considered in the same cluster
CLUSTER_MIN_SAMPLES = 15

MIN_DIST_FILTER = 0.3
MAX_DIST_FILTER = 2.0
    
class BuoyTracker(Node): 
    def __init__(self):
        super().__init__("BuoyTracker")

        # -------- Publishers and Subscribers --------
        self.centroids_pub = self.create_subscription(CloseBuoysCentroids, "/centroids", self.centroids_cb, 1)
        # -------- Atributes --------
        self.centroids_position = None

    def centroids_cb(self, msg):
        centroid_1 = msg.centroid_1
        centroid_2 = msg.centroid_2
        self.get_logger().info(f"RECIEVED CENTROIDS\nCentroid 1: [{centroid_1.range}, {centroid_1.theta}]\nCentroid 2: [{centroid_2.range}, {centroid_2.range}]")
        pass

def main(args=None):
    rclpy.init(args=args)
    node = BuoyTracker()
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()

        