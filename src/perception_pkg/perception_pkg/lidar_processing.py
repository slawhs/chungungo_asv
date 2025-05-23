#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from sklearn.cluster import KMeans

import numpy as np


TIME_INCREMENT = 0.00011
ANGLE_INCREMENT = 0.005806980188935995  #? radians
ANGLE_MIN = -3.1241390705108643
ANGLE_MAX = 3.1415927410125732
RANGE_MIN = 0.15
RANGE_MAX = 12

    
    
class Lidar(Node): 
    def __init__(self):
        super().__init__("Lidar")

        # -------- Publishers and Subscribers --------
        self.laser_sub = self.create_subscription(LaserScan, "/scan", self.lidar_scan_cb, 1)
        self.laser_pub = self.create_publisher(String, "/object_ahead", 1)
        self.front_laser_pub = self.create_publisher(LaserScan, "/front_scan", 1)
        self.centroids_pub = self.create_publisher(LaserScan, "/centroids", 1)
        
        # -------- Atributes --------
        self.laser_ranges = None
        self.polar_samples = None

    def lidar_scan_cb(self, msg: LaserScan):  #? Jorge: 720 muestras - Nosotros 1080 muestras
        self.front_ranges = msg.ranges[1035:] + msg.ranges[:45]  #? 30°
        self.detect_buoys(msg)
        self.publish_front_scan(msg)
        if min(self.front_ranges) < 0.8:
            # self.get_logger().info(f"Object Ahead!")
            string_msg = String()
            string_msg.data = "Object Ahead!"
            self.laser_pub.publish(string_msg)

    def publish_front_scan(self, msg):
        msg.ranges = self.front_ranges
        self.front_laser_pub.publish(msg)

    def detect_buoys(self, msg):
        self.samples_to_polar(msg)
        np.nan_to_num(self.polar_samples, copy=False, posinf=13.0)
        kmeans = KMeans(n_clusters=3, random_state=0).fit(self.polar_samples)
        self.publish_centroids(kmeans.cluster_centers_)

    def samples_to_polar(self, msg):
        angle_left = ANGLE_MAX*2*3/4
        ranges_left = msg.ranges[810:]
        polar_left = []

        for r in ranges_left:
            angle_left += ANGLE_INCREMENT
            if angle_left <= ANGLE_MAX*2:
                polar_left.append(np.array([r, angle_left]))
            else:
                polar_left.append(np.array([r, ANGLE_MAX*2 - 1e-18]))

        angle_right = 0
        ranges_right = msg.ranges[:270]
        polar_right = []
        
        for r in ranges_right:
            angle_right += ANGLE_INCREMENT
            if angle_right <= ANGLE_MAX/2:
                polar_right.append(np.array([r, angle_right]))
            else:
                polar_right.append(np.array([r, (ANGLE_MAX/2) - 1e-18]))

        self.polar_samples = np.array(polar_left + polar_right)

        # print("--------------------------------- NEW ENTRY ---------------------------------")
        # print(self.polar_samples,"\n")
    
    def publish_centroids(self, cluster_centers):
        msg = self.centroid_to_laserscan(cluster_centers)
        self.centroids_pub.publish(msg)
        

    def centroid_to_laserscan(self, cluster_centers):
        msg = LaserScan()
        msg.angle_min = ANGLE_MIN
        msg.angle_max = ANGLE_MAX
        msg.angle_increment = ANGLE_INCREMENT
        msg.range_min = 0.15
        msg.range_max = 12.0
        msg._header._frame_id= "laser"

        ranges = [0.0] * 1080

        # indexes = []

        for centroid in cluster_centers:
            rng = centroid[0]
            ang = centroid[1]
            index = int(round(ang / ANGLE_INCREMENT))
            # indexes.append(index)
            if index >= 1080:
                index = 1080-1

            ranges[index] = rng

        # print(f"indexes = {indexes}")
        msg.ranges = ranges

        return msg

def main(args=None):
    rclpy.init(args=args)
    node = Lidar()
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()