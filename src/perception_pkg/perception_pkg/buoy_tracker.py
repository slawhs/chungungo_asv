#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from chungungo_interfaces.msg import CloseBuoysCentroids, BuoysDetected 
from std_msgs.msg import String

BUOY_RADIUS = 0.15

class BuoyTracker(Node): 
    def __init__(self):
        super().__init__("BuoyTracker")

        # -------- Publishers and Subscribers --------
        self.detect_pub = self.create_publisher(Bool, "/detect_order", 10)

        self.centroids_sub = self.create_subscription(CloseBuoysCentroids, "/centroids", self.centroids_cb, 1)
        self.color_sub = self.create_subscription(String, "/color_detection", self.color_cb, 10)
        self.buoys_sub = self.create_subscription(BuoysDetected, "/buoys_detected", self.buoys_cb, 10)

        # -------- Atributes --------
        self.centroid_1 = None
        self.centroid_2 = None

        self.centroid_1_distance = 12.0
        self.centroid_2_distance = 12.0
        
        timer_period = 0.1 
        self.timer = self.create_timer(timer_period, self.detect_cb)

        self.detected_color = ""

        self.detected_buoys = [None, None, None, None]  # List to hold detected buoys

    def color_cb(self, msg):
        self.detected_color = msg.data 
        self.get_logger().info(f"{self.detected_color} buoy at {self.centroid_1.range:.3f} meters")

    def centroids_cb(self, msg):
        # self.get_logger().info(f"updated position of centroid. Range: {msg.centroid_1.range}")
        self.centroid_1 = msg.centroid_1
        self.centroid_2 = msg.centroid_2

    def detect_cb(self):
        if self.centroid_1 is not None:
            self.send_detect_order(True)

        # if self.centroid_2 is not None:    
        #     self.get_logger().info(f"Second closest buoy is at {self.centroid_2.range:.3f} meters")

    def send_detect_order(self, order):
        detect_msg = Bool()
        detect_msg.data = order
        self.detect_pub.publish(detect_msg)

    def buoys_cb(self, msg: BuoysDetected):
        self.get_logger().info(f"Buoys detected: {msg}")

        for i, buoy in enumerate([msg.buoy_1, msg.buoy_2, msg.buoy_3, msg.buoy_4]):
            if buoy.color == "None":
                self.detected_buoys[i] = buoy
            else:
                self.detected_buoys[i] = None

def main(args=None):
    rclpy.init(args=args)
    node = BuoyTracker()
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()

        