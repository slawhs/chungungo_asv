#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String 
import numpy as np

DIST_KP = 1.0
DIST_KI = 0.0
DIST_KD = 0.0

class ActionProcessing(Node): 
    def __init__(self):
        super().__init__("ActionProcessing")

        # -------- Publishers and Subscribers --------
        self.mode_pub = self.create_publisher(String, "/operation_pub", 1)
        self.mode = "standby"

        # -------- Atributes --------
        timer_period = 0.1 
        self.timer = self.create_timer(timer_period, self.mode_cb)

    def mode_cb(self):
        self.mode = input("introduce mode (standby/joystick/buoy)")
        mode_msg = String()
        mode_msg.data = self.mode
        self.mode_pub.publish(mode_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ActionProcessing()
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()

        