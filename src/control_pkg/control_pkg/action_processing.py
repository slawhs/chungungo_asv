#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from std_msgs.msg import String 

import numpy as np

DIST_KP = 1.0
DIST_KI = 0.0
DIST_KD = 0.0

class ActionProcessing(Node): 
    def __init__(self):
        super().__init__("ActionProcessing")

        # -------- Publishers and Subscribers --------
        self.mode_pub = self.create_publisher(String, "/operation_mode", 1)
        self.joystick_sub = self.create_subscription(Joy, "/joy", self.joystick_mode_cb, 10)
        self.mode = "standby"

        # -------- Atributes --------
        # timer_period = 0.1 
        # self.timer = self.create_timer(timer_period, self.mode_cb)
        self.last_joy_msg = None

    def joystick_mode_cb(self, joy_msg):

        if joy_msg.buttons[4] == 1 and joy_msg.buttons[0] == 1:  # L1 + X
            self.mode = "standby"
        elif joy_msg.buttons[4] == 1 and joy_msg.buttons[3] == 1:  # L1 + Y
            self.mode = "joystick"
        elif joy_msg.buttons[4] == 1 and joy_msg.buttons[2] == 1:  # L1 + B
            self.mode = "buoy"

        if self.last_joy_msg != joy_msg:
            mode_msg = String()
            mode_msg.data = self.mode
            self.mode_pub.publish(mode_msg)
            self.get_logger().info(f"Sent Mode: {self.mode}")
        
        self.last_joy_msg = joy_msg

    def mode_cb(self):
        self.mode = input("Introduce Operation Mode (standby/joystick/buoy): ")
        mode_msg = String()
        mode_msg.data = self.mode
        self.get_logger().info(f"Sent Mode: {self.mode}")
        self.mode_pub.publish(mode_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ActionProcessing()
    rclpy.spin(node)
    rclpy.shutdown()
    final_msg = String()
    final_msg.data = "standby"
    node.mode_pub.publish(final_msg)
    
    
if __name__ == "__main__":
    main()

        