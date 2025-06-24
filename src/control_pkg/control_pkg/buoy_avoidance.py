#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Vector3

class BuoyAvoidance(Node):
    def __init__(self):
        super().__init__("BuoyAvoidance")

        # -------- Publishers, Subscribers, Clients and Services --------

        # -------- Atributes --------

        # -------- Setup Routines --------

     

def main(args=None):
    rclpy.init(args=args)
    node = BuoyAvoidance()
    rclpy.spin(node)


if __name__ == "__main__":
    main()