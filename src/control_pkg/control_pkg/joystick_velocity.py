#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Vector3

from chungungo_interfaces import ThrustersVelocity

class JoystickVelocity(Node):
    def __init__(self):
        super().__init__("JoystickVelocity")

        # -------- Publishers, Subscribers, Clients and Services --------
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_cb, 10)
        self.vel_pub = self.create_publisher(ThrustersVelocity, '/joystick_velocity', 10)

        # -------- Atributes --------
        self.vel_msg = ThrustersVelocity()

        # -------- Setup Routines --------

    def joy_cb(self, joy_msg):
        self.vel_msg.x = joy_msg.axes[1] * 1050.0
        self.vel_msg.y = joy_msg.axes[3] * 1050.0
        
        # Publish the converted values
        self.vel_pub.publish(self.vel_msg)

        
def main(args=None):
    rclpy.init(args=args)
    node = JoystickVelocity()
    rclpy.spin(node)
    
    node.serial.close()
    rclpy.shutdown()


if __name__ == "__main__":
    main()