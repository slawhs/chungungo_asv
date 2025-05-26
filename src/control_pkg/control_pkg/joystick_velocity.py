#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Vector3

class JoystickVelocity(Node):
    def __init__(self):
        super().__init__("JoystickVelocity")

        # -------- Publishers, Subscribers, Clients and Services --------
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_cb, 10)
        self.vel_pub = self.create_publisher(Vector3, '/joystick_velocity', 10)

        # -------- Atributes --------
        self.vector3_msg = Vector3()

        # -------- Setup Routines --------

    def joy_cb(self, joy_msg):
        self.vector3_msg.x = joy_msg.axes[1] * 100.0
        self.vector3_msg.y = joy_msg.axes[3] * 100.0
        self.vector3_msg.z = 0.0
        
        # Publish the converted values
        self.vel_pub.publish(self.vector3_msg)

        

def main(args=None):
    rclpy.init(args=args)
    node = JoystickVelocity()
    rclpy.spin(node)
    
    node.serial.close()
    rclpy.shutdown()


if __name__ == "__main__":
    main()