#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from chungungo_interfaces import VelocityCommand

from serial import Serial

DEVICE_NAME = '/ttyUSB0'
BAUD_RATE = 115200

class Thrusters(Node):
    def __init__(self):
        super().__init__("Thrusters")

        # -------- Publishers, Subscribers and Services --------
        self.vel_cmd_srv = self.create_service(VelocityCommand, '/vel_command', self.vel_command_cb)

        # -------- Atributes --------
        self.serial = None

        # -------- Setup Routines --------
        self.serial_setup()

    def serial_setup(self):
        self.serial = Serial('/dev' + DEVICE_NAME, BAUD_RATE)
        if not self.serial.isOpen():
            self.get_logger().warning('Serial port is not open yet')

    def vel_command_cb(self, request, response):
        vel_cmd = f"{request.left_speed},{request.left_direction},{request.right_speed},{request.right_direction}"
        
        self.serial.write(vel_cmd.encode('utf-8'))
        
        response.left_rpm, response.right_rpm = self.get_rpm_feedback()

        return response

    def get_rpm_feedback(self):
        rpm_feedback = self.serial.readline()

        left_rpm, right_rpm = rpm_feedback.strip().split()
        left_rpm = int(left_rpm)
        right_rpm = int(right_rpm)

        return left_rpm, right_rpm


def main(args=None):
    rclpy.init(args=args)
    node = Thrusters()
    rclpy.spin(node)
    
    node.serial.close()
    rclpy.shutdown()


if __name__ == "__main__":
    main()