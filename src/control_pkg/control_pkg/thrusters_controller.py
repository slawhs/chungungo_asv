#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from chungungo_interfaces.srv import VelocityCommand

from serial import Serial
import time

DEVICE_NAME = '/ttyUSB0'
BAUD_RATE = 115200

class Thrusters(Node):
    def __init__(self):
        super().__init__("Thrusters")

        # -------- Publishers, Subscribers Clients and Services --------
        self.vel_cmd_srv = self.create_service(VelocityCommand, '/vel_command', self.vel_command_cb)

        # -------- Atributes --------
        self.serial = None

        # -------- Setup Routines --------
        self.serial_setup()
        self.get_logger().info("Setup compelted")

    def serial_setup(self):
        self.serial = Serial('/dev' + DEVICE_NAME, BAUD_RATE)
        if not self.serial.isOpen():
            self.get_logger().warning('Serial port is not open yet')
        self.get_logger().info('Serial port connected')

    def vel_command_cb(self, request, response):
        vel_cmd = f"{request.left_speed},{request.left_direction},{request.right_speed},{request.right_direction}\n"
        self.get_logger().info("==================== VELOCITY COMMAND RECIEVED ====================")
        self.get_logger().info(f"Left:{request.left_speed} rpm headed to direction {request.left_direction}")
        self.get_logger().info(f"Right:{request.right_speed} rpm headed to direction {request.right_direction}")

        self.serial.write(vel_cmd.encode("utf-8"))
        # self.serial.readline()

        # response.left_rpm = 0
        # response.right_rpm = 0

        # return response
        
        response.left_rpm, response.right_rpm = self.get_rpm_feedback()

        return response

    def get_rpm_feedback(self):
        self.get_logger().info("Getting feedback...")
        rpm_feedback = self.serial.readline().decode('ascii', errors='replace')
        self.get_logger().info("Feedback recieved")

        left_rpm, right_rpm = rpm_feedback.strip().split("|")
        left_rpm = int(round(float(left_rpm)))
        right_rpm = int(round(float(right_rpm)))

        self.get_logger().info(f"Left: {left_rpm} RPM | Right: {right_rpm} RPM")

        return left_rpm, right_rpm


def main(args=None):
    rclpy.init(args=args)
    node = Thrusters()
    rclpy.spin(node)
    
    node.serial.close()
    rclpy.shutdown()


if __name__ == "__main__":
    main()