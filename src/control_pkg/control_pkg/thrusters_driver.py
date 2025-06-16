#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from chungungo_interfaces.srv import VelocityCommand

from serial import Serial
import time

DEVICE_NAME = '/ttyCH341USB0'
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
        vel_cmd = f"{request.left_velocity},{request.right_velocity}\n"
        self.get_logger().info(f"""
                               ==================== VELOCITY COMMAND RECIEVED ====================
                               Left setpoint:{request.left_velocity} rpm
                               Right setpoint:{request.right_velocity} rpm
                               """)
        # self.get_logger().info(f"Left setpoint:{request.left_velocity} rpm")
        # self.get_logger().info(f"Right setpoint:{request.right_velocity} rpm")

        self.serial.write(vel_cmd.encode("utf-8"))
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