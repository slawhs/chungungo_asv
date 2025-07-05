#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from chungungo_interfaces.srv import VelocityCommand

from serial import Serial
import time

DEVICE_NAME = '/ttyUSB1'
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
        self.get_logger().info(f"[Setpoints] Left: {request.left_velocity} RPM | Right: {request.right_velocity} RPM")
        self.serial.write(vel_cmd.encode("utf-8"))
        response.left_rpm, response.right_rpm = self.get_rpm_feedback()

        return response

    def get_rpm_feedback(self):
        # self.get_logger().info("Getting feedback...")
        self.serial.flushInput()
        rpm_esp = self.serial.read_until(b'\r')
        rpm_feedback = str(rpm_esp, encoding='utf-8')
        rpm_feedback = self.serial.readline().decode('utf-8', errors='replace')
        # self.get_logger().info(rpm_feedback)
        # self.get_logger().info("Feedback recieved")

        left_rpm, right_rpm = rpm_feedback.strip().split("|")
        left_rpm = int(round(float(left_rpm)))
        right_rpm = int(round(float(right_rpm)))

        self.get_logger().info(f"[Feedback] Left: {left_rpm} RPM | Right: {right_rpm} RPM")

        return left_rpm, right_rpm


def main(args=None):
    rclpy.init(args=args)
    node = Thrusters()
    rclpy.spin(node)
    
    node.serial.close()
    rclpy.shutdown()


if __name__ == "__main__":
    main()