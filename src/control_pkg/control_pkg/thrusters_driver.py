#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from chungungo_interfaces.srv import VelocityCommand

from serial import Serial

class Thrusters(Node):
    def __init__(self):
        super().__init__("Thrusters")

        self.parameters_setup()

        # -------- Publishers, Subscribers Clients and Services --------
        self.vel_cmd_srv = self.create_service(VelocityCommand, '/vel_command', self.vel_command_cb)
        self.mode_sub = self.create_subscription(String, "/operation_mode", self.mode_cb, 10)


        # -------- Atributes --------
        self.serial = None
        self.operation_mode = "standby"

        # -------- Setup Routines --------
        self.serial_setup()
        self.get_logger().info("Setup compelted")

    def parameters_setup(self):
        self.declare_parameter("device_name", 'ttyUSB1')
        self.declare_parameter("baud_rate", 115200)

        self.device_name = self.get_parameter("device_name").value
        self.baud_rate = self.get_parameter("baud_rate").value

    def serial_setup(self):
        self.serial = Serial('/dev/' + self.device_name, self.baud_rate)
        if not self.serial.isOpen():
            self.get_logger().warning('Serial port is not open yet')
        self.get_logger().info('Serial port connected')

    def mode_cb(self, mode_msg):
        self.operation_mode = mode_msg.data

    def vel_command_cb(self, request, response):
        vel_cmd = f"{request.left_velocity},{request.right_velocity}\n"
        if self.operation_mode != "standby":
            self.get_logger().info(f"[Setpoints] Left: {request.left_velocity} RPM | Right: {request.right_velocity} RPM")
        self.serial.write(vel_cmd.encode("utf-8"))
        # response.left_rpm, response.right_rpm = self.get_rpm_feedback()

        return response

    def get_rpm_feedback(self):
        # self.get_logger().info("Getting feedback...")
        self.serial.flushInput()
        rpm_esp = self.serial.read_until(b'\r')
        rpm_feedback = str(rpm_esp, encoding='utf-8')
        # rpm_feedback = self.serial.readline().decode('utf-8', errors='replace')
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