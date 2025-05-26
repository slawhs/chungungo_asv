#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from serial import Serial

DEVICE_NAME = '/dev/ttyUSB0'


class SerialThrusterController(Node):
    def __init__(self):
        super().__init__("SerialThrusterController")

        # -------- Publishers and Subscribers --------
        # self.left_motor_sub(String,)
        # -------- Atributes --------
        self.timer = self.create_timer(0.2, self.timer_cb)

        # -------- Setup Routines --------
        self.setup_serial()

    def setup_serial(self):
        
        self.serial = Serial(DEVICE_NAME, baudrate=115200)
        if not self.serial.isOpen():
            self.get_logger().error('Serial port is closed')

    def timer_cb(self):
        vel1, dir1 = input("ingrese vel1,dir1: ").strip().split(",")
        vel2, dir2 = input("ingrese vel2,dir2: ").strip().split(",")

        rpm_msg = f"{vel1},{dir1},{vel2},{dir2}"
        self.serial.write(rpm_msg.encode('utf-8'))

 
        rpm_feedback = self.serial.readline()
        rpm_feedback = rpm_feedback.decode('utf-8')

        rpm_motor1, rpm_motor2 = rpm_feedback.strip().split(" | ")

        rpm_motor1 = float(rpm_motor1)
        rpm_motor2 = float(rpm_motor2)

        print(f"Motor 1 = {rpm_motor1} rpm")
        print(f"Motor 2 = {rpm_motor2} rpm")


def main(args=None):
    rclpy.init(args=args)
    node = SerialThrusterController()
    rclpy.spin(node)
    
    node.serial.close()
    rclpy.shutdown()


if __name__ == "__main__":
    main()