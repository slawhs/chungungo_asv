#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from chungungo_interfaces import VelocityCommand


class ThrustersVelMux(Node):
    def __init__(self):
        super().__init__("ThrustersVelocityMux")

        # -------- Publishers, Subscribers and Services --------
        #* crear subscriber de nodos de teleop, control de boyas, control de distancias. 
        self.vel_cmd_cli = self.create_client(VelocityCommand, '/vel_command')
        while not self.vel_cmd_cli.wait_for_service(timeout_sec=0.5):
            self.get_logger().info('Service not available, waiting again...')

        # -------- Atributes --------
        timer_period = 0.1  # Segundos
        self.vel_mux_timer = self.create_timer(timer_period, self.vel_mux_cb)
        self.request = VelocityCommand.Request()

        # -------- Setup Routines --------

    def vel_mux_cb(self):
        self.request.left_speed, self.request.left_direction = input("Ingrese vel1,dir1").strip().split(",")
        self.request.right_speed, self.request.right_direction = input("Ingrese vel2,dir2").strip().split(",")
        self.future = self.vel_cmd_cli.call_async(self.request)

        if self.future.done():
            try:
                response = self.future.result()
            except Exception as e:
                self.get_logger().info(f"Service call failed {e}")
            else:
                self.get_logger().info(f"RPM Left = {response.left_rpm}")
                self.get_logger().info(f"RPM Right = {response.right_rpm}")
   


def main(args=None):
    rclpy.init(args=args)
    node = ThrustersVelMux()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == "__main__":
    main()