#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from chungungo_interfaces.srv import VelocityCommand
from chungungo_interfaces.msg import ThrustersVelocity
from geometry_msgs.msg import Vector3  #? Do custom interface for joystick_speed

class ThrustersVelMux(Node):
    def __init__(self):
        super().__init__("ThrustersVelocityMux")

        # -------- Publishers, Subscribers and Services --------
        #* crear subscriber de nodos de teleop, control de boyas, control de distancias. 
        self.joystick_vel_sub = self.create_subscription(ThrustersVelocity, '/joystick_velocity', self.joystick_vel_cb, 10)
        self.buoy_distance_vel_sub = self.create_subscription()

        self.vel_cmd_cli = self.create_client(VelocityCommand, '/vel_command')
        while not self.vel_cmd_cli.wait_for_service(timeout_sec=0.5):
            self.get_logger().info('Service not available, waiting again...')

        # -------- Atributes --------
        timer_period = 0.1  # Segundos
        self.vel_mux_timer = self.create_timer(timer_period, self.vel_mux_cb)
        self.request = VelocityCommand.Request()
        
        self.joystick_left_vel = 0.0
        self.joystick_right_vel = 0.0

        # -------- Setup Routines --------

    def joystick_vel_cb(self, msg):
        self.joystick_left_vel = msg.left_velocity
        self.joystick_right_vel = msg.right_velocity

    def vel_mux_cb(self):
        # Joystick velocity
        self.request.left_velocity = self.joystick_left_vel
        self.request.right_velocity = self.joystick_right_vel
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