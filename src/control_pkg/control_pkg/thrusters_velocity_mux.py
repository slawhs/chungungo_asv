#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from chungungo_interfaces.srv import VelocityCommand
from geometry_msgs.msg import Vector3  #? Do custom interface for joystick_speed

class ThrustersVelMux(Node):
    def __init__(self):
        super().__init__("ThrustersVelocityMux")

        # -------- Publishers, Subscribers and Services --------
        #* crear subscriber de nodos de teleop, control de boyas, control de distancias. 
        self.joystick_vel_sub = self.create_subscription(Vector3, '/joystick_velocity', self.joystick_vel_cb, 10)

        self.vel_cmd_cli = self.create_client(VelocityCommand, '/vel_command')
        while not self.vel_cmd_cli.wait_for_service(timeout_sec=0.5):
            self.get_logger().info('Service not available, waiting again...')

        # -------- Atributes --------
        timer_period = 0.1  # Segundos
        self.vel_mux_timer = self.create_timer(timer_period, self.vel_mux_cb)
        self.request = VelocityCommand.Request()
        
        self.left_speed = 0.0
        self.left_direction = 0.0
        self.right_speed = 0.0
        self.right_direction = 0.0

        # -------- Setup Routines --------

    def joystick_vel_cb(self, msg):

        # Set speed
        self.left_speed = int(abs(msg.x))
        self.right_speed = int(abs(msg.y))

        # Set direction
        if msg.x > 0:
            self.left_direction = 1
        elif msg.x < 0:
            self.left_direction = -1
        elif msg.x == 0:
            self.left_direction = 0

        if msg.y > 0:
            self.right_direction = 1
        elif msg.y < 0:
            self.right_direction = -1
        elif msg.y == 0:
            self.right_direction = 0

        

    def vel_mux_cb(self):
        self.request.left_speed = int(self.left_speed)
        self.request.left_direction = int(self.left_direction)
        self.request.right_speed = int(self.right_speed)
        self.request.right_direction = int(self.right_direction)
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