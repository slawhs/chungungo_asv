#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32
from chungungo_interfaces.msg import ThrustersVelocity
from control_pkg.pid_controller import PIDController
from time import time
import numpy as np

# TODO:
# CALIBRAR PID
# AJUSTAR ESCALON
# COMPROBAR VELOCIDAD

VELOCITY_KP = 1.0
VELOCITY_KI = 0.0
VELOCITY_KD = 0.0
MIN_SATURATION = -2.0
MAX_SATURATION = 2.0

class Cola:
    def __init__(self, max_size):
        self.items = []
        self.max_size = max_size
    
    def add(self, item):
        if len(self.items) > self.max_size:
            self.items.pop(0)
        self.items.append(item)
    
    def average(self):
        if len(self.items) > self.max_size:
            a = np.average(self.items)
        else:
            a = 0
        return a

class IMUVelocityController(Node):
    def __init__(self):
        super().__init__("IMUVelocityController")

        # -------- Publishers and Subscribers --------
        self.vel_sub = self.create_subscription(Float32, "/imu_ctrl/setpoint", self.set_desired_velocity, 1)
        self.imu_sub = self.create_subscription(Imu, "/bno055/imu", self.imu_cb, 1)
        self.vel_pub = self.create_publisher(ThrustersVelocity, '/imu_ctrl/velocity', 10)

        # -------- Atributes --------
        self.vel_msg = ThrustersVelocity()
        
        # timer_period = 0.01 
        # self.timer = self.create_timer(timer_period, self.control_cb)

        # PID Controller
        self.Ts = 0.01
        self.velocity_controller = PIDController(Kp=VELOCITY_KP, Ki=VELOCITY_KI, Kd=VELOCITY_KD, min_saturation=MIN_SATURATION,max_saturation=MAX_SATURATION, Ts=0.01)
        self.desired_vel = 0.0
        
        self.linear_vel = 0.0
        self.last_time = time()
        self.last_acel = None
        self.linear_acceleration = 0.0
        self.aceleraciones = Cola(100)

    def set_desired_velocity(self, desired_velocity):
        """Set the desired velocity for the thrusters."""
        self.desired_vel = desired_velocity.data
        self.get_logger().info(f"Desired Velocity set to: {self.desired_vel:.2f} m/s")


    def imu_cb(self, imu_msg):
        # Revisar si el vector es el que corresponde.
        self.linear_acceleration = np.round(imu_msg.linear_acceleration.x, 4)
        tiempo = time()
        self.aceleraciones.add(self.linear_acceleration)
        dt = tiempo - self.last_time
        average = self.aceleraciones.average()
        if self.last_acel is not None:
            delta_v = (average) * dt
            self.linear_vel += delta_v

        self.last_acel = self.linear_acceleration
        self.last_time = tiempo
        # Conversion automatica KM/H
        # self.linear_vel.x = round(self.linear_acceleration.x * (self.delta_t) * 3.6, 3) + self.last_vel.x
        # self.linear_vel.y = round(self.linear_acceleration.y * (self.delta_t) * 3.6, 3)
        # self.linear_vel.z = round(self.linear_acceleration.z * (self.delta_t) * 3.6, 3)
        # self.get_logger().info(f"Linear Velocity_x: ({(self.linear_vel.x):.2f}, {(self.linear_vel.y):.2f}, {(self.linear_vel.z):.2f}) km/h")
        self.get_logger().info(f"Linear Velocity_x: {(self.linear_vel):.2f}\t m/s")
        self.get_logger().info(f"Linear acel_x: {(self.linear_acceleration):.2f}\t m/s")

        # self.get_logger().info(f"Linear Velocity_y: {(self.linear_vel):.2f}\t m/s")
        # self.get_logger().info(f"Linear Velocity_z: {(self.linear_vel):.2f}\t m/s")

        # self.velocity_control()



    def velocity_control(self):
        # Controlador de velocidad
        control_effort = self.velocity_controller.pid(setpoint=self.desired_vel, feedback=self.linear_vel)

        # No estoy muy seguro de esto, hay que agregar un escalon?
        self.vel_msg.left_velocity = control_effort
        self.vel_msg.right_velocity = control_effort

        # self.get_logger().info(f"Linear Velocity: {self.linear_vel:.2f} m/s")
        self.get_logger().info(f"Linear Velocity_x: ({(self.linear_vel.x):.2f}, {(self.linear_vel.y):.2f}, {(self.linear_vel.z):.2f}) km/h")


        # self.get_logger().info(f"Control Effort: {control_effort:.2f}")
        self.vel_pub.publish(self.vel_msg)
        pass


def main(args=None):
    rclpy.init(args=args)
    node = IMUVelocityController()
    rclpy.spin(node)
    
    node.serial.close()
    rclpy.shutdown()


if __name__ == "__main__":
    main()