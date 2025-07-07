#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import lgpio
from std_msgs.msg import String 


class Relay(Node):
    def __init__(self):
        super().__init__("Relay")


        # -------- Publishers, Subscribers Clients and Services --------
        self.mode_sub = self.create_subscription(String, "/operation_mode", self.mode_cb, 10)
        # -------- Atributes --------
        self.gpio = lgpio.gpiochip_open(0)
        self.gpio_number = 17 
        
        self.turn_off()

    def mode_cb(self, mode_msg):
        if mode_msg.data == "standby":
            self.turn_off()
        else:
            self.turn_on()
    
    def turn_on(self):
        lgpio.gpio_write(self.gpio, self.gpio_number, 0)
        self.get_logger().info("Relay turned ON")
    
    def turn_off(self):
        lgpio.gpio_write(self.gpio, self.gpio_number, 1)
        self.get_logger().info("Relay turned OFF")

def main(args=None):
    rclpy.init(args=args)
    node = Relay()
    rclpy.spin(node)
    
    node.serial.close()
    rclpy.shutdown()


if __name__ == "__main__":
    main()