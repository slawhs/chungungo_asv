#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3Stamped
from chungungo_interfaces.msg import HSVColor

import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

N_CAM = 2

class ColorPicker(Node):
    def __init__(self): 
        super().__init__("ColorPicker")

        # -------- Publishers and Subscribers --------
        self.image_sub = self.create_subscription(Image, "/camera", self.recieve_image_cb, 10)
        self.red_pub = self.create_publisher(HSVColor, "/color_picker/red", 1)
        self.green_pub = self.create_publisher(HSVColor, "/color_picker/green", 1)
        
        # -------- Atributes --------
        self.red_hsv = np.array([None, None, None])
        self.green_hsv = np.array([None, None, None])
        self.left_clicks = 0
        self.right_clicks = 0
        self.points = []

        # -------- Setup Routines --------
        self.setup_camera()

    def setup_camera(self):
        cv2.namedWindow("ColorPicker")
        cv2.setMouseCallback("ColorPicker", self.mouse_cb)

        self.cap = cv2.VideoCapture(N_CAM, cv2.CAP_V4L)
        self.bridge = CvBridge()
        self.cap.read()

    def recieve_image_cb(self, img):
        # self.get_logger().info("Recieving video frame")
        self.frame = self.bridge.imgmsg_to_cv2(img)
        
        cv2.imshow("ColorPicker", self.frame)
        cv2.waitKey(1)


    def mouse_cb(self, event, x, y, flags, param):

        hsv_frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
        if event == cv2.EVENT_LBUTTONDOWN and self.left_clicks:
            self.red_hsv = hsv_frame[y,x]
            print(f"RED HSV = {self.red_hsv}")

        if event == cv2.EVENT_RBUTTONDOWN:
            self.green_hsv = hsv_frame[y,x]
            print(f"GREEN HSV = {self.green_hsv}")
    
        if (self.red_hsv.all() != None) and (self.green_hsv.all() != None):
            self.send_color_msgs()
            exit()
    

    def send_color_msgs(self):
            red_msg = HSVColor()
            red_msg.color = 0 
            red_msg.h = int(self.red_hsv[0])
            red_msg.s = int(self.red_hsv[1])
            red_msg.v = int(self.red_hsv[2])
            self.red_pub.publish(red_msg)

            green_msg = HSVColor()
            green_msg.color = 1
            green_msg.h = int(self.green_hsv[0])
            green_msg.s = int(self.green_hsv[1])
            green_msg.v = int(self.green_hsv[2])
            self.green_pub.publish(green_msg)



def main(args=None):
    rclpy.init(args=args)
    node = ColorPicker()
    
    rclpy.spin(node)
    rclpy.shutdown()
    node.cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()