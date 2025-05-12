#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from chungungo_interfaces.msg import HSVColor

import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

N_CAM = 2
BUOY_AREA_TH = 10


class Camera(Node):
    def __init__(self):
        super().__init__("Camera")

        # -------- Publishers and Subscribers --------
        self.image_pub = self.create_publisher(Image, "/camera", 10)
        self.red_hsv_sub = self.create_subscription(HSVColor, "/color_picker/red", self.update_thresholds_cb, 1)
        self.green_hsv_sub = self.create_subscription(HSVColor, "/color_picker/green", self.update_thresholds_cb, 1)

        # -------- Atributes --------
        timer_period = 0.1  # Segundos
        self.camera_timer = self.create_timer(timer_period, self.recieve_image_cb)

        self.lower_red = np.array([0, 0, 0])
        self.upper_red = np.array([0, 0, 0])

        self.lower_green = np.array([0, 0, 0])
        self.upper_green = np.array([0, 0, 0])

        # -------- Setup Routines --------
        self.setup_camera()


    def setup_camera(self):
        self.cap = cv2.VideoCapture(N_CAM, cv2.CAP_V4L)
        self.bridge = CvBridge()
        self.cap.read()


    def recieve_image_cb(self):
        ret, cv_frame = self.cap.read()
        # cv2.flip(cv_frame, 1, cv_frame)
        img_msg = self.bridge.cv2_to_imgmsg(cv_frame, encoding='bgr8')
        # cv2.imshow("Camera", cv_frame)
        self.color_masks(ret, cv_frame)
        cv2.waitKey(1)

        try:
            self.image_pub.publish(img_msg)
            
        except CvBridgeError as e:
            print("Couldn't transform from cv2 to image msg")
            print(e)


    def update_thresholds_cb(self, data):
        if data.color == 0:  # Rojo
            self.lower_red = np.array([data.h_low, 100, 100]) 
            self.upper_red = np.array([data.h_high, 255, 255])

        if data.color == 1:  # Verde
            self.lower_green = np.array([data.h_low, 100, 100]) 
            self.upper_green = np.array([data.h_high, 255, 255])

    def color_masks(self, ret, cv_frame):
        frame_smooth = cv2.GaussianBlur(cv_frame, (5,5), 0)
        hsv_frame = cv2.cvtColor(frame_smooth, cv2.COLOR_BGR2HSV)
        
        # ------------ Red mask ------------

        red_mask = cv2.inRange(hsv_frame, self.lower_red, self.upper_red)
        red_edges = cv2.Canny(red_mask, 100, 255)  # check
        red_contours, _ = cv2.findContours(red_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in red_contours:
            if cv2.contourArea(contour) > BUOY_AREA_TH:
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cX_red = int(M["m10"] / M["m00"])
                    cY_red = int(M["m01"] / M["m00"])
                else:
                    cX_red, cY_red = 0, 0

                cv2.circle(cv_frame, (cX_red, cY_red), 1, (255, 0, 0), -1)
                cv2.drawContours(cv_frame, [contour], -1, (255, 0, 0), 2)
        
        red_mask_frame = cv2.bitwise_and(frame_smooth, frame_smooth, mask=red_mask)

        # ------------ Green mask ------------

        green_mask = cv2.inRange(hsv_frame, self.lower_green, self.upper_green)
        green_edges = cv2.Canny(green_mask, 100, 255)  # check
        green_contours, _ = cv2.findContours(green_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in green_contours:
            if cv2.contourArea(contour) > BUOY_AREA_TH:
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cX_green = int(M["m10"] / M["m00"])
                    cY_green = int(M["m01"] / M["m00"])
                else:
                    cX_green, cY_green = 0, 0

                cv2.circle(cv_frame, (cX_green, cY_green), 1, (255, 0, 0), -1)
                cv2.drawContours(cv_frame, [contour], -1, (255, 0, 0), 2)
        
        green_mask_frame = cv2.bitwise_and(frame_smooth, frame_smooth, mask=green_mask)

        buoy_mask_frame = cv2.bitwise_or(red_mask_frame, green_mask_frame)

        cv2.imshow('Camera', cv_frame)
        cv2.imshow('MaskedBuoys', buoy_mask_frame)


def main(args=None):
    rclpy.init(args=args)
    node = Camera()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()