#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from chungungo_interfaces.msg import HSVColor

import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

from buoy import Buoy


class Camera():
    def __init__(self):
        # -------- Parameters Setup --------
        self.declare_parameter("n_cam", 2)
        self.declare_parameter("buoy_area_th", 200.0)
        self.declare_parameter("max_iterations", 10)
        self.declare_parameter("debug", True)

        self.n_cam = self.get_parameter("n_cam").value
        self.buoy_area_th = self.get_parameter("buoy_area_th").value
        self.max_iterations = self.get_parameter("max_iterations").value
        self.debug = self.get_parameter("debug").value

        # -------- Publishers and Subscribers --------
        self.image_pub = self.create_publisher(Image, "/camera", 10)
        self.masked_image_pub = self.create_publisher(Image, "/masked_buoys", 10)
        self.color_pub = self.create_publisher(String, "/color_detection", 10)

        self.detect_sub = self.create_subscription(Bool, "/detect_order", self.detect_cb, 10)
        self.red_hsv_sub = self.create_subscription(HSVColor, "/color_picker/red", self.update_thresholds_cb, 1)
        self.green_hsv_sub = self.create_subscription(HSVColor, "/color_picker/green", self.update_thresholds_cb, 1)

        # -------- Atributes --------
        timer_period = 0.1  # Segundos
        self.camera_timer = self.create_timer(timer_period, self.recieve_image_cb)

        self.lower_red = np.array([167, 100, 50])
        self.upper_red = np.array([7, 255, 255])

        self.lower_green = np.array([50, 70, 25])
        self.upper_green = np.array([80, 255, 255])

        self.detect = False
        self.detected_color = ""

        self.red_hue_wrap = False

        self.min_x_red = float('inf')
        self.max_x_red = float('-inf')

        self.min_x_green = float('inf')
        self.max_x_green = float('-inf')

        self.buoys_array = [None, None, None, None]  # [red_left, red_right, green_left, green_right]

        # -------- Setup Routines --------
        self.setup_camera()


    def setup_camera(self):
        self.cap = cv2.VideoCapture(self.n_cam, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error("Camera could not be opened")
        self.bridge = CvBridge()
        self.cap.read()

    def detect_cb(self, msg):
        self.detect = msg.data
        if self.detect:
            self.publish_color(self.detected_color)
            self.detect = False

    def recieve_image_cb(self):
        ret, cv_frame = self.cap.read()
    
        buoy_mask_frame = self.color_masks(ret, cv_frame)

        if self.debug:
            self.buoy_status()

        cv2.waitKey(1)

        try:
            self.publish_images(cv_frame, buoy_mask_frame)
            
        except CvBridgeError as e:
            print("Couldn't transform from cv2 to image msg")
            print(e)

    def update_thresholds_cb(self, data):
        if data.color == 0:  # Red
            self.lower_red = np.array([data.h_low, 100, 50]) 
            self.upper_red = np.array([data.h_high, 255, 255])
            self.red_hue_wrap = data.h_low > data.h_high  # Red Hue wraparound happening

        if data.color == 1:  # Green
            self.lower_green = np.array([data.h_low, 100, 50]) 
            self.upper_green = np.array([data.h_high, 255, 255])

    def buoy_status(self):
        count = 0
        for buoy in self.buoys_array:
            if buoy is not None:
                print(buoy)
            else:
                print("No buoy detected at index", count)
            count += 1
        print("")

    def color_masks(self, ret, cv_frame):
        h_low = self.lower_red[0]
        h_high = self.upper_red[0]
        self.red_hue_wrap = h_low > h_high

        frame_smooth = cv2.GaussianBlur(cv_frame, (5, 5), 0)
        hsv_frame = cv2.cvtColor(frame_smooth, cv2.COLOR_BGR2HSV)

        # ------------ Red mask ------------
        if self.red_hue_wrap:   # if red Hue values are around 0
            red_mask_1 = cv2.inRange(hsv_frame, np.array((0, self.lower_red[1], self.lower_red[2])), self.upper_red)
            red_mask_2 = cv2.inRange(hsv_frame, self.lower_red, np.array((179, self.upper_red[1], self.upper_red[2])))
            red_mask = cv2.bitwise_or(red_mask_1, red_mask_2)

        else: 
            red_mask = cv2.inRange(hsv_frame, self.lower_red, self.upper_red)

        red_mask_frame = cv2.bitwise_and(frame_smooth, frame_smooth, mask=red_mask)
        self.process_contours(red_mask, cv_frame, 'red', 0, 1, [self.min_x_red], [self.max_x_red])

        # ------------ Green mask ------------
        green_mask = cv2.inRange(hsv_frame, self.lower_green, self.upper_green)

        green_mask_frame = cv2.bitwise_and(frame_smooth, frame_smooth, mask=green_mask)
        self.process_contours(green_mask, cv_frame, 'green', 2, 3, [self.min_x_green], [self.max_x_green])

        if self.debug:
            buoy_mask_frame = cv2.bitwise_or(red_mask_frame, green_mask_frame)

            cv2.imshow('Camera', cv_frame)
            cv2.imshow('MaskedBuoys', buoy_mask_frame)

    def process_contours(self, mask, frame, color, left_id, right_id, min_ref, max_ref):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key=lambda c: cv2.boundingRect(c)[0])
        min_ref[0], max_ref[0] = float('inf'), float('-inf')

        for contour in contours:
            if cv2.contourArea(contour) > self.buoy_area_th:
                epsilon = 0.01 * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)
                M = cv2.moments(approx)
                if M["m00"] == 0:
                    continue
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                x = approx[:, 0, 0]
                min_x = x.min()
                max_x = x.max()

                if self.debug:
                    y = approx[:, 0, 1]
                    y_min_x = y[x == min_x].min()
                    y_max_x = y[x == max_x].max()

                if min_x <= min_ref[0]:
                    min_ref[0] = min_x
                    self.set_buoys(left_id, color, min_x)
                elif max_x >= max_ref[0]:
                    max_ref[0] = max_x
                    self.set_buoys(right_id, color, max_x)

                if self.debug:
                    cv2.drawContours(frame, [approx], -1, (255, 0, 0), 2)
                    cv2.circle(frame, (min_x, y_min_x), 5, (0, 0, 255), -1)
                    cv2.circle(frame, (max_x, y_max_x), 5, (0, 255, 255), -1)
                    cv2.circle(frame, (cX, cY), 3, (255, 255, 255), -1)
                
        self.check_buoy_timeout(left_id, min_ref[0], 'left')
        self.check_buoy_timeout(right_id, max_ref[0], 'right')

    def check_buoy_timeout(self, id, x_ref, side):
        buoy = self.buoys_array[id]
        if buoy and ((side == 'left' and x_ref == float('inf')) or (side == 'right' and x_ref == float('-inf'))):
            buoy.limit_counter += 1
            if buoy.limit_counter >= self.max_iterations:
                self.buoys_array[id] = None

    def set_buoys(self, id, color, limit):
        buoy = Buoy(id, color, limit)
        self.buoys_array[id] = buoy

    def publish_color(self, color):
        color_msg = String()
        color_msg.data = color
        self.color_pub.publish(color_msg)

    def publish_images(self, cv_frame, buoy_mask_frame):
        img_msg = self.bridge.cv2_to_imgmsg(cv_frame, encoding='bgr8')

        buoy_mask_bgr = cv2.cvtColor(buoy_mask_frame, cv2.COLOR_HSV2BGR)
        buoy_mask_msg = self.bridge.cv2_to_imgmsg(buoy_mask_bgr, encoding='bgr8')


        self.image_pub.publish(img_msg)
        self.masked_image_pub.publish(buoy_mask_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Camera()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()