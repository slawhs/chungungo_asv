#!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node

# from std_msgs.msg import String
# from std_msgs.msg import Bool
# from sensor_msgs.msg import Image
# from chungungo_interfaces.msg import HSVColor

import numpy as np
import cv2
# from cv_bridge import CvBridge, CvBridgeError

N_CAM = 1
BUOY_AREA_TH = 200
MAX_RED_BUOYS = 2
MAX_GREEN_BUOYS = 2
MAX_ITERATIONS = 10


class Camera():
    def __init__(self):
        # -------- Publishers and Subscribers --------
        # self.image_pub = self.create_publisher(Image, "/camera", 10)
        # self.color_pub = self.create_publisher(String, "/color_detection", 10)

        # self.detect_sub = self.create_subscription(Bool, "/detect_order", self.detect_cb, 10)
        # self.red_hsv_sub = self.create_subscription(HSVColor, "/color_picker/red", self.update_thresholds_cb, 1)
        # self.green_hsv_sub = self.create_subscription(HSVColor, "/color_picker/green", self.update_thresholds_cb, 1)

        # -------- Atributes --------
        timer_period = 0.1  # Segundos
        # self.camera_timer = self.create_timer(timer_period, self.recieve_image_cb)

        self.lower_red = np.array([167, 100, 50])
        self.upper_red = np.array([7, 255, 255])

        self.lower_green = np.array([50, 70, 25])
        self.upper_green = np.array([80, 255, 255])

        self.detect = False
        self.detected_color = ""

        self.min_x_red = float('inf')
        self.max_x_red = float('-inf')

        self.min_x_green = float('inf')
        self.max_x_green = float('-inf')

        self.buoys_array = [None, None, None, None]  # [red_left, red_right, green_left, green_right]

        self.red_hue_wrap = False

        # -------- Setup Routines --------
        self.setup_camera()
        print("Camera initialized")


    def setup_camera(self):
        self.cap = cv2.VideoCapture(N_CAM, cv2.CAP_DSHOW)#, cv2.CAP_V4L2)
        # self.bridge = CvBridge()
        self.cap.read()

    def recieve_image_cb(self):
        ret, cv_frame = self.cap.read()
        # cv2.flip(cv_frame, 1, cv_frame)
        # img_msg = self.bridge.cv2_to_imgmsg(cv_frame, encoding='bgr8')
        # cv2.imshow("Camera", cv_frame)
        
        self.buoy_status()

        self.color_masks(ret, cv_frame)
        cv2.waitKey(1)

        # try:
        #     self.image_pub.publish(img_msg)
            
        # except CvBridgeError as e:
        #     print("Couldn't transform from cv2 to image msg")
        #     print(e)

    def buoy_status(self):
        count = 0
        for buoy in self.buoys_array:
            if buoy is not None:
                print(buoy)
            else:
                print("No buoy detected at index", count)
            count += 1
        print("")

    def update_thresholds_cb(self, data):
        if data.color == 0:  # Rojo
            self.lower_red = np.array([data.h_low, 100, 50]) 
            self.upper_red = np.array([data.h_high, 255, 255])
            self.red_hue_wrap = data.h_low > data.h_high  # Red Hue wraparound happening

        if data.color == 1:  # Verde
            self.lower_green = np.array([data.h_low, 100, 50]) 
            self.upper_green = np.array([data.h_high, 255, 255])

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

        red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        sorted_contours = sorted(red_contours, key=lambda c: cv2.boundingRect(c)[0])
        self.min_x_red = float('inf')
        self.max_x_red = float('-inf')

        for contour in sorted_contours:
            # print("Contour found")
            area = cv2.contourArea(contour)
            if area > BUOY_AREA_TH:
                # Aproximar contorno suavizado
                epsilon = 0.01 * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)

                M = cv2.moments(approx)
                if M["m00"] != 0:
                    cX_red = int(M["m10"] / M["m00"])
                    cY_red = int(M["m01"] / M["m00"])
                else:
                    cX_red, cY_red = 0, 0

                x_coords = approx[:, 0, 0]
                y_coords = approx[:, 0, 1]
                min_x = x_coords.min()
                max_x = x_coords.max()
                # print("min_x:", min_x, "max_x:", max_x)
                y_min_x = y_coords[x_coords == min_x].min()
                y_max_x = y_coords[x_coords == max_x].max()

                if min_x <= self.min_x_red: # this contour is the leftmost red buoy
                    # id for leftmost red buoy = 0
                    self.min_x_red = min_x
                    # print("Leftmost red buoy detected at:", min_x)
                    self.set_buoys(0, "red", min_x)

                elif max_x >= self.max_x_red: # this contour is the rightmost red buoy
                    # id for rightmost red buoy = 1
                    self.max_x_red = max_x
                    # print("Rightmost red buoy detected at:", max_x, '\n')
                    self.set_buoys(1, "red", max_x)
                    

                cv2.circle(cv_frame, (min_x, y_min_x), 10, (0, 0, 0), -1)
                cv2.circle(cv_frame, (max_x, y_max_x), 10, (255, 255, 0), -1)

                cv2.circle(cv_frame, (cX_red, cY_red), 1, (255, 0, 0), -1)
                cv2.drawContours(cv_frame, [approx], -1, (255, 0, 0), 2)

        right_red_buoy = self.buoys_array[1]
        if right_red_buoy is not None and self.max_x_red == float('-inf'):
            right_red_buoy.limit_counter += 1
            if right_red_buoy.limit_counter >= MAX_ITERATIONS:
                self.buoys_array[1] = None

        left_red_buoy = self.buoys_array[0]
        if left_red_buoy is not None and self.min_x_red == float('inf'):
            left_red_buoy.limit_counter += 1
            if left_red_buoy.limit_counter >= MAX_ITERATIONS:
                self.buoys_array[0] = None

        red_mask_frame = cv2.bitwise_and(frame_smooth, frame_smooth, mask=red_mask)

        # ------------ Green mask ------------
        green_mask = cv2.inRange(hsv_frame, self.lower_green, self.upper_green)

        green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        green_contours = sorted(green_contours, key=lambda c: cv2.boundingRect(c)[0])
        self.min_x_green = float('inf')
        self.max_x_green = float('-inf')

        for contour in green_contours:
            area = cv2.contourArea(contour)
            if area > BUOY_AREA_TH:
                epsilon = 0.01 * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)

                M = cv2.moments(approx)
                if M["m00"] != 0:
                    cX_green = int(M["m10"] / M["m00"])
                    cY_green = int(M["m01"] / M["m00"])
                else:
                    cX_green, cY_green = 0, 0

                x_coords = approx[:, 0, 0]
                y_coords = approx[:, 0, 1]
                min_x = x_coords.min()
                max_x = x_coords.max()
                # print("min_x:", min_x, "max_x:", max_x)
                y_min_x = y_coords[x_coords == min_x].min()
                y_max_x = y_coords[x_coords == max_x].max()

                if min_x <= self.min_x_green: # this contour is the leftmost green buoy
                    # id for leftmost green buoy = 2
                    self.min_x_green = min_x
                    # print("Leftmost green buoy detected at:", min_x)
                    self.set_buoys(2, "green", min_x)

                elif max_x >= self.max_x_green: # this contour is the rightmost green buoy
                    # id for rightmost green buoy = 3
                    self.max_x_green = max_x
                    # print("Rightmost green buoy detected at:", max_x, '\n')
                    self.set_buoys(3, "green", max_x)

                cv2.circle(cv_frame, (min_x, y_min_x), 10, (0, 0, 0), -1)
                cv2.circle(cv_frame, (max_x, y_max_x), 10, (255, 255, 0), -1)

                cv2.circle(cv_frame, (cX_green, cY_green), 1, (255, 0, 0), -1)
                cv2.drawContours(cv_frame, [approx], -1, (255, 0, 0), 2)

        right_green_buoy = self.buoys_array[3]
        if right_green_buoy is not None and self.max_x_green == float('-inf'):
            right_green_buoy.limit_counter += 1
            if right_green_buoy.limit_counter >= MAX_ITERATIONS:
                self.buoys_array[3] = None

        left_green_buoy = self.buoys_array[2]
        if left_green_buoy is not None and self.min_x_green == float('inf'):
            left_green_buoy.limit_counter += 1
            if left_green_buoy.limit_counter >= MAX_ITERATIONS:
                self.buoys_array[2] = None

        green_mask_frame = cv2.bitwise_and(frame_smooth, frame_smooth, mask=green_mask)

        buoy_mask_frame = cv2.bitwise_or(red_mask_frame, green_mask_frame)

        cv2.imshow('Camera', cv_frame)
        cv2.imshow('MaskedBuoys', buoy_mask_frame)

    def set_buoys(self, id, color, limit):
        buoy = Buoy(id, color, limit)
        self.buoys_array[id] = buoy

    # def publish_color(self, color):
    #     color_msg = String()
    #     color_msg.data = color
    #     self.color_pub.publish(color_msg)


class Buoy():
    def __init__(self, id: int, color: str, limit: int, counter: int = 0):
        self.id = id
        self.color = color

        self.limit = limit
        self.limit_counter = counter

        self.distance = None
        self.angle = None

        self.name = ""

        self.set_name()

    def __str__(self):
        return f"{self.name}, Distance: {self.distance}, Angle: {self.angle}"

    def get_id(self):
        return self.id

    def get_color(self):
        return self.color
    
    def set_distance(self, distance):
        self.distance = distance

    def get_distance(self):
        return self.distance
    
    def set_angle(self, angle):
        self.angle = angle

    def get_angle(self):
        return self.angle
    
    def set_name(self):
        if self.id % 2 == 0:
            self.name = f"Buoy {self.id} - {self.color} - Left"
        else:
            self.name = f"Buoy {self.id} - {self.color} - Right"


def main(args=None):
    # rclpy.init(args=args)
    node = Camera()
    # rclpy.spin(node)

    # rclpy.shutdown()
    while True:
        node.recieve_image_cb()
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

if __name__ == "__main__":
    main()