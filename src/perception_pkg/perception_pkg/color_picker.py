#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from chungungo_interfaces.msg import HSVColor

import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

N_CAM = 0
RED_WRAP_HUE = 20

class ColorPicker(Node):
    def __init__(self): 
        super().__init__("ColorPicker")

        # -------- Publishers and Subscribers --------
        self.image_sub = self.create_subscription(Image, "/camera", self.recieve_image_cb, 10)
        self.red_pub = self.create_publisher(HSVColor, "/color_picker/red", 1)
        self.green_pub = self.create_publisher(HSVColor, "/color_picker/green", 1)
        
        # -------- Atributes --------
        self.lower_red = np.array([None, None, None])
        self.upper_red = np.array([None, None, None])
        
        self.lower_green = np.array([None, None, None])
        self.upper_green = np.array([None, None, None])

        self.left_clicks = 0
        self.right_clicks = 0
        
        self.red_points = np.zeros((4, 2), dtype=np.int32)
        self.green_points = np.zeros((4, 2), dtype=np.int32)

        # -------- Setup Routines --------
        self.setup_camera()
        self.get_logger().info("Finished Setup")

    def setup_camera(self):
        cv2.namedWindow("ColorPicker")
        cv2.setMouseCallback("ColorPicker", self.select_area_cb)

        # self.cap = cv2.VideoCapture(N_CAM, cv2.CAP_V4L)
        self.bridge = CvBridge()
        # self.cap.read()


    def recieve_image_cb(self, img):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(img)
            
            # Draw selected points
            for i in range(self.left_clicks):
                cv2.circle(self.frame, tuple(self.red_points[i]), 5, (0,0,255), -1)
            for i in range(self.right_clicks):
                cv2.circle(self.frame, tuple(self.green_points[i]), 5, (0,255,0), -1)

            if self.left_clicks == 4:
                poly_points = self.red_points.reshape((-1, 1, 2))
                cv2.polylines(self.frame, [poly_points], isClosed=(self.left_clicks==4), color=(0,0,255), thickness=2)
                
                red_pixels = self.get_pixels_in_area(self.red_points)

                self.lower_red, self.upper_red = self.get_hsv_range(red_pixels)
            
            if self.right_clicks == 4:
                poly_points = self.green_points.reshape((-1, 1, 2))
                cv2.polylines(self.frame, [poly_points], isClosed=(self.right_clicks==4), color=(0,255,0), thickness=2)
    
                green_pixels = self.get_pixels_in_area(self.green_points)

                self.lower_green, self.upper_green = self.get_hsv_range(green_pixels)

            key = cv2.waitKey(1)
            
            if key == 13 and self.left_clicks == 4 and self.right_clicks == 4:
                self.send_color_msgs()

            if key == ord('q'):
                exit()

            if key == ord('r'):
                self.reset_atributes()
                self.get_logger().info("Atributes reseted")


            cv2.imshow("ColorPicker", self.frame)
            cv2.waitKey(1)

        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge error: {e}")


    def reset_atributes(self):
        self.left_clicks = 0
        self.right_clicks = 0
        self.red_points = np.zeros((4, 2), dtype=np.int32)
        self.green_points = np.zeros((4, 2), dtype=np.int32)


    # def get_hsv_range(self, masked_pixels):
    #     h_min, s_min, v_min = np.min(masked_pixels, axis=0)
    #     h_max, s_max, v_max = np.max(masked_pixels, axis=0)

    #     lower_hsv = np.array([
    #         max(0, h_min),
    #         max(0, s_min),
    #         max(0, v_min)
    #     ])

    #     upper_hsv = np.array([
    #         min(179, h_max),
    #         min(255, s_max),
    #         min(255, v_max)
    #     ])

    #     return lower_hsv, upper_hsv

    def get_hsv_range(self, masked_pixels):
        hsv = masked_pixels.astype(np.float32)
        h, s, v = hsv[:, 0], hsv[:, 1], hsv[:, 2]

        h_shifted = h.copy()
        h_shifted[h < RED_WRAP_HUE] += 180  # Shift hues < 20 to "unwrap" red around 0

        h_low = np.percentile(h_shifted, 5) % 180
        h_high = np.percentile(h_shifted, 95) % 180

        s_low = np.percentile(s, 5)
        s_high = np.percentile(s, 95)

        v_low = np.percentile(v, 5)
        v_high = np.percentile(v, 95)

        lower_hsv = np.array([
            max(0, h_low),
            max(0, s_low),
            max(0, v_low)
        ]).astype(np.uint8)

        upper_hsv = np.array([
            min(179, h_high),
            min(255, s_high),
            min(255, v_high)
        ]).astype(np.uint8)

        return lower_hsv, upper_hsv


    def get_pixels_in_area(self, points):
        hsv_frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
        
        mask = np.zeros(self.frame.shape[:2], dtype=np.uint8)
        cv2.fillConvexPoly(mask, points, 255)
        
        masked_pixels = hsv_frame[mask == 255] 
        
        return masked_pixels
    

    def select_area_cb(self, event, x, y, flags, param):
        
        if event == cv2.EVENT_LBUTTONDOWN and self.left_clicks < 4:
            self.red_points[self.left_clicks] = [x, y]
            self.left_clicks += 1

        elif event ==cv2.EVENT_RBUTTONDOWN and self.right_clicks < 4:
            self.green_points[self.right_clicks] = [x,y]
            self.right_clicks += 1


    def send_color_msgs(self):
        red_msg = HSVColor()
        red_msg.color = 0 
        
        red_msg.h_low = int(self.lower_red[0])
        red_msg.s_low = int(self.lower_red[1])
        red_msg.v_low = int(self.lower_red[2])
        
        red_msg.h_high = int(self.upper_red[0])
        red_msg.s_high = int(self.upper_red[1])
        red_msg.v_high = int(self.upper_red[2])
        
        self.red_pub.publish(red_msg)

        print(f"Lower Red: {self.lower_red}")
        print(f"Upper Red: {self.upper_red}")

        green_msg = HSVColor()
        green_msg.color = 1

        green_msg.h_low = int(self.lower_green[0])
        green_msg.s_low = int(self.lower_green[1])
        green_msg.v_low = int(self.lower_green[2])
        
        green_msg.h_high = int(self.upper_green[0])
        green_msg.s_high = int(self.upper_green[1])
        green_msg.v_high = int(self.upper_green[2])

        print(f"Lower Green: {self.lower_green}")
        print(f"Upper Green: {self.upper_green}")

        self.green_pub.publish(green_msg)

        self.reset_atributes()

        self.get_logger().info("Color messages sent")




def main(args=None):
    rclpy.init(args=args)
    node = ColorPicker()
    
    rclpy.spin(node)
    
    rclpy.shutdown()
    node.cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()