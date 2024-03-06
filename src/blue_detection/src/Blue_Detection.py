#!/usr/bin/env python3
# This Python file uses the following encoding: utf-8

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage

import cv2
import numpy as np


class Blue_Detection(Node):

    def __init__(self):
        super().__init__('Blue_Detection')
        self.publisher_ = self.create_publisher(CompressedImage, 'blue_publisher', 10)

        self.subscription = self.create_subscription(CompressedImage,'/scara/image/compressed', self.blue_treatement,10)
        self.subscription


    def blue_treatement(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        img = frame
        hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        lower_blue = np.array([100,150,0])
        upper_blue = np.array([140,255,255])

        mask = cv2.inRange(img,lower_blue,upper_blue)

        cnts,hie = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
        cv2.drawContours(frame,cnts,-1,(0,255,0),3)

        compressed_img_msg = CompressedImage()
        _, img_encoded = cv2.imencode('.jpg', frame)
        compressed_img_msg.data = img_encoded.tobytes()
        compressed_img_msg.header = msg.header
        compressed_img_msg.format = msg.format

        self.publisher_.publish(compressed_img_msg)


def main(args=None):
    rclpy.init(args=args)

    blue_detection = Blue_Detection()

    rclpy.spin(blue_detection)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    blue_detection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

