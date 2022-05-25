"""
Author: windzu
Date: 2022-03-23 10:30:25
LastEditTime: 2022-03-23 10:33:32
LastEditors: windzu
Description: 
FilePath: /monocular_camera_calibration/test/get_frame.py
@Copyright (C) 2021-2022 plusgo Company Limited. All rights reserved.
@Licensed under the Apache License, Version 2.0 (the License)
"""
import cv2
import numpy as np
import os
import sys
import math
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
import rospy
import time, threading

# local
sys.path.append("../")
from common.enum_common import FrameInputMode


class ROSImageConverter:
    def __init__(self, ros_topic):
        self.ros_topic = ros_topic
        self.frame = None
        if self.ros_topic.endswith("compressed"):
            self.image_type = "CompressedImage"
        else:
            self.image_type = "Image"

        # create a thread to start the subscriber
        self.mutex = threading.Lock()
        self.thread = threading.Thread(target=self.start_subscriber)
        self.thread.start()

    def start_subscriber(self):
        if self.image_type == "CompressedImage":
            self.image_sub = rospy.Subscriber(self.ros_topic, CompressedImage, self.callback)
        else:
            self.image_sub = rospy.Subscriber(self.ros_topic, Image, self.callback)
        rospy.spin()

    def callback(self, data):
        if self.image_type == "CompressedImage":
            cmprs_img_msg = data
            str_msg = cmprs_img_msg.data
            buf = np.ndarray(shape=(1, len(str_msg)), dtype=np.uint8, buffer=cmprs_img_msg.data)
            frame = cv2.imdecode(buf, cv2.IMREAD_ANYCOLOR)
        else:
            frame = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)

        self.mutex.acquire()
        self.frame = frame
        self.mutex.release()

    def read(self):
        if self.frame is None:
            return False, None
        else:
            self.mutex.acquire()
            frame = self.frame
            self.mutex.release()
            return True, frame

    def release(self):
        self.image_sub.unregister()

    def isOpened(self):
        return True


class GetFrame:
    def __init__(self, input_mode, device_name, ros_topic):
        self.input_mode = input_mode
        self.device_name = device_name
        self.ros_topic = ros_topic
        self.camera_info = None

        self.raw_frame = None
        if self.input_mode == FrameInputMode.DEVICE_NAME:
            self.cap = cv2.VideoCapture(self.device_name)
        elif self.input_mode == FrameInputMode.ROS_TOPIC:
            self.cap = ROSImageConverter(self.ros_topic)

        if not self.cap.isOpened():
            raise Exception("Failed to open camera")

    def read(self):
        frame = None
        while True:
            ret, frame = self.cap.read()
            if ret:
                break
            else:
                time.sleep(0.01)
        return frame

    def set_camera_info(self, camera_info):
        self.camera_info = camera_info

    def get_undistort_frame(self, frame=None):
        if frame is None:
            frame = self.read()
        camera_info = self.camera_info

        frame = cv2.remap(frame, camera_info.map1, camera_info.map2, cv2.INTER_LINEAR)
        return frame

    def get_bird_view_frame(self, frame=None):
        if frame is None:
            frame = self.read()
        camera_info = self.camera_info
        if camera_info is None:
            print("camera_info is None")
            raise Exception("camera_info is None")

        frame = cv2.remap(frame, camera_info.map1, camera_info.map2, cv2.INTER_LINEAR)
        frame = cv2.warpPerspective(frame, camera_info.homography_matrix, camera_info.mask_size)
        return frame

    def get_bird_view_frame_with_mask(self, frame=None):
        if frame is None:
            frame = self.read()
        camera_info = self.camera_info
        if camera_info is None:
            print("camera_info is None")
            raise Exception("camera_info is None")

        frame = cv2.remap(frame, camera_info.map1, camera_info.map2, cv2.INTER_LINEAR)
        frame = cv2.warpPerspective(frame, camera_info.homography_matrix, camera_info.mask_size)
        frame = cv2.bitwise_and(frame, frame, mask=camera_info.mask)
        return frame

    def release(self):
        self.cap.release()

