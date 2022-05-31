"""
Author: windzu
Date: 2022-04-09 14:00:40
LastEditTime: 2022-04-09 14:00:41
LastEditors: windzu
Description: 
FilePath: /windzu_ws/src/tools/calibration_tools/surround_view_calibration/surround_view_calibration_node.py
@Copyright (C) 2021-2022 plusgo Company Limited. All rights reserved.
@Licensed under the Apache License, Version 2.0 (the License)
"""

import rospy
import cv2
import numpy as np
import sys

# local
sys.path.append("../../")
from common.enum_common import CameraInfoCheckLevel
from utils.camera_utils import camera_info_check, calculate_cameras_mask, PointSelector


class SurroundViewCalibrationNode:
    def __init__(self, front_cap=None, left_cap=None, right_cap=None, back_cap=None):
        self.front_cap = front_cap
        self.left_cap = left_cap
        self.right_cap = right_cap
        self.back_cap = back_cap

    def _set_current_param(self, current_camera_id, current_cap, current_cc_info, current_camera_info):
        self.current_camera_id = current_camera_id
        self.current_cap = current_cap
        self.current_cc_info = current_cc_info
        self.current_camera_info = current_camera_info

    def start(self):
        print("*******start set scale and shift*******")
        print("waiting frame...")
        # cap = self.front_cap
        cap = self.current_cap
        camera_info = self.current_camera_info
        cc_info = self.current_cc_info
        camera_info = self.current_camera_info

        current_frame = cap.read()
        window_name = "start set scale and shift"
        cv2.namedWindow(window_name)
        scale_x_progress_trackbar_name = " scale x "
        scale_y_progress_trackbar_name = " scale y "
        shift_x_progress_trackbar_name = " shift x "
        shift_y_progress_trackbar_name = " shift y "

        cv2.createTrackbar(scale_x_progress_trackbar_name, window_name, 0, 100, self._on_scale_x_trackbar)
        cv2.createTrackbar(scale_y_progress_trackbar_name, window_name, 0, 100, self._on_scale_y_trackbar)
        cv2.createTrackbar(shift_x_progress_trackbar_name, window_name, 0, 600, self._on_shift_x_trackbar)
        cv2.createTrackbar(shift_y_progress_trackbar_name, window_name, 0, 600, self._on_shift_y_trackbar)
        # 设置初始默认值
        scale_x = int(camera_info.scale_xy[0] * 100)
        scale_y = int(camera_info.scale_xy[1] * 100)
        shift_x = int(camera_info.shift_xy[0] + 300)
        shift_y = int(camera_info.shift_xy[1] + 300)
        cv2.setTrackbarPos(scale_x_progress_trackbar_name, window_name, scale_x)
        cv2.setTrackbarPos(scale_y_progress_trackbar_name, window_name, scale_y)
        cv2.setTrackbarPos(shift_x_progress_trackbar_name, window_name, shift_x)
        cv2.setTrackbarPos(shift_y_progress_trackbar_name, window_name, shift_y)
        # 调整拉伸参数
        while True:
            camera_info.map1 = None
            camera_info.map2 = None
            ret, camera_info = camera_info_check(camera_info, InfoCheckLevel.COMPLETED)
            if ret is False:
                print("camera_info check failed")
                break
            else:
                self.current_camera_info = camera_info

            show_frame = cv2.remap(current_frame, camera_info.map1, camera_info.map2, cv2.INTER_LINEAR)
            cv2.imshow(window_name, show_frame)
            key = cv2.waitKey(1)
            if key == ord("q"):
                cv2.destroyAllWindows()
                break
            elif key == 13:  # enter
                cv2.destroyAllWindows()
                print("*******choose src points*******")
                # get src points
                src_points = None
                point_selector = PointSelector(show_frame)
                choice = point_selector.loop()
                if choice > 0:
                    src_points = np.float32(point_selector.keypoints)
                    src_points = src_points.reshape((-1, 1, 2))
                camera_info.homography_matrix = self._calculate_homography_matrix(src_points)
                break
        self.current_camera_info = camera_info

    def stitching(self):
        """结合单应性矩阵计算拼接图像,然后将mask的信息保存到配置文件中去

        Args:
            cap_dict (dict): 四个camera的cap, key为camera_id
            camera_info_dict (dict): 四个camera的camera_info, key为camera_id
        """
        print("*******start stitching*******")
        front_cap = self.front_cap
        left_cap = self.left_cap
        right_cap = self.right_cap
        back_cap = self.back_cap
        front_frame = front_cap.get_bird_view_frame()
        left_frame = left_cap.get_bird_view_frame()
        right_frame = right_cap.get_bird_view_frame()
        back_frame = back_cap.get_bird_view_frame()
        # 1. 计算mask

        mask_size = (front_frame.shape[1], front_frame.shape[0])
        print("mask_size:", mask_size)
        (front_mask_points, left_mask_points, right_mask_points, back_mask_points) = calculate_cameras_mask(
            front_frame=front_frame, left_frame=left_frame, right_frame=right_frame, back_frame=back_frame, mask_size=mask_size
        )

        front_mask = self._generate_camera_mask(front_mask_points, mask_size)
        left_mask = self._generate_camera_mask(left_mask_points, mask_size)
        right_mask = self._generate_camera_mask(right_mask_points, mask_size)
        back_mask = self._generate_camera_mask(back_mask_points, mask_size)

        # mask frame
        front_frame = cv2.bitwise_and(front_frame, front_frame, mask=front_mask)
        left_frame = cv2.bitwise_and(left_frame, left_frame, mask=left_mask)
        right_frame = cv2.bitwise_and(right_frame, right_frame, mask=right_mask)
        back_frame = cv2.bitwise_and(back_frame, back_frame, mask=back_mask)

        # add frame
        frame = cv2.add(front_frame, left_frame)
        frame = cv2.add(frame, right_frame)
        frame = cv2.add(frame, back_frame)

        frame = cv2.resize(frame, (500, 690))

        cv2.imshow("stitching frame", frame)
        if cv2.waitKey(0) & 0xFF == ord("q"):
            cv2.destroyAllWindows()

        mask_points_dict = {}
        mask_points_dict["/camera/front_wild"] = front_mask_points
        mask_points_dict["/camera/left_wild"] = left_mask_points
        mask_points_dict["/camera/right_wild"] = right_mask_points
        mask_points_dict["/camera/back_wild"] = back_mask_points

        return mask_points_dict

    def show_result(self):
        # 一切都是完备的，直接拼接
        print("*******show result*******")
        front_cap = self.front_cap
        left_cap = self.left_cap
        right_cap = self.right_cap
        back_cap = self.back_cap

        # todo
        # tone_mapping test

        while True:
            front_frame = front_cap.get_bird_view_frame_with_mask()
            left_frame = left_cap.get_bird_view_frame_with_mask()
            right_frame = right_cap.get_bird_view_frame_with_mask()
            back_frame = back_cap.get_bird_view_frame_with_mask()

            # add frame
            frame = cv2.add(front_frame, left_frame)
            frame = cv2.add(frame, right_frame)
            frame = cv2.add(frame, back_frame)

            cv2.imshow("result", frame)
            key = cv2.waitKey(1)
            if key == ord("q"):
                cv2.destroyAllWindows()
                break

    def _on_scale_x_trackbar(self, scale_x_progress):
        self.current_camera_info.scale_xy[0] = scale_x_progress / 100

    def _on_scale_y_trackbar(self, scale_y_progress):
        self.current_camera_info.scale_xy[1] = scale_y_progress / 100

    def _on_shift_x_trackbar(self, shift_x_progress):
        self.current_camera_info.shift_xy[0] = shift_x_progress - 300

    def _on_shift_y_trackbar(self, shift_y_progress):
        self.current_camera_info.shift_xy[1] = shift_y_progress - 300

    def _calculate_homography_matrix(self, src_points):
        if self.current_camera_id == "/camera/front_wild":
            dst_points = self.current_cc_info.dst_points_dict["front"]
        elif self.current_camera_id == "/camera/left_wild":
            dst_points = self.current_cc_info.dst_points_dict["left"]
        elif self.current_camera_id == "/camera/right_wild":
            dst_points = self.current_cc_info.dst_points_dict["right"]
        elif self.current_camera_id == "/camera/back_wild":
            dst_points = self.current_cc_info.dst_points_dict["back"]
        else:
            raise Exception("current_camera_id error")
        src_points = np.float32(src_points).reshape((-1, 1, 2))
        dst_points = np.float32(dst_points).reshape((-1, 1, 2))
        return cv2.getPerspectiveTransform(src_points, dst_points)

    def _generate_camera_mask(self, mask_points, mask_size):
        mask = np.zeros((mask_size[1], mask_size[0]), np.uint8)
        mask_points = np.array(mask_points, np.int32)
        mask_points = mask_points.reshape((-1, 1, 2))
        mask = cv2.fillPoly(mask, [mask_points], 255)
        return mask
