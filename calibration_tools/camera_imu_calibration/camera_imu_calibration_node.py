"""
Author: windzu
Date: 2022-04-09 15:29:29
LastEditTime: 2022-04-09 15:29:30
LastEditors: windzu
Description: 
FilePath: /tools/calibration_tools/camera_imu_calibration/camera_imu_calibration_node.py
@Copyright (C) 2021-2022 plusgo Company Limited. All rights reserved.
@Licensed under the Apache License, Version 2.0 (the License)
"""

import rospy
import cv2
import sys
import numpy as np
import math
import scipy
from scipy.linalg import expm, norm
import tf

# import ros transform

# local
sys.path.append("../../")
from common.enum_common import CameraInfoCheckLevel


class CameraIMUCalibrationNode:
    """相机和imu的外参手动标定"""

    def __init__(self, get_frame, get_rtk, camera_info, tl_info, tf_info):
        self.get_frame = get_frame
        self.get_rtk = get_rtk
        self.camera_info = camera_info
        self.tl_info = tl_info
        self.tf_info = tf_info

        # 存储滑动条所定义的平移和旋转量
        self.base_link_to_camera_translation_x = 0
        self.base_link_to_camera_translation_y = 0
        self.base_link_to_camera_translation_z = 0
        self.base_link_to_camera_rotation_x_offset = 0  # 围绕x轴旋转的偏移量（弧度制）
        self.base_link_to_camera_rotation_y_offset = 0  # 围绕y轴旋转的偏移量（弧度制）
        self.base_link_to_camera_rotation_z_offset = 0  # 围绕z轴旋转的偏移量（弧度制）

        # 因为opencv 的 trackbar不支持设置负的范围和setp size，所以需要自己根据精度需求做一下后处理
        # 需求如下：
        #   平移：
        #       imu三个方向的平移范围最大值 ： 10m (考虑到卡车的长度，imu在车尾，相机在车头) [-10m,10m]->[0m,20m]
        #       imu三个方向的平移step size ： 1cm
        #       平移范围[0m,20m]->[0mm,2000mm]
        #   旋转：
        #       imu三个方向的旋转偏移量最大值 ： 45度  [-45,45]->[0,90]
        #       imu三个方向的旋转偏移量step size ： 0.1度
        #       平移范围[0,90]->[0,900]
        # NOTE : 后处理的时候需要做对应的缩放
        self.__translation_slider_max = 2000
        self.__translation_slider_step_size = 100  # 缩放尺度为100倍，2000/100=20m
        self.__rotation_offset_slider_max = 900
        self.__rotation_offset_slider_step_size = 10  # 缩放100倍，900/10=90度

    def start(self):
        print("*******start collect sample*******")
        window_name = "collecting sample"

        # 如果是初次标定，则没有R和T的信息，需要初始化一下，初始化默认先围绕
        if self.tf_info.rotation is None or self.tf_info.translation is None:
            self.tf_info.rotation = np.array([0, 0, 0, 1])
            self.tf_info.translation = np.array([0, 0, 0])
        self.__opencv_trackbar_init(window_name)

        while True:

            frame = self.get_frame.read()
            car_position = self.get_rtk.get_car_position()

            ret, center, roi_rect_pts = self.calculate_roi(mode="manual")
            if ret is True:
                cv2.circle(frame, center, 10, (0, 255, 0), 4)
                cv2.rectangle(frame, roi_rect_pts[0], roi_rect_pts[1], (255, 0, 0), 2)
            cv2.imshow(window_name, frame)

            key = cv2.waitKey(30)
            if key == ord("q"):
                cv2.destroyAllWindows()
                return
            if key == 13:  # enter
                cv2.destroyAllWindows()
                return

    def show_result(self):
        print("*******start show result*******")
        window_name = "show result"

        while True:
            frame = self.get_frame.read()

            ret, center, roi_rect_pts = self.calculate_roi(mode="show_mode")
            if ret is True:
                cv2.circle(frame, center, 10, (0, 255, 0), 4)
                cv2.rectangle(frame, roi_rect_pts[0], roi_rect_pts[1], (255, 0, 0), 2)
            cv2.imshow(window_name, frame)

            key = cv2.waitKey(30)
            if key == ord("q"):
                cv2.destroyAllWindows()
                return

    def __opencv_trackbar_init(self, window_name):
        """通过opencv创建滑动条"""
        window_name = window_name
        cv2.namedWindow(window_name)

        # trackbar_name = "Alpha x %d" % alpha_slider_max
        imu_to_camera_translation_x_progress_trackbar_name = " translation_x "
        imu_to_camera_translation_y_progress_trackbar_name = " translation_y "
        imu_to_camera_translation_z_progress_trackbar_name = " translation_z "
        imu_to_camera_rotation_offset_x_progress_trackbar_name = " rotation_offset_x "
        imu_to_camera_rotation_offset_y_progress_trackbar_name = " rotation_offset_y "
        imu_to_camera_rotation_offset_z_progress_trackbar_name = " rotation_offset_z "

        # 创建opencv滑动条
        cv2.createTrackbar(
            imu_to_camera_translation_x_progress_trackbar_name,
            window_name,
            0,
            self.__translation_slider_max,
            self.__on_translation_x_trackbar,
        )
        cv2.createTrackbar(
            imu_to_camera_translation_y_progress_trackbar_name,
            window_name,
            0,
            self.__translation_slider_max,
            self.__on_translation_y_trackbar,
        )
        cv2.createTrackbar(
            imu_to_camera_translation_z_progress_trackbar_name,
            window_name,
            0,
            self.__translation_slider_max,
            self.__on_translation_z_trackbar,
        )
        cv2.createTrackbar(
            imu_to_camera_rotation_offset_x_progress_trackbar_name,
            window_name,
            0,
            self.__rotation_offset_slider_max,
            self.__on_rotation_offset_x_trackbar,
        )
        cv2.createTrackbar(
            imu_to_camera_rotation_offset_y_progress_trackbar_name,
            window_name,
            0,
            self.__rotation_offset_slider_max,
            self.__on_rotation_offset_y_trackbar,
        )
        cv2.createTrackbar(
            imu_to_camera_rotation_offset_z_progress_trackbar_name,
            window_name,
            0,
            self.__rotation_offset_slider_max,
            self.__on_rotation_offset_z_trackbar,
        )

        # 滑动条设置初始默认值
        translation_x = int(self.__translation_slider_max / 2)
        translation_y = int(self.__translation_slider_max / 2)
        translation_z = int(self.__translation_slider_max / 2)
        rotation_offset_x = int(self.__rotation_offset_slider_max / 2)
        rotation_offset_y = int(self.__rotation_offset_slider_max / 2)
        rotation_offset_z = int(self.__rotation_offset_slider_max / 2)
        cv2.setTrackbarPos(imu_to_camera_translation_x_progress_trackbar_name, window_name, translation_x)
        cv2.setTrackbarPos(imu_to_camera_translation_y_progress_trackbar_name, window_name, translation_y)
        cv2.setTrackbarPos(imu_to_camera_translation_z_progress_trackbar_name, window_name, translation_z)
        cv2.setTrackbarPos(imu_to_camera_rotation_offset_x_progress_trackbar_name, window_name, rotation_offset_x)
        cv2.setTrackbarPos(imu_to_camera_rotation_offset_y_progress_trackbar_name, window_name, rotation_offset_y)
        cv2.setTrackbarPos(imu_to_camera_rotation_offset_z_progress_trackbar_name, window_name, rotation_offset_z)

    def __on_translation_x_trackbar(self, progress):
        translation_x = (progress - (self.__translation_slider_max / 2)) / self.__translation_slider_step_size
        self.base_link_to_camera_translation_x = translation_x

    def __on_translation_y_trackbar(self, progress):
        translation_y = (progress - (self.__translation_slider_max / 2)) / self.__translation_slider_step_size
        self.base_link_to_camera_translation_y = translation_y

    def __on_translation_z_trackbar(self, progress):
        translation_z = (progress - (self.__translation_slider_max / 2)) / self.__translation_slider_step_size
        self.base_link_to_camera_translation_z = translation_z

    def __on_rotation_offset_x_trackbar(self, progress):
        # 通过滑动条设置的值，计算出滑动条设置的值对应的弧度值
        rotation_x_offset = (
            (progress - (self.__rotation_offset_slider_max / 2))
            * (math.pi / 180)
            / self.__rotation_offset_slider_step_size
        )
        # 默认围绕y轴先旋转
        self.base_link_to_camera_rotation_x_offset = rotation_x_offset

    def __on_rotation_offset_y_trackbar(self, progress):
        rotation_offset_y = (
            (progress - (self.__rotation_offset_slider_max / 2))
            * (math.pi / 180)
            / self.__rotation_offset_slider_step_size
        )
        self.base_link_to_camera_rotation_y_offset = rotation_offset_y

    def __on_rotation_offset_z_trackbar(self, progress):
        rotation_offset_z = (
            (progress - (self.__rotation_offset_slider_max / 2))
            * (math.pi / 180)
            / self.__rotation_offset_slider_step_size
        )
        self.base_link_to_camera_rotation_z_offset = rotation_offset_z

    def calculate_roi(self, mode):
        """计算世界坐标点在相机坐标系内的位置，需要提供如下信息
        map坐标系到base_link的tf信息
        base_link到相机坐标系的tf信息
        相机内参
        目标点的map坐标系下的位置(默认utm坐标)
        Args:
            mode(str): 计算采用的参数模式
                show_mode:根据tf_info的参数来提供base_link_to_camera_transform
                calculate_mode:根据滑动条设置的参数来提供base_link_to_camera_transform
        """

        def calculate_map_to_base_link_transform(car_position):
            # 平移矩阵
            T = np.array(
                [[1, 0, 0, car_position.x], [0, 1, 0, car_position.y], [0, 0, 1, car_position.z], [0, 0, 0, 1]]
            )
            # 旋转矩阵
            z_axis = [0, 0, 1]
            R = self.rotate_mat(z_axis, car_position.yaw)
            map_to_base_link_transform = np.eye(4)
            map_to_base_link_transform[:3, :3] = R
            map_to_base_link_transform[:3, 3] = T[:3, 3]

            return map_to_base_link_transform

        def calculate_base_link_to_camera_transform_from_manual(
            translation_x, translation_y, translation_z, rotation_x_offset, rotation_y_offset, rotation_z_offset
        ):
            """通过滑动条设置的值,计算出该值对应的base_link到相机坐标系的tf信息
            这个计算中因为要旋转多次,所以有一个旋转顺序,旋转顺序按照xyz三个轴的方向如下:
            1. 先旋转x轴,逆方向90度
            2. 再旋转y轴,0度
            3. 再旋转z轴,逆方向90度

            Args:
                translation_x (_type_): _description_
                translation_y (_type_): _description_
                translation_z (_type_): _description_
                rotation_x_offset (_type_): 相机坐标系下围绕x轴角度偏转:对应base_link下的y轴的逆方向
                rotation_y_offset (_type_): 相机坐标系下围绕y轴角度偏转:对应base_link下的z轴的逆方向
                rotation_z_offset (_type_): 相机坐标系下围绕x轴角度偏转:对应base_link下的x轴的正方向
            """
            rotation_x = -math.pi / 2 + rotation_z_offset
            rotation_y = 0 - rotation_x_offset
            rotation_z = -math.pi / 2 - rotation_y_offset

            # 平移矩阵
            T = np.array([[1, 0, 0, translation_x], [0, 1, 0, translation_y], [0, 0, 1, translation_z], [0, 0, 0, 1]])
            # 旋转矩阵
            x_axis = [1, 0, 0]
            y_axis = [0, 1, 0]
            z_axis = [0, 0, 1]
            x_R = self.rotate_mat(x_axis, rotation_x)
            y_R = self.rotate_mat(y_axis, rotation_y)
            z_R = self.rotate_mat(z_axis, rotation_z)
            R = z_R.dot(y_R).dot(x_R)
            base_link_to_camera_transform = np.eye(4)
            base_link_to_camera_transform[:3, :3] = R
            base_link_to_camera_transform[:3, 3] = T[:3, 3]

            return base_link_to_camera_transform

        def calculate_base_link_to_camera_transform_from_tf_info(tf_info):
            rotation = tf_info.rotation
            translation = tf_info.translation
            # 四元数转换为旋转矩阵
            R = tf.transformations.quaternion_matrix(rotation)
            base_link_to_camera_transform = np.eye(4)
            base_link_to_camera_transform[:3, :3] = R[:3, :3]
            base_link_to_camera_transform[:3, 3] = translation
            return base_link_to_camera_transform

        def revise_roi_bbox(targt, padding_ratio, camera_info):
            """根据投影计算出的红绿灯点,并指定一个padding宽度,计算出一个不超过图像的bbox,并返回center和roi的bbox"""
            if targt[2] < 0:
                return False, None, None
            roi_center = (int(targt[0]), int(targt[1]))
            img_width = camera_info.resolution[0]
            img_height = camera_info.resolution[1]
            padding = int(img_width * padding_ratio)
            if roi_center[0] < 0 or roi_center[0] > img_width or roi_center[1] < 0 or roi_center[1] > img_height:
                return False, None, None
            pt1_x = roi_center[0] - padding
            pt1_y = roi_center[1] - padding
            pt2_x = roi_center[0] + padding
            pt2_y = roi_center[1] + padding
            if pt1_x < 0:
                pt1_x = 0
            if pt1_y < 0:
                pt1_y = 0
            if pt2_x > img_width - 1:
                pt2_x = img_width - 1
            if pt2_y > img_height - 1:
                pt2_y = img_height - 1

            pt1 = (pt1_x, pt1_y)
            pt2 = (pt2_x, pt2_y)
            return True, roi_center, (pt1, pt2)

        tl_info = self.tl_info
        car_position = self.get_rtk.get_car_position()
        camera_info = self.camera_info

        print("yaw:", car_position.yaw)

        map_to_base_link_transform = calculate_map_to_base_link_transform(car_position)
        if mode == "show_mode":
            base_link_to_camera_transform = calculate_base_link_to_camera_transform_from_tf_info(self.tf_info)
        else:
            base_link_to_camera_transform = calculate_base_link_to_camera_transform_from_manual(
                translation_x=self.base_link_to_camera_translation_x,
                translation_y=self.base_link_to_camera_translation_y,
                translation_z=self.base_link_to_camera_translation_z,
                rotation_x_offset=self.base_link_to_camera_rotation_x_offset,
                rotation_y_offset=self.base_link_to_camera_rotation_y_offset,
                rotation_z_offset=self.base_link_to_camera_rotation_z_offset,
            )
            self.update_tf_info(base_link_to_camera_transform)
        print("base_link_to_camera_transform:", base_link_to_camera_transform)
        camera_matrix = np.concatenate((camera_info.intrinsics_matrix, np.zeros((3, 1))), axis=1)
        tl_position = np.array([[tl_info.x], [tl_info.y], [tl_info.z], [1]])
        target = self.map_to_pixel(
            map_to_base_link_transform, base_link_to_camera_transform, camera_matrix, tl_position
        )

        return revise_roi_bbox(target, 0.2, camera_info)

    def update_tf_info(self, transform):
        """将变换矩阵转换为4元数和平移向量存储在tf_info中"""
        translation_vector = transform[:3, 3]
        quaternion = tf.transformations.quaternion_from_matrix(transform)  # 需要transform为4x4矩阵
        self.tf_info.rotation = quaternion
        self.tf_info.translation = translation_vector

    @staticmethod
    def rotate_mat(axis, radian, dim=3):
        """计算按照某个轴旋转弧度的旋转矩阵
        Args:
            axis: 旋转轴, 可以是x, y, z轴,对三个轴的表示分别为 x_axis = [1, 0, 0] y_axis = [0, 1, 0] z_axis = [0, 0, 1]
            radian: 旋转弧度
            dim: 输出旋转矩阵的维度,默认为3, 可选4
        Returns:
            旋转矩阵: np.array
        """
        rot_matrix = expm(np.cross(np.eye(3), axis / norm(axis) * radian))
        if dim == 3:
            return rot_matrix
        elif dim == 4:
            m1 = np.concatenate((rot_matrix, np.zeros((1, 3))), axis=0)
            m2 = np.concatenate((m1, np.zeros((4, 1))), axis=1)
            m2[3, 3] = 1
            return m2

    @staticmethod
    def map_to_pixel(map_to_base_link_transform, base_link_to_camera_transform, camera_matrix, tl_position):
        """计算世界坐标点在相机的像素坐标系内的位置，需要提供如下信息
        map坐标系到base_link的tf信息
        base_link到相机坐标系的tf信息
        相机内参
        目标点的map坐标系下的位置(默认utm坐标)

        Args:
            map_to_base_link_transform(np.array): map坐标系到base_link的tf信息
            base_link_to_camera_transform(np.array): base_link到相机坐标系的tf信息
            camera_matrix(np.array): 相机内参(3x4)
            tl_position(np.array): 目标点的map坐标系下的位置(默认utm坐标)(4x1)
        """

        target = np.linalg.inv(map_to_base_link_transform).dot(tl_position)
        target = np.linalg.inv(base_link_to_camera_transform).dot(target)
        target = camera_matrix.dot(target)
        target = target / target[2]

        return target
