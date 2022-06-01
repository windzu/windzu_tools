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

# import ros transform

# local
sys.path.append("../../")
from common.enum_common import InfoCheckLevel


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
        if self.tf_info.R is None or self.tf_info.T is None:
            self.tf_info.R = np.eye(3)

        self.__opencv_trackbar_init(window_name)

        while True:

            frame = self.get_frame.read()
            tl_info = self.tl_info
            car_position = self.get_rtk.get_car_position()
            camera_info = self.camera_info

            ret, center, roi_rect_pts = self.__calculate_roi(
                tl_info=tl_info, car_position=car_position, camera_info=camera_info
            )
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
        info_check_level = InfoCheckLevel.IMU_TO_CAMERA
        self.camera_info.info_check(info_check_level)
        print("*******start show result*******")
        window_name = "show result"

        while True:
            frame = self.get_frame.read()
            tl_info = self.tl_info
            car_position = self.get_rtk.get_car_position()
            camera_info = self.camera_info

            ret, center, roi_rect_pts = self.__calculate_roi(
                tl_info=tl_info, car_position=car_position, camera_info=camera_info
            )
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
        self.tf_info.T[0] = translation_x

    def __on_translation_y_trackbar(self, progress):
        translation_y = (progress - (self.__translation_slider_max / 2)) / self.__translation_slider_step_size
        self.tf_info.T[1] = translation_y

    def __on_translation_z_trackbar(self, progress):
        translation_z = (progress - (self.__translation_slider_max / 2)) / self.__translation_slider_step_size
        self.tf_info.T[2] = translation_z

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

    @staticmethod
    def __map_to_pixel(map_to_base_link_transform, base_link_to_camera_transform, camera_matrix, tl_position):
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
        return target[0], target[1]

    def calculate_roi(self, car_position, tl_info, camera_info):
        """计算世界坐标点在相机坐标系内的位置，需要提供如下信息
        map坐标系到base_link的tf信息
        base_link到相机坐标系的tf信息
        相机内参
        目标点的map坐标系下的位置(默认utm坐标)

        Args:
            tl_info (TLInfo): 存储一个红绿灯的位置、状态等信息
            car_position (CarPosition): 存储一个车的位置、yaw等信息
            camera_info (CameraInfo): 存储一个相机的很多信息，内外参数等
        """

        def calculate_map_to_base_link_transform(car_position):
            # 平移矩阵
            T = np.array(
                [[1, 0, 0, car_position.x], [0, 1, 0, car_position.y], [0, 0, 1, car_position.z], [0, 0, 0, 1]]
            )
            # 旋转矩阵
            z_axis = [0, 0, 1]
            R = rotate_mat(z_axis, car_position.yaw, 4)
            map_to_base_link_transform = np.eye(4)
            map_to_base_link_transform[:3, :3] = R
            map_to_base_link_transform[:3, 3] = T[:3, 3]

            return map_to_base_link_transform

        def calculate_base_link_to_camera_transform(
            translation_x, translation_y, translation_z, rotation_x, rotation_y, rotation_z
        ):
            """通过滑动条设置的值,计算出该值对应的base_link到相机坐标系的tf信息
            这个计算中因为要旋转多次，所以有一个旋转顺序，旋转顺序如下：
            1. 先旋转y轴,正方向90度
            2. 再旋转x轴,逆方向90度
            3. 再旋转z轴,正方向90度

            Args:
                translation_x (_type_): _description_
                translation_y (_type_): _description_
                translation_z (_type_): _description_
                rotation_x (_type_): _description_
                rotation_y (_type_): _description_
                rotation_z (_type_): _description_
            """
            pass

        def revise_roi_bbox(targt, padding_ratio, camera_info):
            """根据投影计算出的红绿灯点,并指定一个padding宽度,计算出一个不超过图像的bbox,并返回center和roi的bbox"""
            if targt[2] < 0:
                return False, None, None
            roi_center = (int(targt[0] / targt[2]), int(targt[1] / targt[2]))
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

        map_to_base_link_transform = calculate_map_to_base_link_transform(car_position)
        base_link_to_camera_transform = np.eye(4)

        x_axis = [1, 0, 0]
        y_axis = [0, 1, 0]
        z_axis = [0, 0, 1]

        # imu坐标到相机坐标的变换
        # 先平移到相机的中心，再围绕其中两个轴旋转，再围绕三个轴微小的旋转
        imu_to_camera_translation_matrix = np.eye(4)
        imu_to_camera_translation_matrix = np.array(
            [
                [1, 0, 0, -camera_info.imu_to_camera_translation_xyz[0]],
                [0, 1, 0, -camera_info.imu_to_camera_translation_xyz[1]],
                [0, 0, 1, -camera_info.imu_to_camera_translation_xyz[2]],
                [0, 0, 0, 1],
            ]
        )
        # 先围绕y轴正方向旋转90度，再围绕z逆方向旋转90度
        imu_to_camera_rotation_matrix = np.dot(rotate_mat(z_axis, math.pi / 2, 4), rotate_mat(y_axis, -math.pi / 2, 4))
        imu_to_camera_tranformation_matrix = np.dot(imu_to_camera_rotation_matrix, imu_to_camera_translation_matrix)
        # 小角度微调
        camera_offset_rotation_matrix = np.eye(4)
        camera_x_offset_rotation_matrix = rotate_mat(x_axis, -camera_info.imu_to_camera_rotation_offset_xyz[0], 4)
        camera_y_offset_rotation_matrix = rotate_mat(y_axis, -camera_info.imu_to_camera_rotation_offset_xyz[1], 4)
        camera_z_offset_rotation_matrix = rotate_mat(z_axis, -camera_info.imu_to_camera_rotation_offset_xyz[2], 4)
        camera_offset_rotation_matrix = np.dot(camera_x_offset_rotation_matrix, camera_y_offset_rotation_matrix)
        camera_offset_rotation_matrix = np.dot(camera_offset_rotation_matrix, camera_z_offset_rotation_matrix)
        # 微调后合并
        imu_to_camera_tranformation_matrix = np.dot(camera_offset_rotation_matrix, imu_to_camera_tranformation_matrix)

        # world to pixel transformation
        intrinsics_matrix = camera_info.intrinsics_matrix
        camera_to_pixel_translation_matrix = np.eye(4)
        camera_to_pixel_translation_matrix = np.concatenate((intrinsics_matrix, np.zeros((1, 3))), axis=0)
        camera_to_pixel_translation_matrix = np.concatenate(
            (camera_to_pixel_translation_matrix, np.zeros((4, 1))), axis=1
        )
        camera_to_pixel_translation_matrix[3, 3] = 1

        world_to_pixel_tranformation_matrix = np.eye(4)
        world_to_camera_tranformation_matrix = np.dot(
            imu_to_camera_tranformation_matrix, world_to_imu_tranformation_matrix
        )
        world_to_pixel_tranformation_matrix = np.dot(
            camera_to_pixel_translation_matrix, world_to_camera_tranformation_matrix
        )

        tl_position = np.array([[tl_info.x], [tl_info.y], [tl_info.z], [1]])
        # debug
        print("tl_position:", tl_position.flatten())
        print("car_position:", car_position.x, car_position.y, car_position.z, car_position.yaw)
        # tl_in_imu_position = np.dot(world_to_imu_tranformation_matrix, tl_position)
        # print("tl_in_imu_position:", tl_in_imu_position)
        tl_in_camera_position = np.dot(world_to_camera_tranformation_matrix, tl_position)
        print("tl_in_camera_position:", tl_in_camera_position)

        targt = np.dot(world_to_pixel_tranformation_matrix, tl_position)
        targt = targt.flatten().tolist()
        padding_ratio = 0.1
        return revise_roi_bbox(targt, padding_ratio, camera_info)

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
