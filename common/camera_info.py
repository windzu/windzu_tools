"""
Author: windzu
Date: 2022-04-14 17:30:30
LastEditTime: 2022-04-14 17:30:31
LastEditors: windzu
Description: 
FilePath: /windzu_ws/src/tools/common/camera_info.py
@Copyright (C) 2021-2022 plusgo Company Limited. All rights reserved.
@Licensed under the Apache License, Version 2.0 (the License)
"""
from logging import raiseExceptions
import sys
import cv2
import numpy as np

# local
sys.path.append("../")
from common.enum_common import CameraModel, CameraInfoCheckLevel


class CameraInfo:
    def __init__(
        self,
        camera_id,
        input_mode,
        device_name,
        ros_topic,
        camera_model,
        resolution,
        intrinsics_matrix=None,
        distortion_coefficients=None,
        scale_xy=None,
        shift_xy=None,
        mask_size=None,
        mask=None,
        mask_ploygon_corner_points=None,
        homography_matrix=None,
        imu_to_camera_translation_xyz=None,
        imu_to_camera_rotation_offset_xyz=None,
    ):
        # 确定一下相机图像的输入方式
        self.camera_id = camera_id
        self.input_mode = input_mode
        self.device_name = device_name
        self.ros_topic = ros_topic
        # 在标定伊始，相机的模型和分辨率都是必然已知的(与相机配置文件内容一致)
        self.camera_model = camera_model
        self.resolution = resolution

        # 在标定完成后，最终需要保存的内容(与相机配置文件内容一致)
        self.intrinsics_matrix = intrinsics_matrix
        self.distortion_coefficients = distortion_coefficients
        self.scale_xy = scale_xy
        self.shift_xy = shift_xy
        self.mask_size = mask_size
        self.mask = mask
        self.mask_ploygon_corner_points = mask_ploygon_corner_points
        self.homography_matrix = homography_matrix

        # imu to camera 外参
        self.imu_to_camera_translation_xyz = imu_to_camera_translation_xyz  # 旋转前三个轴的平移量
        self.imu_to_camera_rotation_offset_xyz = imu_to_camera_rotation_offset_xyz  # 旋转后三个轴的旋转微调量

        # 标定过程中临时变量
        ## 外参旋转和平移量
        self.rvecs = None
        self.tvecs = None
        # map
        self.map1 = None
        self.map2 = None
        # 状态量
        self.reproj_err = None
        self.ok = False

    def info_check(self, info_check_level):
        """对camera_info的完整性进行检查,如果camera_info中的相关信息不完整,则返回False,检查分几个等级
        1. BASE: 基础检查,只检查camera_model 和 resolution
            - camera_model
            - resolution
        2. ADVANCED: 检查相机内参和畸变系数,如果是鱼眼还需要检查scale_xy和shift_xy
            - intrinsics_matrix
            - distortion_coefficients
            * scale_xy
            * shift_xy
        3. COMPLETED : 完全的完备性检查,包括上述所有检查,并且如果map没有计算,还会将map计算。
        一个标定完内参的相机必须得能完成这个检查！！
            - map1 , map2
        4. SURROUND_SPECIAL : 专门用于检查用于环视的检查,包括上述所有检查,还要检查mask_size mask homography_matrix
            - mask_size
            - mask
            - homography_matrix
        """

        if info_check_level == CameraInfoCheckLevel.BASE:
            self.__base_check()
        elif info_check_level == CameraInfoCheckLevel.ADVANCED:
            self.__base_check()
            self.__advanced_check()
        elif info_check_level == CameraInfoCheckLevel.COMPLETED:
            self.__base_check()
            self.__advanced_check()
            self.__completed_check()
        elif info_check_level == CameraInfoCheckLevel.SURROUND_SPECIAL:
            self.__base_check()
            self.__advanced_check()
            self.__completed_check()
            self.__surround_special_check()
        elif info_check_level == CameraInfoCheckLevel.IMU_TO_CAMERA:
            self.__imu_to_camera_check()

    def __base_check(self):
        if not isinstance(self.camera_model, CameraModel):
            raise Exception("camera_model is not CameraModel")
        if self.resolution is None or len(self.resolution) != 2:
            raise Exception("resolution is not 2-element list")

    def __advanced_check(self):
        if self.intrinsics_matrix is None or len(self.intrinsics_matrix) != 3:
            raise Exception("intrinsics_matrix is not 3-element list")
        if self.distortion_coefficients is None:
            raise Exception("distortion_coefficients is None")
        if self.camera_model is CameraModel.FISHEYE:
            # 如果是鱼眼相机，则一定要检测scale_xy和shift_xy
            if self.scale_xy is None or len(self.scale_xy) != 2:
                raise Exception("scale_xy is not 2-element list")
            if self.shift_xy is None or len(self.shift_xy) != 2:
                raise Exception("shift_xy is not 2-element list")

    def __completed_check(self):
        if self.map1 is None or self.map2 is None:
            print("no map1 or map2 , now calculate it")
            if self.camera_model is CameraModel.FISHEYE:
                new_mat = self.intrinsics_matrix.copy()
                new_mat[0, 0] *= self.scale_xy[0]
                new_mat[1, 1] *= self.scale_xy[1]
                new_mat[0, 2] += self.shift_xy[0]
                new_mat[1, 2] += self.shift_xy[1]
                self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(
                    self.intrinsics_matrix,
                    self.distortion_coefficients,
                    np.eye(3, 3),
                    new_mat,
                    self.resolution,
                    cv2.CV_16SC2,
                )
            elif self.camera_model is CameraModel.PINHOLE:
                self.map1, self.map2 = cv2.initUndistortRectifyMap(
                    self.intrinsics_matrix,
                    self.distortion_coefficients,
                    np.eye(3, 3),
                    self.intrinsics_matrix,
                    self.resolution,
                    cv2.CV_16SC2,
                )
            else:
                raise Exception("camera_model is not FISHEYE or PINHOLE")

        def __surround_special_check(self):
            if self.camera_model is CameraModel.FISHEYE:
                if self.mask_size is None:
                    raise Exception("mask_size is None")
                if self.mask is None:
                    raise Exception("mask is None")
                if self.homography_matrix is None:
                    raise Exception("homography_matrix is None")
            else:
                raise Exception("camera_model is not FISHEYE")

    def __surround_special_check(self):
        pass

    def __imu_to_camera_check(self):
        if self.imu_to_camera_translation_xyz is None or len(self.imu_to_camera_translation_xyz) != 3:
            raise Exception("imu_to_camera_translation_xyz is not 3-element list")
        if self.imu_to_camera_rotation_offset_xyz is None or len(self.imu_to_camera_rotation_offset_xyz) != 3:
            raise Exception("imu_to_camera_rotation_xyz is not 3-element list")

    def echo(self):
        print("[ print camera_info ] : ")
        print("     camera_model: ", self.camera_model)
        print("     resolution: ", self.resolution)
        print("     intrinsics_matrix: ", self.intrinsics_matrix)
        print("     distortion_coefficients: ", self.distortion_coefficients)
        print("     scale_xy: ", self.scale_xy)
        print("     shift_xy: ", self.shift_xy)
        print("     mask_size: ", self.mask_size)
        print("     homography_matrix: ", self.homography_matrix)
        print("     mask_ploygon_corner_points: ", self.mask_ploygon_corner_points)
        print("     imu_to_camera_translation_xyz: ", self.imu_to_camera_translation_xyz)
        print("     imu_to_camera_rotation_offset_xyz: ", self.imu_to_camera_rotation_offset_xyz)
        print("     reproj_err:", self.reproj_err)
        print("     ok:", self.ok)
