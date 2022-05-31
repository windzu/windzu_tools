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
from common.enum_common import CameraModel, CameraInfoCheckLevel, FrameInputMode


class CameraInfo:
    def __init__(self, camera_id, camera_config):
        """_summary_

        Args:
            camera_info (_type_): 存储相机基础信息的字典。如果是环视，则还包含环视拼接的相关信息。
        """
        # 基础信息
        self.camera_id = camera_id
        self.serialize(camera_config)

        # 环视拼接特有的信息（optinal）

        # 外参信息（optinal），不通过camera_info传入
        # imu to camera 外参
        self.imu_to_camera_translation_xyz = None  # 旋转前三个轴的平移量
        self.imu_to_camera_rotation_offset_xyz = None  # 旋转后三个轴的旋转微调量

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

    def serialize_camera_config(self, camera_config):
        """从原始字典格式解析相机的配置信息"""
        # 解析相机基础信息
        self.serialize_basic_camera_config(camera_config)

        # 解析环视拼接所需信息
        self.serialize_surround_camera_config(camera_config)

    def serialize_basic_camera_config(self, camera_config):
        """解析相机基础信息,camera_config 的字典信息,来自于直接加载的yaml文件:
            camera_model
            input_mode
            ros_topic
            resolution
            intrinsics_matrix
            distortion_coefficients

        Args:
            camera_config (_type_): camera_config 的字典信息,来自于直接加载的yaml文件
        """

        def prase_camera_model(camera_config):
            camera_model = camera_config["camera_model"]
            if camera_model == "pinhole":
                camera_model = CameraModel.PINHOLE
            elif camera_model == "fisheye":
                camera_model = CameraModel.FISHEYE
            else:
                raise ValueError("camera_model is invalid")
            return camera_model

        def parse_input_mode(camera_config):
            input_mode = camera_config["input_mode"]
            if input_mode == "ros_topic":
                input_mode = FrameInputMode.ROS_TOPIC
            elif input_mode == "device_name":
                input_mode = FrameInputMode.DEVICE_NAME
            else:
                raise ValueError("input_mode is not invalid")
            return input_mode

        def parse_device_name(camera_config):
            if "device_name" in camera_config.keys() and camera_config["device_name"] is not None:
                device_name = camera_config["device_name"]
                return device_name
            else:
                raise ValueError("device_name is not invalid")

        def parse_ros_topic(camera_config):
            if "ros_topic" in camera_config.keys() and camera_config["ros_topic"] is not None:
                ros_topic = camera_config["ros_topic"]
                return ros_topic
            else:
                raise ValueError("ros_topic is invalid")

        def parse_resolution(camera_config):
            if "resolution" in camera_config.keys() and camera_config["resolution"] is not None:
                resolution = camera_config["resolution"]
                return resolution
            else:
                raise ValueError("resolution is invalid")

        def parse_intrinsics_matrix(camera_config):
            if "intrinsics_matrix" in camera_config.keys() and camera_config["intrinsics_matrix"] is not None:
                intrinsics_matrix = camera_config["intrinsics_matrix"]
                intrinsics_matrix = np.array(intrinsics_matrix, dtype=np.float32)
                intrinsics_matrix = intrinsics_matrix.reshape(3, 3)
                return intrinsics_matrix
            else:
                raise ValueError("intrinsics_matrix is invalid")

        def parse_distortion_coefficients(camera_config):
            if (
                "distortion_coefficients" in camera_config.keys()
                and camera_config["distortion_coefficients"] is not None
            ):
                distortion_coefficients = camera_config["distortion_coefficients"]
                distortion_coefficients = np.array(distortion_coefficients, dtype=np.float32)
                return distortion_coefficients
            else:
                raise ValueError("distortion_coefficients is invalid")

        # 相机的基本信息
        self.camera_model = prase_camera_model(camera_config)
        self.input_mode = parse_input_mode(camera_config)
        self.device_name = parse_device_name(camera_config)
        self.ros_topic = parse_ros_topic(camera_config)
        self.resolution = parse_resolution(camera_config)
        self.intrinsics_matrix = parse_intrinsics_matrix(camera_config)
        self.distortion_coefficients = parse_distortion_coefficients(camera_config)

    def serialize_surround_camera_config(self, camera_config):
        """解析相机关于环视的信息,camera_config的字典信息,来自于直接加载的yaml文件:
        scale_xy : 图像的缩放比例
        shift_xy : 图像的平移量
        mask_size:环视拼接的mask大小,即拼接图的总大小
        mask:该相机对应的mask区域,是一个四边梯形
        ploygon_corner_points:上述mask区域的四个角点坐标
        homography_matrix:环视相机进行变换的homography矩阵
        """

        def parse_scale_xy(camera_config, camera_model):
            if "scale_xy" in camera_config.keys():
                scale_xy = camera_config["scale_xy"]
                return scale_xy
            elif camera_model == CameraModel.FISHEYE:
                scale_xy = [1, 1]
                return scale_xy
            else:
                return None

        def parse_shift_xy(camera_config, camera_model):
            if "shift_xy" in camera_config.keys():
                shift_xy = camera_config["shift_xy"]
                return shift_xy
            elif camera_model == CameraModel.FISHEYE:
                shift_xy = [0, 0]
                return shift_xy
            else:
                return None

        def parse_mask_size(camera_config):
            if "mask_size" in camera_config.keys() and camera_config["mask_size"] is not None:
                mask_size = camera_config["mask_size"]
                return mask_size
            else:
                return None

        def parse_mask(camera_config, mask_size):
            """将mask的四个角点坐标转换为cv2格式的mask"""
            if mask_size is None:
                return None
            mask = np.zeros((mask_size[1], mask_size[0]), np.uint8)
            if "mask" in camera_config.keys() and camera_config["mask"] is not None:
                mask_polygon_corner_points = camera_config["mask"]
                mask_polygon_corner_points = np.array(mask_polygon_corner_points, np.int32)
                mask_polygon_corner_points = mask_polygon_corner_points.reshape((-1, 1, 2))
                mask = cv2.fillPoly(mask, [mask_polygon_corner_points], 255)
            else:
                mask = None
            return mask

        def parse_ploygon_corner_points(camera_config):
            """将mask的四个角点坐标转换为角点坐标单独存储"""
            if "mask" in camera_config.keys() and camera_config["mask"] is not None:
                mask_polygon_corner_points = camera_config["mask"]
            else:
                mask_polygon_corner_points = None
            return mask_polygon_corner_points

        def parse_homography_matrix(camera_config):
            if "homography_matrix" in camera_config.keys() and camera_config["homography_matrix"] is not None:
                homography_matrix = camera_config["homography_matrix"]
                homography_matrix = np.array(homography_matrix, dtype=np.float32)
                homography_matrix = homography_matrix.reshape(3, 3)
                return homography_matrix
            else:
                return None

        self.scale_xy = parse_scale_xy(camera_config, self.camera_model)
        self.shift_xy = parse_shift_xy(camera_config, self.camera_model)
        self.mask_size = parse_mask_size(camera_config)
        self.mask = parse_mask(camera_config, self.mask_size)
        self.mask_polygon_corner_points = parse_ploygon_corner_points(camera_config)
        self.homography_matrix = parse_homography_matrix(camera_config)

    def deserialize(self):
        pass

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
