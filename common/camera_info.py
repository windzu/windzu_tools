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
import sys
import cv2
import numpy as np

# local
sys.path.append("../")
from common.enum_common import CameraModel, CameraInfoCheckLevel, FrameInputMode


class CameraInfo:
    def __init__(self, camera_id, raw_camera_config):
        """存储相机的基本信息,包括相机的基本信息,环视拼接所需信息

        Args:
            camera_id (_type_): 相机的id
            camera_config (_type_): 相机的配置信息,来自于直接加载的yaml文件
        """
        self.raw_camera_config = raw_camera_config
        # 基础信息
        self.camera_id = camera_id
        self.camera_model = None
        self.input_mode = None
        self.device_name = None
        self.ros_topic = None
        self.resolution = None
        self.intrinsics_matrix = None
        self.distortion_coefficients = None

        # 环视拼接特有信息
        self.scale_xy = None
        self.shift_xy = None
        self.mask_size = None
        self.mask_corners = None
        self.mask = None  # 不存在于yaml文件中，需要根据mask_corners计算生成
        self.homography_matrix = None

        # 解析字典,构造相机信息
        self.serialize_camera_config(raw_camera_config)

        # 暂时不考虑让camera info存储任何与外参相关的信息
        # # 相机坐标系与其他坐标系外参信息，不在构造类时传入
        # ## imu坐标系到该相机坐标系的外参
        # self.imu_to_camera_rotation = None
        # self.imu_to_camera_translation_xyz = None  # 旋转前三个轴的平移量
        # self.imu_to_camera_rotation_offset_xyz = None  # 旋转后三个轴的旋转微调量
        # ## 某个相机坐标系到该相机坐标系的外参
        # ## 某个lidar坐标系到该相机坐标系的外参

        # 单目标定结果
        self.reproj_err = None  # 重投影误差
        self.rvecs = None  # 标定板坐标系到相机坐标系的旋转矩阵
        self.tvecs = None  # 标定板坐标系到相机坐标系的平移矩阵
        self.ok = False

        # 根据相机内参和畸变计算的map
        self.map1 = None
        self.map2 = None

    def serialize_camera_config(self, raw_camera_config):
        """从原始字典格式解析相机的配置信息"""
        # 解析相机基础信息
        self.__serialize_basic_camera_config(raw_camera_config)

        # 解析环视拼接所需信息
        self.__serialize_surround_camera_config(raw_camera_config)

    def __serialize_basic_camera_config(self, raw_camera_config):
        """解析相机基础信息,camera_config 的字典信息,来自于直接加载的yaml文件:
        camera_model : 相机的成像模型,目前有pinehole和fisheye
        input_mode : 相机的输入模式,目前有ros_topic和device_name
        device_name : 如果输入采用的是device_name,则需要指定有效的设备名称
        ros_topic : 如果输入采用是rostopic,则需要提供有效的ros_topic
        resolution : 相机的分辨率,格式为(width, height)
        intrinsics_matrix : 内参矩阵 (可选参数,默认为None)
        distortion_coefficients : 畸变系数 (可选参数,默认为None)
        """

        def serialize_camera_model(camera_config):
            camera_model = camera_config["camera_model"]
            if camera_model == "pinhole":
                camera_model = CameraModel.PINHOLE
            elif camera_model == "fisheye":
                camera_model = CameraModel.FISHEYE
            else:
                raise ValueError("camera_model is invalid")
            return camera_model

        def serialize_input_mode(camera_config):
            input_mode = camera_config["input_mode"]
            if input_mode == "ros_topic":
                input_mode = FrameInputMode.ROS_TOPIC
            elif input_mode == "device_name":
                input_mode = FrameInputMode.DEVICE_NAME
            else:
                raise ValueError("input_mode is not invalid")
            return input_mode

        def serialize_device_name(camera_config):
            if "device_name" in camera_config.keys() and camera_config["device_name"] is not None:
                device_name = camera_config["device_name"]
                return device_name
            else:
                raise ValueError("device_name is not invalid")

        def serialize_ros_topic(camera_config):
            if "ros_topic" in camera_config.keys() and camera_config["ros_topic"] is not None:
                ros_topic = camera_config["ros_topic"]
                return ros_topic
            else:
                raise ValueError("ros_topic is invalid")

        def serialize_resolution(camera_config):
            if "resolution" in camera_config.keys() and camera_config["resolution"] is not None:
                resolution = camera_config["resolution"]
                return resolution
            else:
                raise ValueError("resolution is invalid")

        def serialize_intrinsics_matrix(camera_config):
            if "intrinsics_matrix" in camera_config.keys() and camera_config["intrinsics_matrix"] is not None:
                intrinsics_matrix = camera_config["intrinsics_matrix"]
                intrinsics_matrix = np.array(intrinsics_matrix, dtype=np.float32)
                intrinsics_matrix = intrinsics_matrix.reshape(3, 3)
                return intrinsics_matrix
            else:
                return None

        def serialize_distortion_coefficients(camera_config):
            if (
                "distortion_coefficients" in camera_config.keys()
                and camera_config["distortion_coefficients"] is not None
            ):
                distortion_coefficients = camera_config["distortion_coefficients"]
                distortion_coefficients = np.array(distortion_coefficients, dtype=np.float32)
                return distortion_coefficients
            else:
                return None

        # 相机的基本信息
        self.camera_model = serialize_camera_model(raw_camera_config)
        self.input_mode = serialize_input_mode(raw_camera_config)
        self.device_name = serialize_device_name(raw_camera_config)
        self.ros_topic = serialize_ros_topic(raw_camera_config)
        self.resolution = serialize_resolution(raw_camera_config)
        self.intrinsics_matrix = serialize_intrinsics_matrix(raw_camera_config)
        self.distortion_coefficients = serialize_distortion_coefficients(raw_camera_config)

    def __serialize_surround_camera_config(self, raw_camera_config):
        """解析相机关于环视的信息,camera_config的字典信息,来自于直接加载的yaml文件:
        scale_xy : 图像的缩放比例
        shift_xy : 图像的平移量
        mask_size:环视拼接的mask大小,即拼接图的总大小
        ploygon_corner_points:上述mask区域的四个角点坐标
        mask:该相机对应的mask区域,是一个四边梯形(通过ploygon_corner_points生成的,原始yaml中不存储该信息)
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

        def parse_mask_corners(camera_config):
            """将mask的四个角点坐标转换为角点坐标单独存储"""
            if "mask_corners" in camera_config.keys() and camera_config["mask_corners"] is not None:
                mask_corners = camera_config["mask_corners"]
            else:
                mask_corners = None
            return mask_corners

        def parse_mask(camera_config, mask_size):
            """将mask的四个角点mask_corners换为cv2格式的mask"""
            if mask_size is None:
                return None
            mask = np.zeros((mask_size[1], mask_size[0]), np.uint8)
            if "mask_corners" in camera_config.keys() and camera_config["mask_corners"] is not None:
                mask_corners = camera_config["mask_corners"]
                mask_corners = np.array(mask_corners, np.int32)
                mask_corners = mask_corners.reshape((-1, 1, 2))
                mask = cv2.fillPoly(mask, [mask_corners], 255)
            else:
                mask = None
            return mask

        def parse_homography_matrix(camera_config):
            if "homography_matrix" in camera_config.keys() and camera_config["homography_matrix"] is not None:
                homography_matrix = camera_config["homography_matrix"]
                homography_matrix = np.array(homography_matrix, dtype=np.float32)
                homography_matrix = homography_matrix.reshape(3, 3)
                return homography_matrix
            else:
                return None

        self.scale_xy = parse_scale_xy(raw_camera_config, self.camera_model)
        self.shift_xy = parse_shift_xy(raw_camera_config, self.camera_model)
        self.mask_size = parse_mask_size(raw_camera_config)
        self.mask_polygon_corner_points = parse_mask_corners(raw_camera_config)
        self.mask = parse_mask(raw_camera_config, self.mask_size)
        self.homography_matrix = parse_homography_matrix(raw_camera_config)

    def deserialize_camera_config(self):
        self.__deserialize_basic_camera_config()
        self.__deserialize_surround_camera_config()
        return self.raw_camera_config

    def __deserialize_basic_camera_config(self):
        """将相机的基本信息反序列化为适合yaml存储的基本格式,包括如下几个信息
        intrinsics_matrix:相机内参可能会更新
        distortion_coefficients:相机畸变参数可能会更新
        """

        def deserialize_intrinsics_matrix(intrinsics_matrix):
            if intrinsics_matrix is None or len(intrinsics_matrix) != 3:
                return None
            else:
                return intrinsics_matrix.flatten().tolist()

        def deserialize_distortion_coefficients(distortion_coefficients):
            if distortion_coefficients is None or len(distortion_coefficients) < 4:
                return None
            else:
                return distortion_coefficients.flatten().tolist()

        # 如果有有效的内参矩阵,则反序列化为yaml格式；否则，将此项删除
        intrinsics_matrix = deserialize_intrinsics_matrix(self.intrinsics_matrix)
        if intrinsics_matrix is not None:
            self.raw_camera_config["intrinsics_matrix"] = intrinsics_matrix
        else:
            if "intrinsics_matrix" in self.raw_camera_config.keys():
                self.raw_camera_config.pop("intrinsics_matrix")

        # 如果有有效的畸变系数,则反序列化为yaml格式；否则，将此项删除
        distortion_coefficients = deserialize_distortion_coefficients(self.distortion_coefficients)
        if distortion_coefficients is not None:
            self.raw_camera_config["distortion_coefficients"] = distortion_coefficients
        else:
            if "distortion_coefficients" in self.raw_camera_config.keys():
                self.raw_camera_config.pop("distortion_coefficients")

    def __deserialize_surround_camera_config(self):
        """将相机的关于环视拼接附加信息反序列化为适合yaml存储的基本格式,包括如下几个信息
        scale_xy:相机的缩放比例
        shift_xy:相机的平移量
        mask_size:相机的mask大小可能会更新
        mask:相机的mask可能会更新
        homography_matrix:相机的homography矩阵可能会更新
        """

        def deserialize_scale_xy(scale_xy):
            if scale_xy is None:
                return None
            else:
                return scale_xy

        def deserialize_shift_xy(shift_xy):
            if shift_xy is None:
                return None
            else:
                return shift_xy

        def deserialize_mask_size(mask_size):
            if mask_size is None:
                return None
            else:
                return np.array(mask_size).astype(np.int32).flatten().tolist()

        def deserialize_parse_mask_corners(parse_mask_corners):
            """mask的角点序列化"""
            if parse_mask_corners is None:
                return None
            else:
                return parse_mask_corners

        def deserialize_homography_matrix(homography_matrix):
            if homography_matrix is None or len(homography_matrix) != 3:
                return None
            else:
                return homography_matrix.flatten().tolist()

        scale_xy = deserialize_scale_xy(self.scale_xy)
        if scale_xy is not None:
            self.raw_camera_config["scale_xy"] = scale_xy
        else:
            if "scale_xy" in self.raw_camera_config.keys():
                self.raw_camera_config.pop("scale_xy")

        shift_xy = deserialize_shift_xy(self.shift_xy)
        if shift_xy is not None:
            self.raw_camera_config["shift_xy"] = shift_xy
        else:
            if "shift_xy" in self.raw_camera_config.keys():
                self.raw_camera_config.pop("shift_xy")

        mask_size = deserialize_mask_size(self.mask_size)
        if mask_size is not None:
            self.raw_camera_config["mask_size"] = mask_size
        else:
            if "mask_size" in self.raw_camera_config.keys():
                self.raw_camera_config.pop("mask_size")

        mask_corners = deserialize_parse_mask_corners(self.mask_polygon_corner_points)
        if mask_corners is not None:
            self.raw_camera_config["mask_corners"] = mask_corners
        else:
            if "mask_corners" in self.raw_camera_config.keys():
                self.raw_camera_config.pop("mask_corners")

        homography_matrix = deserialize_homography_matrix(self.homography_matrix)
        if homography_matrix is not None:
            self.raw_camera_config["homography_matrix"] = homography_matrix
        else:
            if "homography_matrix" in self.raw_camera_config.keys():
                self.raw_camera_config.pop("homography_matrix")

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
            self.__surround_view_check()

    def __base_check(self):
        """检查相机模型、分辨率"""
        if not isinstance(self.camera_model, CameraModel):
            raise Exception("camera_model is not CameraModel")
        if self.resolution is None or len(self.resolution) != 2:
            raise Exception("resolution is not 2-element list")

    def __advanced_check(self):
        """检查相机内参数、畸变系数
        如果是鱼眼,还要检查scale_xy和shift_xy
        """
        if self.intrinsics_matrix is None or len(self.intrinsics_matrix) != 3:
            raise Exception("intrinsics_matrix is not 3-element list")
        if self.distortion_coefficients is None:
            raise Exception("distortion_coefficients is None")
        if self.camera_model is CameraModel.FISHEYE:
            # 如果是鱼眼相机，则一定要检测scale_xy和shift_xy
            # 如果不存在则使用默认值
            if self.scale_xy is None:
                print("scale_xy is None , will use default value")
                self.shift_xy = [0, 0]
            elif len(self.scale_xy) != 2:
                raise Exception("scale_xy is not 2-element list")

            if self.shift_xy is None:
                print("shift_xy is None , will use default value")
                self.shift_xy = [0, 0]
            elif len(self.shift_xy) != 2:
                raise Exception("shift_xy is not 2-element list")

    def __completed_check(self):
        if self.map1 is None or self.map2 is None:
            print("map1 or map2 is none, will calculate it")
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

    def __surround_view_check(self):
        if self.camera_model is CameraModel.FISHEYE:
            if self.mask_size is None:
                raise Exception("mask_size is None")
            if self.mask is None:
                raise Exception("mask is None")
            if self.homography_matrix is None:
                raise Exception("homography_matrix is None")
        else:
            raise Exception("camera_model is not FISHEYE")

    def echo(self):
        print("[ print camera_info ] : ")
        print("     camera_model: ", self.camera_model)
        print("     resolution: ", self.resolution)
        print("     intrinsics_matrix: ", self.intrinsics_matrix)
        print("     distortion_coefficients: ", self.distortion_coefficients)
        print("     scale_xy: ", self.scale_xy)
        print("     shift_xy: ", self.shift_xy)
        print("     mask_size: ", self.mask_size)
        print("     mask_corners: ", self.mask_corners)
        print("     homography_matrix: ", self.homography_matrix)
        print("     reproj_err:", self.reproj_err)
        print("     ok:", self.ok)
