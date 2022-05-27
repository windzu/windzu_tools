"""
Author: windzu
Date: 2022-04-14 16:40:05
LastEditTime: 2022-04-14 16:40:06
LastEditors: windzu
Description: 
FilePath: /windzu_ws/src/tools/utils/parse_camera_config.py
@Copyright (C) 2021-2022 plusgo Company Limited. All rights reserved.
@Licensed under the Apache License, Version 2.0 (the License)
"""

import yaml
import numpy as np
import cv2
import sys

sys.path.append("../")
from common.enum_common import CameraModel, FrameInputMode
from common.camera_info import CameraInfo


def parse_camera_config(config_path):
    """读取配置文件，获得各个相机的参数
    只有camera_id存储与list中,其他的存储为字典,字典中的key为相机id,value为相机参数
    Return:
        camera_id_list (list): 相机配置编号
        camera_config_dict (dict): 解析格式后的配置,key为参数名,value为dict,其key为相机id,value为相机参数
        camera_raw_config_dict (dict): yaml直接load的原始格式,用于更新参数并存储
    """
    camera_config_path = config_path
    with open(camera_config_path, "r") as f:
        camera_raw_config_dict = yaml.load(f)

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
            raise ValueError("input_mode is not supported")
        return input_mode

    def parse_device_name(camera_config):
        return camera_config["device_name"]

    def parse_ros_topic(camera_config):
        return camera_config["ros_topic"]

    def parse_resolution(camera_config):
        return camera_config["resolution"]

    def parse_intrinsics_matrix(camera_config):
        if "intrinsics_matrix" in camera_config.keys() and camera_config["intrinsics_matrix"] is not None:
            intrinsics_matrix = camera_config["intrinsics_matrix"]
            intrinsics_matrix = np.array(intrinsics_matrix, dtype=np.float32)
            intrinsics_matrix = intrinsics_matrix.reshape(3, 3)
            return intrinsics_matrix
        else:
            return None

    def parse_distortion_coefficients(camera_config):
        if "distortion_coefficients" in camera_config.keys() and camera_config["distortion_coefficients"] is not None:
            distortion_coefficients = camera_config["distortion_coefficients"]
            distortion_coefficients = np.array(distortion_coefficients, dtype=np.float32)
            return distortion_coefficients
        else:
            return None

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
        """将mask的四个角点坐标转换为cv2的mask

        Args:
            camera_config (_type_): _description_
            mask_size (_type_): _description_

        Returns:
            _type_: _description_
        """
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

    def parse_imu_to_camera_translation(camera_config):
        if "imu_to_camera_translation_xyz" in camera_config.keys():
            imu_to_camera_translation_list = camera_config["imu_to_camera_translation_xyz"]
            return imu_to_camera_translation_list
        else:
            return None

    def parse_imu_to_camera_rotation_offset(camera_config):
        if "imu_to_camera_rotation_offset_xyz" in camera_config.keys():
            imu_to_camera_rotation_offset_list = camera_config["imu_to_camera_rotation_offset_xyz"]
            return imu_to_camera_rotation_offset_list
        else:
            return None

    camera_id_list = []

    # 迭代camera_raw_config_dict,解析相机参数
    # key 就是 camera_id
    # value 就是 没一个camera_id对应的具体camera_config
    camera_info_dict = {}
    for key, value in camera_raw_config_dict.items():
        # **单独针对环视拼接的参数解析**
        # 环视拼接需要预先生成mask，即CameraInfo中的mask，这是一张“图像”
        # 但是在将mask参数写入配置文件的时候是以四个角点坐标的形式写入的，那么生成mask和保存mask的时候存在两个问题
        # * 生成mask时 ： 不仅仅需要mask的角点信息，还需要整个mask的图像尺寸，这通过传入mask_size来解决
        # * 保存mask时 ： 将mask从图像格式提取出四个角点坐标存储太复杂，引入mask_ploygon_corner_points来单独存储点坐标
        mask_size = parse_mask_size(value)
        mask = parse_mask(value, mask_size)
        mask_ploygon_corner_points = parse_ploygon_corner_points(value)

        # **因为鱼眼相机需要加载scale_xy和shift_xy参数，当配置文件中没有这两个参数时，需要自己生成**
        # 所以需要根据相机的模型来判断是否需要生成scale_xy和shift_xy
        camera_model = prase_camera_model(value)

        camera_info = CameraInfo(
            input_mode=parse_input_mode(value),
            device_name=parse_device_name(value),
            ros_topic=parse_ros_topic(value),
            camera_model=camera_model,
            resolution=parse_resolution(value),
            intrinsics_matrix=parse_intrinsics_matrix(value),
            distortion_coefficients=parse_distortion_coefficients(value),
            scale_xy=parse_scale_xy(value, camera_model),
            shift_xy=parse_shift_xy(value, camera_model),
            homography_matrix=parse_homography_matrix(value),
            imu_to_camera_translation_xyz=parse_imu_to_camera_translation(value),
            imu_to_camera_rotation_offset_xyz=parse_imu_to_camera_rotation_offset(value),
            mask_size=mask_size,
            mask=mask,
            mask_ploygon_corner_points=mask_ploygon_corner_points,
        )
        camera_id_list.append(key)
        camera_info_dict[key] = camera_info

    return camera_id_list, camera_info_dict, camera_raw_config_dict
