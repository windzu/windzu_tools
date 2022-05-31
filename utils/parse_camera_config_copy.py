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


def parse_camera_config(camera_config_path):
    """读取配置文件，获得各个相机的参数
    只有camera_id存储与list中,其他的存储为字典,字典中的key为相机id,value为相机参数
    Args:
        camera_config_path (str): 相机配置文件路径
    Return:
        all_camera_info (dict): 所有camera_info解析后的格式
        all_raw_camera_info (dict): 所有camera_info原始格式,是yaml直接读取的格式
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
        if "distortion_coefficients" in camera_config.keys() and camera_config["distortion_coefficients"] is not None:
            distortion_coefficients = camera_config["distortion_coefficients"]
            distortion_coefficients = np.array(distortion_coefficients, dtype=np.float32)
            return distortion_coefficients
        else:
            raise ValueError("distortion_coefficients is invalid")

    with open(camera_config_path, "r") as f:
        all_raw_camera_info = yaml.load(f)

    all_camera_info = {}
    for key, value in all_raw_camera_info.items():
        camera_info = {}
        camera_info["camera_id"] = key
        camera_info["camera_model"] = prase_camera_model(value)
        camera_info["input_mode"] = parse_input_mode(value)
        camera_info["device_name"] = parse_device_name(value)
        camera_info["ros_topic"] = parse_ros_topic(value)
        camera_info["resolution"] = parse_resolution(value)
        camera_info["intrinsics_matrix"] = parse_intrinsics_matrix(value)
        camera_info["distortion_coefficients"] = parse_distortion_coefficients(value)
        all_camera_info[key] = camera_info

    return all_camera_info, all_raw_camera_info
