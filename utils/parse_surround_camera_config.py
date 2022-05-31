"""
Author: windzu
Date: 2022-05-31 17:32:01
LastEditTime: 2022-05-31 17:32:15
LastEditors: windzu
Description: 
FilePath: /windzu_tools/utils/parse_surround_camera_config.py
@Copyright (C) 2021-2022 plusgo Company Limited. All rights reserved.
@Licensed under the Apache License, Version 2.0 (the License)
"""
import sys

# local
sys.path.append("../")

from utils.parse_camera_config import parse_surround_camera_config


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


                    # **单独针对环视拼接的参数解析**
        # 环视拼接需要预先生成mask，即CameraInfo中的mask，这是一张“图像”
        # 但是在将mask参数写入配置文件的时候是以四个角点坐标的形式写入的，那么生成mask和保存mask的时候存在两个问题
        # * 生成mask时 ： 不仅仅需要mask的角点信息，还需要整个mask的图像尺寸，这通过传入mask_size来解决
        # * 保存mask时 ： 将mask从图像格式提取出四个角点坐标存储太复杂，引入mask_ploygon_corner_points来单独存储点坐标
        mask_size = parse_mask_size(value)
        mask = parse_mask(value, mask_size)
        mask_ploygon_corner_points = parse_ploygon_corner_points(value)