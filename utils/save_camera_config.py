"""
Author: windzu
Date: 2022-04-14 18:07:08
LastEditTime: 2022-04-14 18:07:09
LastEditors: windzu
Description: 
FilePath: /windzu_ws/src/tools/utils/save_camera_config.py
@Copyright (C) 2021-2022 plusgo Company Limited. All rights reserved.
@Licensed under the Apache License, Version 2.0 (the License)
"""
import numpy as np
import yaml


def save_camera_config(camera_config_path, camera_id, camera_info_dict, camera_raw_config_dict):
    """
    保存camera配置
    Args:
        camera_config_path (str): camera配置路径
        camera_id (str): 修改了配置的camera id
        camera_info_list (dict): camera info list
        camera_raw_config_dict (dict): camera原始格式配置字典
    """

    def serialize_intrinsics_matrix(intrinsics_matrix):
        if intrinsics_matrix is None or len(intrinsics_matrix) != 3:
            return None
        else:
            return intrinsics_matrix.flatten().tolist()

    def serialize_distortion_coefficients(distortion_coefficients):
        if distortion_coefficients is None or len(distortion_coefficients) < 4:
            return None
        else:
            return distortion_coefficients.flatten().tolist()

    def serialize_mask_size(mask_size):
        if mask_size is None:
            return None
        else:
            return np.array(mask_size).astype(np.int32).flatten().tolist()

    def serialize_mask(mask_ploygon_corner_points):
        """mask的角点序列化
        """
        if mask_ploygon_corner_points is None:
            return None
        else:
            return mask_ploygon_corner_points

    def serialize_homography_matrix(homography_matrix):
        if homography_matrix is None or len(homography_matrix) != 3:
            return None
        else:
            return homography_matrix.flatten().tolist()

    def serialize_scale_xy(scale_xy):
        if scale_xy is None:
            return None
        else:
            return scale_xy

    def serialize_shift_xy(shift_xy):
        if shift_xy is None:
            return None
        else:
            return shift_xy

    def serialize_imu_to_camera_translation_xyz(imu_to_camera_translation_xyz):
        if imu_to_camera_translation_xyz is None:
            return None
        else:
            return imu_to_camera_translation_xyz

    def serialize_imu_to_camera_rotation_offset_xyz(imu_to_camera_rotation_offset_xyz):
        if imu_to_camera_rotation_offset_xyz is None:
            return None
        else:
            return imu_to_camera_rotation_offset_xyz

    def serialize_camera_info(camera_info, camera_config):
        # 内参矩阵 np.array -> list
        intrinsics_matrix = serialize_intrinsics_matrix(camera_info.intrinsics_matrix)
        if intrinsics_matrix is not None:
            camera_config["intrinsics_matrix"] = intrinsics_matrix
        # 畸变系数 np.array -> list
        distortion_coefficients = serialize_distortion_coefficients(camera_info.distortion_coefficients)
        if distortion_coefficients is not None:
            camera_config["distortion_coefficients"] = distortion_coefficients
        # 环视的mask np.array -> list
        mask_size = serialize_mask_size(camera_info.mask_size)
        if mask_size is not None:
            camera_config["mask_size"] = mask_size
        # 环视的mask的多边形角点 np.array -> list(将mask_ploygon_corner_points转换为list)
        mask_ploygon_corner_points = serialize_mask(camera_info.mask_ploygon_corner_points)
        if mask_ploygon_corner_points is not None:
            camera_config["mask"] = mask_ploygon_corner_points  # mask存储点
        # 环视的单应矩阵 np.array -> list
        homography_matrix = serialize_homography_matrix(camera_info.homography_matrix)
        if homography_matrix is not None:
            camera_config["homography_matrix"] = homography_matrix
        # 鱼眼scale_xy np.array -> list
        scale_xy = serialize_scale_xy(camera_info.scale_xy)
        if scale_xy is not None:
            camera_config["scale_xy"] = scale_xy
        # 鱼眼shift_xy np.array -> list
        shift_xy = serialize_shift_xy(camera_info.shift_xy)
        if shift_xy is not None:
            camera_config["shift_xy"] = shift_xy
        # imu to camera 外参
        imu_to_camera_translation_xyz = serialize_imu_to_camera_translation_xyz(camera_info.imu_to_camera_translation_xyz)
        if imu_to_camera_translation_xyz is not None:
            camera_config["imu_to_camera_translation_xyz"] = imu_to_camera_translation_xyz
        # imu to camera 旋转
        imu_to_camera_rotation_offset_xyz = serialize_imu_to_camera_rotation_offset_xyz(camera_info.imu_to_camera_rotation_offset_xyz)
        if imu_to_camera_rotation_offset_xyz is not None:
            camera_config["imu_to_camera_rotation_offset_xyz"] = imu_to_camera_rotation_offset_xyz

    # judge camera_id is key in camera_id_list
    if camera_id not in camera_raw_config_dict.keys():
        print("camera_id is not in camera_id_list")
        return

    camera_info = camera_info_dict[camera_id]
    camera_config = camera_raw_config_dict[camera_id]
    serialize_camera_info(camera_info, camera_config)

    with open(camera_config_path, "w") as f:
        yaml.dump(camera_raw_config_dict, f, default_flow_style=False)
        return True

