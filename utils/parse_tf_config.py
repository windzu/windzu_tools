"""
Author: windzu
Date: 2022-05-30 10:00:19
LastEditTime: 2022-05-30 13:55:49
LastEditors: windzu
Description: 
FilePath: /windzu_tools/utils/parse_tf_config.py
@Copyright (C) 2021-2022 plusgo Company Limited. All rights reserved.
@Licensed under the Apache License, Version 2.0 (the License)
"""

import yaml
import numpy as np
import cv2
import sys
from scipy.spatial.transform import Rotation

sys.path.append("../")
from common.enum_common import CameraModel, FrameInputMode
from common.camera_info import CameraInfo


def parse_tf_config(config_path):
    """读取tf配置文件"""

    def parse_R(tf_config):
        """解析旋转矩阵R为四元数"""
        if "R" in tf_config.keys() and tf_config["R"] is not None:
            R = tf_config["R"]
            R = np.array(R, dtype=np.float32)
            R = R.reshape(3, 3)
            R = Rotation.from_matrix(R)  # 将tvec转换为四元数
            R = R.as_quat()
            R = tuple(R)  # 再转为元组
            return R
        else:
            return None

    def parse_T(tf_config):
        if "T" in tf_config.keys() and tf_config["T"] is not None:
            T = tf_config["T"]
            T = np.array(T, dtype=np.float32)
            T = T.reshape(
                3,
            )
            T = tuple(T)
            return T
        else:
            return None

    with open(config_path, "r") as f:
        tf_raw_config_dict = yaml.load(f)

    # 迭代camera_raw_config_dict,解析相机参数
    # key 就是 camera_id
    # value 就是 没一个camera_id对应的具体camera_config
    all_tf_info_dict = {}
    for key, value in tf_raw_config_dict.items():
        tf_info_dict = {}
        child_frame_id = key.split("_to_")[0]
        father_frame_id = key.split("_to_")[1]
        tf_info_dict["father_frame_id"] = father_frame_id
        tf_info_dict["child_frame_id"] = child_frame_id
        tf_info_dict["R"] = parse_R(value)
        tf_info_dict["T"] = parse_T(value)

        all_tf_info_dict[key] = tf_info_dict

    return all_tf_info_dict
