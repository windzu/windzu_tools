"""
Author: windzu
Date: 2022-05-30 10:00:19
LastEditTime: 2022-05-31 16:52:32
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
    """读取tf配置文件
    将旋转矩阵读取为四元数元组(x,x,x,x)
    将平移向量读取为元组(x,y,z)
    Args:
        config_path (str): tf配置文件路径
    Returns:
        all_tf_info: 所有tf_info解析后的格式
        all_raw_tf_info: 所有tf_info原始格式,是yaml直接读取的格式

    """

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
        all_raw_tf_info = yaml.load(f)

    # 迭代camera_raw_config_dict,解析相机参数
    # key 就是 camera_id
    # value 就是 没一个camera_id对应的具体camera_config
    all_tf_info = {}
    for key, value in all_raw_tf_info.items():
        tf_info = {}
        child_frame_id = key.split("_to_")[0]
        father_frame_id = key.split("_to_")[1]
        tf_info["father_frame_id"] = father_frame_id
        tf_info["child_frame_id"] = child_frame_id
        tf_info["R"] = parse_R(value)
        tf_info["T"] = parse_T(value)

        all_tf_info[key] = tf_info

    return all_tf_info, all_raw_tf_info
