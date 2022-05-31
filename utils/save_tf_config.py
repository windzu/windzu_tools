"""
Author: windzu
Date: 2022-05-31 16:45:36
LastEditTime: 2022-05-31 17:07:39
LastEditors: windzu
Description: 
FilePath: /windzu_tools/utils/save_tf_config.py
@Copyright (C) 2021-2022 plusgo Company Limited. All rights reserved.
@Licensed under the Apache License, Version 2.0 (the License)
"""
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
from scipy.spatial.transform import Rotation


def save_tf_config(father_frame_id, child_frame_id, R, T, all_tf_info, all_raw_tf_info, tf_config_path):
    """
    保存外参配置，如果之前存在同名外参，则覆盖;如果不存在同名外参，则添加
    Args:
        father_frame_id (str): tf树的父节点frame_id
        child_frame_id (str): tf树的子节点frame_id
        R (np.array): 旋转矩阵
        T (np.array): 平移矩阵
        all_tf_info (dict): tf配置经解析过的配置字典
        all_raw_tf_info (dict): tf原始格式配置字典
        tf_config_path (str): tf参数文件存储路径
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

    tf_id = father_frame_id + "_to_" + child_frame_id

    # 如果之前存在同名外参，则覆盖（all_tf_info，all_raw_tf_info信息保持同步）
    if tf_id in all_tf_info.keys():
        all_tf_info.pop(tf_id)
        all_raw_tf_info.pop(tf_id)

    all_raw_tf_info[tf_id] = {"R": R.flatten().tolist(), "T": T.flatten().tolist()}
    all_tf_info[tf_id] = {"R": parse_R(all_raw_tf_info[tf_id]), "T": parse_T(all_raw_tf_info[tf_id])}

    with open(tf_config_path, "w") as f:
        yaml.dump(all_raw_tf_info, f, default_flow_style=False)
        return True
