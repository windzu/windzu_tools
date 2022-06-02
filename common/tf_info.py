"""
Author: windzu
Date: 2022-06-01 09:07:48
LastEditTime: 2022-06-01 13:45:53
LastEditors: windzu
Description: 
FilePath: /windzu_tools/common/tf_info.py
@Copyright (C) 2021-2022 plusgo Company Limited. All rights reserved.
@Licensed under the Apache License, Version 2.0 (the License)
"""
"""
Author: windzu
Date: 2022-06-01 00:21:17
LastEditTime: 2022-06-01 00:40:36
LastEditors: windzu
Description: 
FilePath: /windzu_tools/common/tf_info.py
@Copyright (C) 2021-2022 plusgo Company Limited. All rights reserved.
@Licensed under the Apache License, Version 2.0 (the License)
"""
import sys
from tkinter.messagebox import NO
import cv2
import numpy as np
from scipy.spatial.transform import Rotation

# local
sys.path.append("../")
from common.enum_common import CameraModel, CameraInfoCheckLevel, FrameInputMode


class TFInfo:
    def __init__(self, tf_id, raw_tf_config=None):
        """存储tf的基本信息,包括旋转和平移

        Args:
            tf_id (_type_): 两个frame的组合id:parent_frame_id_to_child_frame_id
            raw_tf_config (_type_): tf的原始信息
        """
        self.raw_tf_config = raw_tf_config
        # 基础信息
        self.tf_id = tf_id
        self.parent_frame_id = None
        self.child_frame_id = None
        self.rotation = None  # 四元数
        self.translation = None  # 平移向量

        self.serialize_tf_config()

    def serialize_tf_config(self):
        """从原始字典格式解析配置信息，包含旋转和平移
        旋转:原始格式为len为9的list,转换为numpy.array的3x3旋转矩阵 4元数,然后转换为
        平移:原始格式为len为3的list,转换为numpy.array的3x1平移向量
        """

        def serialize_rotation(raw_tf_config):
            """解析旋转矩阵R为四元数元组"""
            if "rotation" in raw_tf_config.keys() and raw_tf_config["rotation"] is not None:
                rotation = raw_tf_config["rotation"]
                rotation = np.array(rotation, dtype=np.float32)
                rotation = rotation.reshape(
                    4,
                )
                return rotation
            else:
                raise Exception("[ tf_info ] : raw_tf_config中没有rotation")

        def serialize_translation(raw_tf_config):
            if "translation" in raw_tf_config.keys() and raw_tf_config["translation"] is not None:
                translation = raw_tf_config["translation"]
                translation = np.array(translation, dtype=np.float32)
                translation = translation.reshape(
                    3,
                )
                return translation
            else:
                raise Exception("[ tf_info ] : raw_tf_config中没有translation")

        self.parent_frame_id = self.tf_id.split("_to_")[0]
        self.child_frame_id = self.tf_id.split("_to_")[1]

        if self.raw_tf_config is None:
            print("[ TFInfo ]  raw_tf_config is None")
            return
        self.rotation = serialize_rotation(self.raw_tf_config)
        self.translation = serialize_translation(self.raw_tf_config)

    def deserialize_tf_config(self):
        if self.raw_tf_config is None:
            self.raw_tf_config = dict()
        self.raw_tf_config["rotation"] = self.rotation.flatten().tolist()
        self.raw_tf_config["translation"] = self.translation.flatten().tolist()
        return self.raw_tf_config

    def echo(self):
        print("[ TFInfo ] parent_frame_id: ", self.parent_frame_id)
        print("[ TFInfo ] child_frame_id: ", self.child_frame_id)
        print("[ TFInfo ] rotation: ", self.rotation)
        print("[ TFInfo ] translation: ", self.translation)
