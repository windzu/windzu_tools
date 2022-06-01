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
        self.R = None
        self.T = None

        # 解析字典,构造相机信息
        self.serialize_tf_config()

    def serialize_tf_config(self):
        """从原始字典格式解析配置信息，包含旋转和平移
        旋转:原始格式为len为9的list,转换为numpy.array的3x3旋转矩阵 4元数,然后转换为
        平移:原始格式为len为3的list,转换为numpy.array的3x1平移向量
        """

        def serialize_R(raw_tf_config):
            """解析旋转矩阵R为四元数元组"""
            if "R" in raw_tf_config.keys() and raw_tf_config["R"] is not None:
                R = raw_tf_config["R"]
                R = np.array(R, dtype=np.float32)
                R = R.reshape(3, 3)
                return R
            else:
                return None

        def serialize_T(raw_tf_config):
            if "T" in raw_tf_config.keys() and raw_tf_config["T"] is not None:
                T = raw_tf_config["T"]
                T = np.array(T, dtype=np.float32)
                T = T.reshape(
                    3,
                )
                return T
            else:
                raise Exception("T is not in raw_tf_config")

        self.parent_frame_id = self.tf_id.split("_to_")[0]
        self.child_frame_id = self.tf_id.split("_to_")[1]

        raw_tf_config = self.raw_tf_config
        if raw_tf_config is None:
            print("raw_tf_config is None")
            return
        self.R = serialize_R(raw_tf_config)
        self.T = serialize_T(raw_tf_config)

    def deserialize_tf_config(self):
        if self.raw_tf_config is None:
            self.raw_tf_config = dict()
        self.raw_tf_config["R"] = self.R.flatten().tolist()
        self.raw_tf_config["T"] = self.T.flatten().tolist()
        return self.raw_tf_config

    def echo(self):
        print("[ tf_info ] : ")
        print("     parent_frame_id: ", self.parent_frame_id)
        print("     child_frame_id: ", self.child_frame_id)
        print("     R: ", self.R)
        print("     T: ", self.T)
