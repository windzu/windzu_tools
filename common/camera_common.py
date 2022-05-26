"""
Author: windzu
Date: 2022-05-25 15:24:57
LastEditTime: 2022-05-26 09:30:38
LastEditors: windzu
Description: 
FilePath: /windzu_tools/common/camera_common.py
@Copyright (C) 2021-2022 plusgo Company Limited. All rights reserved.
@Licensed under the Apache License, Version 2.0 (the License)
"""

import cv2
import numpy as np
import sys


class CalibratorFunctionFlags:
    """设定在标定过程中常用函数的flags , 很多flags默认与非默认的区别是非常大的,要结合实际情况来选择"""

    def __init__(
        self,
        cv2_calibrateCamera_flags=0,
        cv2_fisheye_calibrate_flags=cv2.fisheye.CALIB_FIX_SKEW | cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC,
        cv2_findChessboardCorners_flags=cv2.CALIB_CB_ADAPTIVE_THRESH
        | cv2.CALIB_CB_NORMALIZE_IMAGE
        | cv2.CALIB_CB_FAST_CHECK,
    ):
        self.cv2_calibrateCamera_flags = cv2_calibrateCamera_flags
        self.cv2_fisheye_calibrate_flags = cv2_fisheye_calibrate_flags
        self.cv2_findChessboardCorners_flags = cv2_findChessboardCorners_flags


class CalibratorTargetThreshold:
    """设置标定过程中，指标需要满足的阈值，一般情况下都是使用默认值"""

    def __init__(self, chessboard_max_speed=-1, metrics_ranges=[0.7, 0.7, 0.1, 0.3], metrics_min_distance=0.2):
        """初始化指标阈值

        Args:
            chessboard_max_speed (float): 对标定板移动速度的要求. 当值小于0时表示不考虑移动速度,Defaults to -1.
            param_ranges (list, optional): _description_. Defaults to [0.7, 0.7, 0.4, 0.5].
            param_min_distance (float, optional): corner指标空间中,样本间的最小曼哈顿距离. Defaults to 0.2.
        """
        self.chessboard_max_speed = chessboard_max_speed
        self.metrics_ranges = metrics_ranges
        self.metrics_min_distance = metrics_min_distance
