"""
Author: windzu
Date: 2022-05-25 21:54:08
LastEditTime: 2022-05-27 00:35:04
LastEditors: windzu
Description: 
FilePath: /windzu_tools/common/enum_common.py
@Copyright (C) 2021-2022 plusgo Company Limited. All rights reserved.
@Licensed under the Apache License, Version 2.0 (the License)
"""
from enum import Enum

# Supported camera models
class CameraModel(Enum):
    PINHOLE = 0
    FISHEYE = 1


class FrameInputMode(Enum):
    ROS_TOPIC = 0
    DEVICE_NAME = 1  # linux /dev/video* or windows video*


# Supported calibration patterns
class Patterns(Enum):
    CHESSBOARD = 0
    CHARUCO = 1
    CC = 2  # Calibration cloth,目前只有一个标定布,就叫CC,以后有新的图案在区别标定布名称


class CameraInfoCheckLevel(Enum):
    """相机信息信息检查级别"""

    # ！！！ 标准检查模式 ！！！#
    BASE = 0  # camera_model resolution
    ADVANCED = 1  # intrinsics_matrix distortion_coefficients *scale_xy *shift_xy
    COMPLETED = 2  # map1 map2

    # ！！！ 特定场景检查模式 ！！！#
    ## 环视拼接检查级别 ##
    SURROUND_SPECIAL = 3
    ## camera与imu外参标定检查级别 ##
    IMU_TO_CAMERA = 4
    ## 双目外参标定检查级别 ##
    STEREO_CALIBRATION = 5  # 双目标定检查级别
