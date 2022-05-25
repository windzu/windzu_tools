"""
Author: windzu
Date: 2022-04-15 10:00:04
LastEditTime: 2022-04-16 15:13:48
LastEditors: windzu
Description: 
FilePath: /windzu_ws/src/tools/common/enum_common.py
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


class InfoCheckLevel(Enum):
    BASE = 0
    ADVANCED = 1
    COMPLETED = 2
    SURROUND_SPECIAL = 3  # 专门用于环视拼接的特殊检查级别
    IMU_TO_CAMERA = 4
