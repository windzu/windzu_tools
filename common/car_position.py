"""
Author: windzu
Date: 2022-04-11 16:14:41
LastEditTime: 2022-04-18 10:50:07
LastEditors: windzu
Description: 
FilePath: /windzu_ws/src/tools/common/car_position.py
@Copyright (C) 2021-2022 plusgo Company Limited. All rights reserved.
@Licensed under the Apache License, Version 2.0 (the License)
"""


class CarPosition:
    def __init__(self, latitude, longtitude, x, y, z, yaw):
        self.latitude = latitude
        self.longtitude = longtitude
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw  # enu下正东为0,正北为pi/2,rad
