"""
Author: windzu
Date: 2022-04-11 15:32:29
LastEditTime: 2022-04-14 10:24:46
LastEditors: windzu
Description: 
FilePath: /windzu_ws/src/tools/common/tl_info.py
@Copyright (C) 2021-2022 plusgo Company Limited. All rights reserved.
@Licensed under the Apache License, Version 2.0 (the License)
"""


class TLInfo:
    def __init__(self, tl_id, latitude, longtitude, x, y, z, direction, state=None):
        self.tl_id = tl_id
        self.latitude = latitude
        self.longtitude = longtitude
        self.x = x
        self.y = y
        self.z = z
        self.direction = direction
        self.state = state

