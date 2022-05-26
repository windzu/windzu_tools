"""
Author: windzu
Date: 2022-05-26 09:24:34
LastEditTime: 2022-05-26 09:24:49
LastEditors: windzu
Description: 
FilePath: /windzu_tools/common/chessboard_info.py
@Copyright (C) 2021-2022 plusgo Company Limited. All rights reserved.
@Licensed under the Apache License, Version 2.0 (the License)
"""
import cv2
import numpy as np
import sys

# local
sys.path.append("../")
from common.enum_common import Patterns


class ChessboardInfo:
    """存储标定所使用棋盘格标定板信息,棋盘格类型的标定板计划支持opencv中的两种
    1. 普通的棋盘格
    2. charuco标定板(在普通棋盘格的基础上增加了角码的标定，可以提供方向信息)(目前尚不支持)
    """

    def __init__(self, pattern=Patterns.CHESSBOARD, n_cols=0, n_rows=0, square_size=0.0):
        self.pattern = pattern
        self.n_cols = n_cols
        self.n_rows = n_rows
        self.square_size = square_size
        self.BOARD = np.array(
            [
                [(j * self.square_size, i * self.square_size, 0.0)]
                for i in range(self.n_rows)
                for j in range(self.n_cols)
            ],
            dtype=np.float32,
        )
