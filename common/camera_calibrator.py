"""
Author: windzu
Date: 2022-05-26 09:10:13
LastEditTime: 2022-05-26 11:47:51
LastEditors: windzu
Description: 
FilePath: /windzu_tools/common/camera_calibrator.py
@Copyright (C) 2021-2022 plusgo Company Limited. All rights reserved.
@Licensed under the Apache License, Version 2.0 (the License)
"""
from enum import Enum
import cv2
import numpy as np
import sys

# local
sys.path.append("../")
from common.enum_common import Patterns, CameraModel, InfoCheckLevel
from common.camera_common import CalibratorFunctionFlags


class HandleResult(object):
    """
    存储handleframe的返回结果
    """

    def __init__(self):
        self.params = None


class CameraCalibrator:
    """相机标定的基类"""

    def __init__(self, chessboard_info):
        # public
        ######## 相机标定需要的基础信息
        self.calibrated = False
        self.chessboard_info = chessboard_info
        self.calibrator_function_flags = CalibratorFunctionFlags()

    @staticmethod
    def _undistort_points(src, camera_info):
        """角点去畸变
        Params:
            src (np.ndarray): 原始图像中检测到的角点
            camera_info (CameraInfo): 相机信息
        Returns:
            np.ndarray: 去畸变后的角点
        """
        if camera_info.camera_model == CameraModel.PINHOLE:
            return cv2.undistortPoints(
                src,
                camera_info.intrinsics_matrix,
                camera_info.distortion_coefficients,
                np.eye(3, 3),
                camera_info.intrinsics_matrix,
            )
        elif camera_info.camera_model == CameraModel.FISHEYE:
            new_mat = camera_info.intrinsics_matrix.copy()
            new_mat[0, 0] *= camera_info.scale_xy[0]
            new_mat[1, 1] *= camera_info.scale_xy[1]
            new_mat[0, 2] += camera_info.shift_xy[0]
            new_mat[1, 2] += camera_info.shift_xy[1]
            return cv2.fisheye.undistortPoints(
                src,
                camera_info.intrinsics_matrix,
                camera_info.distortion_coefficients,
                np.eye(3, 3),
                new_mat,
            )
        return src
