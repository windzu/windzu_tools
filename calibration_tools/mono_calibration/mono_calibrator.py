"""
Author: windzu
Date: 2022-04-09 11:43:10
LastEditTime: 2022-04-15 14:36:56
LastEditors: windzu
Description: 
FilePath: /windzu_ws/src/tools/calibration_tools/mono_calibration/mono_calibrator.py
@Copyright (C) 2021-2022 plusgo Company Limited. All rights reserved.
@Licensed under the Apache License, Version 2.0 (the License)
"""
import cv2
import sys

# local
sys.path.append("../")
from common.enum_common import Patterns
from common.camera_calibrator import CameraCalibrator, HandleResult


class MonoHandleResult(HandleResult):
    def __init__(self):
        HandleResult.__init__(self)
        self.resized_img_with_corners = None
        self.linear_error = -1.0


class MonoCalibrator(CameraCalibrator):
    """
    单目相机标定类,继承自Calibrator类
    此类的设计目标：
        1. 类变量 : 尽量不含有专属的类变量,因为Calibrator类应该包含了一切单目标定需要的信息
        2. 类函数 : 函数应该被设计只是用于单目标定使用,而如果存在与双目标定类似的函数需求,应该尝试通过继承Calibrator类来实现
    """

    def __init__(self, *args, **kwargs):
        super(MonoCalibrator, self).__init__(*args, **kwargs)

    def handle_frame(self, frame):
        """单目图像的处理入口函数，只需要传入图像(因为是实时进行,称之为frame)，即可进行处理，处理目前分两种类型
        1. 如果相机是已经标定过的,则对输入进行校正
        2. 如果相机是未标定过的,则采集图像进行标定

            Returns a MonoDrawable message with the display image and progress info.
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        linear_error = -1

        resized_img, corners, downsampled_corners, ids, board, (x_scale, y_scale) = self._downsample_and_detect(gray)

        # 如果标定是已经完成的,则将角点坐标进行校正、缩放然后绘制在校正缩放后的图像上
        if self.calibrated:
            undistorted_gray = self._remap(gray)
            if x_scale != 1.0 or y_scale != 1.0:
                resized_undistorted_gray = cv2.resize(undistorted_gray, (resized_img.shape[1], resized_img.shape[0]))

            resized_undistorted_img = cv2.cvtColor(resized_undistorted_gray, cv2.COLOR_GRAY2BGR)

            # 角点校正、计算线性误差、缩放、绘制角点
            if corners is not None:
                # Report linear error
                undistorted_corners = self._undistort_points(corners)
                linear_error = self._linear_error(undistorted_corners, ids, self._chessboard_info)
                resized_undistorted_corners = undistorted_corners.copy()
                resized_undistorted_corners[:, :, 0] /= x_scale
                resized_undistorted_corners[:, :, 1] /= y_scale
                cv2.drawChessboardCorners(resized_undistorted_img, (board.n_cols, board.n_rows), resized_undistorted_corners, True)

                # debug
                # print linear_error
                print("linear_error:", linear_error)
            resized_img_with_corners = resized_undistorted_img
        else:
            resized_gray = cv2.cvtColor(resized_img, cv2.COLOR_GRAY2BGR)
            if corners is not None:
                if board.pattern == Patterns.CHESSBOARD:
                    cv2.drawChessboardCorners(resized_gray, (board.n_cols, board.n_rows), downsampled_corners, True)
                else:
                    pass  # TODO Draw other corner types

                # 对本次检测的角点信息与角点数据库中的数据进行比较,如果是符合要求的角点,则加入数据库，反之则丢弃本次检测的角点
                params = self._get_parameters(corners, ids, board, (gray.shape[1], gray.shape[0]))
                if self._is_good_sample(params, corners, ids, self._last_frame_corners, self._last_frame_ids):
                    self._db.append((params, gray))
                    if board.pattern == Patterns.CHESSBOARD:
                        self._good_corners.append((corners, None, board))
                    else:
                        pass
                    print(
                        ("*** Added sample %d, p_x = %.3f, p_y = %.3f, p_size = %.3f, skew = %.3f" % tuple([len(self._db)] + params))
                    )
            resized_img_with_corners = resized_gray

        self._last_frame_corners = corners
        self._last_frame_ids = ids
        mono_handle_result = MonoHandleResult()
        mono_handle_result.resized_img_with_corners = resized_img_with_corners
        mono_handle_result.params = self._compute_goodenough()
        mono_handle_result.linear_error = linear_error
        return mono_handle_result
