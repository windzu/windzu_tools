"""
Author: windzu
Date: 2022-05-25 15:33:47
LastEditTime: 2022-05-25 16:34:18
LastEditors: windzu
Description: 
FilePath: /windzu_tools/calibration_tools/stereo_calibration/stereo_calibrator.py
@Copyright (C) 2021-2022 plusgo Company Limited. All rights reserved.
@Licensed under the Apache License, Version 2.0 (the License)
"""
import cv2
import sys

# local
sys.path.append("../")
from common.enum_common import Patterns
from common.camera_calibrator import CameraCalibrator, HandleResult


class StereoHandleResult(HandleResult):
    def __init__(self):
        HandleResult.__init__(self)
        self.resized_img_with_corners = None
        self.linear_error = -1.0


class StereoCalibrator(CameraCalibrator):
    """
    单目相机标定类,继承自Calibrator类
    此类的设计目标：
        1. 类变量 : 尽量不含有专属的类变量,因为Calibrator类应该包含了一切单目标定需要的信息
        2. 类函数 : 函数应该被设计只是用于单目标定使用,而如果存在与双目标定类似的函数需求,应该尝试通过继承Calibrator类来实现
    """

    def __init__(self, chessboard_info, master_camera_info, slaver_camera_info):
        super(StereoCalibrator, self).__init__()
        self._good_corners = {}
        self._good_corners["master"] = []
        self._good_corners["slaver"] = []

        # public
        # 这两个标志位应为public,外部需要认为控制他们的状态
        self.calibrated = False  # 标定是否完成的标志位
        self.goodenough = False  # 采集的样本是否达到要求的标志位
        # protected
        self._last_frame_corners = None
        self._last_frame_ids = None
        self._chessboard_info = chessboard_info
        # private
        self.__master_camera_info = master_camera_info
        self.__slaver_camera_info = slaver_camera_info

    def handle_frame(self, master_frame, slaver_frame):
        """单目图像的处理入口函数，只需要传入图像(因为是实时进行,称之为frame)，即可进行处理，处理目前分两种类型
        1. 如果相机是已经标定过的,则对输入进行校正
        2. 如果相机是未标定过的,则采集图像进行标定

            Returns a MonoDrawable message with the display image and progress info.
        """
        master_gray = cv2.cvtColor(master_frame, cv2.COLOR_BGR2GRAY)
        slaver_gray = cv2.cvtColor(slaver_frame, cv2.COLOR_BGR2GRAY)
        linear_error = -1

        (
            master_resized_img,
            master_corners,
            master_downsampled_corners,
            ids,
            board,
            (master_x_scale, master_y_scale),
        ) = self._downsample_and_detect(master_gray)

        (
            slaver_resized_img,
            slaver_corners,
            slaver_downsampled_corners,
            ids,
            board,
            (slaver_x_scale, slaver_y_scale),
        ) = self._downsample_and_detect(slaver_gray)

        # 如果标定是已经完成的,则将slaver检测的角点重投影到master上,并计算误差
        if self.calibrated:
            undistorted_master_gray = self._remap(master_gray, self.__master_camera_info)
            undistorted_slaver_gray = self._remap(slaver_gray, self.__slaver_camera_info)
            if master_x_scale != 1.0 or master_y_scale != 1.0:
                resized_master_undistorted_gray = cv2.resize(
                    undistorted_master_gray, (master_resized_img.shape[1], master_resized_img.shape[0])
                )
            if slaver_x_scale != 1.0 or slaver_y_scale != 1.0:
                resized_slaver_undistorted_gray = cv2.resize(
                    undistorted_slaver_gray, (slaver_resized_img.shape[1], slaver_resized_img.shape[0])
                )

            resized_master_undistorted_img = cv2.cvtColor(resized_master_undistorted_gray, cv2.COLOR_GRAY2BGR)
            resized_slaver_undistorted_img = cv2.cvtColor(resized_slaver_undistorted_gray, cv2.COLOR_GRAY2BGR)

            # 角点校正、缩放、绘制角点
            if master_corners is not None:
                undistorted_master_corners = self._undistort_points(master_corners, self.__master_camera_info)
                # 重投影到slaver上

                resized_undistorted_corners = undistorted_corners.copy()
                resized_undistorted_corners[:, :, 0] /= x_scale
                resized_undistorted_corners[:, :, 1] /= y_scale
                cv2.drawChessboardCorners(
                    resized_undistorted_img,
                    (board.n_cols, board.n_rows),
                    resized_undistorted_corners,
                    True,
                )

                # debug
            resized_img_with_corners = resized_undistorted_img
        else:
            # 如果还没标定完成，就检测左右角点计算误差，不断迭代，直到达到预期值，就结束
            master_resized_gray = cv2.cvtColor(master_resized_img, cv2.COLOR_GRAY2BGR)
            slaver_resized_gray = cv2.cvtColor(slaver_resized_img, cv2.COLOR_GRAY2BGR)

            if master_corners is not None and slaver_corners is not None:
                if board.pattern == Patterns.CHESSBOARD:
                    cv2.drawChessboardCorners(
                        master_resized_gray,
                        (board.n_cols, board.n_rows),
                        master_downsampled_corners,
                        True,
                    )
                    cv2.drawChessboardCorners(
                        slaver_resized_gray,
                        (board.n_cols, board.n_rows),
                        slaver_downsampled_corners,
                        True,
                    )
                else:
                    pass  # TODO Draw other corner types

                # 对本次检测的角点信息与角点数据库中的数据进行比较,如果是符合要求的角点,则加入数据库，反之则丢弃本次检测的角点
                self._good_corners["master"].append(master_corners)
                self._good_corners["slave"].append(slaver_corners)
                print("add sample corners")
            master_resized_img_with_corners = master_resized_gray
            slaver_resized_img_with_corners = slaver_resized_gray

        stereo_handle_result = StereoHandleResult()
        stereo_handle_result.master_resized_img_with_corners = master_resized_img_with_corners
        stereo_handle_result.slaver_resized_img_with_corners = slaver_resized_img_with_corners
        stereo_handle_result.params = self._compute_goodenough()
        return stereo_handle_result

    def _compute_goodenough(self):
        """重写计算双目足够好的函数"""
        boards = [self._chessboard_info.BOARD for i in range(len(self._good_corners["master"]))]
        ret, CM1, dist1, CM2, dist2, R, T, E, F = cv2.stereoCalibrate(
            boards,
            self._good_corners["master"],
            self._good_corners["slaver"],
            self.__master_camera_info.intrinsics_matrix,
            self.__master_camera_info.distortion_coefficients,
            self.__slaver_camera_info.intrinsics_matrix,
            self.__slaver_camera_info.distortion_coefficients,
            self.size,
            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 1, 1e-5),
            flags=cv2.CALIB_FIX_INTRINSIC,
        )
        return ret, R, T
