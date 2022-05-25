"""
Author: windzu
Date: 2022-03-23 10:37:36
LastEditTime: 2022-03-23 10:37:37
LastEditors: windzu
Description: 
FilePath: /monocular_camera_calibration/test/calibration_node.py
@Copyright (C) 2021-2022 plusgo Company Limited. All rights reserved.
@Licensed under the Apache License, Version 2.0 (the License)
"""
import rospy
import cv2
import sys

# local
sys.path.append("../../")
from utils.get_frame import GetFrame
from utils.camera_utils import camera_info_check
from common.camera_common import CalibratorFunctionFlags, CalibratorTargetThreshold, InfoCheckLevel
from calibrator import Calibrator, MonoCalibrator


class MonoCalibrationNode:
    """给calibrator提供外部支持,让calibrator只负责标定,其他操作都交给外部
    1.初始化calibrator需要的信息:
        - 标定板的基本信息 ChessboardInfo
        - 所要标定的相机的基本信息 CameraInfo
        - 标定中用到的方法的标志位 CalibratorFunctionFlags
        - 标定中用到的目标阈值 CalibratorTargetThreshold
    2. 标定过程中需要输入的信息:
        - 相机的图像
    3. 标定过程中接收的信息:
        - 标定过程中的一些标定状态信息
    4. 标定结束后接收的信息
        - 相机的内参
    """

    def __init__(self, cap, chessboard_info, camera_info):
        self.cap = cap
        self.chessboard_info = chessboard_info
        self.camera_info = camera_info

        # calibrator的一些个性化的参数设置,无特设需求保持默认即可，无需改动
        self.calibrator_function_flags = CalibratorFunctionFlags()
        self.calibrator_target_threshold = CalibratorTargetThreshold()

        self.calibrator = None

    def start(self):
        def on_trackbar(alpha_slider):
            pass

        self.calibrator = MonoCalibrator(
            chessboard_info=self.chessboard_info,
            camera_info=self.camera_info,
            calibrator_function_flags=self.calibrator_function_flags,
            calibrator_target_threshold=self.calibrator_target_threshold,
        )
        print("*******start collect sample*******")
        window_name = "collecting sample"
        cv2.namedWindow(window_name)
        slider_max = 100
        # trackbar_name = "Alpha x %d" % alpha_slider_max
        x_progress_trackbar_name = " X "
        y_progress_trackbar_name = " Y "
        size_progress_trackbar_name = " SIZE "
        skew_progress_trackbar_name = " SKEW "

        cv2.createTrackbar(x_progress_trackbar_name, window_name, 0, slider_max, on_trackbar)
        cv2.createTrackbar(y_progress_trackbar_name, window_name, 0, slider_max, on_trackbar)
        cv2.createTrackbar(size_progress_trackbar_name, window_name, 0, slider_max, on_trackbar)
        cv2.createTrackbar(skew_progress_trackbar_name, window_name, 0, slider_max, on_trackbar)
        while True:
            frame = self.cap.read()
            mono_handle_result = self.calibrator.handle_frame(frame)
            cv2.imshow(window_name, mono_handle_result.resized_img_with_corners)
            if mono_handle_result.params is None:
                x_progress = 0
                y_progress = 0
                size_progress = 0
                skew_progress = 0
            else:
                x_progress = int(mono_handle_result.params[0][3] * 100)
                y_progress = int(mono_handle_result.params[1][3] * 100)
                size_progress = int(mono_handle_result.params[2][3] * 100)
                skew_progress = int(mono_handle_result.params[3][3] * 100)
            cv2.setTrackbarPos(x_progress_trackbar_name, window_name, x_progress)
            cv2.setTrackbarPos(y_progress_trackbar_name, window_name, y_progress)
            cv2.setTrackbarPos(size_progress_trackbar_name, window_name, size_progress)
            cv2.setTrackbarPos(skew_progress_trackbar_name, window_name, skew_progress)
            key = cv2.waitKey(1)
            if key == ord("q"):
                cv2.destroyAllWindows()
                return
            if key == 13:  # enter
                self.calibrator.goodenough = True
            if self.calibrator.goodenough is True:
                cv2.destroyAllWindows()
                break
        print("*******start calibrate*******")
        self.calibrator.do_calibration()
        print("*******calibrate finished*******")

    def show_result(self):
        info_check_level = InfoCheckLevel.COMPLETED
        ret, self.camera_info = camera_info_check(self.camera_info, info_check_level)
        if ret is False:
            print("camera_info is not valid")
            return
        # debug
        print(self.camera_info)
        print("start show result!!")
        if self.calibrator is None:
            self.calibrator = MonoCalibrator(
                chessboard_info=self.chessboard_info,
                camera_info=self.camera_info,
                calibrator_function_flags=self.calibrator_function_flags,
                calibrator_target_threshold=self.calibrator_target_threshold,
            )
        self.calibrator.calibrated = True
        while True:
            frame = self.cap.read()
            mono_handle_result = self.calibrator.handle_frame(frame)
            cv2.imshow("mono_handle_result", mono_handle_result.resized_img_with_corners)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                cv2.destroyAllWindows()
                break
