"""
Author: windzu
Date: 2022-04-09 11:16:23
LastEditTime: 2022-04-09 11:16:24
LastEditors: windzu
Description: 
FilePath: /windzu_ws/src/tools/calibration_tools/mono_calibration/mono_calibration_node.py
@Copyright (C) 2021-2022 plusgo Company Limited. All rights reserved.
@Licensed under the Apache License, Version 2.0 (the License)
"""
import rospy
import cv2
import sys

# local
sys.path.append("../../")
from common.enum_common import InfoCheckLevel
from common.camera_common import CalibratorFunctionFlags, CalibratorTargetThreshold
from calibration_tools.stereo_calibration.stereo_calibrator import StereoCalibrator


class StereoCalibrationNode:
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

    def __init__(
        self,
        master_get_frame,
        master_camera_info,
        slave_get_frame,
        slave_camera_info,
        chessboard_info,
        calibrator_function_flags=CalibratorFunctionFlags(),
        calibrator_target_threshold=CalibratorTargetThreshold(),
    ):
        self.master_get_frame = master_get_frame
        self.master_camera_info = master_camera_info
        self.slave_get_frame = slave_get_frame
        self.slave_camera_info = slave_camera_info
        self.chessboard_info = chessboard_info
        self.calibrator_function_flags = calibrator_function_flags
        self.calibrator_target_threshold = calibrator_target_threshold

        self.calibrator = None

    def start(self):
        def on_trackbar(alpha_slider):
            pass

        self.stereo_calibrator = StereoCalibrator(
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
        ret_progress_trackbar_name = "preproject error "

        cv2.createTrackbar(ret_progress_trackbar_name, window_name, 0, slider_max, on_trackbar)
        while True:
            master_frame = self.master_get_frame.read()
            slave_frame = self.slave_get_frame.read()
            stereo_handle_result = self.stereo_calibrator.handle_frame(master_frame, slave_frame)
            cv2.imshow(window_name, stereo_handle_result.resized_img_with_corners)
            if stereo_handle_result.params is None:
                ret_progress = 0
            else:
                ret_progress = int(stereo_handle_result.params[0][3] * 100)
            cv2.setTrackbarPos(ret_progress_trackbar_name, window_name, ret_progress)

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
        self.calibrator._do_calibration()
        print("*******calibrate finished*******")

    def show_result(self):
        info_check_level = InfoCheckLevel.COMPLETED
        self.camera_info.info_check(info_check_level)

        # debug
        print(self.camera_info)
        print("start show result!!")
        if self.calibrator is None:
            self.calibrator = StereoCalibrator(
                chessboard_info=self.chessboard_info,
                camera_info=self.camera_info,
                calibrator_function_flags=self.calibrator_function_flags,
                calibrator_target_threshold=self.calibrator_target_threshold,
            )
        self.calibrator.calibrated = True
        while True:
            frame = self.get_frame.read()
            mono_handle_result = self.calibrator.handle_frame(frame)
            cv2.imshow("mono_handle_result", mono_handle_result.resized_img_with_corners)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                cv2.destroyAllWindows()
                break
