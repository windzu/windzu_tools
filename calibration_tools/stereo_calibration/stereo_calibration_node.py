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
from common.enum_common import CameraInfoCheckLevel
from common.camera_common import CalibratorFunctionFlags, CalibratorTargetThreshold
from calibration_tools.stereo_calibration.stereo_calibrator import StereoCalibrator


class StereoCalibrationNode:
    """给calibrator提供外部支持,让calibrator只负责标定,其他操作都交给外部"""

    def __init__(
        self,
        master_get_frame,
        slaver_get_frame,
        master_camera_info,
        slaver_camera_info,
        chessboard_info,
    ):
        self.master_get_frame = master_get_frame
        self.slaver_get_frame = slaver_get_frame
        self.master_camera_info = master_camera_info
        self.slaver_camera_info = slaver_camera_info
        self.chessboard_info = chessboard_info
        self.calibrator = None

    def start(self):
        def on_trackbar(alpha_slider):
            pass

        self.calibrator = StereoCalibrator(
            chessboard_info=self.chessboard_info,
            master_camera_info=self.master_camera_info,
            slaver_camera_info=self.slaver_camera_info,
        )
        print("*******start collect sample*******")
        window_name = "collecting sample"
        cv2.namedWindow(window_name)
        slider_max = 100
        progress_trackbar_name = "collect progress "
        cv2.createTrackbar(progress_trackbar_name, window_name, 0, slider_max, on_trackbar)
        while True:
            master_frame = self.master_get_frame.read()
            slaver_frame = self.slaver_get_frame.read()
            ok, stereo_handle_result = self.calibrator.handle_frame(master_frame, slaver_frame)
            cv2.imshow(window_name, stereo_handle_result.show_master_img)
            cv2.imshow("slaver_windows", stereo_handle_result.show_slaver_img)
            if ok is False:
                progress = 0
            else:
                progress = int(stereo_handle_result.progress)
            cv2.setTrackbarPos(progress_trackbar_name, window_name, progress)

            key = cv2.waitKey(1)
            if key == ord("q"):
                cv2.destroyAllWindows()
                return
            if key == 13:  # enter

                self.calibrator._do_calibration()

                self.calibrator.calibrated = True
            if self.calibrator.calibrated is True:
                cv2.destroyAllWindows()
                break
        cv2.destroyAllWindows()

    def show_result(self):
        camera_info_check_level = CameraInfoCheckLevel.COMPLETED
        self.master_camera_info.info_check(camera_info_check_level)
        self.slaver_camera_info.info_check(camera_info_check_level)

        print("*******start show result*******")
        if self.calibrator is None:
            self.calibrator = StereoCalibrator(
                chessboard_info=self.chessboard_info,
                master_camera_info=self.master_camera_info,
                slaver_camera_info=self.slaver_camera_info,
            )
            self.calibrator.calibrated = True
        window_name = "show result"
        cv2.namedWindow(window_name)

        while True:
            master_frame = self.master_get_frame.read()
            slaver_frame = self.slaver_get_frame.read()
            ok, stereo_handle_result = self.calibrator.handle_frame(master_frame, slaver_frame)
            cv2.imshow("mono_handle_result", mono_handle_result.resized_img_with_corners)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                cv2.destroyAllWindows()
                break
