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
# sys.path.append("../")
from common.enum_common import Patterns
from common.camera_calibrator import CameraCalibrator, HandleResult
from common.enum_common import CameraModel
from utils.get_corners import get_all_good_corners_from_images, quick_get_good_corners


class StereoHandleResult(HandleResult):
    def __init__(self):
        HandleResult.__init__(self)
        self.show_master_img = None
        self.show_slave_img = None
        self.progress = 0
        self.reproj_error = 0.0


class StereoCalibrator(CameraCalibrator):
    """双目标定"""

    def __init__(self, chessboard_info, master_camera_info, slaver_camera_info):
        super(StereoCalibrator, self).__init__(chessboard_info)
        self.master_camera_info = master_camera_info
        self.slaver_camera_info = slaver_camera_info

        # 存储采集到的数据，包括达标的master和slaver的images、重投影误差
        # {"master_images": [], "slaver_images": [],"reproj_errors": []}
        self.stored_data = {}
        self.stored_data["master_images"] = []
        self.stored_data["slaver_images"] = []
        self.stored_data["reproj_errors"] = []

        self.R = None
        self.T = None
        self.reproj_error = None

        # 一些固定阈值设置
        self.reproj_error_thresh = 20
        self.min_samples = 20

    def handle_frame(self, master_frame, slaver_frame):

        master_gray = cv2.cvtColor(master_frame, cv2.COLOR_BGR2GRAY)
        slaver_gray = cv2.cvtColor(slaver_frame, cv2.COLOR_BGR2GRAY)

        stereo_handle_result = StereoHandleResult()
        board_cols = self.chessboard_info.n_cols
        board_rows = self.chessboard_info.n_rows

        img_width = self.master_camera_info.resolution[0]
        img_height = self.master_camera_info.resolution[1]

        # 首先快速检测角点
        (
            master_ok,
            master_corners,
            master_resized_img,
            master_downsampled_corners,
            (master_x_scale, master_y_scale),
        ) = quick_get_good_corners(master_gray, board_cols, board_rows)
        (
            slaver_ok,
            slaver_corners,
            slaver_resized_img,
            slaver_downsampled_corners,
            (slaver_x_scale, slaver_y_scale),
        ) = quick_get_good_corners(slaver_gray, board_cols, board_rows)

        if self.calibrated is False:
            # 如果标定是未完成的,则
            #   1. 绘制快速检测的角点
            #   2. 进行标定并计算重投影误差
            #   3. 判断投影误差是否超过阈值然后决定是否存储本次数据：master_image slaver_image reproj_error
            #   4. 检查是否已经采集到足够数据，如果是，则进行正式标定
            #   5. 返回必要的信息
            master_resized_gray = cv2.cvtColor(master_resized_img, cv2.COLOR_GRAY2BGR)
            slaver_resized_gray = cv2.cvtColor(slaver_resized_img, cv2.COLOR_GRAY2BGR)

            if master_ok and slaver_ok:
                # 1. 绘制快速检测的角点
                cv2.drawChessboardCorners(
                    master_resized_gray,
                    (board_cols, board_rows),
                    master_downsampled_corners,
                    True,
                )
                cv2.drawChessboardCorners(
                    slaver_resized_gray,
                    (board_cols, board_rows),
                    slaver_downsampled_corners,
                    True,
                )
                # 2. 进行标定并计算重投影误差
                ret, CM1, dist1, CM2, dist2, R, T, E, F = cv2.stereoCalibrate(
                    [self.chessboard_info.BOARD],
                    [master_downsampled_corners],
                    [slaver_downsampled_corners],
                    self.master_camera_info.intrinsics_matrix,
                    self.master_camera_info.distortion_coefficients,
                    self.slaver_camera_info.intrinsics_matrix,
                    self.slaver_camera_info.distortion_coefficients,
                    (img_width, img_height),
                    R=self.R,
                    T=self.T,
                    criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 1, 1e-5),
                    flags=cv2.CALIB_FIX_INTRINSIC,
                )
                # debug
                print("current sample reproj error:", ret)
                # 3. 判断投影误差是否超过阈值然后决定是否存储本次数据：master_image slaver_image reproj_error
                if ret < self.reproj_error_thresh:
                    self.stored_data["master_images"].append(master_gray)
                    self.stored_data["slaver_images"].append(slaver_gray)
                    self.stored_data["reproj_errors"].append(ret)
                # 4. 检查是否已经采集到足够数据，如果是，则进行正式标定
                if len(self.stored_data["master_images"]) >= self.min_samples:
                    self._do_calibration()
                # 5. 同时检测到角点，并完成计算，返回必要的信息
                stereo_handle_result.show_master_img = master_resized_gray
                stereo_handle_result.show_slaver_img = slaver_resized_gray
                stereo_handle_result.progress = int((len(self.stored_data["master_images"]) / self.min_samples) * 100)
                stereo_handle_result.reproj_error = ret
                return True, stereo_handle_result
            else:
                # 5. 未同时检测到角点，未完成计算，返回必要的信息
                stereo_handle_result.show_master_img = master_resized_gray
                stereo_handle_result.show_slaver_img = slaver_resized_gray
                stereo_handle_result.progress = int((len(self.stored_data["master_images"]) / self.min_samples) * 100)
                stereo_handle_result.reproj_error = -1
                return False, stereo_handle_result

    def _do_calibration(self):
        """数据采集完毕后，使用收集到的数据进行正式标定"""
        print("*******start calibrate*******")
        master_images = self.stored_data["master_images"]
        slaver_images = self.stored_data["slaver_images"]

        board_cols = self.chessboard_info.n_cols
        board_rows = self.chessboard_info.n_rows
        checkerboard_flags = self.calibrator_function_flags.cv2_findChessboardCorners_flags
        calibration_flags = self.calibrator_function_flags
        master_corners, raw_master_corners = get_all_good_corners_from_images(
            master_images, board_cols, board_rows, checkerboard_flags
        )
        slaver_corners, raw_slaver_corners = get_all_good_corners_from_images(
            slaver_images, board_cols, board_rows, checkerboard_flags
        )

        # 是否同时采集到判断
        master_corners = []
        slaver_corners = []
        if len(raw_master_corners) == len(raw_slaver_corners):
            for i in range(len(raw_master_corners)):
                if raw_master_corners[i][0] == True and raw_slaver_corners[i][0] == True:
                    master_corners.append(raw_master_corners[i][1])
                    slaver_corners.append(raw_slaver_corners[i][1])
        if len(master_corners) > 0 and len(master_corners) == len(slaver_corners):
            print("get all good corners")
        else:
            print("len of corners is different")

        self._do_calibration_from_corners(master_corners, slaver_corners, calibration_flags)

    def _do_calibration_from_corners(self, master_corners, slaver_corners, calibrate_flags=None):
        boards = [self.chessboard_info.BOARD for i in range(len(master_corners))]
        img_width = self.master_camera_info.resolution[0]
        img_height = self.master_camera_info.resolution[1]
        if self.master_camera_info.camera_model == CameraModel.PINHOLE:
            (
                self.reproj_error,
                cameraMatrix1,
                distCoeffs1,
                cameraMatrix2,
                distCoeffs2,
                self.R,
                self.T,
                E,
                F,
            ) = cv2.stereoCalibrate(
                boards,
                master_corners,
                slaver_corners,
                self.master_camera_info.intrinsics_matrix,
                self.master_camera_info.distortion_coefficients,
                self.slaver_camera_info.intrinsics_matrix,
                self.slaver_camera_info.distortion_coefficients,
                (img_width, img_height),
                criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 1, 1e-5),
                flags=cv2.CALIB_FIX_INTRINSIC,
            )
            R1, R2, P1, P2, Q, validPixROI1, validPixROI2 = cv2.stereoRectify(
                self.master_camera_info.intrinsics_matrix,
                self.master_camera_info.distortion_coefficients,
                self.slaver_camera_info.intrinsics_matrix,
                self.slaver_camera_info.distortion_coefficients,
                (img_width, img_height),
                self.R,
                self.T,
                alpha=1,
            )
        elif self.master_camera_info.camera_model == CameraModel.FISHEYE:
            (
                self.reproj_error,
                cameraMatrix1,
                distCoeffs1,
                cameraMatrix2,
                distCoeffs2,
                self.R,
                self.T,
            ) = cv2.fisheye.stereoCalibrate(
                boards,
                master_corners,
                slaver_corners,
                self.master_camera_info.intrinsics_matrix,
                self.master_camera_info.distortion_coefficients,
                self.slaver_camera_info.intrinsics_matrix,
                self.slaver_camera_info.distortion_coefficients,
                (img_width, img_height),
            )
        # TODO : camera info check
        self.calibrated = True
