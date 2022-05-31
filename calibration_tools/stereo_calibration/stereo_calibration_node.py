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
import numpy as np

# local
sys.path.append("../../")
from common.enum_common import CameraInfoCheckLevel
from common.camera_common import CalibratorFunctionFlags, CalibratorTargetThreshold
from calibration_tools.stereo_calibration.stereo_calibrator import StereoCalibrator
from utils.get_corners import get_all_good_corners_from_images, quick_get_good_corners
from sensor_msgs.msg import PointCloud2, PointField


class StereoCalibrationNode:
    """给calibrator提供外部支持,让calibrator只负责标定,其他操作都交给外部"""

    def __init__(
        self,
        master_get_frame,
        slaver_get_frame,
        master_camera_info,
        slaver_camera_info,
        tf_info,
        chessboard_info,
    ):
        self.master_get_frame = master_get_frame
        self.slaver_get_frame = slaver_get_frame
        self.master_camera_info = master_camera_info
        self.slaver_camera_info = slaver_camera_info
        self.tf_info = tf_info
        self.chessboard_info = chessboard_info
        self.calibrator = None
        self.reproj_error = None  # 保存最后的标定误差

    def start(self):
        def on_trackbar(alpha_slider):
            pass

        self.calibrator = StereoCalibrator(
            chessboard_info=self.chessboard_info,
            master_camera_info=self.master_camera_info,
            slaver_camera_info=self.slaver_camera_info,
        )
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
        # 保存标定结果
        self.reproj_error = self.calibrator.reproj_error
        self.tf_info.R = self.calibrator.R
        self.tf_info.T = self.calibrator.T

    def show_result(self):
        def convert_board_to_4d(board_cols, board_rows, square_size):
            """将棋盘格转换为4维空间中的坐标"""

            corners_4d = np.array(
                [
                    [[j * square_size], [i * square_size], [0.0], [1]]
                    for i in range(board_rows)
                    for j in range(board_cols)
                ],
                dtype=np.float32,
            )
            return corners_4d

        def calculate_chessboard_corners_3d_points(rvecs, tvecs, corners_4d):
            """从棋盘世界坐标系变换到相机坐标系

            Args:
                rvecs (_type_): 旋转向量
                tvecs (_type_): 平移向量
                corners_4d (_type_): 棋盘格子的4维坐标
            """

            def convert_rvecs_tvecs_to_transform_matrix(rvecs, tvecs):
                """将rvecs和tvecs转换为4x4变换矩阵"""
                world_to_camera_tranformation_matrix = np.eye(4)
                rotation_matrix = cv2.Rodrigues(rvecs)[0]
                world_to_camera_tranformation_matrix[:3, :3] = rotation_matrix

                world_to_camera_tranformation_matrix[:3, 3] = tvecs.reshape(3)

                return world_to_camera_tranformation_matrix

            world_to_camera_tranformation_matrix = convert_rvecs_tvecs_to_transform_matrix(rvecs, tvecs)

            point_3d_list = []
            for i in range(corners_4d.shape[0]):
                point_3d = np.dot(world_to_camera_tranformation_matrix, corners_4d[i])
                point_3d_list.append(point_3d)

            # convert list to numpy array
            point_3d_array = np.array(point_3d_list)
            point_3d_array = np.reshape(point_3d_array, (point_3d_array.shape[0], point_3d_array.shape[1]))
            # drop the last column
            point_3d_array = point_3d_array[:, :3]
            return point_3d_array

        def convert_numpy_to_pointcloud2(camera_id, rvecs, tvecs, board_4d):
            """将numpy数组转换为pointcloud2格式"""
            # 计算棋盘格的3d坐标
            result = calculate_chessboard_corners_3d_points(rvecs, tvecs, board_4d)
            # convert result to PointCloud2
            points = np.zeros((result.shape[0], result.shape[1]), dtype=np.float32)
            for i in range(result.shape[0]):
                points[i, :] = result[i, :]
            msg = PointCloud2()
            msg.header.stamp = rospy.Time().now()
            msg.header.frame_id = camera_id
            msg.height = 1
            msg.width = len(points)
            msg.fields = [
                PointField("x", 0, PointField.FLOAT32, 1),
                PointField("y", 4, PointField.FLOAT32, 1),
                PointField("z", 8, PointField.FLOAT32, 1),
            ]
            msg.is_bigendian = False
            msg.point_step = 12
            msg.row_step = msg.point_step * points.shape[0]
            msg.is_dense = False
            msg.data = np.asarray(points, np.float32).tostring()
            return msg

        camera_info_check_level = CameraInfoCheckLevel.COMPLETED
        self.master_camera_info.info_check(camera_info_check_level)
        self.slaver_camera_info.info_check(camera_info_check_level)

        # handle board info
        board_cols = self.chessboard_info.n_cols
        board_rows = self.chessboard_info.n_rows
        square_size = self.chessboard_info.square_size
        board_4d = convert_board_to_4d(board_cols, board_rows, square_size)
        master_camera_publisher = rospy.Publisher(self.master_camera_info.camera_id, PointCloud2, queue_size=2)
        slaver_camera_publisher = rospy.Publisher(self.slaver_camera_info.camera_id, PointCloud2, queue_size=2)

        while True:
            master_frame = self.master_get_frame.read()
            slaver_frame = self.slaver_get_frame.read()

            (
                master_ok,
                master_corners,
                master_resized_img,
                master_downsampled_corners,
                (x_scale, y_scale),
            ) = quick_get_good_corners(master_frame, board_cols, board_rows)
            (
                slaver_ok,
                slaver_corners,
                slaver_resized_img,
                slaver_downsampled_corners,
                (x_scale, y_scale),
            ) = quick_get_good_corners(slaver_frame, board_cols, board_rows)

            if master_ok and slaver_ok:

                # sovle_pnp
                master_ret, master_rvecs, master_tvecs = cv2.solvePnP(
                    self.chessboard_info.BOARD,
                    np.array(master_corners, dtype=np.float32),
                    self.master_camera_info.intrinsics_matrix,
                    self.master_camera_info.distortion_coefficients,
                )
                slaver_ret, slaver_rvecs, slaver_tvecs = cv2.solvePnP(
                    self.chessboard_info.BOARD,
                    np.array(slaver_corners, dtype=np.float32),
                    self.slaver_camera_info.intrinsics_matrix,
                    self.slaver_camera_info.distortion_coefficients,
                )

                if master_ret and slaver_ret:
                    master_pointcloud2_msg = convert_numpy_to_pointcloud2(
                        self.master_camera_info.camera_id, master_rvecs, master_tvecs, board_4d
                    )
                    slaver_pointcloud2_msg = convert_numpy_to_pointcloud2(
                        self.slaver_camera_info.camera_id, slaver_rvecs, slaver_tvecs, board_4d
                    )
                    master_camera_publisher.publish(master_pointcloud2_msg)
                    slaver_camera_publisher.publish(slaver_pointcloud2_msg)
                # draw_chessboard
                cv2.drawChessboardCorners(
                    master_resized_img,
                    (board_cols, board_rows),
                    master_downsampled_corners,
                    True,
                )
                cv2.drawChessboardCorners(
                    slaver_resized_img,
                    (board_cols, board_rows),
                    slaver_downsampled_corners,
                    True,
                )
            cv2.imshow("master_windows", master_resized_img)
            cv2.imshow("slaver_windows", slaver_resized_img)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                cv2.destroyAllWindows()
                break
