"""
Author: windzu
Date: 2022-03-22 13:13:18
LastEditTime: 2022-03-22 13:13:20
LastEditors: windzu
Description: 
FilePath: /monocular_camera_calibration/test/calibrator.py
@Copyright (C) 2021-2022 plusgo Company Limited. All rights reserved.
@Licensed under the Apache License, Version 2.0 (the License)
"""
from enum import Enum
import cv2
from cv2 import checkChessboard
import numpy as np
import math
import sys

# local
sys.path.append("../")
from common.enum_common import Patterns, CameraModel, InfoCheckLevel


class HandleResult(object):
    """
    Passed to CalibrationNode after image handled. Allows plotting of images
    with detected corner points
    """

    def __init__(self):
        self.params = None


class CameraCalibrator:
    """相机标定的基类"""

    def __init__(self, chessboard_info, calibrator_function_flags, calibrator_target_threshold):
        # public
        # 相机标定需要的基础信息
        self.chessboard_info = chessboard_info
        # 标定状态信息
        self.calibrated = False  # 标定是否完成的标志位
        self.goodenough = False  # 采集的样本是否达到要求的标志位
        # protected
        self._db = []

        self._last_frame_corners = None
        self._last_frame_ids = None
        # private
        self.__calibrator_function_flags = calibrator_function_flags
        self.__calibrator_target_threshold = calibrator_target_threshold

    #     def _remap(self, src):
    #         return cv2.remap(src, self.__camera_info.map1, self.__camera_info.map2, cv2.INTER_LINEAR)
    #
    #     def _remap(self, src, camera_info):
    #         return cv2.remap(src, camera_info.map1, camera_info.map2, cv2.INTER_LINEAR)

    def _compute_goodenough(self):
        """遍历所有已采样样本，计算评价指标在各个维度上的进度
        Returns:
            (list):[
                ( 'X' , min_p_x , max_p_x , x_progress ),
                ( 'Y' , min_p_y , max_p_y , y_progress ),
                ( 'Size' , min_p_size , max_p_size , size_progress ),
                ( 'Skew' , min_p_skew , max_p_skew , skew_progress )
            ]
        """
        if not self._db:
            return None

        # 遍历db中所有的params,统计当前db中params在四个维度上的上下限(统计指标空间的边界)
        all_params = [sample[0] for sample in self._db]
        min_params = all_params[0]
        max_params = all_params[0]
        for params in all_params[1:]:
            min_params = self.__lmin(min_params, params)
            max_params = self.__lmax(max_params, params)
        # Don't reward small size or skew
        min_params = [min_params[0], min_params[1], 0.0, 0.0]

        # 根据上下限与预设的进度阈值进行比较,得到当前标定在四个维度上的进度，并以此进度作为标定的进度
        # 1. 当进度达到设定阈值的时候，返回True，意味着已经采集了足够多的样本了，可以进行标定
        # 2. 当进度还没有达到阈值的，可以根据四个维度阈值与目标的差距，从而得知应该还需要采集哪些数据 (暂时不考虑 size 和 skew 的情况)
        #     1. 如果 X 过小，表示在x方向要再多拍一些样本
        #     2. 如果 Y 过小，表示在y方向要再多拍一些样本
        # NOTE : 如果已经采集了足够多的样本，但进度还是不符合需求，这种情况可能是以为阈值设置的不合理，我们也认为可以用于标定了
        progress = [
            min((hi - lo) / r, 1.0)
            for (lo, hi, r) in zip(min_params, max_params, self.__calibrator_target_threshold.param_ranges)
        ]
        self.goodenough = (len(self._db) >= 40) or all([p == 1.0 for p in progress])

        return list(zip(self.__param_names, min_params, max_params, progress))

    def _quick_get_good_corners(self, img):
        """通过降采样以实现快速角点检测
        Args:
            img: 原图
        Returns:
            (ok, resized_img , corners, downsampled_corners, (x_scale, y_scale)).
                ok: 是否检测到角点
                resized_img : 缩放后的图像
                corners: 原图角点
                downsampled_corners :缩放后图像中检测到的角点
                (x_scale, y_scale): x y方向缩放的尺度比例
        """
        # 原图过大的话就缩放至与VGA(640*480)尺寸类似的大小，通过计算面积而得到缩放系数
        height = img.shape[0]
        width = img.shape[1]
        scale = math.sqrt((width * height) / (640.0 * 480.0))
        # scale = math.sqrt((width * height) / (512.0 * 384.0))
        if scale > 1.0:
            resized_img = cv2.resize(img, (int(width / scale), int(height / scale)))
        else:
            resized_img = img

        # 计算缩放系数
        # NOTE ： 缩放后的图像进行角点检测后，还需将其还原至原图大小，并将角点也缩放回原图大小，所以需要知道缩放系数才能计算
        x_scale = float(width) / resized_img.shape[1]
        y_scale = float(height) / resized_img.shape[0]

        (ok, downsampled_corners) = self._get_good_corners(resized_img, self.chessboard_info)

        # 缩放回原尺寸后再进行cv2.cornerSubPix , 这样就可以做到实时的角点检测
        corners = None
        if ok:
            if scale > 1.0:
                corners_unrefined = downsampled_corners.copy()
                corners_unrefined[:, :, 0] *= x_scale
                corners_unrefined[:, :, 1] *= y_scale
                radius = int(math.ceil(scale))
                if len(img.shape) == 3 and img.shape[2] == 3:
                    mono = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                else:
                    mono = img
                cv2.cornerSubPix(
                    mono,
                    corners_unrefined,
                    (radius, radius),
                    (-1, -1),
                    (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1),
                )
                corners = corners_unrefined
            else:
                corners = downsampled_corners
        else:
            corners = None

        return (ok, resized_img, corners, downsampled_corners, (x_scale, y_scale))

    def _get_all_good_corners_from_images(self, images):
        """检测图像列表中所有图像的角点
        Args:
            images (list): 图像列表
        Returns:
            goodcorners (list): [ corners... ]
        """
        self.size = (images[0].shape[1], images[0].shape[0])
        chessboard_info = self.chessboard_info

        corners = [self._get_good_corners(i, chessboard_info) for i in images]

        all_good_corners = [co for (ok, co) in corners if ok]
        if not all_good_corners:
            raise Exception("No corners found in images!")
        return all_good_corners

    @staticmethod
    def _do_calibration_from_corners(corners, chessboard_info, camera_info, calibrate_flags):
        """通过所有采集的图像的角点计算相机内参
        Args:
            corners (list):每张图corner的列表
            chessboard_info (ChessboardInfo): 棋盘信息
            camera_info (CameraInfo): 相机信息
            good (list): [ (corners, ids, ChessboardInfo) ]
            calibrate_flags (int): 标定标志

        """
        boards = [chessboard_info.BOARD for i in range(len(corners))]

        if camera_info.camera_model == CameraModel.PINHOLE:
            print("mono pinhole calibration...")
            (
                reproj_err,
                camera_info.intrinsics_matrix,
                camera_info.distortion_coefficients,
                camera_info.rvecs,
                camera_info.tvecs,
            ) = cv2.calibrateCamera(
                boards,
                corners,
                camera_info.resolution,
                np.eye(3, 3),
                np.zeros((5, 1)),
                flags=calibrate_flags.cv2_calibrateCamera_flags,
            )
            print("pinhole calibration reproj_err : ", reproj_err)

        elif camera_info.camera_model == CameraModel.FISHEYE:
            print("mono fisheye calibration...")
            # WARNING: cv2.fisheye.calibrate wants float64 points
            corners = np.asarray(corners, dtype=np.float64)
            boards = np.asarray(boards, dtype=np.float64)
            (
                reproj_err,
                camera_info.intrinsics_matrix,
                camera_info.distortion_coefficients,
                camera_info.rvecs,
                camera_info.tvecs,
            ) = cv2.fisheye.calibrate(
                boards,
                corners,
                camera_info.resolution,
                np.eye(3, 3),
                np.zeros((4, 1)),
                flags=calibrate_flags.cv2_fisheye_calibrate_flags,
            )
            print("fisheye calibration reproj_err : ", reproj_err)

        info_check_level = InfoCheckLevel.COMPLETED
        camera_info.info_check(info_check_level)
        return True, camera_info
        # # R is identity matrix for monocular calibration
        # self.R = np.eye(3, dtype=np.float64)
        # self.P = np.zeros((3, 4), dtype=np.float64)

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

    @staticmethod
    def _get_good_corners(img, chessboard_info, refine=True, checkerboard_flags=0):
        """检测更好的角点。原因如下：
        1. 对检测到的角点进行了有效性检查，排除了距离边缘过近的角点
        2. 对检测到的角点进行了重新排序，保证顺序为左上，右上，右下，左下
        3. 使用了cv2.cornerSubPix进行了精细化,使得角点更加精确

        Args:
            img (np.ndarray): 原始图像
            chessboard_info (_type_): 标定板信息
            checkerboard_flags (_type_): 检测角点的参数

        Returns:
            (ok, corners): ok是否检测到足够的角点,corners是检测到的角点
        """

        def pdist(p1, p2):
            """
            计算两点距离. p1 = (x, y), p2 = (x, y)
            """
            return math.sqrt(math.pow(p1[0] - p2[0], 2) + math.pow(p1[1] - p2[1], 2))

        h = img.shape[0]
        w = img.shape[1]
        if len(img.shape) == 3 and img.shape[2] == 3:
            mono = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        else:
            mono = img
        (ok, corners) = cv2.findChessboardCorners(
            mono, (chessboard_info.n_cols, chessboard_info.n_rows), flags=checkerboard_flags
        )
        if not ok:
            return (ok, corners)

        # 对于靠近边缘的角点，如果与边缘距离小于BORDER像素(默认设置为8)，则设置ok为false从而丢弃该检测结果，
        # NOTE:这对于低分辨率的相机来不友好，因为8像素对于非常低像素图像而言不是一个小数目
        BORDER = 8
        if not all(
            [
                (BORDER < corners[i, 0, 0] < (w - BORDER)) and (BORDER < corners[i, 0, 1] < (h - BORDER))
                for i in range(corners.shape[0])
            ]
        ):
            ok = False

        # 确保角点数组的顺序为:从上到下,从左到右
        # NOTE :
        #   1. 行列不同的棋盘的摆放方位与cv2.findChessboardCorners中设置的棋盘尺寸一致，例如棋盘尺寸为(6,9) (cols,rows)，则棋盘拜访必须是长边竖直，短边水平
        #   2. 行列相同，则无所谓摆放的方向
        # a. 使用行列不同的棋盘格，角点顺序可能会反方向，通过判断第一个和最后一个角点的y坐标来判断，如果想法则将corners数组翻转
        if chessboard_info.n_rows != chessboard_info.n_cols:
            if corners[0, 0, 1] > corners[-1, 0, 1]:
                corners = np.copy(np.flipud(corners))
        # 2. 如果是行列相同的标定板，则检测的角点顺序有四种可能，分别是:
        #    1. 从上到下 (正确)
        #    2. 从下到上 (需要翻转)
        #    3. 从右到左 (需要逆时针旋转90度)
        #    4. 从左到右 (需要逆时针旋转3x90度)
        else:
            direction_corners = (corners[-1] - corners[0]) >= np.array([[0.0, 0.0]])

            if not np.all(direction_corners):
                if not np.any(direction_corners):
                    corners = np.copy(np.flipud(corners))
                elif direction_corners[0][0]:
                    corners = np.rot90(corners.reshape(chessboard_info.n_rows, chessboard_info.n_cols, 2)).reshape(
                        chessboard_info.n_cols * chessboard_info.n_rows, 1, 2
                    )
                else:
                    corners = np.rot90(corners.reshape(chessboard_info.n_rows, chessboard_info.n_cols, 2), 3).reshape(
                        chessboard_info.n_cols * chessboard_info.n_rows, 1, 2
                    )

        # 按照行和列的方向遍历角点，找到距离最近的角点间的距离，设置其作为cv2.cornerSubPix的搜寻半径，这样可以在保证速度情况下最大范围的进行亚像素搜索
        # NOTE : 这参考其ros的角点检测代码，个人认为亚像素搜索没必要这么大范围，除非是分辨率很低的情况或者是棋盘格square size很小，这样做可以确定搜索的边界；
        # 而对于高像素的图像，随便设置为5就可以了，两个角点间的距离肯定不会小于5，而5像素范围内就可以很好的搜索
        # 不过ros的这种做法确实更具rubust
        if refine and ok:
            min_distance = float("inf")
            for row in range(chessboard_info.n_rows):
                for col in range(chessboard_info.n_cols - 1):
                    index = row * chessboard_info.n_cols + col
                    min_distance = min(min_distance, pdist(corners[index, 0], corners[index + 1, 0]))
            for row in range(chessboard_info.n_rows - 1):
                for col in range(chessboard_info.n_cols):
                    index = row * chessboard_info.n_cols + col
                    min_distance = min(
                        min_distance, pdist(corners[index, 0], corners[index + chessboard_info.n_cols, 0])
                    )
            radius = int(math.ceil(min_distance * 0.5))
            cv2.cornerSubPix(
                mono, corners, (radius, radius), (-1, -1), (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)
            )

        return (ok, corners)
