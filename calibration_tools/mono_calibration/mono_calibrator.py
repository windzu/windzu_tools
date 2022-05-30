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
import math
import numpy as np

# local
# sys.path.append("../")
from common.camera_calibrator import CameraCalibrator, HandleResult
from common.enum_common import CameraModel, CameraInfoCheckLevel
from utils.get_corners import get_all_good_corners_from_images, quick_get_good_corners


class MonoHandleResult(HandleResult):
    def __init__(self):
        HandleResult.__init__(self)
        self.show_img = None  # 返回的用于显示的图像

        #### 标定未完成时需要返回的参数
        self.progress = None  # 标定进度
        #### 标定完成时需要返回的参数
        self.linear_error = -1.0  # 线性误差


class MonoCalibrator(CameraCalibrator):
    """
    单目相机标定
    """

    def __init__(self, chessboard_info, camera_info):
        super(MonoCalibrator, self).__init__(chessboard_info)
        self.camera_info = camera_info

        # 存储采集到的数据，包括达标的images和对应的metrics、corners
        # {"images": [], "metrics": [], "corners": []}
        self.stored_data = {}
        self.stored_data["images"] = []
        self.stored_data["metrics"] = []
        self.stored_data["corners"] = []

        ########## static parameters ##########
        # 单目标定的四个指标名字
        #   X : x方向的覆盖率
        #   Y : y方向的覆盖率
        #   Size : 棋盘格面积占图像面积的比例
        #   Skew : 拐角覆盖率
        self.metrics_name = ["X", "Y", "Size", "Skew"]
        self.metrics_min_distance = 0.2  # metrics的曼哈顿距离阈值
        self.metrics_ranges = [0.7, 0.7, 0.1, 0.3]  # metrics达标范围

    def handle_frame(self, frame):
        """处理输入的图像，处理目前分两种类型
        1. 如果相机是已经标定过的,则对输入进行校正
        2. 如果相机是未标定过的,则采集图像进行标定
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        linear_error = -1
        mono_handle_result = MonoHandleResult()
        board_cols = self.chessboard_info.n_cols
        board_rows = self.chessboard_info.n_rows

        # 首先快速检测角点
        (ok, corners, resized_img, downsampled_corners, (x_scale, y_scale)) = quick_get_good_corners(
            gray, board_cols, board_rows
        )

        if self.calibrated:
            # 如果标定是已经完成的,则
            #   1. 图像去畸变,然后缩放
            #   2. 角点去畸变
            #   3. 计算liner error
            #   4. 缩放去畸变后的角点，绘制在缩放且去畸变后的图像上
            #   5. 返回必要的信息

            # 1. 图像去畸变
            undistorted_gray = cv2.remap(gray, self.camera_info.map1, self.camera_info.map2, cv2.INTER_LINEAR)
            if x_scale != 1.0 or y_scale != 1.0:
                resized_undistorted_gray = cv2.resize(undistorted_gray, (resized_img.shape[1], resized_img.shape[0]))
            resized_undistorted_img = cv2.cvtColor(resized_undistorted_gray, cv2.COLOR_GRAY2BGR)

            if ok:
                # 2. 角点去畸变
                undistorted_corners = self._undistort_points(corners, self.camera_info)
                # 3. 计算liner error
                linear_error = self.__linear_error(undistorted_corners, board_cols, board_rows)
                # 4. 缩放去畸变后的角点，绘制在缩放且去畸变后的图像上
                resized_undistorted_corners = undistorted_corners.copy()
                resized_undistorted_corners[:, :, 0] /= x_scale
                resized_undistorted_corners[:, :, 1] /= y_scale
                cv2.drawChessboardCorners(
                    resized_undistorted_img, (board_cols, board_rows), resized_undistorted_corners, True
                )

                # print linear_error
                print("linear_error:", linear_error)

                # 5. 返回必要的信息
                mono_handle_result.show_img = resized_undistorted_img
                mono_handle_result.linear_error = linear_error
                return True, mono_handle_result
            else:
                # 角点检测失败
                mono_handle_result.show_img = resized_undistorted_img
                return False, mono_handle_result
        else:
            # 如果标定是未完成的,则
            #   1. 绘制快速检测的角点
            #   2. 计算角点的metrics，并结合历史数据进行其质量进行判断决定是否保存
            #   3. 检查是否已经采集到足够数据，如果是，则进行标定
            #   4. 返回必要的信息
            resized_gray = cv2.cvtColor(resized_img, cv2.COLOR_GRAY2BGR)
            if ok:
                # 1. 绘制快速检测的角点
                cv2.drawChessboardCorners(resized_gray, (board_cols, board_rows), downsampled_corners, True)
                # 2. 计算角点的metrics，并结合历史数据进行其质量进行判断决定是否保存
                # 对本次检测的角点信息与角点数据库中的数据进行比较,如果是符合要求的角点,则加入数据库，反之则丢弃本次检测的角点
                current_metrics = self._calculate_metrics(corners)
                if self._judge_metrics_is_qualified(current_metrics):
                    self.stored_data["images"].append(gray)
                    self.stored_data["metrics"].append(current_metrics)
                    self.stored_data["corners"].append(corners)
                    print(
                        (
                            "*** Added sample %d, p_x = %.3f, p_y = %.3f, p_size = %.3f, skew = %.3f"
                            % tuple([len(self.stored_data["images"])] + current_metrics)
                        )
                    )

                # 3. 检查是否已经采集到足够数据，如果是，则进行标定
                # 计算数据是否采集足够，如果采集足够，则进行标定
                collection_if_completed, progress = self._compute_collection_progress()
                if collection_if_completed:
                    print("*******start calibrate*******")
                    self._do_calibration()
                    print("*******calibrate finished*******")
                    self.calibrated = True

                # 4. 返回必要的信息
                mono_handle_result.show_img = resized_gray
                mono_handle_result.progress = progress

                return True, mono_handle_result
            else:
                mono_handle_result.show_img = resized_gray
                return False, mono_handle_result

    def _calculate_metrics(self, corners):
        """根据当前的角点和棋盘信息计算单目标定的几个指标

        Args:
            corners (_type_): 采集的所有角点

        Returns:
            list: [p_x, p_y, p_size, skew] 四个指标
        """

        def get_outside_corners(corners):
            """
            返回角点中相对于标定板的四个拐角，顺序分别为,(up_left, up_right, down_right, down_left).
            Args:
                corners (list): 角点
                xdim (int): 棋盘格的x方向的格子数
                ydim (int): 棋盘格的y方向的格子数
            Returns:
                list: [(up_left, up_right, down_right, down_left)] 四个角点
            """
            # 似乎有bug
            # up_left = corners[0, 0]
            # up_right = corners[xdim - 1, 0]
            # down_right = corners[-1, 0]
            # down_left = corners[-xdim, 0]

            up_left = corners[0, 0]
            up_right = corners[-1, 0]
            down_right = corners[-1, -1]
            down_left = corners[0, -1]

            return (up_left, up_right, down_right, down_left)

        def calculate_area(corners):
            """
            根据四边形的四个顶点计算四边形的面积（通过向量叉乘计算）
            """
            (up_left, up_right, down_right, down_left) = corners
            a = up_right - up_left
            b = down_right - up_right
            c = down_left - down_right
            p = b + c
            q = a + b
            return abs(p[0] * q[1] - p[1] * q[0]) / 2.0

        def calculate_skew(corners):
            """通过计算四边形右上角度与90度的偏差来近似的表示检测出的棋盘角点的倾斜度,即skew指标
            0 <= skew < =1 0为无倾斜,1为倾斜无限大
            """
            # TODO Using three nearby interior corners might be more robust, outside corners occasionally
            # get mis-detected
            up_left, up_right, down_right, _ = corners

            def angle(a, b, c):
                """返回直线ab,bc的夹角 (弧度制)"""
                ab = a - b
                cb = c - b
                return math.acos(np.dot(ab, cb) / (np.linalg.norm(ab) * np.linalg.norm(cb)))

            skew = min(1.0, 2.0 * abs((math.pi / 2.0) - angle(up_left, up_right, down_right)))
            return skew

        (width, height) = self.camera_info.resolution
        Xs = corners[:, :, 0]  # 所有 corner x坐标
        Ys = corners[:, :, 1]  # 所有 corner y坐标
        outside_corners = get_outside_corners(corners)

        area = calculate_area(outside_corners)

        # X Y metrics
        border = math.sqrt(area)
        p_x = min(1.0, max(0.0, (np.mean(Xs) - border / 2) / (width - border)))
        p_y = min(1.0, max(0.0, (np.mean(Ys) - border / 2) / (height - border)))

        # Size metrics
        p_size = math.sqrt(area / (width * height))

        # Skew metrics
        skew = calculate_skew(outside_corners)

        return [p_x, p_y, p_size, skew]

    def _judge_metrics_is_qualified(self, current_metrics):
        """判断当前检测出的角点是否是一个合格样本
        通过判断: 棋盘格的偏移量是否在预先设置的范围内 (通过计算当前角点指标与历史角点指标的曼哈顿距离，距离太小则丢弃)

        Args:
            current_metric (tuple): 当前检测出corner的图像的corners的评价指标
            corners (list): 当前检测出corner的图像的corners
            last_frame_corners (list): 上一张检测出corner的图像的corners
        Returns: True or False
        """

        def param_distance(p1, p2):
            """计算指标空间中两个样本的曼哈顿距离"""
            return sum([abs(a - b) for (a, b) in zip(p1, p2)])

        # 没有历史角点，则直接返回True
        if len(self.stored_data["metrics"]) == 0:
            return True

        history_metrics_list = self.stored_data["metrics"]
        d = min([param_distance(current_metrics, p) for p in history_metrics_list])
        # TODO What's a good threshold here? Should it be configurable?
        if d <= self.metrics_min_distance:
            return False

        return True

    def _compute_collection_progress(self):
        """计算收集数据的进度
        遍历所有已采样样本，计算评价指标在各个维度上的进度
        Returns:
            (list):[
                ( 'X' , min_p_x , max_p_x , x_progress ),
                ( 'Y' , min_p_y , max_p_y , y_progress ),
                ( 'Size' , min_p_size , max_p_size , size_progress ),
                ( 'Skew' , min_p_skew , max_p_skew , skew_progress )
            ]
        """

        def lmin(seq1, seq2):
            """两个等长度序列中对应位置的最小值"""
            return [min(a, b) for (a, b) in zip(seq1, seq2)]

        def lmax(seq1, seq2):
            """两个等长度序列中对应位置的最大值"""
            return [max(a, b) for (a, b) in zip(seq1, seq2)]

        if not self.stored_data:
            return False, None

        # 遍历统计当前metrics在四个维度上的上下限(统计指标空间的边界)
        metrics_list = self.stored_data["metrics"]
        min_metrics = metrics_list[0]
        max_metrics = metrics_list[0]
        for i in metrics_list[1:]:
            min_metrics = lmin(min_metrics, i)
            max_metrics = lmax(max_metrics, i)
        # Don't reward small size or skew
        min_metrics = [min_metrics[0], min_metrics[1], 0.0, 0.0]

        # 根据上下限与预设的进度阈值进行比较,得到当前标定在四个维度上的进度，并以此进度作为标定的进度
        # 1. 当进度达到设定阈值的时候，返回True，意味着已经采集了足够多的样本了，可以进行标定
        # 2. 当进度还没有达到阈值的，可以根据四个维度阈值与目标的差距，从而得知应该还需要采集哪些数据 (暂时不考虑 size 和 skew 的情况)
        #     1. 如果 X 过小，表示在x方向要再多拍一些样本
        #     2. 如果 Y 过小，表示在y方向要再多拍一些样本
        # NOTE : 如果已经采集了足够多的样本，但进度还是不符合需求，这种情况可能是以为阈值设置的不合理，我们也认为可以用于标定了
        progress = [min((hi - lo) / r, 1.0) for (lo, hi, r) in zip(min_metrics, max_metrics, self.metrics_ranges)]
        collection_if_completed = (len(self.stored_data["images"]) >= 40) or all([p == 1.0 for p in progress])
        return collection_if_completed, list(zip(self.metrics_name, min_metrics, max_metrics, progress))

    def _do_calibration(self):
        """数据采集完毕后，使用收集到的数据进行正式标定
        NOTE : 之前采集数据时,为了速度,将图像大小降采样至VGA(640*480)再检测角点，在正式标定的时候，要使用原图检测角点
        """
        images = self.stored_data["images"]
        board_cols = self.chessboard_info.n_cols
        board_rows = self.chessboard_info.n_rows
        checkerboard_flags = self.calibrator_function_flags.cv2_findChessboardCorners_flags
        calibration_flags = self.calibrator_function_flags
        all_good_corners, corners = get_all_good_corners_from_images(images, board_cols, board_rows, checkerboard_flags)
        self._do_calibration_from_corners(all_good_corners, calibration_flags)
        self.calibrated = True

    def _do_calibration_from_corners(self, corners, calibrate_flags):
        """通过所有采集的图像的角点计算相机内参。
        标定方法根据相机类型自适应
        Args:
            corners (list):所有采集的图像的角点
            calibrate_flags (int): 标定标志
        """
        boards = [self.chessboard_info.BOARD for i in range(len(corners))]

        if self.camera_info.camera_model == CameraModel.PINHOLE:
            print("mono pinhole calibration...")
            (
                reproj_err,
                self.camera_info.intrinsics_matrix,
                self.camera_info.distortion_coefficients,
                self.camera_info.rvecs,
                self.camera_info.tvecs,
            ) = cv2.calibrateCamera(
                boards,
                corners,
                self.camera_info.resolution,
                np.eye(3, 3),
                np.zeros((5, 1)),
                flags=calibrate_flags.cv2_calibrateCamera_flags,
            )
            print("pinhole calibration reproj_err : ", reproj_err)

        elif self.camera_info.camera_model == CameraModel.FISHEYE:
            print("mono fisheye calibration...")
            # WARNING: cv2.fisheye.calibrate wants float64 points
            corners = np.asarray(corners, dtype=np.float64)
            boards = np.asarray(boards, dtype=np.float64)
            (
                reproj_err,
                self.camera_info.intrinsics_matrix,
                self.camera_info.distortion_coefficients,
                self.camera_info.rvecs,
                self.camera_info.tvecs,
            ) = cv2.fisheye.calibrate(
                boards,
                corners,
                self.camera_info.resolution,
                np.eye(3, 3),
                np.zeros((4, 1)),
                flags=calibrate_flags.cv2_fisheye_calibrate_flags,
            )
            print("fisheye calibration reproj_err : ", reproj_err)

        info_check_level = CameraInfoCheckLevel.COMPLETED
        self.camera_info.info_check(info_check_level)
        self.calibrated = True

    @staticmethod
    def __linear_error(corners, board_cols, board_rows):

        """
        计算棋盘角点的线性误差。误差=每个角点距离所在行列直线的距离的RMSE
        """

        if corners is None:
            return None

        corners = np.squeeze(corners)

        def pt2line(x0, y0, x1, y1, x2, y2):
            """点到直线的距离:(x0,y0)到(x1,y1)与(x2,y2)构成的直线的距离"""
            return abs((x2 - x1) * (y1 - y0) - (x1 - x0) * (y2 - y1)) / math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

        n_pts = board_cols * board_rows

        ids = np.arange(n_pts).reshape((n_pts, 1))

        ids_to_idx = dict((ids[i, 0], i) for i in range(len(ids)))

        errors = []
        for row in range(board_rows):
            row_min = row * board_cols
            row_max = (row + 1) * board_cols
            pts_in_row = [x for x in ids if row_min <= x < row_max]

            # not enough points to calculate error
            if len(pts_in_row) <= 2:
                continue

            left_pt = min(pts_in_row)[0]
            right_pt = max(pts_in_row)[0]
            x_left = corners[ids_to_idx[left_pt], 0]
            y_left = corners[ids_to_idx[left_pt], 1]
            x_right = corners[ids_to_idx[right_pt], 0]
            y_right = corners[ids_to_idx[right_pt], 1]

            for pt in pts_in_row:
                if pt[0] in (left_pt, right_pt):
                    continue
                x = corners[ids_to_idx[pt[0]], 0]
                y = corners[ids_to_idx[pt[0]], 1]
                errors.append(pt2line(x, y, x_left, y_left, x_right, y_right))

        if errors:
            return math.sqrt(sum([e**2 for e in errors]) / len(errors))
        else:
            return None
