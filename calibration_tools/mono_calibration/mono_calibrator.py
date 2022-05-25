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
    单目相机标定
    """

    def __init__(self, chessboard_info, camera_info):
        super(MonoCalibrator, self).__init__(chessboard_info)
        self.camera_info = camera_info

        # private
        # 单目标定的四个指标
        #   X : x方向的覆盖率
        #   Y : y方向的覆盖率
        #   Size : 棋盘格面积占图像面积的比例
        #   Skew : 拐角覆盖率
        self.__metrics = ["X", "Y", "Size", "Skew"]
        # 不知道存什么用的
        self._db = []
        # 收集到的角点
        self.__good_corners = []

    def handle_frame(self, frame):
        """处理输入的图像，处理目前分两种类型
        1. 如果相机是已经标定过的,则对输入进行校正
        2. 如果相机是未标定过的,则采集图像进行标定
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        linear_error = -1
        mono_handle_result = MonoHandleResult()

        (ok, resized_img, corners, downsampled_corners, (x_scale, y_scale)) = self._quick_get_good_corners(gray)

        # resized_img, corners, downsampled_corners, ids, board, (x_scale, y_scale) = self._quick_get_good_corners(gray)

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
                cv2.drawChessboardCorners(
                    resized_undistorted_img, (board.n_cols, board.n_rows), resized_undistorted_corners, True
                )

                # debug
                # print linear_error
                print("linear_error:", linear_error)
            resized_img_with_corners = resized_undistorted_img
        else:
            resized_gray = cv2.cvtColor(resized_img, cv2.COLOR_GRAY2BGR)
            if corners is not None:
                cv2.drawChessboardCorners(
                    resized_gray, (self.chessboard_info.n_cols, self.chessboard_info.n_rows), downsampled_corners, True
                )

                # 对本次检测的角点信息与角点数据库中的数据进行比较,如果是符合要求的角点,则加入数据库，反之则丢弃本次检测的角点
                curent_metric = self._calculate_metrics(corners)
                if self._is_qualified_sample(
                    curent_metric, corners, ids, self._last_frame_corners, self._last_frame_ids
                ):
                    self._db.append((params, gray))
                    self._good_corners.append((corners, None, board))

                    print(
                        (
                            "*** Added sample %d, p_x = %.3f, p_y = %.3f, p_size = %.3f, skew = %.3f"
                            % tuple([len(self._db)] + params)
                        )
                    )
            resized_img_with_corners = resized_gray

        self._last_frame_corners = corners
        self._last_frame_ids = ids

        mono_handle_result.resized_img_with_corners = resized_img_with_corners
        mono_handle_result.params = self._compute_goodenough()
        mono_handle_result.linear_error = linear_error
        return mono_handle_result

    def _calculate_metrics(self, corners):
        """根据当前的角点和棋盘信息计算单目标定的几个指标

        Args:
            corners (_type_): 采集的所有角点
            board (_type_): 标定板信息
            size (_type_): 相机的图像尺寸

        Returns:
            list: [p_x, p_y, p_size, skew] 四个指标
        """
        (width, height) = self.camera_info.resolution
        Xs = corners[:, :, 0]  # 所有 corner x坐标
        Ys = corners[:, :, 1]  # 所有 corner y坐标
        outside_corners = self.__get_outside_corners(corners, board)

        area = self.__calculate_area(outside_corners)

        # X Y metrics
        border = math.sqrt(area)
        p_x = min(1.0, max(0.0, (np.mean(Xs) - border / 2) / (width - border)))
        p_y = min(1.0, max(0.0, (np.mean(Ys) - border / 2) / (height - border)))

        # Size metrics
        p_size = math.sqrt(area / (width * height))

        # Skew metrics
        skew = self.__calculate_skew(outside_corners)

        return [p_x, p_y, p_size, skew]

    def _is_qualified_sample(self, curent_metric, corners, last_frame_corners):
        """判断当前检测出的角点是否是一个合格样本
        通过判断: 棋盘格的偏移量是否在预先设置的范围内 (通过计算当前角点指标与历史角点指标的曼哈顿距离，距离太小则丢弃)

        Args:
            curent_metric (tuple): 当前检测出corner的图像的corners的评价指标
            corners (list): 当前检测出corner的图像的corners
            last_frame_corners (list): 上一张检测出corner的图像的corners
        Returns: True or False
        """

        def param_distance(p1, p2):
            """计算指标空间中两个样本的曼哈顿距离"""
            return sum([abs(a - b) for (a, b) in zip(p1, p2)])

        if not self._db:
            return True

        db_params = [sample[0] for sample in self._db]
        d = min([param_distance(params, p) for p in db_params])
        # TODO What's a good threshold here? Should it be configurable?
        if d <= self.__calibrator_target_threshold.param_min_distance:
            return False

        return True

    def _do_calibration(self):
        """数据采集完毕后，使用收集到的数据进行正式标定
        NOTE : 之前采集数据时,为了速度,将图像大小降采样至VGA(640*480)再检测角点，在正式标定的时候，要使用原图检测角点
        """
        if not self._good_corners:
            print("**** Collecting good corners from database! ****")  # DEBUG
            images = [i for (p, i) in self._db]
            self._good_corners = self.collect_corners(images)
        self.size = (self._db[0][1].shape[1], self._db[0][1].shape[0])  # TODO Needs to be set externally
        self._cal_fromcorners(self._good_corners)
        self.calibrated = True

    @staticmethod
    def __get_outside_corners(corners, board):
        """
        返回角点中相对于标定板的四个拐角，顺序分别为,(up_left, up_right, down_right, down_left).
        """
        xdim = board.n_cols
        ydim = board.n_rows

        up_left = corners[0, 0]
        up_right = corners[xdim - 1, 0]
        down_right = corners[-1, 0]
        down_left = corners[-xdim, 0]

        return (up_left, up_right, down_right, down_left)

    @staticmethod
    def __calculate_skew(corners):
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

    @staticmethod
    def __calculate_area(corners):
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

    @staticmethod
    def __linear_error(corners, ids, board):

        """
        计算棋盘角点的线性误差。误差=每个角点距离所在行列直线的距离的RMSE
        """

        if corners is None:
            return None

        corners = np.squeeze(corners)

        def pt2line(x0, y0, x1, y1, x2, y2):
            """点到直线的距离:(x0,y0)到(x1,y1)与(x2,y2)构成的直线的距离"""
            return abs((x2 - x1) * (y1 - y0) - (x1 - x0) * (y2 - y1)) / math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

        n_cols = board.n_cols
        n_rows = board.n_rows
        n_pts = n_cols * n_rows

        if ids is None:
            ids = np.arange(n_pts).reshape((n_pts, 1))

        ids_to_idx = dict((ids[i, 0], i) for i in range(len(ids)))

        errors = []
        for row in range(n_rows):
            row_min = row * n_cols
            row_max = (row + 1) * n_cols
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

    @staticmethod
    def __lmin(seq1, seq2):
        """两个等长度序列中对应位置的最小值"""
        return [min(a, b) for (a, b) in zip(seq1, seq2)]

    @staticmethod
    def __lmax(seq1, seq2):
        """两个等长度序列中对应位置的最大值"""
        return [max(a, b) for (a, b) in zip(seq1, seq2)]

    @staticmethod
    def __pdist(p1, p2):
        """
        两点距离. p1 = (x, y), p2 = (x, y)
        """
        return math.sqrt(math.pow(p1[0] - p2[0], 2) + math.pow(p1[1] - p2[1], 2))
