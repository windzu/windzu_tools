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
import numpy as np
import math
import time
import sys

# local
sys.path.append("../../")
from common.camera_common import CameraModel, Patterns, ChessboardInfo, InfoCheckLevel
from utils.camera_utils import parse_cameras_config, camera_info_check


def lmin(seq1, seq2):
    """ Pairwise minimum of two sequences """
    return [min(a, b) for (a, b) in zip(seq1, seq2)]


def lmax(seq1, seq2):
    """ Pairwise maximum of two sequences """
    return [max(a, b) for (a, b) in zip(seq1, seq2)]


def _pdist(p1, p2):
    """
    Distance bwt two points. p1 = (x, y), p2 = (x, y)
    """
    return math.sqrt(math.pow(p1[0] - p2[0], 2) + math.pow(p1[1] - p2[1], 2))


def _get_outside_corners(corners, board):
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


def _calculate_skew(corners):
    """通过计算四边形右上角度与90度的偏差来近似的表示检测出的棋盘角点的倾斜度
    0 <= skew < =1 0为无倾斜,1为倾斜无限大
    """
    # TODO Using three nearby interior corners might be more robust, outside corners occasionally
    # get mis-detected
    up_left, up_right, down_right, _ = corners

    def angle(a, b, c):
        """返回直线ab,bc的夹角 (弧度制)
        """
        ab = a - b
        cb = c - b
        return math.acos(np.dot(ab, cb) / (np.linalg.norm(ab) * np.linalg.norm(cb)))

    skew = min(1.0, 2.0 * abs((math.pi / 2.0) - angle(up_left, up_right, down_right)))
    return skew


def _calculate_area(corners):
    """
    通过四边形的四个顶点计算四边形的面积，通过向量叉乘计算
    """
    (up_left, up_right, down_right, down_left) = corners
    a = up_right - up_left
    b = down_right - up_right
    c = down_left - down_right
    p = b + c
    q = a + b
    return abs(p[0] * q[1] - p[1] * q[0]) / 2.0


def _get_corners(img, board, refine=True, checkerboard_flags=0):
    """
    从图像中检测chessboard的角点
    """
    h = img.shape[0]
    w = img.shape[1]
    if len(img.shape) == 3 and img.shape[2] == 3:
        mono = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        mono = img
    (ok, corners) = cv2.findChessboardCorners(mono, (board.n_cols, board.n_rows), flags=checkerboard_flags)
    if not ok:
        return (ok, corners)

    # 对于靠近边缘的角点，如果与边缘距离小于BORDER像素(默认设置为8)，则设置ok为false从而丢弃该检测结果，
    # NOTE:这对于低分辨率的相机来不友好，因为8像素对于非常低像素图像而言不是一个小数目
    BORDER = 8
    if not all(
        [(BORDER < corners[i, 0, 0] < (w - BORDER)) and (BORDER < corners[i, 0, 1] < (h - BORDER)) for i in range(corners.shape[0])]
    ):
        ok = False

    # 确保角点数组的顺序为:从上到下,从左到右
    # NOTE :
    #   1. 行列不同的棋盘的摆放方位与cv2.findChessboardCorners中设置的棋盘尺寸一致，例如棋盘尺寸为(6,9) (cols,rows)，则棋盘拜访必须是长边竖直，短边水平
    #   2. 行列相同，则无所谓摆放的方向
    # a. 使用行列不同的棋盘格，角点顺序可能会反方向，通过判断第一个和最后一个角点的y坐标来判断，如果想法则将corners数组翻转
    if board.n_rows != board.n_cols:
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
                corners = np.rot90(corners.reshape(board.n_rows, board.n_cols, 2)).reshape(board.n_cols * board.n_rows, 1, 2)
            else:
                corners = np.rot90(corners.reshape(board.n_rows, board.n_cols, 2), 3).reshape(board.n_cols * board.n_rows, 1, 2)

    # 按照行和列的方向遍历角点，找到距离最近的角点间的距离，设置其作为cv2.cornerSubPix的搜寻半径，这样可以在保证速度情况下最大范围的进行亚像素搜索
    # NOTE : 这参考其ros的角点检测代码，个人认为亚像素搜索没必要这么大范围，除非是分辨率很低的情况或者是棋盘格square size很小，这样做可以确定搜索的边界；
    # 而对于高像素的图像，随便设置为5就可以了，两个角点间的距离肯定不会小于5，而5像素范围内就可以很好的搜索
    # 不过ros的这种做法确实更具rubust
    if refine and ok:
        min_distance = float("inf")
        for row in range(board.n_rows):
            for col in range(board.n_cols - 1):
                index = row * board.n_cols + col
                min_distance = min(min_distance, _pdist(corners[index, 0], corners[index + 1, 0]))
        for row in range(board.n_rows - 1):
            for col in range(board.n_cols):
                index = row * board.n_cols + col
                min_distance = min(min_distance, _pdist(corners[index, 0], corners[index + board.n_cols, 0]))
        radius = int(math.ceil(min_distance * 0.5))
        cv2.cornerSubPix(mono, corners, (radius, radius), (-1, -1), (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1))

    return (ok, corners)


class CalibrationException(Exception):
    pass


class HandleResult(object):
    """
    Passed to CalibrationNode after image handled. Allows plotting of images
    with detected corner points
    """

    def __init__(self):
        self.params = None


class MonoHandleResult(HandleResult):
    def __init__(self):
        HandleResult.__init__(self)
        self.resized_img_with_corners = None
        self.linear_error = -1.0


class Calibrator:
    """相机标定的基类
    NOTE : 后续的单目相机（小孔和鱼眼）的标定、双目相机标定等均要继承自此类
    此类的设计目标：
        1. 类变量 : 包含标定所需的所有数据，包括相机类型、内参、畸变系数、所使用标定板信息等
        2. 类函数 : 包含标定步骤中通用的方法，包括标定板的角点检测、检测到的角点质量的评估

    """

    def __init__(self, chessboard_info, camera_info, calibrator_function_flags, calibrator_target_threshold):
        self.chessboard_info = chessboard_info
        self.camera_info = camera_info
        self.calibrator_function_flags = calibrator_function_flags
        self.calibrator_target_threshold = calibrator_target_threshold

        # 临时状态量
        self.db = []
        self.good_corners = []
        self.calibrated = False  # 标定是否完成的标志位
        self.goodenough = False  # 采集的样本是否达到要求的标志位
        self.last_frame_corners = None
        self.last_frame_ids = None

    def remap(self, src):
        return cv2.remap(src, self.camera_info.map1, self.camera_info.map2, cv2.INTER_LINEAR)

    def get_parameters(self, corners, ids, board, size):
        """计算当前corners的评价指标,并返回评价指标
        
        Args:
            corners (list): 检测出的角点(坐标是相较与原图)
            ids (none): 暂未使用
            board (ChessboardInfo): 所使用的标定板的信息
            size (tuple): 原图尺寸 (width,height)
        Returns:
            p_x (float): 标定板在x方向上的偏移量 范围[0,1],0表示标定板在图像最左边,1表示标定板在图像最右边
            p_y (float): 标定板在y方向上的偏移量 范围[0,1],0表示标定板在图像最上边,1表示标定板在图像最下边
            p_size (float): 标定板占原图的比例 范围[0,1]
            Skew (float): 标定板的倾斜度 范围[0,1]
        """
        (width, height) = size

        Xs = corners[:, :, 0]  # 所有 corner x坐标
        Ys = corners[:, :, 1]  # 所有 corner y坐标
        if board.pattern == Patterns.CHESSBOARD:
            outside_corners = _get_outside_corners(corners, board)
        else:
            outside_corners = None
            pass

        # 根据检测出来的棋盘格角点的四个拐角点所构成的四边行，计算其一些属性值(理论上这个四边行一定不是矩形)
        # area : 四边行面积
        # skew : 四边行的拐角相较于直角的偏差(可以用此值来衡量该四边行与矩形的相似度)
        area = _calculate_area(outside_corners)
        skew = _calculate_skew(outside_corners)

        # 计算棋盘格靠近边缘程度
        border = math.sqrt(area)
        p_x = min(1.0, max(0.0, (np.mean(Xs) - border / 2) / (width - border)))
        p_y = min(1.0, max(0.0, (np.mean(Ys) - border / 2) / (height - border)))

        # 计算棋盘格面积占图像面积的比例
        p_size = math.sqrt(area / (width * height))
        params = [p_x, p_y, p_size, skew]
        return params

    def is_slow_moving(self, corners, ids, last_frame_corners, last_frame_ids):
        """ 通过与上一帧读比，计算棋盘格的移动速度(平均移动距离)，然后与预先设置的移速阈值对比，
        如果移速低于阈值,则认为棋盘格移动很慢.返回True,否则返回False
            """
        # If we don't have previous frame corners, we can't accept the sample
        if last_frame_corners is None:
            return False
        if ids is None:
            num_corners = len(corners)
            corner_deltas = (corners - last_frame_corners).reshape(num_corners, 2)
        else:
            corner_deltas = []
            last_frame_ids = list(last_frame_ids.transpose()[0])
            for i, c_id in enumerate(ids):
                try:
                    last_i = last_frame_ids.index(c_id)
                    corner_deltas.append(corners[i] - last_frame_corners[last_i])
                except ValueError:
                    pass
            corner_deltas = np.concatenate(corner_deltas)

        # Average distance travelled overall for all corners
        average_motion = np.average(np.linalg.norm(corner_deltas, axis=1))
        return average_motion <= self.calibrator_target_threshold.chessboard_max_speed

    def is_good_sample(self, params, corners, ids, last_frame_corners, last_frame_ids):
        """ 判断当前检测出的角点是否是一个好的样本
        通过两方面判断:
        1. 棋盘格的偏移量是否在预先设置的范围内 (通过计算当前角点指标与历史角点指标的曼哈顿距离，距离太小则丢弃)
        2. 棋盘格的移动速度是否低于阈值 (移动速度不能太快，一般不启用)

        Args:
            params (tuple): 当前检测出corner的图像的corners的评价指标
            corners (list): 当前检测出corner的图像的corners
            ids (none)): 尚未启用
            last_frame_corners (list): 上一张检测出corner的图像的corners
            last_frame_ids (none): 尚未启用
        Returns:
            True: 如果当前检测出的corners的评价指标比上一帧的corners的评价指标高，则返回True
            False: 否则返回False
        """
        if not self.db:
            return True

        def param_distance(p1, p2):
            """计算指标空间中两个样本的曼哈顿距离"""
            return sum([abs(a - b) for (a, b) in zip(p1, p2)])

        db_params = [sample[0] for sample in self.db]
        d = min([param_distance(params, p) for p in db_params])
        # print "d = %.3f" % d #DEBUG
        # TODO What's a good threshold here? Should it be configurable?
        if d <= self.calibrator_target_threshold.param_min_distance:
            return False

        if self.calibrator_target_threshold.chessboard_max_speed > 0:
            if not self.is_slow_moving(corners, ids, last_frame_corners, last_frame_ids):
                return False

        # All tests passed, image should be good for calibration
        return True

    _param_names = ["X", "Y", "Size", "Skew"]

    def compute_goodenough(self):
        """遍历所有已采样样本，计算评价指标在各个维度上的进度
        Returns:
            (list):[
                ( 'X' , min_p_x , max_p_x , x_progress ),
                ( 'Y' , min_p_y , max_p_y , y_progress ),
                ( 'Size' , min_p_size , max_p_size , size_progress ),
                ( 'Skew' , min_p_skew , max_p_skew , skew_progress )
            ]
        """
        if not self.db:
            return None

        # 遍历db中所有的params,统计当前db中params在四个维度上的上下限(统计指标空间的边界)
        all_params = [sample[0] for sample in self.db]
        min_params = all_params[0]
        max_params = all_params[0]
        for params in all_params[1:]:
            min_params = lmin(min_params, params)
            max_params = lmax(max_params, params)
        # Don't reward small size or skew
        min_params = [min_params[0], min_params[1], 0.0, 0.0]

        # 根据上下限与预设的进度阈值进行比较,得到当前标定在四个维度上的进度，并以此进度作为标定的进度
        # 1. 当进度达到设定阈值的时候，返回True，意味着已经采集了足够多的样本了，可以进行标定
        # 2. 当进度还没有达到阈值的，可以根据四个维度阈值与目标的差距，从而得知应该还需要采集哪些数据 (暂时不考虑 size 和 skew 的情况)
        #     1. 如果 X 过小，表示在x方向要再多拍一些样本
        #     2. 如果 Y 过小，表示在y方向要再多拍一些样本
        # NOTE : 如果已经采集了足够多的样本，但进度还是不符合需求，这种情况可能是以为阈值设置的不合理，我们也认为可以用于标定了
        progress = [
            min((hi - lo) / r, 1.0) for (lo, hi, r) in zip(min_params, max_params, self.calibrator_target_threshold.param_ranges)
        ]
        self.goodenough = (len(self.db) >= 40) or all([p == 1.0 for p in progress])

        return list(zip(self._param_names, min_params, max_params, progress))

    def get_corners(self, img, refine=True):
        """
        根据标定板类型，选择不同的检测角点方法，并返回检测到的角点
        NOTE : 返回的检测到的角点是从上到下、从左到右的顺序排列的

        Returns (ok, corners, ids, board)
        """
        if self.chessboard_info.pattern == Patterns.CHESSBOARD:
            (ok, corners) = _get_corners(
                img=img,
                board=self.chessboard_info,
                refine=refine,
                checkerboard_flags=self.calibrator_function_flags.cv2_findChessboardCorners_flags,
            )
            ids = None
        else:
            # 暂时不支持其他类型的标定板
            pass
        if ok:
            return (ok, corners, ids, self.chessboard_info)
        return (False, None, None, None)

    def downsample_and_detect(self, img):
        """
        直接对原图进行角点检测速度太慢，将其降采样后检测，如果检测到结果再缩放回原图大小，再次使用亚像素检测，这样可以做到实时的检测
        NOTE : 因为如果在降采样后的图像中能检测出角点，那么在原图中大概率也是可以检测出的，其中为了保证缩放后的检测速度,
        将尺寸一律降采样至VGA为标准,通过计算面积比得到缩放系数

        Returns (resized_img, corners, downsampled_corners, ids, board, (x_scale, y_scale)).
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

        # 根据标定板类型，选择不同的检测角点方法，并返回检测到的角点
        # NOTE : 目前只支持检测棋盘格角点，其他类型的标定板暂时不支持
        if self.chessboard_info.pattern == Patterns.CHESSBOARD:
            (ok, downsampled_corners, ids, board) = self.get_corners(resized_img, refine=True)

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
            # TODO ： 支持其他类型的标定板的检测
            # 对于其他类型的标定板的角点检测暂时不支持(据说对圆形检测很快，不需要缩放都可以实时检测)
            pass

        return (resized_img, corners, downsampled_corners, ids, board, (x_scale, y_scale))

    def collect_corners(self, images):
        """检测图像列表中所以图像的角点
        Args:
            images (list): 图像列表
        Returns:
            goodcorners (list): [ (corners, ids, ChessboardInfo) ]
        """
        self.size = (images[0].shape[1], images[0].shape[0])
        corners = [self.get_corners(i) for i in images]

        goodcorners = [(co, ids, b) for (ok, co, ids, b) in corners if ok]
        if not goodcorners:
            raise CalibrationException("No corners found in images!")
        return goodcorners

    def cal_fromcorners(self, good):
        """通过所有采集的图像的角点计算相机内参
        Params:
            good (list): [ (corners, ids, ChessboardInfo) ]
        """

        (corners, ids, _) = zip(*good)
        # opts = self.mk_object_points(boards)
        boards = [self.chessboard_info.BOARD for i in range(len(good))]
        # If FIX_ASPECT_RATIO flag set, enforce focal lengths have 1/1 ratio
        # intrinsics_in = np.eye(3, dtype=np.float64)

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
                self.size,
                np.eye(3, 3),
                np.zeros((5, 1)),
                flags=self.calibrator_function_flags.cv2_calibrateCamera_flags,
            )
            # OpenCV returns more than 8 coefficients (the additional ones all zeros) when CALIB_RATIONAL_MODEL is set.
            # The extra ones include e.g. thin prism coefficients, which we are not interested in.
            # self.camera_info.distortion_coefficients = dist_coeffs.flat[:8].reshape(-1, 1)
            print("pinhole calibration reproj_err : ", reproj_err)
            # debug
            print("pinhole calibration intrinsics_matrix : ", self.camera_info.intrinsics_matrix)

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
                self.size,
                np.eye(3, 3),
                np.zeros((4, 1)),
                flags=self.calibrator_function_flags.cv2_fisheye_calibrate_flags,
            )
            print("fisheye calibration reproj_err : ", reproj_err)

        info_check_level = InfoCheckLevel.COMPLETED
        ret, self.camera_info = camera_info_check(self.camera_info, info_check_level)
        if not ret:
            raise CalibrationException("Camera info check failed")
        # R is identity matrix for monocular calibration
        self.R = np.eye(3, dtype=np.float64)
        self.P = np.zeros((3, 4), dtype=np.float64)

    def do_calibration(self):
        """数据采集完毕后，使用收集到的数据进行正式标定
        NOTE : 之前采集数据时,为了速度,将图像大小降采样至VGA(640*480)再检测角点，在正式标定的时候，要使用原图检测角点
        """
        if not self.good_corners:
            print("**** Collecting good corners from database! ****")  # DEBUG
            images = [i for (p, i) in self.db]
            self.good_corners = self.collect_corners(images)
        self.size = (self.db[0][1].shape[1], self.db[0][1].shape[0])  # TODO Needs to be set externally
        self.cal_fromcorners(self.good_corners)
        self.calibrated = True
        # DEBUG
        # print((self.report()))
        # print((self.ost()))

    def undistort_points(self, src):
        """角点去畸变
        Params:
            src (np.ndarray): 原始图像中检测到的角点

            """
        if self.camera_info.camera_model == CameraModel.PINHOLE:
            return cv2.undistortPoints(
                src,
                self.camera_info.intrinsics_matrix,
                self.camera_info.distortion_coefficients,
                np.eye(3, 3),
                self.camera_info.intrinsics_matrix,
            )
        elif self.camera_info.camera_model == CameraModel.FISHEYE:
            new_mat = self.camera_info.intrinsics_matrix.copy()
            new_mat[0, 0] *= self.camera_info.scale_xy[0]
            new_mat[1, 1] *= self.camera_info.scale_xy[1]
            new_mat[0, 2] += self.camera_info.shift_xy[0]
            new_mat[1, 2] += self.camera_info.shift_xy[1]
            return cv2.fisheye.undistortPoints(
                src, self.camera_info.intrinsics_matrix, self.camera_info.distortion_coefficients, np.eye(3, 3), new_mat
            )

    @staticmethod
    def linear_error(corners, ids, board):

        """
        Returns the linear error for a set of corners detected in the unrectified image.
        """

        if corners is None:
            return None

        corners = np.squeeze(corners)

        def pt2line(x0, y0, x1, y1, x2, y2):
            """ 点到直线的距离
            point is (x0, y0), line is (x1, y1, x2, y2) 
            """
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
            return math.sqrt(sum([e ** 2 for e in errors]) / len(errors))
        else:
            return None


class MonoCalibrator(Calibrator):
    """
    单目相机标定类,继承自Calibrator类
    此类的设计目标：
        1. 类变量 : 尽量不含有专属的类变量,因为Calibrator类应该包含了一切单目标定需要的信息
        2. 类函数 : 函数应该被设计只是用于单目标定使用,而如果存在与双目标定类似的函数需求,应该尝试通过继承Calibrator类来实现
    """

    is_mono = True  # TODO Could get rid of is_mono

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

        resized_img, corners, downsampled_corners, ids, board, (x_scale, y_scale) = self.downsample_and_detect(gray)

        # 如果标定是已经完成的,则将角点坐标进行校正、缩放然后绘制在校正缩放后的图像上
        if self.calibrated:
            undistorted_gray = self.remap(gray)
            if x_scale != 1.0 or y_scale != 1.0:
                resized_undistorted_gray = cv2.resize(undistorted_gray, (resized_img.shape[1], resized_img.shape[0]))

            resized_undistorted_img = cv2.cvtColor(resized_undistorted_gray, cv2.COLOR_GRAY2BGR)

            # 角点校正、计算线性误差、缩放、绘制角点
            if corners is not None:
                # Report linear error
                undistorted_corners = self.undistort_points(corners)
                linear_error = self.linear_error(undistorted_corners, ids, self.chessboard_info)
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
                params = self.get_parameters(corners, ids, board, (gray.shape[1], gray.shape[0]))
                if self.is_good_sample(params, corners, ids, self.last_frame_corners, self.last_frame_ids):
                    self.db.append((params, gray))
                    if board.pattern == Patterns.CHESSBOARD:
                        self.good_corners.append((corners, None, board))
                    else:
                        pass
                    print(("*** Added sample %d, p_x = %.3f, p_y = %.3f, p_size = %.3f, skew = %.3f" % tuple([len(self.db)] + params)))
                end_time = time.time()
            resized_img_with_corners = resized_gray

        self.last_frame_corners = corners
        self.last_frame_ids = ids
        mono_handle_result = MonoHandleResult()
        mono_handle_result.resized_img_with_corners = resized_img_with_corners
        mono_handle_result.params = self.compute_goodenough()
        mono_handle_result.linear_error = linear_error
        return mono_handle_result


class StereoCalibrator(Calibrator):
    def __init__():
        pass
