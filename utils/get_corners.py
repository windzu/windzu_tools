"""
Author: windzu
Date: 2022-05-26 11:08:46
LastEditTime: 2022-05-26 11:09:11
LastEditors: windzu
Description: 
FilePath: /windzu_tools/utils/get_corners.py
@Copyright (C) 2021-2022 plusgo Company Limited. All rights reserved.
@Licensed under the Apache License, Version 2.0 (the License)
"""
from enum import Enum
import cv2
import numpy as np
import math
import sys


def get_good_corners(img, board_cols, board_rows, checkerboard_flags=0):
    """检测更好的角点。原因如下：
    1. 对检测到的角点进行了有效性检查，排除了距离边缘过近的角点
    2. 对检测到的角点进行了重新排序，保证顺序为左上，右上，右下，左下
    3. 使用了cv2.cornerSubPix进行了精细化,使得角点更加精确

    Args:
        img (np.ndarray): 原始图像
        chessboard_info (_type_): 标定板信息
        checkerboard_flags (_type_): 检测角点的参数

    Returns:
        (ok, corners): ok是否检测到角点,corners是检测到的角点
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
    (ok, corners) = cv2.findChessboardCorners(mono, (board_cols, board_rows), flags=checkerboard_flags)
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
    if board_rows != board_cols:
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
                corners = np.rot90(corners.reshape(board_rows, board_cols, 2)).reshape(board_cols * board_rows, 1, 2)
            else:
                corners = np.rot90(corners.reshape(board_rows, board_cols, 2), 3).reshape(board_cols * board_rows, 1, 2)

    # 按照行和列的方向遍历角点，找到距离最近的角点间的距离，设置其作为cv2.cornerSubPix的搜寻半径，这样可以在保证速度情况下最大范围的进行亚像素搜索
    # NOTE : 这参考其ros的角点检测代码，个人认为亚像素搜索没必要这么大范围，除非是分辨率很低的情况或者是棋盘格square size很小，这样做可以确定搜索的边界；
    # 而对于高像素的图像，随便设置为5就可以了，两个角点间的距离肯定不会小于5，而5像素范围内就可以很好的搜索
    # 不过ros的这种做法确实更具rubust
    if ok:
        min_distance = float("inf")
        for row in range(board_rows):
            for col in range(board_cols - 1):
                index = row * board_cols + col
                min_distance = min(min_distance, pdist(corners[index, 0], corners[index + 1, 0]))
        for row in range(board_rows - 1):
            for col in range(board_cols):
                index = row * board_cols + col
                min_distance = min(min_distance, pdist(corners[index, 0], corners[index + board_cols, 0]))
        radius = int(math.ceil(min_distance * 0.5))
        cv2.cornerSubPix(
            mono, corners, (radius, radius), (-1, -1), (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)
        )

    return (ok, corners)


def get_all_good_corners_from_images(images, board_cols, board_rows, checkerboard_flags=0):
    """检测图像列表中所有图像的角点
    Args:
        images (list): 图像列表
    Returns:
        all_good_corners (list): [ corners... ],所有检测到的角点
        corners ;[[ok,corners],...],检测到的角点的原始数据,也包含是否检测到的状态
    """
    corners = [get_good_corners(i, board_cols, board_rows, checkerboard_flags) for i in images]

    all_good_corners = [co for (ok, co) in corners if ok]
    if not all_good_corners:
        raise Exception("No corners found in images!")
    return all_good_corners, corners


def quick_get_good_corners(img, board_cols, board_rows):
    """通过降采样以实现快速角点检测
    Args:
        img: 原图
    Returns:
        (ok, corners , resized_img , downsampled_corners, (x_scale, y_scale)).
            ok: 是否检测到角点
            corners: 原图角点
            resized_img : 缩放后的图像
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

    (ok, downsampled_corners) = get_good_corners(resized_img, board_cols, board_rows)

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

    return (ok, corners, resized_img, downsampled_corners, (x_scale, y_scale))
