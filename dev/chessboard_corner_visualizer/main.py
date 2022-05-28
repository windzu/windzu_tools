"""
Author: windzu
Date: 2022-05-28 02:09:45
LastEditTime: 2022-05-28 02:13:33
LastEditors: windzu
Description: 
FilePath: /windzu_tools/dev/detect_chessboard_3d_points/main.py
@Copyright (C) 2021-2022 plusgo Company Limited. All rights reserved.
@Licensed under the Apache License, Version 2.0 (the License)
"""
from re import T
import cv2
import sys
import math
import numpy as np
import rospy
import tf
from sensor_msgs.msg import PointCloud2, PointField
import time, threading

# local
sys.path.append("../../")
from common.camera_calibrator import CameraCalibrator, HandleResult
from common.enum_common import CameraModel, CameraInfoCheckLevel
from common.chessboard_info import ChessboardInfo
from utils.parse_camera_config import parse_camera_config
from utils.get_corners import get_all_good_corners_from_images, quick_get_good_corners
from utils.get_frame import GetFrame


class ChessBoardCornerVisualizer(threading.Thread):  # 继承父类threading.Thread
    def __init__(
        self, thread_id, thread_name, camera_id, camera_info_dict, board_cols, board_rows, square_size, tf_listener=None
    ):
        threading.Thread.__init__(self)
        self.thread_id = thread_id
        self.thread_name = thread_name
        self.camera_id = camera_id
        self.camera_info_dict = camera_info_dict
        self.board_cols = board_cols
        self.board_rows = board_rows
        self.square_size = square_size
        self.tf_listener = tf_listener

        if self.info_init():
            print("info init success")
        else:
            raise Exception("info init failed")

    def info_init(self):
        self.camera_info = self.camera_info_dict[self.camera_id]
        info_check_level = CameraInfoCheckLevel.COMPLETED
        self.camera_info.info_check(info_check_level)
        self.get_frame = GetFrame(
            input_mode=self.camera_info.input_mode,
            device_name=self.camera_info.device_name,
            ros_topic=self.camera_info.ros_topic,
        )
        self.chessboard_info = ChessboardInfo(
            n_cols=self.board_cols,
            n_rows=self.board_rows,
            square_size=self.square_size,
        )
        self.corners_4d = self.convert_board_to_4d(self.board_cols, self.board_rows, self.square_size)
        self.ros_pub = rospy.Publisher(self.camera_id, PointCloud2, queue_size=2)
        return True

    def run(self):  # 把要执行的代码写到run函数里面 线程在创建后会直接运行run函数
        print("Starting " + self.thread_name)

        while True:
            frame = self.get_frame.read()
            if frame is None:
                continue
            (ok, corners, resized_img, downsampled_corners, (x_scale, y_scale)) = quick_get_good_corners(
                frame, self.board_cols, self.board_rows
            )
            if ok:
                # sovle_pnp
                ret, rvecs, tvecs = cv2.solvePnP(
                    self.chessboard_info.BOARD,
                    np.array(corners, dtype=np.float32),
                    self.camera_info.intrinsics_matrix,
                    self.camera_info.distortion_coefficients,
                )
                if ret:
                    # 计算棋盘格的3d坐标
                    result = self.calculate_chessboard_corners_3d_points(rvecs, tvecs, self.corners_4d)
                    # convert result to PointCloud2
                    points = np.zeros((result.shape[0], result.shape[1]), dtype=np.float32)
                    for i in range(result.shape[0]):
                        points[i, :] = result[i, :]
                    print(points)
                    msg = PointCloud2()
                    msg.header.stamp = rospy.Time().now()
                    msg.header.frame_id = self.camera_id
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
                    self.ros_pub.publish(msg)
                    print(self.camera_id, " publish pointcloud2")
            # cv2.drawChessboardCorners(resized_img, (self.board_cols, self.board_rows), downsampled_corners, ok)  # 画棋盘角点
            # cv2.imshow(self.camera_id, resized_img)
            # if cv2.waitKey(1) & 0xFF == ord("q"):
            #     cv2.destroyAllWindows()
            #     break

    @staticmethod
    def convert_board_to_4d(board_cols, board_rows, square_size):
        """
        将棋盘格转换为4维空间中的坐标
        """

        corners_4d = np.array(
            [[[j * square_size], [i * square_size], [0.0], [1]] for i in range(board_rows) for j in range(board_cols)],
            dtype=np.float32,
        )
        print("corners_4d shape: ", corners_4d.shape)
        return corners_4d

    @staticmethod
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


def main():
    rospy.init_node("test")
    # 常量设置
    board_cols = 5
    board_rows = 8
    square_size = 21.4  # mm
    camera_config_path = "../../config/stereo_camera_config_template.yaml"
    camera_id_list, camera_info_dict, camera_raw_config_dict = parse_camera_config(camera_config_path)

    master_camera_id = "/camera/master_camera"
    master_thread = ChessBoardCornerVisualizer(
        thread_id=1,
        thread_name="Thread-1",
        camera_id=master_camera_id,
        camera_info_dict=camera_info_dict,
        board_cols=board_cols,
        board_rows=board_rows,
        square_size=square_size,
    )
    slaver_camera_id = "/camera/slaver_camera"
    slaver_thread = ChessBoardCornerVisualizer(
        thread_id=2,
        thread_name="Thread-2",
        camera_id=slaver_camera_id,
        camera_info_dict=camera_info_dict,
        board_cols=board_cols,
        board_rows=board_rows,
        square_size=square_size,
    )

    master_thread.start()
    slaver_thread.start()

    # master_thread.join()
    # slaver_thread.join()


if __name__ == "__main__":
    main()
