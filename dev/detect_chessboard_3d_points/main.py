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

# local
sys.path.append("../../")
from common.camera_calibrator import CameraCalibrator, HandleResult
from common.enum_common import CameraModel, CameraInfoCheckLevel
from utils.parse_camera_config import parse_camera_config
from utils.get_corners import get_all_good_corners_from_images, quick_get_good_corners
from utils.get_frame import GetFrame


def main():
    rospy.init_node("test")
    camera_config_path = "../../config/mono_camera_config_template.yaml"
    camera_id = "/camera/mono_test"
    camera_id_list, camera_info_dict, camera_raw_config_dict = parse_camera_config(camera_config_path)
    camera_info = camera_info_dict[camera_id]

    info_check_level = CameraInfoCheckLevel.COMPLETED
    camera_info.info_check(info_check_level)
    get_frame = GetFrame(
        input_mode=camera_info.input_mode, device_name=camera_info.device_name, ros_topic=camera_info.ros_topic
    )

    # 常量设置
    board_cols = 5
    board_rows = 8
    while True:
        frame = get_frame.read()
        if frame is None:
            continue
        (ok, corners, resized_img, downsampled_corners, (x_scale, y_scale)) = quick_get_good_corners(
            frame, board_cols, board_rows
        )
        if ok:
            cv2.drawChessboardCorners(resized_img, (board_cols, board_rows), downsampled_corners, ok)  # 画棋盘角点
            cv2.imshow("corners", resized_img)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                cv2.destroyAllWindows()
                break
        else:
            continue


if __name__ == "__main__":
    main()
