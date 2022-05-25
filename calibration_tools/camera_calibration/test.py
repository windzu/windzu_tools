"""
Author: windzu
Date: 2022-03-26 18:38:52
LastEditTime: 2022-04-08 13:57:28
LastEditors: windzu
Description: 
FilePath: /windzu_ws/src/tools/calibration_tools/camera_calibration/test.py
@Copyright (C) 2021-2022 plusgo Company Limited. All rights reserved.
@Licensed under the Apache License, Version 2.0 (the License)
"""
import sys
import rospy

# local
sys.path.append("../../")
from utils.camera_utils import parse_cameras_config
from camera_gui import MonoCalibrationGUI, SurroundViewGUI


def main():
    rospy.init_node("test")
    camera_config_path = "./test_config.yaml"
    gui = MonoCalibrationGUI(camera_config_path)
    # gui = SurroundViewGUI(camera_config_path)


if __name__ == "__main__":
    main()
