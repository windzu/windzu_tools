"""
Author: windzu
Date: 2022-05-25 15:24:57
LastEditTime: 2022-05-31 16:34:15
LastEditors: windzu
Description: 
FilePath: /windzu_tools/calibration_tools/surround_view_calibration/main.py
@Copyright (C) 2021-2022 plusgo Company Limited. All rights reserved.
@Licensed under the Apache License, Version 2.0 (the License)
"""
import sys
import rospy

# local
sys.path.append("../../")
from surround_view_calibration_gui import SurroundViewCalibrationGUI


def main():
    rospy.init_node("test")
    camera_config_path = "../../config/surround_view.yaml"
    gui = SurroundViewCalibrationGUI(camera_config_path)


if __name__ == "__main__":
    main()
