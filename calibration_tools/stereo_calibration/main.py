"""
Author: windzu
Date: 2022-05-25 15:33:47
LastEditTime: 2022-05-26 17:10:30
LastEditors: windzu
Description: 
FilePath: /windzu_tools/calibration_tools/stereo_calibration/main.py
@Copyright (C) 2021-2022 plusgo Company Limited. All rights reserved.
@Licensed under the Apache License, Version 2.0 (the License)
"""
import sys
import rospy

# local
from stereo_calibration_gui import StereoCalibrationGUI


def main():
    rospy.init_node("test")
    camera_config_path = "../../config/stereo_camera_config_template.yaml"
    gui = StereoCalibrationGUI(camera_config_path)


if __name__ == "__main__":
    main()
