"""
Author: windzu
Date: 2022-05-25 15:33:47
LastEditTime: 2022-05-31 17:15:36
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
    camera_config_path = "../../config/camera_config_test.yaml"
    tf_config_path = "../../config/tf_config_test.yaml"
    gui = StereoCalibrationGUI(camera_config_path, tf_config_path)


if __name__ == "__main__":
    main()
