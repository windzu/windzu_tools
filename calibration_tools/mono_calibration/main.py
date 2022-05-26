"""
Author: windzu
Date: 2022-05-25 15:24:57
LastEditTime: 2022-05-26 14:07:02
LastEditors: windzu
Description: 
FilePath: /windzu_tools/calibration_tools/mono_calibration/main.py
@Copyright (C) 2021-2022 plusgo Company Limited. All rights reserved.
@Licensed under the Apache License, Version 2.0 (the License)
"""

import sys
import rospy

# local
from mono_calibration_gui import MonoCalibrationGUI


def main():
    rospy.init_node("test")
    camera_config_path = "../../config/mono_camera_config_template.yaml"
    gui = MonoCalibrationGUI(camera_config_path)


if __name__ == "__main__":
    main()
