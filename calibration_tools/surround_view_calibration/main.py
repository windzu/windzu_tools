"""
Author: windzu
Date: 2022-04-09 13:59:05
LastEditTime: 2022-04-09 13:59:37
LastEditors: windzu
Description: 
FilePath: /windzu_ws/src/tools/calibration_tools/surround_view_calibration/main.py
@Copyright (C) 2021-2022 plusgo Company Limited. All rights reserved.
@Licensed under the Apache License, Version 2.0 (the License)
"""
"""
Author: windzu
Date: 2022-04-09 11:59:06
LastEditTime: 2022-04-09 11:59:07
LastEditors: windzu
Description: 
FilePath: /windzu_ws/src/tools/calibration_tools/mono_calibration/main.py
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
    camera_config_path = "/home/wind/windzu_ws/config/surround_view.yaml"
    gui = SurroundViewCalibrationGUI(camera_config_path)


if __name__ == "__main__":
    main()
