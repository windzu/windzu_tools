'''
Author: windzu
Date: 2022-05-16 10:51:21
LastEditTime: 2022-05-24 15:02:37
LastEditors: windzu
Description: 
FilePath: /windzu_ws/src/tools/calibration_tools/camera_imu_calibration/main.py
@Copyright (C) 2021-2022 plusgo Company Limited. All rights reserved.
@Licensed under the Apache License, Version 2.0 (the License)
'''

#/usr/bin/python3
import sys
import rospy

# local
sys.path.append("../../")
from camera_imu_calibartion_gui import CameraIMUCalibrationGUI


def main():
    rospy.init_node("test")
    camera_config_path = "../../../../config/imu_to_camera_config_test.yaml"
    hdmap_config_path = "../../../../hdmap/weitang_park.json"
    get_rtk_topics = {"gps_topic": "/rtk_gps", "imu_topic": "/rtk_imu"}

    gui = CameraIMUCalibrationGUI(
        camera_config_path=camera_config_path, hdmap_config_path=hdmap_config_path, get_rtk_topics=get_rtk_topics
    )


if __name__ == "__main__":
    main()
