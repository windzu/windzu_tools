"""
Author: windzu
Date: 2022-06-08 09:25:00
LastEditTime: 2022-06-08 09:26:22
LastEditors: windzu
Description: 
FilePath: /windzu_tools/dev/imu_test/main.py
@Copyright (C) 2021-2022 plusgo Company Limited. All rights reserved.
@Licensed under the Apache License, Version 2.0 (the License)
"""
import sys
import tf2_ros
import rospy
import yaml

# local
sys.path.append("../../")
from utils.parse_hdmap import parse_hdmap
from utils.get_frame import GetFrame
from common.camera_info import CameraInfo


def main():
    # config path
    camera_config_path = "../../config/camera_config_test.yaml"
    hdmap_config_path = "../../hdmap/jianghuai.json"
    ## 读取配置文件
    # 读取相机配置文件
    with open(camera_config_path, "r") as f:
        all_raw_camera_config = yaml.load(f)
    camera_id_list = [key for key in all_raw_camera_config.keys()]
    # 读取hdmap文件
    all_tl_info = parse_hdmap(hdmap_config_path)

    camera_id = "/camera/jianghuai"
    tl_id = "tl_01"
    camera_info = CameraInfo(camera_id, all_raw_camera_config[camera_id])
    camera_get_frame = GetFrame(
        input_mode=camera_info.input_mode,
        device_name=camera_info.device_name,
        ros_topic=camera_info.ros_topic,
    )

    tl_info = all_tl_info[tl_id]

    # tf listener
    listener = tf2_ros.TransformListener()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # tf listener get transform
        (trans, rot) = listener.lookupTransform("/map", camera_id, rospy.Time(0))
        print("trans:{}".format(trans))


if __name__ == "__main__":
    main()
