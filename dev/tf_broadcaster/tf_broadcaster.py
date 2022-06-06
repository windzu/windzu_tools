"""
Author: windzu
Date: 2022-06-01 09:07:48
LastEditTime: 2022-06-06 16:46:00
LastEditors: windzu
Description: 
FilePath: /windzu_tools/dev/tf_broadcaster/tf_broadcaster.py
@Copyright (C) 2021-2022 plusgo Company Limited. All rights reserved.
@Licensed under the Apache License, Version 2.0 (the License)
"""
# ros tf publisher
import rospy
import tf
import sys
from scipy.spatial.transform import Rotation
import numpy as np
import yaml


#
import tf2_ros
import geometry_msgs.msg


# local
sys.path.append("../../")
from common.tf_info import TFInfo
from utils.parse_tf_config import parse_tf_config


def main():
    rospy.init_node("ststic_tf_publish")

    # static tf broadcaster
    static_broadcaster = tf2_ros.StaticTransformBroadcaster()
    tf_config_path = "../../config/tf_config_test.yaml"
    with open(tf_config_path, "r") as f:
        all_raw_tf_config = yaml.load(f)
    static_transformStamped_list = []
    for key, value in all_raw_tf_config.items():
        tf_info = TFInfo(key, value)
        static_transformStamped = geometry_msgs.msg.TransformStamped()
        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = tf_info.parent_frame_id
        static_transformStamped.child_frame_id = tf_info.child_frame_id
        static_transformStamped.transform.translation.x = float(tf_info.translation[0])
        static_transformStamped.transform.translation.y = float(tf_info.translation[1])
        static_transformStamped.transform.translation.z = float(tf_info.translation[2])

        static_transformStamped.transform.rotation.x = float(tf_info.rotation[0])
        static_transformStamped.transform.rotation.y = float(tf_info.rotation[1])
        static_transformStamped.transform.rotation.z = float(tf_info.rotation[2])
        static_transformStamped.transform.rotation.w = float(tf_info.rotation[3])
        static_transformStamped_list.append(static_transformStamped)
        # static_broadcaster.sendTransform(static_transformStamped)
        print("[ static_tf_publisher ] : send static tf: {}".format(tf_info.tf_id))
        print("[ static_tf_publisher ] : from {} to {}".format(tf_info.parent_frame_id, tf_info.child_frame_id))
    print("[ tf_broadcaster ] : static tf broadcast finished")
    static_broadcaster.sendTransform(static_transformStamped_list)
    rospy.spin()


if __name__ == "__main__":
    main()
