"""
Author: windzu
Date: 2022-05-30 09:59:43
LastEditTime: 2022-05-30 13:39:04
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

# local
sys.path.append("../../")
from utils.parse_tf_config import parse_tf_config


def main():
    publish_rate = 10
    rospy.init_node("ststic_tf_publish")
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(publish_rate)
    # publish static tf
    tf_config_file_path = "../../config/tf_config.yaml"
    all_tf_info_dict = parse_tf_config(tf_config_file_path)

    while not rospy.is_shutdown():
        for kye, value in all_tf_info_dict.items():
            # print(kye, value)
            br.sendTransform(
                value["T"], value["R"], rospy.Time.now(), value["child_frame_id"], value["father_frame_id"]
            )
        rate.sleep()
        print("publish static tf")


if __name__ == "__main__":
    main()
