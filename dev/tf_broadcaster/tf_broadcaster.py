"""
Author: windzu
Date: 2022-06-01 09:07:48
LastEditTime: 2022-06-01 13:49:46
LastEditors: windzu
Description: 
FilePath: /windzu_tools/dev/tf_broadcaster/tf_broadcaster.py
@Copyright (C) 2021-2022 plusgo Company Limited. All rights reserved.
@Licensed under the Apache License, Version 2.0 (the License)
"""
"""
Author: windzu
Date: 2022-05-31 19:43:10
LastEditTime: 2022-06-01 01:27:02
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

# local
sys.path.append("../../")
from common.tf_info import TFInfo
from utils.parse_tf_config import parse_tf_config


def main():
    publish_rate = 10
    rospy.init_node("ststic_tf_publish")
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(publish_rate)
    # publish static tf
    tf_config_path = "../../config/tf_config_test.yaml"
    print("************************************************")
    print("[ TFBroadcaster ] loading tf config files")
    with open(tf_config_path, "r") as f:
        all_raw_tf_config = yaml.load(f)
    print("************************************************")
    print("[ TFBroadcaster ] publishing tf")
    while not rospy.is_shutdown():
        for key, value in all_raw_tf_config.items():
            tf_info = TFInfo(key, value)

            # convert numpy.array to quaternion and translation
            R = Rotation.from_matrix(tf_info.R)  # 将（3,3）转换为四元数
            R = R.as_quat()
            R = tuple(R)

            T = tuple(tf_info.T)  # (3,)转换为tuple

            br.sendTransform(
                translation=T,
                rotation=R,
                time=rospy.Time.now(),
                child=tf_info.child_frame_id,
                parent=tf_info.parent_frame_id,
            )
        rate.sleep()
    print("*** tf publish finished ***")


if __name__ == "__main__":
    main()
