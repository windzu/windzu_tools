"""
Author: windzu
Date: 2022-05-28 13:20:55
LastEditTime: 2022-05-28 13:26:50
LastEditors: windzu
Description: 
FilePath: /windzu_tools/dev/chessboard_corner_visualizer/tf_broadcast.py
@Copyright (C) 2021-2022 plusgo Company Limited. All rights reserved.
@Licensed under the Apache License, Version 2.0 (the License)
"""
# ros tf publisher
import rospy
import tf
from scipy.spatial.transform import Rotation as R
import numpy as np

rotation_matrix = np.array(
    [
        [0.58489424, -0.05242351, 0.80941368],
        [-0.06964007, 0.99097863, 0.11450592],
        [-0.80811447, -0.12334148, 0.57596692],
    ]
)
translation_vector = np.array([-804.20484107, -91.50419771, 827.76415155])
r = R.from_matrix(rotation_matrix)
quaternion = r.as_quat()
quaternion = tuple(quaternion)
translation_vector = tuple(translation_vector)


def main():
    rospy.init_node("tf_test")
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(30.0)

    father_frame_id = "/camera/slaver_camera"
    child_frame_id = "/camera/master_camera"

    while not rospy.is_shutdown():
        br.sendTransform(translation_vector, quaternion, rospy.Time.now(), child_frame_id, father_frame_id)
        rate.sleep()


if __name__ == "__main__":
    main()
