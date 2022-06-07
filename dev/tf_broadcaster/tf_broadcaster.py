"""
Author: windzu
Date: 2022-06-07 09:25:26
LastEditTime: 2022-06-07 18:06:50
LastEditors: windzu
Description: 
FilePath: /windzu_tools/dev/tf_broadcaster/tf_broadcaster.py
@Copyright (C) 2021-2022 plusgo Company Limited. All rights reserved.
@Licensed under the Apache License, Version 2.0 (the License)
"""
import sys
import rospy
import numpy as np
import tf
import tf2_ros
from scipy.linalg import expm, norm
import geometry_msgs.msg

# local
sys.path.append("../../")
from utils.get_rtk import GetRTK


def calculate_map_to_base_link_tf(car_position, map_frame_id="/map", base_link_frame_id="/base_link"):
    # get quaternion from euler angle
    quaternion = tf.transformations.quaternion_from_euler(0, 0, car_position.yaw)
    transformStamped = geometry_msgs.msg.TransformStamped()
    transformStamped.header.stamp = rospy.Time.now()
    transformStamped.header.frame_id = map_frame_id
    transformStamped.child_frame_id = base_link_frame_id
    transformStamped.transform.translation.x = car_position.x
    transformStamped.transform.translation.y = car_position.y
    transformStamped.transform.translation.z = car_position.z
    transformStamped.transform.rotation.x = quaternion[0]
    transformStamped.transform.rotation.y = quaternion[1]
    transformStamped.transform.rotation.z = quaternion[2]
    transformStamped.transform.rotation.w = quaternion[3]

    return transformStamped


def main():
    rospy.init_node("tf_publisher")
    gps_topic = "/rtk_gps"
    imu_topic = "/rtk_imu"
    get_rtk = GetRTK(gps_topic=gps_topic, imu_topic=imu_topic)

    # tf2 broadcaster
    tf2_broadcaster = tf2_ros.TransformBroadcaster()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        car_position = get_rtk.get_car_position()

        if car_position is not None:

            map_to_base_link_tf = calculate_map_to_base_link_tf(car_position)
            # publish tf
            tf2_broadcaster.sendTransform(map_to_base_link_tf)
            print(
                "[ tf_publisher ] : from {} to {}".format(
                    map_to_base_link_tf.header.frame_id, map_to_base_link_tf.child_frame_id
                )
            )
        rate.sleep()


if __name__ == "__main__":
    main()
