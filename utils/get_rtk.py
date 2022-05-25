"""
Author: windzu
Date: 2022-04-09 17:18:33
LastEditTime: 2022-04-11 14:09:48
LastEditors: windzu
Description: 
FilePath: /windzu_ws/src/tools/utils/get_rtk.py
@Copyright (C) 2021-2022 plusgo Company Limited. All rights reserved.
@Licensed under the Apache License, Version 2.0 (the License)
"""
"""
Author: windzu
Date: 2022-04-09 17:18:33
LastEditTime: 2022-04-09 17:18:34
LastEditors: windzu
Description: 
FilePath: /tools/utils/get_rtk.py
@Copyright (C) 2021-2022 plusgo Company Limited. All rights reserved.
@Licensed under the Apache License, Version 2.0 (the License)
"""
import sys
import rospy
import message_filters
from sensor_msgs.msg import NavSatFix, Imu
import time, threading
import tf
import math

# test lib from https://pypi.org/project/utm/
import utm
from pyproj import Transformer

# TwistStamped
from geometry_msgs.msg import TwistStamped

# local
sys.path.append("../")
from common.car_position import CarPosition


class GetRTK:
    """订阅三个rostopic,解析获取想要的信息
    """

    def __init__(self, gps_topic, imu_topic):
        self.gps_topic = gps_topic
        self.imu_topic = imu_topic
        # self.vel_topic = vel_topic

        self.car_position = None

        # 参数1：WGS84地理坐标系统 对应 epsg编号：4326
        # 参数2：坐标系WKID 中国沿海 UTM区号为WGS_1984_UTM_Zone_49N 对应 epsg编号 32649
        self.transformer = Transformer.from_crs("epsg:4326", "epsg:32650")

        self.mutex = threading.Lock()
        self.thread = threading.Thread(target=self.sub_init)
        self.thread.start()

    def get_car_position(self):
        self.mutex.acquire()
        car_position = self.car_position
        self.mutex.release()
        return car_position

    def sub_init(self):
        gps_sub = message_filters.Subscriber(self.gps_topic, NavSatFix)
        # vel_sub = message_filters.Subscriber(self.vel_topic, TwistStamped)
        imu_sub = message_filters.Subscriber(self.imu_topic, Imu)

        ts = message_filters.TimeSynchronizer([gps_sub, imu_sub], 10)
        ts.registerCallback(self.rtk_callback)
        rospy.spin()

    def rtk_callback(self, gps_msg, imu_msg):
        """
        :param gps_msg: NavSatFix
        :param vel_msg: TwistStamped (已经不使用)
        :param imu_msg: Imu
        :return:
        """
        latitude = gps_msg.latitude
        longtitude = gps_msg.longitude
        # convert imu orientation to quaternion and then to euler
        q = imu_msg.orientation
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])

        # 因为yaw的定义不统一，yaw分几种情况
        # 1 正方向与右手坐标系正方向相同，需要补偿90度
        yaw = math.pi / 2 + yaw
        # 2 正方向与右手坐标系正方向不同，需要补偿90度
        # yaw = math.pi / 2 - yaw
        # 3 yaw就是yaw

        # print("yaw:", yaw)

        # u = utm.from_latlon(latitude, longtitude)
        (x, y) = self.transformer.transform(latitude, longtitude)

        # self.car_position = CarPosition(latitude=latitude, longtitude=longtitude, x=u[0], y=u[1], z=0, yaw=yaw)
        self.car_position = CarPosition(latitude=latitude, longtitude=longtitude, x=x, y=y, z=0, yaw=yaw)


if __name__ == "__main__":
    rospy.init_node("get_rtk", anonymous=True)
    gps_topic = "/rtk_gps"
    imu_topic = "/rtk_imu"
    # vel_topic = "/rtk_velocity"
    rtk = GetRTK(gps_topic=gps_topic, imu_topic=imu_topic)
    while True:
        car_position = rtk.get_car_position()
        if car_position is not None:
            pass
            print(car_position.x, car_position.y, car_position.yaw)
        time.sleep(0.1)
