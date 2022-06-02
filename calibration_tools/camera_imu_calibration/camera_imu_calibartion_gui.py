"""
Author: windzu
Date: 2022-02-28 21:31:54
LastEditTime: 2022-03-08 16:38:08
LastEditors: windzu
Description: 
FilePath: /windzu_ws/src/tools/calibration_tools/monocular_camera_calibration/script/gui.py
@Copyright (C) 2021-2022 plusgo Company Limited. All rights reserved.
@Licensed under the Apache License, Version 2.0 (the License)
"""
from abc import abstractclassmethod
from numpy import double
import cv2
import yaml
import numpy as np
import sys
import tkinter as tk
from tkinter import ttk  # 导入ttk模块，因为下拉菜单控件在ttk中

# local
sys.path.append("../../")
from common.chessboard_info import ChessboardInfo
from common.gui import GUI
from common.tf_info import TFInfo
from camera_imu_calibration_node import CameraIMUCalibrationNode
from common.camera_info import CameraInfo
from utils.get_frame import GetFrame
from utils.get_rtk import GetRTK
from utils.parse_hdmap import parse_hdmap


# CameraIMUCalibrationGUI
# +───────────────────────────────+──────────────────────────────────+
# | camera_id_label               | camera_id_combobox               |
# +───────────────────────────────+──────────────────────────────────+
# | tl_id_label                   | tl_id_combobox                   |
# +───────────────────────────────+──────────────────────────────────+
# | set_param_button              | start_button                     |
# +───────────────────────────────+──────────────────────────────────+
# | show_result_button            | save_button                      |
# +───────────────────────────────+──────────────────────────────────+
# | exit_button                   |                                  |
# +───────────────────────────────+──────────────────────────────────+


class CameraIMUCalibrationGUI(GUI):
    def __init__(self, camera_config_path, tf_config_path, hdmap_config_path, gps_topic, imu_topic):
        super(CameraIMUCalibrationGUI, self).__init__()
        self.camera_config_path = camera_config_path
        self.tf_config_path = tf_config_path
        self.hdmap_config_path = hdmap_config_path
        self.gps_topic = gps_topic
        self.imu_topic = imu_topic
        self.node = None

        # get rtk init
        self.get_rtk = GetRTK(gps_topic=self.gps_topic, imu_topic=self.imu_topic)

        # 未初始化参数
        self.camera_info = None
        self.camera_get_frame = None
        self.tf_info = None

        # 为填充参数
        self.camera_id_list = []
        self.tl_id_list = []

        # 待分配参数
        self.all_raw_camera_config = {}
        self.all_raw_tf_config = {}
        self.all_tl_info = {}  # 解析后的hdmap

        # 加载相机配置文件、tf配置文件、hdmap
        self.__loading_files()
        self.__gui_init()
        print("[ GUI ] init success")
        print("************************************************")

    def set_param_callback(self):
        (camera_id, tl_id) = self.__get_params_from_gui()

        # tl info init
        self.tl_info = self.all_tl_info[tl_id]
        # tf info init
        tf_id = "/imu" + "_to_" + camera_id
        if tf_id not in self.all_raw_tf_config.keys():
            self.tf_info = TFInfo(tf_id)
        else:
            self.tf_info = TFInfo(tf_id, self.all_raw_tf_config[tf_id])
        # camera info init
        self.camera_info = CameraInfo(camera_id, self.all_raw_camera_config[camera_id])

        # get frame init
        self.camera_get_frame = GetFrame(
            input_mode=self.camera_info.input_mode,
            device_name=self.camera_info.device_name,
            ros_topic=self.camera_info.ros_topic,
        )

        self.node = CameraIMUCalibrationNode(
            get_frame=self.camera_get_frame,
            get_rtk=self.get_rtk,
            camera_info=self.camera_info,
            tl_info=self.tl_info,
            tf_info=self.tf_info,
        )
        # echo result
        self.camera_info.echo()
        self.tf_info.echo()
        print("[ GUI ] set params success")
        print("************************************************")

    def start_callback(self):
        if self.node is None:
            print("please set params first!")
            return
        self.node.start()

        # 信息同步
        self.camera_info = self.node.camera_info
        self.tf_info = self.node.tf_info

    def show_result_callback(self):
        print("[ GUI ] show result")
        self.node.show_result()
        print("************************************************")

    def save_callback(self):
        if self.node is None:
            raise Exception("[ GUI ] please set params first!")

        tf_id = self.tf_info.tf_id
        self.all_raw_tf_config[tf_id] = self.tf_info.deserialize_tf_config()

        with open(self.tf_config_path, "w") as f:
            yaml.dump(self.all_raw_tf_config, f, default_flow_style=False)
        print("[ GUI ] save tf config success")
        self.tf_info.echo()
        print("************************************************")

    def exit_callback(self):
        cv2.destroyAllWindows()
        self.win.quit()
        self.win.destroy()
        print("[ GUI ] exit success")
        print("************************************************")

    def __loading_files(self):
        """读取配置文件"""
        # 读取相机配置文件
        with open(self.camera_config_path, "r") as f:
            self.all_raw_camera_config = yaml.load(f)
        self.camera_id_list = [key for key in self.all_raw_camera_config.keys()]

        # 读取tf配置文件
        with open(self.tf_config_path, "r") as f:
            self.all_raw_tf_config = yaml.load(f)

        # 读取hdmap文件
        self.all_tl_info = parse_hdmap(self.hdmap_config_path)
        self.tl_id_list = [key for key in self.all_tl_info.keys()]

    def __gui_init(self):
        # create root window
        self.win = tk.Tk()
        self.win.title("camera calibration")
        self.win.geometry("650x400")

        # camera id
        self.camera_id_label = ttk.Label(self.win, text="camera id:")
        self.camera_id_combobox = ttk.Combobox(self.win)
        self.camera_id_combobox["values"] = self.camera_id_list
        self.camera_id_combobox.current(0)
        self.camera_id_get_value = self.camera_id_combobox.get()
        self.camera_id_combobox["state"] = "readonly"
        # tl id
        self.tl_id_label = ttk.Label(self.win, text="tl id:")
        self.tl_id_combobox = ttk.Combobox(self.win)
        self.tl_id_combobox["values"] = self.tl_id_list
        self.tl_id_combobox.current(0)
        self.tl_id_get_value = self.tl_id_combobox.get()
        self.tl_id_combobox["state"] = "readonly"

        # ttk button
        self.set_param_button = ttk.Button(self.win, text="set_params", command=self.set_param_callback)
        self.start_button = ttk.Button(self.win, text="start", command=self.start_callback)
        self.show_result_button = ttk.Button(self.win, text="show_result", command=self.show_result_callback)
        self.save_button = ttk.Button(self.win, text="save", command=self.save_callback)
        self.exit_button = ttk.Button(self.win, text="exit", command=self.exit_callback)

        # layout
        self.__gui_layout()
        # loop
        self.win.mainloop()

    def __gui_layout(self):
        row_count = 1
        # layout combobox
        self.camera_id_label.grid(row=row_count, column=1, sticky="E")
        self.camera_id_combobox.grid(row=row_count, column=2, sticky="NW")
        row_count += 1
        ################################################################################
        self.tl_id_label.grid(row=row_count, column=1, sticky="E")
        self.tl_id_combobox.grid(row=row_count, column=2, sticky="NW")
        row_count += 1
        ################################################################################
        # layout button
        self.set_param_button.grid(row=row_count, column=1)
        self.start_button.grid(row=row_count, column=2)
        row_count += 1
        ################################################################################
        self.show_result_button.grid(row=row_count, column=1)
        self.save_button.grid(row=row_count, column=2)
        row_count += 1
        ################################################################################
        self.exit_button.grid(row=row_count, column=1)
        row_count += 1

    def __get_params_from_gui(self):
        camera_id = self.camera_id_combobox.get()
        tl_id = self.tl_id_combobox.get()
        return (camera_id, tl_id)
