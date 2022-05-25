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
from utils.get_frame import GetFrame
from utils.get_rtk import GetRTK
from utils.parse_hdmap import parse_hdmap
from utils.save_camera_config import save_camera_config
from utils.parse_camera_config import parse_camera_config
from common.camera_common import ChessboardInfo
from common.gui import GUI
from camera_imu_calibration_node import CameraIMUCalibrationNode


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
    def __init__(self, camera_config_path, hdmap_config_path, get_rtk_topics):
        super(CameraIMUCalibrationGUI, self).__init__()
        self.camera_config_path = camera_config_path
        self.hdmap_config_path = hdmap_config_path
        self.get_rtk_topics = get_rtk_topics
        self.get_rtk = GetRTK(gps_topic=self.get_rtk_topics["gps_topic"], imu_topic=self.get_rtk_topics["imu_topic"])
        self.node = None
        self.__parse_files_init()
        self.__gui_init()

    def set_param_callback(self):
        (camera_id, tl_id) = self.__get_params_from_gui()
        tl_info = self.tl_info_dict[tl_id]
        camera_info = self.camera_info_dict[camera_id]
        get_frame = GetFrame(input_mode=camera_info.input_mode, device_name=camera_info.device_name, ros_topic=camera_info.ros_topic)
        get_rtk = self.get_rtk
        self.node = CameraIMUCalibrationNode(get_frame=get_frame, get_rtk=get_rtk, camera_info=camera_info, tl_info=tl_info)
        camera_info.echo()
        print("**** set params success ****")

    def start_callback(self):
        if self.node is None:
            print("please set params first!")
            return
        self.node.start()
        # echo result
        self.node.camera_info.echo()

    def show_result_callback(self):
        print("show result")
        self.node.show_result()

    def save_callback(self):
        if self.node is None:
            raise Exception("please set params first!")
        camera_id = self.camera_id_combobox.get()
        self.camera_info_dict[camera_id] = self.node.camera_info
        save_camera_config(self.camera_config_path, camera_id, self.camera_info_dict, self.camera_raw_config_dict)
        print("save success")

    def exit_callback(self):
        cv2.destroyAllWindows()
        self.win.quit()
        self.win.destroy()
        print("exit success")

    def __parse_files_init(self):
        print("**** parse file init ****")
        self.camera_id_list, self.camera_info_dict, self.camera_raw_config_dict = parse_camera_config(self.camera_config_path)
        self.tl_id_list, self.tl_info_dict = parse_hdmap(self.hdmap_config_path)

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

        # ttk label for show result
        self.show_state_label = ttk.Label(self.win, text="nothing happened")

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
        ################################################################################
        # use two column to show result
        self.show_state_label.grid(row=row_count, column=1, columnspan=2, sticky="NW")
        row_count += 1

    def __get_params_from_gui(self):
        camera_id = self.camera_id_combobox.get()
        tl_id = self.tl_id_combobox.get()
        return (camera_id, tl_id)
