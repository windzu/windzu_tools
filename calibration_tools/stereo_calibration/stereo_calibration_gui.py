"""
Author: windzu
Date: 2022-04-09 11:10:32
LastEditTime: 2022-04-09 11:10:33
LastEditors: windzu
Description: 
FilePath: /windzu_ws/src/tools/calibration_tools/mono_calibration/mono_gui.py
@Copyright (C) 2021-2022 plusgo Company Limited. All rights reserved.
@Licensed under the Apache License, Version 2.0 (the License)
"""

from numpy import double
import cv2
import yaml
import numpy as np
import sys
import tkinter as tk
from tkinter import ttk


# local
sys.path.append("../../")
from common.camera_info import CameraInfo
from common.tf_info import TFInfo
from common.chessboard_info import ChessboardInfo
from common.gui import GUI
from utils.get_frame import GetFrame
from utils.save_tf_config import save_tf_config

from stereo_calibration_node import StereoCalibrationNode


# StereoCalibrationGUI
# +───────────────────────────────+──────────────────────────────────+
# | master_master_camera_id_label        | master_camera_id_combobox               |
# +───────────────────────────────+──────────────────────────────────+
# | slaver_master_camera_id_label        | master_camera_id_combobox               |
# +───────────────────────────────+──────────────────────────────────+
# | chessboard_size_label         | chessboard_size_combobox         |
# +───────────────────────────────+──────────────────────────────────+
# | chessboard_square_size_label  | chessboard_square_size_combobox  |
# +───────────────────────────────+──────────────────────────────────+
# | set_param_button              | start_button                     |
# +───────────────────────────────+──────────────────────────────────+
# | show_result_button            | save_button                      |
# +───────────────────────────────+──────────────────────────────────+
# | exit_button                   |                                  |
# +───────────────────────────────+──────────────────────────────────+


class StereoCalibrationGUI(GUI):
    def __init__(self, camera_config_path, tf_config_path):
        super(StereoCalibrationGUI, self).__init__()
        self.camera_config_path = camera_config_path
        self.tf_config_path = tf_config_path
        self.node = None

        # 为填充参数
        self.camera_id_list = []

        # 未初始化参数
        self.master_camera_info = None
        self.slaver_camera_info = None
        self.master_camera_get_frame = None
        self.slaver_camera_get_frame = None
        self.tf_info = None
        self.chessboard_info = None

        # 待分配参数
        self.all_raw_camera_config = {}
        self.all_raw_tf_config = {}

        self.__loading_files()
        self.__gui_init()
        print("[ GUI ] init success")
        print("************************************************")

    def set_param_callback(self):
        (
            master_camera_id,
            slaver_camera_id,
            chessboard_size,
            square_size,
        ) = self.__get_params_from_gui()

        # camera info init
        self.master_camera_info = CameraInfo(master_camera_id, self.all_raw_camera_config[master_camera_id])
        self.slaver_camera_info = CameraInfo(slaver_camera_id, self.all_raw_camera_config[slaver_camera_id])

        # tf info init
        # tf_id = master_camera_id + "_to_" + slaver_camera_id
        tf_id = slaver_camera_id + "_to_" + master_camera_id
        self.tf_info = TFInfo(tf_id)

        # get frame init
        self.master_camera_get_frame = GetFrame(
            input_mode=self.master_camera_info.input_mode,
            device_name=self.master_camera_info.device_name,
            ros_topic=self.master_camera_info.ros_topic,
        )
        self.slaver_camera_get_frame = GetFrame(
            input_mode=self.slaver_camera_info.input_mode,
            device_name=self.slaver_camera_info.device_name,
            ros_topic=self.slaver_camera_info.ros_topic,
        )

        # chessboard info init
        self.chessboard_info = ChessboardInfo(
            n_cols=chessboard_size[0],
            n_rows=chessboard_size[1],
            square_size=square_size,
        )

        # node init
        self.node = StereoCalibrationNode(
            master_get_frame=self.master_camera_get_frame,
            slaver_get_frame=self.slaver_camera_get_frame,
            master_camera_info=self.master_camera_info,
            slaver_camera_info=self.slaver_camera_info,
            tf_info=self.tf_info,
            chessboard_info=self.chessboard_info,
        )
        # echo result
        self.master_camera_info.echo()
        self.slaver_camera_info.echo()
        print("[ GUI ] set params success")
        print("************************************************")

    def start_callback(self):
        if self.node is None:
            print("[ GUI ] error , please set params first!")
            return
        self.node.start()
        print("[ GUI ] start success and complete calibration")
        # 标定完成，信息同步，保存参数
        self.master_camera_info = self.node.master_camera_info
        self.slaver_camera_info = self.node.slaver_camera_info
        self.tf_info = self.node.tf_info
        # echo result
        self.master_camera_info.echo()
        self.slaver_camera_info.echo()
        self.tf_info.echo()
        print("[ GUI ] reproj error: {}".format(self.node.reproj_error))
        print("************************************************")

    def show_result_callback(self):
        print("[ GUI ] show result")
        self.node.show_result()
        print("************************************************")

    def save_callback(self):
        if self.node is None:
            raise Exception("please set params first!")

        tf_id = self.tf_info.tf_id
        self.all_raw_tf_config[tf_id] = self.tf_info.deserialize_tf_config()

        with open(self.tf_config_path, "w") as f:
            yaml.dump(self.all_raw_tf_config, f, default_flow_style=False)

        print("[ GUI ] save tf config success")
        self.tf_info.echo()
        print("************************************************")

    def exit_callback(self):
        cv2.destroyAllWindows()
        self.win.destroy()
        self.win.quit()
        print("[ GUI ] exit success")
        print("************************************************")

    def __loading_files(self):
        """读取yaml配置文件"""
        # 读取相机配置文件
        with open(self.camera_config_path, "r") as f:
            self.all_raw_camera_config = yaml.load(f)
        self.camera_id_list = [key for key in self.all_raw_camera_config.keys()]

        # 读取tf配置文件
        with open(self.tf_config_path, "r") as f:
            self.all_raw_tf_config = yaml.load(f)

    def __gui_init(self):
        # create root window
        self.win = tk.Tk()
        self.win.title("camera calibration")
        self.win.geometry("650x400")

        # master camera id
        self.master_camera_id_label = ttk.Label(self.win, text="master camera id:")
        self.master_camera_id_combobox = ttk.Combobox(self.win)

        self.master_camera_id_combobox["values"] = self.camera_id_list
        self.master_camera_id_combobox.current(0)
        self.master_camera_id_get_value = self.master_camera_id_combobox.get()
        self.master_camera_id_combobox["state"] = "readonly"

        # slaver camera id
        self.slaver_camera_id_label = ttk.Label(self.win, text="slaver camera id:")
        self.slaver_camera_id_combobox = ttk.Combobox(self.win)

        self.slaver_camera_id_combobox["values"] = self.camera_id_list
        self.slaver_camera_id_combobox.current(0)
        self.slaver_camera_id_get_value = self.slaver_camera_id_combobox.get()
        self.slaver_camera_id_combobox["state"] = "readonly"

        # mannual entry chessboard size and square size
        # chessboard size
        self.chessboard_size_label = ttk.Label(self.win, text="chessboard size:")
        self.chessboard_size_combobox = ttk.Combobox(self.win)
        self.chessboard_size_combobox["values"] = ("5x8", "7x8", "8x9")
        self.chessboard_size_combobox.current(0)
        self.chessboard_size_get_value = self.chessboard_size_combobox.get()
        self.chessboard_size_combobox["state"] = "read-write"
        # chessboard square size
        self.chessboard_square_size_label = ttk.Label(self.win, text="chessboard square size(mm):")
        self.chessboard_square_size_combobox = ttk.Combobox(self.win)
        self.chessboard_square_size_combobox["values"] = "26"
        self.chessboard_square_size_combobox.current(0)
        self.chessboard_square_size_get_value = self.chessboard_square_size_combobox.get()
        self.chessboard_square_size_combobox["state"] = "read-write"

        # ttk button
        self.set_param_button = ttk.Button(self.win, text="set_params", command=self.set_param_callback)
        self.start_button = ttk.Button(self.win, text="start_collect", command=self.start_callback)
        self.show_result_button = ttk.Button(self.win, text="show_result", command=self.show_result_callback)
        self.save_button = ttk.Button(self.win, text="save", command=self.save_callback)
        self.exit_button = ttk.Button(self.win, text="exit", command=self.exit_callback)

        # layout
        self.__gui_layout()
        self.win.mainloop()

    def __gui_layout(self):
        row_count = 1
        # layout combobox
        # master camera id
        self.master_camera_id_label.grid(row=row_count, column=1, sticky="E")
        self.master_camera_id_combobox.grid(row=row_count, column=2, sticky="NW")
        row_count += 1
        ################################################################################
        # slaver camera id
        self.slaver_camera_id_label.grid(row=row_count, column=1, sticky="E")
        self.slaver_camera_id_combobox.grid(row=row_count, column=2, sticky="NW")
        row_count += 1
        ################################################################################
        self.chessboard_size_label.grid(row=row_count, column=1, sticky="E")
        self.chessboard_size_combobox.grid(row=row_count, column=2, sticky="NW")
        row_count += 1
        ################################################################################
        self.chessboard_square_size_label.grid(row=row_count, column=1, sticky="E")
        self.chessboard_square_size_combobox.grid(row=row_count, column=2, sticky="NW")
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

    def __get_params_from_gui(self):
        master_camera_id = self.master_camera_id_combobox.get()
        slaver_camera_id = self.slaver_camera_id_combobox.get()
        chessboard_size = [int(x) for x in self.chessboard_size_combobox.get().split("x")]
        square_size = double(self.chessboard_square_size_combobox.get())
        return (master_camera_id, slaver_camera_id, chessboard_size, square_size)
