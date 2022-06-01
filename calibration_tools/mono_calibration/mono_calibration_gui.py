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
from tkinter import ttk  # 导入ttk模块，因为下拉菜单控件在ttk中

# local
sys.path.append("../../")
from common.camera_info import CameraInfo
from common.chessboard_info import ChessboardInfo
from common.gui import GUI
from mono_calibration_node import MonoCalibrationNode
from utils.get_frame import GetFrame
from utils.save_camera_config import save_camera_config
from utils.parse_camera_config import parse_camera_config


# MonoCalibrationGUI
# +───────────────────────────────+──────────────────────────────────+
# | camera_id_label               | camera_id_combobox               |
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


class MonoCalibrationGUI(GUI):
    def __init__(self, camera_config_path):
        super(MonoCalibrationGUI, self).__init__()
        self.camera_config_path = camera_config_path
        self.node = None

        # 为填充参数
        self.camera_id_list = []

        # 未初始化参数
        self.camera_info = None
        self.camera_get_frame = None
        self.chessboard_info = None

        # 待分配参数
        self.all_raw_camera_config = {}

        self.__loading_files()
        self.__gui_init()
        print("[ GUI ] init success")
        print("************************************************")

    def set_param_callback(self):
        (camera_id, chessboard_size, square_size) = self.__get_params_from_gui()
        self.camera_info = CameraInfo(camera_id, self.all_raw_camera_config[camera_id])

        self.camera_get_frame = GetFrame(
            input_mode=self.camera_info.input_mode,
            device_name=self.camera_info.device_name,
            ros_topic=self.camera_info.ros_topic,
        )

        # chessboard info init
        self.chessboard_info = ChessboardInfo(
            n_cols=chessboard_size[0],
            n_rows=chessboard_size[1],
            square_size=square_size,
        )

        # node init
        self.node = MonoCalibrationNode(
            get_frame=self.camera_get_frame, chessboard_info=self.chessboard_info, camera_info=self.camera_info
        )
        # echo result
        self.camera_info.echo()
        print("[ GUI ] set params success")
        print("************************************************")

    def start_callback(self):
        if self.node is None:
            print("please set params first!")
            return
        self.node.start()
        print("[ GUI ] start success and complete calibration")
        self.camera_info = self.node.camera_info
        # echo result
        self.camera_info.echo()
        print("[ GUI ] reproj error: {}".format(self.node.reproj_error))
        print("************************************************")

    def show_result_callback(self):
        print("[ GUI ] show result")
        self.node.show_result()
        print("************************************************")

    def save_callback(self):
        if self.node is None:
            raise Exception("please set params first!")

        camera_id = self.camera_info.camera_id
        self.all_raw_camera_config[camera_id] = self.camera_info.deserialize_camera_config()

        with open(self.camera_config_path, "w") as f:
            yaml.dump(self.all_raw_camera_config, f, default_flow_style=False)

        print("[ GUI ] camera_id : ", camera_id)
        print("[ GUI ] intrinsics_matrix: ", self.camera_info.intrinsics_matrix)
        print("[ GUI ] distortion_coeffs: ", self.camera_info.distortion_coeffs)
        print("************************************************")

    def exit_callback(self):
        cv2.destroyAllWindows()
        self.win.destroy()
        self.win.quit()
        print("[ GUI ] exit success")
        print("************************************************")

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
        self.camera_id_label.grid(row=row_count, column=1, sticky="E")
        self.camera_id_combobox.grid(row=row_count, column=2, sticky="NW")
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
        camera_id = self.camera_id_combobox.get()
        chessboard_size = [int(x) for x in self.chessboard_size_combobox.get().split("x")]
        square_size = double(self.chessboard_square_size_combobox.get())
        return (camera_id, chessboard_size, square_size)

    def __loading_files(self):
        """读取yaml配置文件"""
        # 读取相机配置文件
        with open(self.camera_config_path, "r") as f:
            self.all_raw_camera_config = yaml.load(f)
        self.camera_id_list = [key for key in self.all_raw_camera_config.keys()]
