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
from utils.get_frame import GetFrame
from utils.save_camera_config import save_camera_config
from utils.parse_camera_config import parse_camera_config
from common.chessboard_info import ChessboardInfo
from common.gui import GUI
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
    def __init__(self, camera_config_path):
        super(StereoCalibrationGUI, self).__init__()
        self.camera_config_path = camera_config_path
        self.node = None
        self.__parse_files_init()
        self.__gui_init()

    def set_param_callback(self):
        (
            master_camera_id,
            slaver_camera_id,
            chessboard_size,
            square_size,
        ) = self.__get_params_from_gui()
        master_camera_info = self.camera_info_dict[master_camera_id]
        slaver_camera_info = self.camera_info_dict[slaver_camera_id]
        master_get_frame = GetFrame(
            input_mode=master_camera_info.input_mode,
            device_name=master_camera_info.device_name,
            ros_topic=master_camera_info.ros_topic,
        )
        slaver_get_frame = GetFrame(
            input_mode=slaver_camera_info.input_mode,
            device_name=slaver_camera_info.device_name,
            ros_topic=slaver_camera_info.ros_topic,
        )
        chessboard_info = ChessboardInfo(
            n_cols=chessboard_size[0],
            n_rows=chessboard_size[1],
            square_size=square_size,
        )
        self.node = StereoCalibrationNode(
            master_get_frame=master_get_frame,
            slaver_get_frame=slaver_get_frame,
            master_camera_info=master_camera_info,
            slaver_camera_info=slaver_camera_info,
            chessboard_info=chessboard_info,
        )
        # echo result
        master_camera_info.echo()
        slaver_camera_info.echo()
        print("**** set params success ****")

    def start_callback(self):
        if self.node is None:
            print("please set params first!")
            return
        self.node.start()
        # echo result
        self.node.master_camera_info.echo()
        self.node.slaver_camera_info.echo()

    def show_result_callback(self):
        print("show result")
        self.node.show_result()

    def save_callback(self):
        if self.node is None:
            raise Exception("please set params first!")
        master_camera_id = self.master_camera_id_combobox.get()
        slaver_camera_id = self.slaver_camera_id_combobox.get()
        print(master_camera_id + " to " + slaver_camera_id + " R T :")

        print("R: ", self.node.calibrator.R)
        print("T: ", self.node.calibrator.T)

        print("print sucess")
        # self.camera_info_dict[camera_id] = self.node.camera_info
        # save_camera_config(
        #     self.camera_config_path,
        #     camera_id,
        #     self.camera_info_dict,
        #     self.camera_raw_config_dict,
        # )
        # print("save success")

    def exit_callback(self):
        cv2.destroyAllWindows()
        self.win.quit()
        self.win.destroy()
        print("exit success")

    def __parse_files_init(self):
        """按需解析配置文件，将其实例化为类对象
        本次需要解析文件：
            相机配置文件
        """
        print("*** parse files init ***")
        (
            self.camera_id_list,
            self.camera_info_dict,
            self.camera_raw_config_dict,
        ) = parse_camera_config(self.camera_config_path)

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

        # ttk label for show result
        self.show_state_label = ttk.Label(self.win, text="nothing happened")

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
        # use two column to show result
        self.show_state_label.grid(row=row_count, column=1, columnspan=2, sticky="NW")
        row_count += 1

    def __get_params_from_gui(self):
        master_camera_id = self.master_camera_id_combobox.get()
        slaver_camera_id = self.slaver_camera_id_combobox.get()
        chessboard_size = [int(x) for x in self.chessboard_size_combobox.get().split("x")]
        square_size = double(self.chessboard_square_size_combobox.get())
        return (master_camera_id, slaver_camera_id, chessboard_size, square_size)
