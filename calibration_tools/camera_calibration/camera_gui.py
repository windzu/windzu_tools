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
from utils.camera_utils import parse_cameras_config, save_camera_config, camera_info_check
from utils.get_frame import GetFrame
from common.camera_common import ChessboardInfo, CCInfo, Patterns, CameraInfo, CameraModel, FrameInputMode, InfoCheckLevel
from calibration_node import MonoCalibrationNode
from surround_view_node import SurroundViewNode


class GUI(object):
    def __init__(self, camera_config_path):
        self.camera_config_path = camera_config_path
        self.camera_id_list, self.camera_config_dict, self.camera_raw_config_dict = parse_cameras_config(camera_config_path)

    @abstractclassmethod
    def init_gui(self):
        print("init_gui")

    @abstractclassmethod
    def gui_layout(self):
        print("gui_layout")

    @abstractclassmethod
    def set_param_callback(self):
        print("set_param_callback")

    @abstractclassmethod
    def start_callback(self):
        print("start_callback")

    @abstractclassmethod
    def show_result_callback(self):
        print("show_result_callback")

    @abstractclassmethod
    def save_callback(self):
        print("save_callback")

    @abstractclassmethod
    def _get_params_from_gui(self):
        print("_get_params_from_gui")

    def _get_params_from_camera_config(self, camera_id):
        camera_model = self.camera_config_dict["camera_model_dict"][camera_id]
        input_mode = self.camera_config_dict["input_mode_dict"][camera_id]
        device_name = self.camera_config_dict["device_name_dict"][camera_id]
        ros_topic = self.camera_config_dict["ros_topic_dict"][camera_id]
        resolution = self.camera_config_dict["resolution_dict"][camera_id]
        intrinsics_matrix = self.camera_config_dict["intrinsics_matrix_dict"][camera_id]
        distortion_coefficients = self.camera_config_dict["distortion_coefficients_dict"][camera_id]
        scale_xy = self.camera_config_dict["scale_xy_dict"][camera_id]
        shift_xy = self.camera_config_dict["shift_xy_dict"][camera_id]
        mask_size = self.camera_config_dict["mask_size_dict"][camera_id]
        mask = self.camera_config_dict["mask_dict"][camera_id]
        homography_matrix = self.camera_config_dict["homography_matrix_dict"][camera_id]

        return (
            camera_model,
            input_mode,
            device_name,
            ros_topic,
            resolution,
            intrinsics_matrix,
            distortion_coefficients,
            scale_xy,
            shift_xy,
            mask_size,
            mask,
            homography_matrix,
        )

    def _set_params_to_camera_config(self, camera_id, camera_info):
        self.camera_config_dict["intrinsics_matrix_dict"][camera_id] = camera_info.intrinsics_matrix
        self.camera_config_dict["distortion_coefficients_dict"][camera_id] = camera_info.distortion_coefficients
        self.camera_config_dict["scale_xy_dict"][camera_id] = camera_info.scale_xy
        self.camera_config_dict["shift_xy_dict"][camera_id] = camera_info.shift_xy
        self.camera_config_dict["mask_size_dict"][camera_id] = camera_info.mask_size
        self.camera_config_dict["mask_dict"][camera_id] = camera_info.mask
        self.camera_config_dict["mask_ploygon_corner_points_dict"][camera_id] = camera_info.mask
        self.camera_config_dict["homography_matrix_dict"][camera_id] = camera_info.homography_matrix

    def _get_camera_info_from_camera_config(self, camera_id):
        (
            camera_model,
            input_mode,
            device_name,
            ros_topic,
            resolution,
            intrinsics_matrix,
            distortion_coefficients,
            scale_xy,
            shift_xy,
            mask_size,
            mask,
            homography_matrix,
        ) = self._get_params_from_camera_config(camera_id)

        if camera_model == "pinhole":
            camera_model = CameraModel.PINHOLE
        elif camera_model == "fisheye":
            camera_model = CameraModel.FISHEYE
        else:
            raise ValueError("camera_model is invalid")

        if input_mode == "ros_topic":
            input_mode = FrameInputMode.ROS_TOPIC
        elif input_mode == "device_name":
            input_mode = FrameInputMode.DEVICE_NAME
        else:
            raise ValueError("input_mode is not supported")

        camera_info = CameraInfo(
            camera_model=camera_model,
            resolution=resolution,
            intrinsics_matrix=intrinsics_matrix,
            distortion_coefficients=distortion_coefficients,
            scale_xy=scale_xy,
            shift_xy=shift_xy,
            mask_size=mask_size,
            mask=mask,
            homography_matrix=homography_matrix,
        )
        return (camera_info, input_mode, device_name, ros_topic)

    def _print_camera_info(self, camera_info):
        # debug print param info
        print("[ print camera_info ] : ")
        print("     camera_model: ", camera_info.camera_model)
        print("     resolution: ", camera_info.resolution)
        print("     intrinsics_matrix: ", camera_info.intrinsics_matrix)
        print("     distortion_coefficients: ", camera_info.distortion_coefficients)
        print("     scale_xy: ", camera_info.scale_xy)
        print("     shift_xy: ", camera_info.shift_xy)
        print("     mask_size: ", camera_info.mask_size)
        print("     mask: ", camera_info.mask)
        print("     homography_matrix: ", camera_info.homography_matrix)

    def _create_caps_with_camera_info(self, camera_id_list, info_check_level):
        """创建所有camera_id所对应相机的camera_info和cap对象

        Args:
            camera_id_list (list)): camera_id list

        Returns:
            cap_dict (dict): cap dict with camera info
        """
        cap_dict = {}

        for camera_id in camera_id_list:
            (camera_info, input_mode, device_name, ros_topic) = self._get_camera_info_from_camera_config(camera_id)
            cap = GetFrame(input_mode=input_mode, device_name=device_name, ros_topic=ros_topic)
            ret, camera_info = camera_info_check(camera_info=camera_info, info_check_level=info_check_level)
            if ret is False:
                print("camera_info check failed")
                raise ValueError("camera_info check failed")
            cap.set_camera_info(camera_info)
            cap_dict[camera_id] = cap
        return cap_dict

    def exit_callback(self):
        cv2.destroyAllWindows()
        self.win.quit()
        self.win.destroy()
        print("exit success")


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
        super(MonoCalibrationGUI, self).__init__(camera_config_path)
        self.mono_calibration_node = None
        self.init_gui()

    def init_gui(self):
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

        # ttk label for show result
        self.show_state_label = ttk.Label(self.win, text="nothing happened")

        # layout
        self.gui_layout()
        # loop
        self.win.mainloop()

    def gui_layout(self):
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
        # use two column to show result
        self.show_state_label.grid(row=row_count, column=1, columnspan=2, sticky="NW")
        row_count += 1

    def set_param_callback(self):
        (camera_id, chessboard_size, square_size) = self._get_params_from_gui()
        (camera_info, input_mode, device_name, ros_topic) = self._get_camera_info_from_camera_config(camera_id)
        cap = GetFrame(input_mode=input_mode, device_name=device_name, ros_topic=ros_topic)
        if not cap.cap.isOpened():
            print("failed to get frame/open camera , please check camera config or camera device")
            return
        chessboard_info = ChessboardInfo(n_cols=chessboard_size[0], n_rows=chessboard_size[1], square_size=square_size)
        self.mono_calibration_node = MonoCalibrationNode(cap=cap, chessboard_info=chessboard_info, camera_info=camera_info)
        # debug
        self._print_camera_info(camera_info)
        print("[ set params success ]")

    def start_callback(self):
        if self.mono_calibration_node is None:
            print("please set params first!")
            return
        self.mono_calibration_node.start()
        # debug
        camera_info = self.mono_calibration_node.camera_info
        self._print_camera_info(camera_info)

    def show_result_callback(self):
        print("show result")
        self.mono_calibration_node.show_result()

    def save_callback(self):
        camera_id = self.camera_id_combobox.get()
        if self.mono_calibration_node is None:
            print("please set params first!")
            return
        camera_info = self.mono_calibration_node.camera_info
        print("camera_info: ", camera_info.camera_model)
        # 标定得到的结果存在camera_info中，将其转存至self.camera_config_dict
        self._set_params_to_camera_config(camera_id, camera_info)
        ret = save_camera_config(self.camera_config_path, self.camera_id_list, self.camera_config_dict, self.camera_raw_config_dict)
        if ret:
            print("save camera config success")
        else:
            print("save camera config failed")

    def _get_params_from_gui(self):
        camera_id = self.camera_id_combobox.get()
        chessboard_size = [int(x) for x in self.chessboard_size_combobox.get().split("x")]
        square_size = double(self.chessboard_square_size_combobox.get())
        return (camera_id, chessboard_size, square_size)


# SurroundViewGUI
# +────────────────────────────+───────────────────────────────+
# | camera_id_label            | camera_id_combobox            |
# +────────────────────────────+───────────────────────────────+
# | pattern_size_label         | pattern_size_combobox         |
# +────────────────────────────+───────────────────────────────+
# | pattern_square_size_label  | pattern_square_size_combobox  |
# +────────────────────────────+───────────────────────────────+
# | front_padding_label        | front_padding_combobox        |
# +────────────────────────────+───────────────────────────────+
# | width_padding_label        | width_padding_combobox        |
# +────────────────────────────+───────────────────────────────+
# | back_padding_label         | back_padding_combobox         |
# +────────────────────────────+───────────────────────────────+
# | set_param_button           | start_button                  |
# +────────────────────────────+───────────────────────────────+
# | save_button                | stitching_button              |
# +────────────────────────────+───────────────────────────────+
# | show_result_button         | exit_button                   |
# +────────────────────────────+───────────────────────────────+
class SurroundViewGUI(GUI):
    def __init__(self, camera_config_path):
        super(SurroundViewGUI, self).__init__(camera_config_path)
        self.surround_view_node = None
        self.cc_info = None
        self.cap_dict = {}
        self.camera_info_dict = {}
        self.init_gui()

    def init_gui(self):
        # create root window
        self.win = tk.Tk()
        self.win.title("surround view")
        self.win.geometry("650x400")

        # camera id
        self.camera_id_label = ttk.Label(self.win, text="camera id:")
        self.camera_id_combobox = ttk.Combobox(self.win)
        self.camera_id_combobox["values"] = self.camera_id_list
        self.camera_id_combobox.current(0)
        self.camera_id_get_value = self.camera_id_combobox.get()
        self.camera_id_combobox["state"] = "readonly"

        # mannual entry pattern size and pattern square size
        # pattern size
        self.pattern_size_label = ttk.Label(self.win, text="pattern size:")
        self.pattern_size_combobox = ttk.Combobox(self.win)
        self.pattern_size_combobox["values"] = ("4150x6800", "4000x7100", "4000x7200", "4000x7500")
        self.pattern_size_combobox.current(0)
        self.pattern_size_value = self.pattern_size_combobox.get()
        self.pattern_size_combobox["state"] = "read-write"
        # pattern square size
        self.pattern_square_size_label = ttk.Label(self.win, text="pattern square size(mm):")
        self.pattern_square_size_combobox = ttk.Combobox(self.win)
        self.pattern_square_size_combobox["values"] = ("1000", "1100")
        self.pattern_square_size_combobox.current(0)
        self.pattern_square_size_value = self.pattern_square_size_combobox.get()
        self.pattern_square_size_combobox["state"] = "read-write"
        # front padding
        self.front_padding_label = ttk.Label(self.win, text="front padding(mm):")
        self.front_padding_combobox = ttk.Combobox(self.win)
        self.front_padding_combobox["values"] = ("2000", "2500", "3000")
        self.front_padding_combobox.current(0)
        self.front_padding_value = self.front_padding_combobox.get()
        self.front_padding_combobox["state"] = "read-write"
        # width padding
        self.width_padding_label = ttk.Label(self.win, text="width padding(mm):")
        self.width_padding_combobox = ttk.Combobox(self.win)
        self.width_padding_combobox["values"] = ("2000", "2500", "3000")
        self.width_padding_combobox.current(0)
        self.width_padding_value = self.width_padding_combobox.get()
        self.width_padding_combobox["state"] = "read-write"
        # back padding
        self.back_padding_label = ttk.Label(self.win, text="back padding(mm):")
        self.back_padding_combobox = ttk.Combobox(self.win)
        self.back_padding_combobox["values"] = ("2000", "2500", "3000")
        self.back_padding_combobox.current(0)
        self.back_padding_value = self.back_padding_combobox.get()
        self.back_padding_combobox["state"] = "read-write"

        # ttk button
        self.set_param_button = ttk.Button(self.win, text="set_params", command=self.set_param_callback)
        self.start_button = ttk.Button(self.win, text="start_collect", command=self.start_callback)
        self.save_button = ttk.Button(self.win, text="save", command=self.save_callback)
        self.stitching_button = ttk.Button(self.win, text="stitching", command=self.stitching_callback)
        self.show_result_button = ttk.Button(self.win, text="show_result", command=self.show_result_callback)
        self.exit_button = ttk.Button(self.win, text="exit", command=self.exit_callback)

        # ttk label for show result
        self.show_state_label = ttk.Label(self.win, text="nothing happened")

        # layout
        self.gui_layout()
        # loop
        self.win.mainloop()
        pass

    def gui_layout(self):
        row_count = 1
        # layout combobox
        self.camera_id_label.grid(row=row_count, column=1, sticky="E")
        self.camera_id_combobox.grid(row=row_count, column=2, sticky="NW")
        row_count += 1
        ################################################################################
        self.pattern_size_label.grid(row=row_count, column=1, sticky="E")
        self.pattern_size_combobox.grid(row=row_count, column=2, sticky="NW")
        row_count += 1
        ################################################################################
        self.pattern_square_size_label.grid(row=row_count, column=1, sticky="E")
        self.pattern_square_size_combobox.grid(row=row_count, column=2, sticky="NW")
        row_count += 1
        ################################################################################
        self.front_padding_label.grid(row=row_count, column=1, sticky="E")
        self.front_padding_combobox.grid(row=row_count, column=2, sticky="NW")
        row_count += 1
        ################################################################################
        self.width_padding_label.grid(row=row_count, column=1, sticky="E")
        self.width_padding_combobox.grid(row=row_count, column=2, sticky="NW")
        row_count += 1
        ################################################################################
        self.back_padding_label.grid(row=row_count, column=1, sticky="E")
        self.back_padding_combobox.grid(row=row_count, column=2, sticky="NW")
        row_count += 1
        ################################################################################
        # layout button
        self.set_param_button.grid(row=row_count, column=1, sticky="E")
        self.start_button.grid(row=row_count, column=2, sticky="NW")
        row_count += 1
        ################################################################################
        self.save_button.grid(row=row_count, column=1, sticky="E")
        self.stitching_button.grid(row=row_count, column=2, sticky="NW")
        row_count += 1
        ################################################################################
        self.show_result_button.grid(row=row_count, column=1, sticky="E")
        self.exit_button.grid(row=row_count, column=2, sticky="NW")
        row_count += 1
        ################################################################################
        # use two column to show result
        self.show_state_label.grid(row=row_count, column=1, columnspan=2, sticky="NW")
        row_count += 1

    def set_param_callback(self):
        (camera_id, pattern_size, square_size, front_padding, width_padding, back_padding) = self._get_params_from_gui()
        (camera_info, input_mode, device_name, ros_topic) = self._get_camera_info_from_camera_config(camera_id)
        cap = GetFrame(input_mode=input_mode, device_name=device_name, ros_topic=ros_topic)
        self.cap_dict[camera_id] = cap  # 把每次camera 实例化的cap 存入字典，方便后面的使用

        # cc info 只需要设置一次
        if self.cc_info is None:
            self.cc_info = CCInfo(
                pattern=Patterns.CC,
                pattern_size=pattern_size,
                square_size=square_size,
                front_padding=front_padding,
                width_padding=width_padding,
                back_padding=back_padding,
            )

        if self.surround_view_node is None:
            self.surround_view_node = SurroundViewNode()
            self.surround_view_node._set_current_param(
                current_camera_id=camera_id, current_cap=cap, current_cc_info=self.cc_info, current_camera_info=camera_info
            )
        else:
            self.surround_view_node._set_current_param(
                current_camera_id=camera_id, current_cap=cap, current_cc_info=self.cc_info, current_camera_info=camera_info
            )
        print("set_param success")

    def start_callback(self):
        if self.surround_view_node is None:
            print("please set params first!")
            return
        self.surround_view_node.start()

    def save_callback(self):
        camera_id = self.camera_id_combobox.get()
        if self.surround_view_node is None:
            print("please set params first!")
            return
        camera_info = self.surround_view_node.current_camera_info
        camera_info.mask_size = self.cc_info.img_size
        self._set_params_to_camera_config(camera_id, camera_info)
        ret = save_camera_config(self.camera_config_path, self.camera_id_list, self.camera_config_dict, self.camera_raw_config_dict)
        if ret:
            print("save camera config success")
        else:
            print("save camera config failed")

    def stitching_callback(self):
        print("start stitching")
        camera_id_list = ["/camera/front_wild", "/camera/left_wild", "/camera/right_wild", "/camera/back_wild"]
        info_check_level = InfoCheckLevel.COMPLETED
        cap_dict = self._create_caps_with_camera_info(camera_id_list=camera_id_list, info_check_level=info_check_level)
        front_cap = cap_dict["/camera/front_wild"]
        left_cap = cap_dict["/camera/left_wild"]
        right_cap = cap_dict["/camera/right_wild"]
        back_cap = cap_dict["/camera/back_wild"]
        self.surround_view_node = SurroundViewNode(front_cap=front_cap, left_cap=left_cap, right_cap=right_cap, back_cap=back_cap)

        mask_points_dict = self.surround_view_node.stitching()
        for camera_id in camera_id_list:
            (camera_info, input_mode, device_name, ros_topic) = self._get_camera_info_from_camera_config(camera_id)
            camera_info.mask = mask_points_dict[camera_id]
            self._set_params_to_camera_config(camera_id, camera_info)
        ret = save_camera_config(self.camera_config_path, self.camera_id_list, self.camera_config_dict, self.camera_raw_config_dict)
        if ret:
            print("save camera config success")
        else:
            print("save camera config failed")

    def show_result_callback(self):
        print("start show result")
        self.camera_id_list, self.camera_config_dict, self.camera_raw_config_dict = parse_cameras_config(self.camera_config_path)
        camera_id_list = ["/camera/front_wild", "/camera/left_wild", "/camera/right_wild", "/camera/back_wild"]
        info_check_level = InfoCheckLevel.SURROUND_SPECIAL

        cap_dict = self._create_caps_with_camera_info(camera_id_list=camera_id_list, info_check_level=info_check_level)
        front_cap = cap_dict["/camera/front_wild"]
        left_cap = cap_dict["/camera/left_wild"]
        right_cap = cap_dict["/camera/right_wild"]
        back_cap = cap_dict["/camera/back_wild"]
        self.surround_view_node = SurroundViewNode(front_cap=front_cap, left_cap=left_cap, right_cap=right_cap, back_cap=back_cap)
        self.surround_view_node.show_result()

    def _get_params_from_gui(self):
        camera_id = self.camera_id_combobox.get()
        pattern_size = [int(x) for x in self.pattern_size_combobox.get().split("x")]
        square_size = double(self.pattern_square_size_combobox.get())
        front_padding = double(self.front_padding_combobox.get())
        width_padding = double(self.width_padding_combobox.get())
        back_padding = double(self.back_padding_combobox.get())
        return (camera_id, pattern_size, square_size, front_padding, width_padding, back_padding)


if __name__ == "__main__":
    camera_id_list = ["/camera/front_left", "/camera/front_right"]
    mono_calibration_gui = MonoCalibrationGUI()
    mono_calibration_gui.init_gui()
