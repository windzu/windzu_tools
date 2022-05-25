"""
Author: windzu
Date: 2022-04-09 11:05:05
LastEditTime: 2022-04-15 15:48:07
LastEditors: windzu
Description: 
FilePath: /windzu_ws/src/tools/common/gui.py
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
sys.path.append("../")


class GUI(object):
    def __init__(self):
        print("*** gui init ***")

    @abstractclassmethod
    def set_param_callback(self):
        """设置参数按钮的回调函数
        """
        pass

    @abstractclassmethod
    def start_callback(self):
        """开始按钮的回调函数
        """
        pass

    @abstractclassmethod
    def show_result_callback(self):
        """显示结果按钮的回调函数
        """
        pass

    @abstractclassmethod
    def save_callback(self):
        """保存按钮的回调函数
        """
        pass

    @abstractclassmethod
    def exit_callback(self):
        """退出按钮的回调函数
        """
        pass

    @abstractclassmethod
    def __parse_files_init(self):
        """按需解析配置文件，将其实例化为类对象
        """
        pass

    @abstractclassmethod
    def __gui_init(self):
        """初始化GUI的布局
        """
        pass

    @abstractclassmethod
    def __gui_layout(self):
        """对GUI进行布局
        """
        pass

    @abstractclassmethod
    def __get_params_from_gui(self):
        """从GUI中获取参数
        """
        pass

