# README

## 工程结构

### camera_gui.py

> 所有相机标定相关的gui都在其中实现，其继承自 GUI类，是标定工具进入的接口，目前包含以下几种gui界面

* MonoCalibrationGUI ： 单目的标定GUI
* SurroundViewGUI ： 环视标定GUI

### calibrator.py

> 集成了所有的标定方法，集成自Calibrator类

* MonoCalibrator
* StereoCalibrator

### calibration_node.py

> 对calibrator的包装，为其提供外部支持，例如calibratior准备外部的图像输入、标定板信息等

### surround_view_node.py

> 单独针对环视拼接功能的实现

