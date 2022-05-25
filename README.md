<!--
 * @Author: windzu
 * @Date: 2022-05-18 11:58:40
 * @LastEditTime: 2022-05-18 11:58:56
 * @LastEditors: windzu
 * @Description: 
 * @FilePath: /windzu_ws/src/tools/README.md
 * @Copyright (C) 2021-2022 plusgo Company Limited. All rights reserved.
 * @Licensed under the Apache License, Version 2.0 (the License)
-->
# windzu_tools

## 项目介绍

主要存储标定相关的工具，以及其他的一些小工具

## 软件架构

>  软件架构说明

* calibration_tools : 标定相关的工具
  * mono_calibration ：单目标定工具
  * camera_imu_calibration ： 相机imu的手动标定小工具
  * pattern_tools ： 生成标定板的小工具
  * surround_view_calibation ：环视拼接
* common：通用的基类、枚举、结构体
* utils：功能性函数

## quick start

### 安装依赖

```python
pip3 install -r requirements.txt
```



## 说明文档

# 已知bug
* 关闭窗口bug

