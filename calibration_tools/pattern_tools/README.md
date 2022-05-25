<!--
 * @Author: windzu
 * @Date: 2022-03-04 10:19:57
 * @LastEditTime: 2022-03-07 17:46:48
 * @LastEditors: windzu
 * @Description: 
 * @FilePath: /pattern_tools/README.md
 * @Copyright (C) 2021-2022 plusgo Company Limited. All rights reserved.
 * @Licensed under the Apache License, Version 2.0 (the License)
-->
ref:https://docs.opencv.org/4.x/da/d0d/tutorial_camera_calibration_pattern.html

## use

> to generate various calibration svg calibration patterns

### help

* python gen_pattern.py --help

### example

Example

create a checkerboard pattern in file chessboard.svg with 9 rows, 6 columns and a square size of 20mm:

    python gen_pattern.py -o chessboard.svg --rows 9 --columns 6 --type checkerboard --square_size 20
create a circle board pattern in file circleboard.svg with 7 rows, 5 columns and a radius of 15mm:

    python gen_pattern.py -o circleboard.svg --rows 7 --columns 5 --type circles --square_size 15
create a circle board pattern in file acircleboard.svg with 7 rows, 5 columns and a square size of 10mm and less spacing between circle:

    python gen_pattern.py -o acircleboard.svg --rows 7 --columns 5 --type acircles --square_size 10 --radius_rate 2
create a radon checkerboard for findChessboardCornersSB() with markers in (7 4), (7 5), (8 5) cells:

    python gen_pattern.py -o radon_checkerboard.svg --rows 10 --columns 15 --type radon_checkerboard -s 12.1 -m 7 4 7 5 8 5
If you want to change unit use -u option (mm inches, px, m)

If you want to change page size use -w and -h options

If you want to create a ChArUco board read tutorial Detection of ChArUco Corners in opencv_contrib tutorial.