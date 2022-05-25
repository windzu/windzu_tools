"""
Author: windzu
Date: 2022-03-23 11:43:37
LastEditTime: 2022-03-23 11:43:38
LastEditors: windzu
Description: 
FilePath: /monocular_camera_calibration/test/utils.py
@Copyright (C) 2021-2022 plusgo Company Limited. All rights reserved.
@Licensed under the Apache License, Version 2.0 (the License)
"""
import yaml
import numpy as np
import cv2
import sys

sys.path.append("..")
from common.camera_common import CameraModel, Patterns, InfoCheckLevel


def parse_cameras_config(config_path):
    """读取配置文件，获得各个相机的参数
    只有camera_id存储与list中,其他的存储为字典,字典中的key为相机id,value为相机参数
    Return:
        camera_id_list (list): 相机配置编号
        camera_config_dict (dict): 解析格式后的配置,key为参数名,value为dict,其key为相机id,value为相机参数
        camera_raw_config_dict (dict): yaml直接load的原始格式,用于更新参数并存储
    """
    camera_config_path = config_path
    with open(camera_config_path, "r") as f:
        camera_raw_config_dict = yaml.load(f)

    def prase_camera_model(camera_config):
        return camera_config["camera_model"]

    def parse_input_mode(camera_config):
        return camera_config["input_mode"]

    def parse_device_name(camera_config):
        return camera_config["device_name"]

    def parse_ros_topic(camera_config):
        return camera_config["ros_topic"]

    def parse_resolution(camera_config):
        return camera_config["resolution"]

    def parse_intrinsics_matrix(camera_config):
        if "intrinsics_matrix" in camera_config.keys() and camera_config["intrinsics_matrix"] is not None:
            intrinsics_matrix = camera_config["intrinsics_matrix"]
            intrinsics_matrix = np.array(intrinsics_matrix, dtype=np.float32)
            intrinsics_matrix = intrinsics_matrix.reshape(3, 3)
            return intrinsics_matrix
        else:
            return None

    def parse_distortion_coefficients(camera_config):
        if "distortion_coefficients" in camera_config.keys() and camera_config["distortion_coefficients"] is not None:
            distortion_coefficients = camera_config["distortion_coefficients"]
            distortion_coefficients = np.array(distortion_coefficients, dtype=np.float32)
            return distortion_coefficients
        else:
            return None

    def parse_scale_xy(camera_config):
        if "scale_xy" in camera_config.keys():
            scale_xy = camera_config["scale_xy"]
            return scale_xy
        else:
            return None

    def parse_shift_xy(camera_config):
        if "shift_xy" in camera_config.keys():
            shift_xy = camera_config["shift_xy"]
            return shift_xy
        else:
            return None

    def parse_mask_size(camera_config):
        if "mask_size" in camera_config.keys() and camera_config["mask_size"] is not None:
            mask_size = camera_config["mask_size"]
            return mask_size
        else:
            return None

    def parse_mask(camera_config, mask_size):
        if mask_size is None:
            return None
        mask = np.zeros((mask_size[1], mask_size[0]), np.uint8)
        if "mask" in camera_config.keys() and camera_config["mask"] is not None:
            mask_polygon_corner_points = camera_config["mask"]
            mask_polygon_corner_points = np.array(mask_polygon_corner_points, np.int32)
            mask_polygon_corner_points = mask_polygon_corner_points.reshape((-1, 1, 2))
            mask = cv2.fillPoly(mask, [mask_polygon_corner_points], 255)
        else:
            mask = None
        return mask

    def parse_ploygon_corner_points(camera_config):
        if "mask" in camera_config.keys() and camera_config["mask"] is not None:
            mask_polygon_corner_points = camera_config["mask"]
        else:
            mask_polygon_corner_points = None
        return mask_polygon_corner_points

    def parse_homography_matrix(camera_config):
        if "homography_matrix" in camera_config.keys() and camera_config["homography_matrix"] is not None:
            homography_matrix = camera_config["homography_matrix"]
            homography_matrix = np.array(homography_matrix, dtype=np.float32)
            homography_matrix = homography_matrix.reshape(3, 3)
            return homography_matrix
        else:
            return None

    def parse_imu_to_camera_translation(camera_config):
        if "imu_to_camera_translation" in camera_config.keys():
            imu_to_camera_translation_list = camera_config["imu_to_camera_translation"]
            return imu_to_camera_translation_list
        else:
            return None

    def parse_imu_to_camera_rotation_offset(camera_config):
        if "imu_to_camera_rotation_offset" in camera_config.keys():
            imu_to_camera_rotation_offset_list = camera_config["imu_to_camera_rotation_offset"]
            return imu_to_camera_rotation_offset_list
        else:
            return None

    camera_id_list = []
    camera_model_dict = {}
    input_mode_dict = {}
    device_name_dict = {}
    ros_topic_dict = {}
    resolution_dict = {}
    intrinsics_matrix_dict = {}
    distortion_coefficients_dict = {}
    scale_xy_dict = {}
    shift_xy_dict = {}
    mask_size_dict = {}
    mask_dict = {}
    mask_ploygon_corner_points_dict = {}  # 存储mask的多边形角点
    homography_matrix_dict = {}
    imu_to_camera_translation_dict = {}
    imu_to_camera_rotation_offset_dict = {}

    for key, value in camera_raw_config_dict.items():
        camera_id_list.append(key)
        camera_model_dict[key] = prase_camera_model(value)
        input_mode_dict[key] = parse_input_mode(value)
        device_name_dict[key] = parse_device_name(value)
        ros_topic_dict[key] = parse_ros_topic(value)
        resolution_dict[key] = parse_resolution(value)
        intrinsics_matrix_dict[key] = parse_intrinsics_matrix(value)
        distortion_coefficients_dict[key] = parse_distortion_coefficients(value)
        scale_xy_dict[key] = parse_scale_xy(value)
        shift_xy_dict[key] = parse_shift_xy(value)
        mask_size_dict[key] = parse_mask_size(value)  # 必须先解析size，再解析mask(因为构建mask需要size)
        mask_dict[key] = parse_mask(value, mask_size_dict[key])
        mask_ploygon_corner_points_dict[key] = parse_ploygon_corner_points(value)
        homography_matrix_dict[key] = parse_homography_matrix(value)
        imu_to_camera_translation_dict[key] = parse_imu_to_camera_translation(value)  # x y z 方向的平移量
        imu_to_camera_rotation_offset_dict[key] = parse_imu_to_camera_rotation_offset(value)  # 围绕相机坐标系下xyz三个轴的细微旋转量，弧度制

    camera_config_dict = {
        "camera_model_dict": camera_model_dict,
        "input_mode_dict": input_mode_dict,
        "device_name_dict": device_name_dict,
        "ros_topic_dict": ros_topic_dict,
        "resolution_dict": resolution_dict,
        "intrinsics_matrix_dict": intrinsics_matrix_dict,
        "distortion_coefficients_dict": distortion_coefficients_dict,
        "scale_xy_dict": scale_xy_dict,
        "shift_xy_dict": shift_xy_dict,
        "mask_size_dict": mask_size_dict,
        "mask_dict": mask_dict,
        "mask_ploygon_corner_points_dict": mask_ploygon_corner_points_dict,
        "homography_matrix_dict": homography_matrix_dict,
        "imu_to_camera_translation_dict": imu_to_camera_translation_dict,
        "imu_to_camera_rotation_offset_dict": imu_to_camera_rotation_offset_dict,
    }

    return camera_id_list, camera_config_dict, camera_raw_config_dict


def save_camera_config(camera_config_path, camera_id_list, camera_config_dict, camera_raw_config_dict):
    """
    保存camera配置
    Args:
        camera_config_path (str): camera配置路径
        camera_config_dict (dict): camera配置字典
        camera_raw_config_dict (dict): camera原始格式配置字典
    """

    def serialize_intrinsics_matrix(intrinsics_matrix):
        if intrinsics_matrix is None or len(intrinsics_matrix) != 3:
            return None
        else:
            return intrinsics_matrix.flatten().tolist()

    def serialize_distortion_coefficients(distortion_coefficients):
        if distortion_coefficients is None or len(distortion_coefficients) < 4:
            return None
        else:
            return distortion_coefficients.flatten().tolist()

    def serialize_mask_size(mask_size):
        if mask_size is None:
            return None
        else:
            return np.array(mask_size).astype(np.int32).flatten().tolist()

    def serialize_mask(mask_ploygon_corner_points):
        if mask_ploygon_corner_points is None:
            return None
        else:
            return mask_ploygon_corner_points

    def serialize_homography_matrix(homography_matrix):
        if homography_matrix is None or len(homography_matrix) != 3:
            return None
        else:
            return homography_matrix.flatten().tolist()

    for camera_id in camera_id_list:

        intrinsics_matrix = camera_config_dict["intrinsics_matrix_dict"][camera_id]
        if intrinsics_matrix is not None:
            camera_raw_config_dict[camera_id]["intrinsics_matrix"] = serialize_intrinsics_matrix(intrinsics_matrix)
        else:
            print("[ save_camera_config ] intrinsics_matrix is None")

        distortion_coefficients = camera_config_dict["distortion_coefficients_dict"][camera_id]
        if distortion_coefficients is not None:
            camera_raw_config_dict[camera_id]["distortion_coefficients"] = serialize_distortion_coefficients(distortion_coefficients)
        else:
            print("[ save_camera_config ] distortion_coefficients is None")

        # scale_xy 和 shift_xy 只针对fisheye需要
        # scale_xy
        scale_xy = camera_config_dict["scale_xy_dict"][camera_id]
        if scale_xy is not None:
            camera_raw_config_dict[camera_id]["scale_xy"] = scale_xy
        else:
            print("[ save_camera_config ] scale_xy is None")

        # shift_xy
        shift_xy = camera_config_dict["shift_xy_dict"][camera_id]
        if shift_xy is not None:
            camera_raw_config_dict[camera_id]["shift_xy"] = shift_xy
        else:
            print("[ save_camera_config ] shift_xy is None")

        # mask_size mask homography_matrix_dict只针对环视拼接需要
        # mask_size
        mask_size = camera_config_dict["mask_size_dict"][camera_id]
        mask_size_ret = serialize_mask_size(mask_size)
        if mask_size_ret is not None:
            camera_raw_config_dict[camera_id]["mask_size"] = mask_size_ret
        else:
            print("[ save_camera_config ] mask_size is None")
        # mask
        # mask = camera_config_dict["mask_dict"][camera_id]
        mask_ploygon_corner_points = camera_config_dict["mask_ploygon_corner_points_dict"][camera_id]
        mask_ret = serialize_mask(mask_ploygon_corner_points)
        if mask_ret is not None:
            camera_raw_config_dict[camera_id]["mask"] = mask_ret
        else:
            print("[ save_camera_config ] mask is None")

        homography_matrix = camera_config_dict["homography_matrix_dict"][camera_id]
        homography_matrix_ret = serialize_homography_matrix(homography_matrix)
        if homography_matrix_ret is not None:
            camera_raw_config_dict[camera_id]["homography_matrix"] = homography_matrix_ret
        else:
            print("[ save_camera_config ] homography_matrix is None")

    with open(camera_config_path, "w") as f:
        yaml.dump(camera_raw_config_dict, f, default_flow_style=False)
        return True


def camera_info_check(camera_info, info_check_level):
    """对camera_info的完整性进行检查,如果camera_info中的相关信息不完整,则返回False,检查分几个等级
    1. BASE: 基础检查,只检查camera_model 和 resolution
        - camera_model
        - resolution
    2. ADVANCED: 检查相机内参和畸变系数,如果是鱼眼还需要检查scale_xy和shift_xy
        - intrinsics_matrix 
        - distortion_coefficients
        * scale_xy
        * shift_xy
    3. COMPLETED : 完全的完备性检查,包括上述所有检查,并且如果map没有计算,还会将map计算
        - map1 , map2
    4. SURROUND_SPECIAL : 专门用于检查用于环视的检查,包括上述所有检查,还要检查mask_size mask homography_matrix
        - mask_size
        - mask
        - homography_matrix
    """

    def base_check(camera_info):
        if not isinstance(camera_info.camera_model, CameraModel):
            print("camera_info.camera_model is not CameraModel")
            return False, camera_info
        if camera_info.resolution is None or len(camera_info.resolution) != 2:
            print("camera_info.resolution is None or len(camera_info.resolution) != 2")
            return False, camera_info
        return True, camera_info

    def advanced_check(camera_info):
        if camera_info.intrinsics_matrix is None or len(camera_info.intrinsics_matrix) != 3:
            print("camera_info.intrinsics_matrix is None or len(camera_info.intrinsics_matrix) != 3")
            return False, camera_info
        if camera_info.distortion_coefficients is None:
            print("camera_info.distortion_coefficients is None ")
            return False, camera_info

        if camera_info.camera_model is CameraModel.FISHEYE:
            # 如果是鱼眼相机，则一定要检测scale_xy和shift_xy
            if camera_info.scale_xy is None or len(camera_info.scale_xy) != 2:
                print("no scale_xy")
                return False, camera_info
            if camera_info.shift_xy is None or len(camera_info.shift_xy) != 2:
                print("no shift_xy")
                return False, camera_info

        return True, camera_info

    def completed_check(camera_info):
        if camera_info.map1 is None or camera_info.map2 is None:
            print("no map1 or map2 , now calculate it")
            if camera_info.camera_model is CameraModel.FISHEYE:
                new_mat = camera_info.intrinsics_matrix.copy()
                new_mat[0, 0] *= camera_info.scale_xy[0]
                new_mat[1, 1] *= camera_info.scale_xy[1]
                new_mat[0, 2] += camera_info.shift_xy[0]
                new_mat[1, 2] += camera_info.shift_xy[1]
                camera_info.map1, camera_info.map2 = cv2.fisheye.initUndistortRectifyMap(
                    camera_info.intrinsics_matrix,
                    camera_info.distortion_coefficients,
                    np.eye(3, 3),
                    new_mat,
                    camera_info.resolution,
                    cv2.CV_16SC2,
                )
            elif camera_info.camera_model is CameraModel.PINHOLE:
                camera_info.map1, camera_info.map2 = cv2.initUndistortRectifyMap(
                    camera_info.intrinsics_matrix,
                    camera_info.distortion_coefficients,
                    np.eye(3, 3),
                    camera_info.intrinsics_matrix,
                    camera_info.resolution,
                    cv2.CV_16SC2,
                )
            else:
                print("camera model not support")
                return False, camera_info
        return True, camera_info

    def surround_special_check(camera_info):
        if camera_info.camera_model is CameraModel.FISHEYE:
            if camera_info.mask_size is None:
                print("no mask_size")
                return False, camera_info
            if camera_info.mask is None:
                print("no mask")
                return False, camera_info
            if camera_info.homography_matrix is None:
                print("no homography_matrix")
                return False, camera_info
        else:
            return False, camera_info
        return True, camera_info

    if info_check_level == InfoCheckLevel.BASE:
        return base_check(camera_info)
    elif info_check_level == InfoCheckLevel.ADVANCED:
        ret, _ = base_check(camera_info)
        if not ret:
            return False, camera_info
        return advanced_check(camera_info)
    elif info_check_level == InfoCheckLevel.COMPLETED:
        ret, _ = base_check(camera_info)
        if not ret:
            return False, camera_info
        ret, _ = advanced_check(camera_info)
        if not ret:
            return False, camera_info
        return completed_check(camera_info)
    elif info_check_level == InfoCheckLevel.SURROUND_SPECIAL:
        ret, _ = base_check(camera_info)
        if not ret:
            return False, camera_info
        ret, _ = advanced_check(camera_info)
        if not ret:
            return False, camera_info
        ret, _ = completed_check(camera_info)
        if not ret:
            return False, camera_info
        return surround_special_check(camera_info)
    else:
        return False, camera_info


def calculate_cameras_mask(front_frame, left_frame, right_frame, back_frame, mask_size):
    """计算所有相机的mask,每次启动时候计算一次
        1. 首先获取同一时刻所有相机进行单应性变换后的帧，分为 front, left, right, back
        2. 通过转灰度后,根据阈值过滤掉不需要的部分(front保留最上部分,left保留最左部分...),得到初步的mask
        3. 通过bitwise_and计算相邻两个方向的初步mask相交的拐角(因为有视角重叠，所以必然有相交部分),将相交部分拟合为一个矩形，得到靠内部的角点
        4. 找到最靠外的角点,以该角点为基准,连接目标图的四个拐角,得到四个梯形mask
        """

    def get_corner_mask(direction0, frame0, direction1, frame1, position="A", mask_size=None):
        """获取该拐角重叠区域的countour 以及靠内部的点（用于与图像拐角连线进行分割）
            """

        def get_threshold_mask(direction, frame, mask_size):
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            ret, mask_frame = cv2.threshold(gray_frame, 0, 255, cv2.THRESH_BINARY)
            mask_frame = cv2.morphologyEx(mask_frame, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

            # 此时mask_frame分为两块联通区
            # 根据方向选择需要的contour
            # front 需要上面的一块
            # left 需要左边的一块
            # right 需要右边的一块
            # back 需要下面的一块
            contours, hierarchy = cv2.findContours(mask_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            width = mask_size[0]
            height = mask_size[1]
            temp_contour = None
            judge_point = None
            if direction == "front":
                judge_point = (int(width / 2), 50)
            elif direction == "left":
                judge_point = (50, int(height / 2))
            elif direction == "right":
                judge_point = (width - 50, int(height / 2))
            elif direction == "back":
                judge_point = (int(width / 2), height - 50)

            for contour in contours:
                if cv2.pointPolygonTest(contour, judge_point, False) > 0:
                    temp_contour = contour
                    break

            mask = np.zeros((height, width), np.uint8)
            cv2.fillPoly(mask, pts=[temp_contour], color=(255))

            return mask

        def get_max_area_counter(contours):
            if len(contours) == 0:
                print("no contours in corner mask")
                return None
            # get max area counter
            max_area = 0
            max_area_counter_index = 0
            for i in range(len(contours)):
                area = cv2.contourArea(contours[i])
                if area > max_area:
                    max_area = area
                    max_area_counter_index = i
            return contours[max_area_counter_index]

        mask_frame0 = get_threshold_mask(direction0, frame0, mask_size)
        mask_frame1 = get_threshold_mask(direction1, frame1, mask_size)
        corner_mask = cv2.bitwise_and(mask_frame0, mask_frame1)
        contours, hierarchy = cv2.findContours(corner_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        corner_counter = get_max_area_counter(contours)
        x, y, w, h = cv2.boundingRect(corner_counter)
        if position == "A":
            return corner_counter, [x + w, y + h]
        elif position == "B":
            return corner_counter, [x, y + h]
        elif position == "C":
            return corner_counter, [x, y]
        elif position == "D":
            return corner_counter, [x + w, y]

    _, A_corner_mask_point = get_corner_mask("front", front_frame, "left", left_frame, position="A", mask_size=mask_size)
    _, B_corner_mask_point = get_corner_mask("front", front_frame, "right", right_frame, position="B", mask_size=mask_size)
    _, C_corner_mask_point = get_corner_mask("back", back_frame, "right", right_frame, position="C", mask_size=mask_size)
    _, D_corner_mask_point = get_corner_mask("back", back_frame, "left", left_frame, position="D", mask_size=mask_size)

    print("A_corner_mask_point:", A_corner_mask_point)
    print("B_corner_mask_point:", B_corner_mask_point)
    print("C_corner_mask_point:", C_corner_mask_point)
    print("D_corner_mask_point:", D_corner_mask_point)

    # 找到最靠外的角点
    # 确定左边x最小值
    if A_corner_mask_point[0] < D_corner_mask_point[0]:
        D_corner_mask_point[0] = A_corner_mask_point[0]
    else:
        A_corner_mask_point[0] = D_corner_mask_point[0]
    # 确定上边y最小值
    if A_corner_mask_point[1] < B_corner_mask_point[1]:
        B_corner_mask_point[1] = A_corner_mask_point[1]
    else:
        A_corner_mask_point[1] = B_corner_mask_point[1]

    if B_corner_mask_point[0] < C_corner_mask_point[0]:
        B_corner_mask_point[0] = C_corner_mask_point[0]
    else:
        C_corner_mask_point[0] = B_corner_mask_point[0]

    if C_corner_mask_point[1] < D_corner_mask_point[1]:
        C_corner_mask_point[1] = D_corner_mask_point[1]
    else:
        D_corner_mask_point[1] = C_corner_mask_point[1]

    frame_width = mask_size[0]
    frame_height = mask_size[1]
    mask_dict = {}
    mask_dict["/camera/front_wild"] = ((0, 0), (frame_width - 1, 0), B_corner_mask_point, A_corner_mask_point)
    mask_dict["/camera/left_wild"] = ((0, 0), A_corner_mask_point, D_corner_mask_point, (0, frame_height - 1))
    mask_dict["/camera/right_wild"] = (
        (frame_width - 1, 0),
        (frame_width - 1, frame_height - 1),
        C_corner_mask_point,
        B_corner_mask_point,
    )
    mask_dict["/camera/back_wild"] = (
        D_corner_mask_point,
        C_corner_mask_point,
        (frame_width - 1, frame_height - 1),
        (0, frame_height - 1),
    )

    front_mask_points = np.array(mask_dict["/camera/front_wild"]).reshape(-1, 2)
    left_mask_points = np.array(mask_dict["/camera/left_wild"]).reshape(-1, 2)
    right_mask_points = np.array(mask_dict["/camera/right_wild"]).reshape(-1, 2)
    back_mask_points = np.array(mask_dict["/camera/back_wild"]).reshape(-1, 2)

    return (front_mask_points, left_mask_points, right_mask_points, back_mask_points)


class PointSelector(object):
    """ 通过鼠标从opencv显示窗口中选择点
    - press `d` to delete the last points
    - press `q` to quit
    - press `Enter` to confirm
    """

    POINT_COLOR = (0, 0, 255)
    FILL_COLOR = (0, 255, 255)

    def __init__(self, image, title="PointSelector"):
        self.image = image
        self.title = title
        self.keypoints = []

    def draw_image(self):
        """
        Display the selected keypoints and draw the convex hull.
        """
        # the trick: draw on another new image
        new_image = self.image.copy()

        # draw the selected keypoints
        for i, pt in enumerate(self.keypoints):
            cv2.circle(new_image, pt, 6, self.POINT_COLOR, -1)
            cv2.putText(new_image, str(i), (pt[0], pt[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.POINT_COLOR, 2)

        # draw a line if there are two points
        if len(self.keypoints) == 2:
            p1, p2 = self.keypoints
            cv2.line(new_image, p1, p2, self.POINT_COLOR, 2)

        # draw the convex hull if there are more than two points
        if len(self.keypoints) > 2:
            mask = self.create_mask_from_pixels(self.keypoints, self.image.shape)
            new_image = self.draw_mask_on_image(new_image, mask)

        cv2.imshow(self.title, new_image)

    def onclick(self, event, x, y, flags, param):
        """
        Click on a point (x, y) will add this points to the list
        and re-draw the image.
        """
        if event == cv2.EVENT_LBUTTONDOWN:
            print("click ({}, {})".format(x, y))
            self.keypoints.append((x, y))
            self.draw_image()

    def loop(self):
        cv2.namedWindow(self.title)
        cv2.setMouseCallback(self.title, self.onclick, param=())
        cv2.imshow(self.title, self.image)

        while True:
            click = cv2.getWindowProperty(self.title, cv2.WND_PROP_AUTOSIZE)
            if click < 0:
                return False

            key = cv2.waitKey(1) & 0xFF

            # press q to return False
            if key == ord("q"):
                return False

            # press d to delete the last point
            if key == ord("d"):
                if len(self.keypoints) > 0:
                    x, y = self.keypoints.pop()
                    print("Delete ({}, {})".format(x, y))
                    self.draw_image()

            # press Enter to confirm
            if key == 13:
                cv2.destroyWindow(self.title)
                return True

    def create_mask_from_pixels(self, pixels, image_shape):
        """
        Create mask from the convex hull of a list of pixels.
        """
        pixels = np.int32(pixels).reshape(-1, 2)
        hull = cv2.convexHull(pixels)
        mask = np.zeros(image_shape[:2], np.int8)
        cv2.fillConvexPoly(mask, hull, 1, lineType=8, shift=0)
        mask = mask.astype(np.bool)
        return mask

    def draw_mask_on_image(self, image, mask):
        """
        Paint the region defined by a given mask on an image.
        """
        new_image = np.zeros_like(image)
        new_image[:, :] = self.FILL_COLOR
        mask = np.array(mask, dtype=np.uint8)
        new_mask = cv2.bitwise_and(new_image, new_image, mask=mask)
        cv2.addWeighted(image, 1.0, new_mask, 0.5, 0.0, image)
        return image

