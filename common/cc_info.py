import sys

# local
sys.path.append("../")
from common.enum_common import Patterns


class CCInfo:
    """环视标定布(calibration cloth)的相关信息"""

    def __init__(
        self,
        pattern=Patterns.CC,
        pattern_size=(4000, 7000),
        square_size=0.0,
        front_padding=2000,
        width_padding=2000,
        back_padding=2000,
    ):
        self.pattern = pattern
        self.pattern_size = pattern_size
        self.square_size = square_size  # (width,height)
        self.front_padding = front_padding
        self.width_padding = width_padding
        self.back_padding = back_padding
        self.dst_points_dict = {}

        # debug
        print("CCInfo:", self.pattern_size, self.square_size, self.front_padding, self.width_padding, self.back_padding)

        self._cc_info_init()

    def _cc_info_init(self):
        def calculate_size(pattern_size, front_padding, width_padding, back_padding):
            new_size = (
                pattern_size[0] + width_padding * 2,
                pattern_size[1] + front_padding + back_padding,
            )
            width_pixel = 1000  # 固定宽为1000像素
            height_pixel = int((new_size[1] / new_size[0]) * width_pixel)
            img_size = (width_pixel, height_pixel)
            return new_size, img_size

        def calculate_dst_points(size, img_size, square_size, front_padding, width_padding, back_padding):
            # 目前只考虑对角取点的情况
            # 计算四个黑色矩形的坐标
            # 四个黑色矩形编号分别为 左上角A 右上角B 右下角C 左下角D (顺时针)
            # 每一个黑色矩形的四个角点坐标编号分别为 左上角X0 右上角X1 右下角X2 左下角X3,X代指矩形编号,小写表示比例，大写表示pixel尺寸 (顺时针)
            total_width, total_height = size
            a0 = ((width_padding) / total_width, (front_padding) / total_height)
            a1 = ((width_padding + square_size) / total_width, (front_padding) / total_height)
            a2 = ((width_padding + square_size) / total_width, (front_padding + square_size) / total_height)
            a3 = ((width_padding) / total_width, (front_padding + square_size) / total_height)
            b0 = ((total_width - width_padding - square_size) / total_width, (front_padding) / total_height)
            b1 = ((total_width - width_padding) / total_width, (front_padding) / total_height)
            b2 = ((total_width - width_padding) / total_width, (front_padding + square_size) / total_height)
            b3 = (
                (total_width - width_padding - square_size) / total_width,
                (front_padding + square_size) / total_height,
            )
            c0 = (
                (total_width - width_padding - square_size) / total_width,
                (total_height - back_padding - square_size) / total_height,
            )
            c1 = (
                (total_width - width_padding) / total_width,
                (total_height - back_padding - square_size) / total_height,
            )
            c2 = ((total_width - width_padding) / total_width, (total_height - back_padding) / total_height)
            c3 = (
                (total_width - width_padding - square_size) / total_width,
                (total_height - back_padding) / total_height,
            )
            d0 = ((width_padding) / total_width, (total_height - back_padding - square_size) / total_height)
            d1 = (
                (width_padding + square_size) / total_width,
                (total_height - back_padding - square_size) / total_height,
            )
            d2 = ((width_padding + square_size) / total_width, (total_height - back_padding) / total_height)
            d3 = ((width_padding) / total_width, (total_height - back_padding) / total_height)

            A0 = ((int)(a0[0] * img_size[0]), int(a0[1] * img_size[1]))
            A1 = ((int)(a1[0] * img_size[0]), int(a1[1] * img_size[1]))
            A2 = ((int)(a2[0] * img_size[0]), int(a2[1] * img_size[1]))
            A3 = ((int)(a3[0] * img_size[0]), int(a3[1] * img_size[1]))
            B0 = ((int)(b0[0] * img_size[0]), int(b0[1] * img_size[1]))
            B1 = ((int)(b1[0] * img_size[0]), int(b1[1] * img_size[1]))
            B2 = ((int)(b2[0] * img_size[0]), int(b2[1] * img_size[1]))
            B3 = ((int)(b3[0] * img_size[0]), int(b3[1] * img_size[1]))
            C0 = ((int)(c0[0] * img_size[0]), int(c0[1] * img_size[1]))
            C1 = ((int)(c1[0] * img_size[0]), int(c1[1] * img_size[1]))
            C2 = ((int)(c2[0] * img_size[0]), int(c2[1] * img_size[1]))
            C3 = ((int)(c3[0] * img_size[0]), int(c3[1] * img_size[1]))
            D0 = ((int)(d0[0] * img_size[0]), int(d0[1] * img_size[1]))
            D1 = ((int)(d1[0] * img_size[0]), int(d1[1] * img_size[1]))
            D2 = ((int)(d2[0] * img_size[0]), int(d2[1] * img_size[1]))
            D3 = ((int)(d3[0] * img_size[0]), int(d3[1] * img_size[1]))
            # 对角梯形
            front_wild_dst_points = np.array([A0, B1, B3, A2], dtype=np.float32)
            front_wild_dst_points = front_wild_dst_points.reshape((-1, 1, 2))
            left_wild_dst_points = np.array([D3, A0, A2, D1], dtype=np.float32)
            left_wild_dst_points = left_wild_dst_points.reshape((-1, 1, 2))
            right_wild_dst_points = np.array([B1, C2, C0, B3], dtype=np.float32)
            right_wild_dst_points = right_wild_dst_points.reshape((-1, 1, 2))
            back_wild_dst_points = np.array([C2, D3, D1, C0], dtype=np.float32)
            back_wild_dst_points = back_wild_dst_points.reshape((-1, 1, 2))
            dst_points_dict = {}
            dst_points_dict["front"] = front_wild_dst_points
            dst_points_dict["left"] = left_wild_dst_points
            dst_points_dict["right"] = right_wild_dst_points
            dst_points_dict["back"] = back_wild_dst_points
            return dst_points_dict

        self.size, self.img_size = calculate_size(
            self.pattern_size, self.front_padding, self.width_padding, self.back_padding
        )
        self.dst_points_dict = calculate_dst_points(
            self.size, self.img_size, self.square_size, self.width_padding, self.front_padding, self.back_padding
        )
