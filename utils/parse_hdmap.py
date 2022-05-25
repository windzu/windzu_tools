"""
Author: windzu
Date: 2022-04-09 16:46:38
LastEditTime: 2022-04-16 22:37:41
LastEditors: windzu
Description: 
FilePath: /windzu_ws/src/tools/utils/parse_hdmap.py
@Copyright (C) 2021-2022 plusgo Company Limited. All rights reserved.
@Licensed under the Apache License, Version 2.0 (the License)
"""
import sys
import json
import utm
from pyproj import Transformer

# local
sys.path.append("../")
from common.tl_info import TLInfo


def parse_hdmap(hdmap_config_path):
    """hdmap 是json格式,想将其解析为dict
    """
    f = open(hdmap_config_path)
    data = json.load(f)
    f.close()
    data = data["default"]
    tl_id_list = []
    tl_info_dict = {}
    # 参数1：WGS84地理坐标系统 对应 epsg编号：4326
    # 参数2：坐标系WKID 中国沿海 UTM区号为WGS_1984_UTM_Zone_49N 对应 epsg编号 32649
    transformer = Transformer.from_crs("epsg:4326", "epsg:32650")
    for i in data:
        tl_id = i["tl_id"]
        latitude = i["latitude"]
        longtitude = i["longtitude"]
        height = i["height"]
        next_motivation = i["next_motivation"]
        # u = utm.from_latlon(latitude, longtitude)
        (x, y) = transformer.transform(latitude, longtitude)
        # x = u[0]
        # y = u[1]
        z = height

        tl_id_list.append(tl_id)
        tl_info = TLInfo(tl_id=tl_id, latitude=latitude, longtitude=longtitude, x=x, y=y, z=z, direction=next_motivation)
        tl_info_dict[tl_id] = tl_info

    return (tl_id_list, tl_info_dict)
