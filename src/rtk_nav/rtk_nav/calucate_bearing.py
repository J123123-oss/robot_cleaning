#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
def calculate_bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    计算两点间的绝对朝向角（方位角），用于初始点→第一个航点的转向目标
    :param lat1: 起点纬度
    :param lon1: 起点经度
    :param lat2: 终点纬度
    :param lon2: 终点经度
    :return: 绝对朝向角（°，归一化到[-180°, 180°]，与IMU航向角格式一致）
    """
    # 转换为弧度
    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    lat2_rad = math.radians(lat2)
    lon2_rad = math.radians(lon2)
    
    # 方位角公式计算（0°=正北，90°=正东，180°=正南，270°=正西）
    delta_lon = lon2_rad - lon1_rad
    y = math.sin(delta_lon) * math.cos(lat2_rad)
    x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon)
    bearing_rad = math.atan2(y, x)
    
    # 转换为角度并归一化到[0°, 360°]
    bearing_deg = math.degrees(bearing_rad)
    bearing_deg = math.fmod(bearing_deg + 360.0, 360.0)
    # 转换到[-180°, 180°]，与IMU航向角格式统一
    bearing_deg = bearing_deg - 360.0 if bearing_deg > 180.0 else bearing_deg
    return bearing_deg
# 测试代码
if __name__ == "__main__":
    # 示例坐标点
    result = calculate_bearing(30.32166662, 120.07131021,30.32172686,120.07130389)
    print(f"计算结果: {result}")
# 2,120.07130059,30.32166586,84.77
# 3,120.07131021,30.32166662,354.82