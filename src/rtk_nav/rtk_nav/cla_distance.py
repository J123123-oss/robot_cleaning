import math
from typing import Tuple
def latlon_to_utm(lat: float, lon: float) -> Tuple[float, float]:
    """
    经纬度转UTM平面坐标（米）
    :param lat: 纬度
    :param lon: 经度
    :return: (utm_x, utm_y) 平面坐标，单位米
    """
    # WGS84椭球参数
    a = 6378137.0  # 长半轴
    f = 1 / 298.257223563  # 扁率
    e_sq = 2 * f - f ** 2  # 第一偏心率平方

    # 计算UTM投影带号（6度带）
    zone = int((lon + 180) / 6) + 1
    # 中央子午线经度
    lon0 = (zone - 1) * 6 - 180 + 3

    # 转换为弧度
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    lon0_rad = math.radians(lon0)

    # 子午线收敛角计算
    n = a / math.sqrt(1 - e_sq * math.sin(lat_rad) ** 2)
    t = math.tan(lat_rad) ** 2
    c = e_sq * math.cos(lat_rad) ** 2 / (1 - e_sq)
    a1 = math.cos(lat_rad) * (lon_rad - lon0_rad)

    # 计算UTM x坐标（东向）
    x = n * (a1 + (1 - t + c) * a1 ** 3 / 6 + (5 - 18 * t + t ** 2 + 72 * c - 58 * e_sq) * a1 ** 5 / 120)
    # 计算UTM y坐标（北向）
    m = a * ((1 - e_sq / 4 - 3 * e_sq ** 2 / 64 - 5 * e_sq ** 3 / 256) * lat_rad -
            (3 * e_sq / 8 + 3 * e_sq ** 2 / 32 + 45 * e_sq ** 3 / 1024) * math.sin(2 * lat_rad) +
            (15 * e_sq ** 2 / 256 + 45 * e_sq ** 3 / 1024) * math.sin(4 * lat_rad) -
            (35 * e_sq ** 3 / 3072) * math.sin(6 * lat_rad))
    y = m + n * math.tan(lat_rad) * (a1 ** 2 / 2 + (5 - t + 9 * c + 4 * c ** 2) * a1 ** 4 / 24 +
                                    (61 - 58 * t + t ** 2 + 600 * c - 330 * e_sq) * a1 ** 6 / 720)

    # 东向偏移500km，避免负数
    x += 500000.0
    # 南半球偏移10000km（本方案默认北半球，若需支持南半球可添加判断）
    if lat < 0:
        y += 10000000.0

    return x, y
def calc_distance_to_waypoint(lon1: float, lat1: float, lon2: float, lat2: float) -> float:

    

    # 转换为UTM平面坐标
    x1, y1 = latlon_to_utm(lat1, lon1)
    x2, y2 = latlon_to_utm(lat2, lon2)
    raw_distance = math.hypot(x2 - x1, y2 - y1)

    # 计算平面直线距离（米）
    distance = math.hypot(x2 - x1, y2 - y1)
    return distance
if __name__ == "__main__":
    # 示例测试
    # 120.071336, 纬度=30.321683
    lon1= 120.071336  #120.07133471383334  #120.0713299929999
    lat1= 30.321683  #30.321682564333333  #30.321686846833337
    lon2=120.071337 #120.07133778
    lat2=30.321686 #30.32168392
    distance = calc_distance_to_waypoint(lon1, lat1, lon2, lat2)
    print(f"距离目标航点的距离: {distance:.2f} 米")