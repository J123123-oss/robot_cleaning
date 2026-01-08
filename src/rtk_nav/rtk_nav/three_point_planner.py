#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
import matplotlib
import matplotlib.pyplot as plt
from pyproj import Proj, transform
import utm
import rclpy
from rclpy.node import Node
import datetime
from matplotlib.patches import FancyArrowPatch
import os
import numpy as np

# 设置中文字体
# matplotlib.rcParams["font.family"] = ["WenQuanYi Micro Hei", "Heiti TC", "SimHei"]
matplotlib.rcParams['axes.unicode_minus'] = False

def degrees_to_radians(degrees):
    return degrees * math.pi / 180.0

def radians_to_degrees(radians):
    return radians * 180.0 / math.pi

def get_utm_coords(lat, lon):
    """将经纬度转换为UTM坐标（东向、北向、区号、字母）"""
    return utm.from_latlon(lat, lon)

def get_latlon_from_utm(easting, northing, zone_number, zone_letter):
    """将UTM坐标转换为经纬度"""
    return utm.to_latlon(easting, northing, zone_number, zone_letter)

def calculate_heading_angles(path_latlon):
    """
    计算轨迹中每个点的航向角（单位：度，0°为北，顺时针递增）
    
    参数:
        path_latlon: 轨迹点列表，格式为[(lon1, lat1), (lon2, lat2), ...]
    
    返回:
        headings: 航向角列表，长度与path_latlon相同
    """
    headings = []
    if len(path_latlon) <= 1:
        # 若轨迹点不足2个，航向角默认为0
        return [0.0] * len(path_latlon)
    
    for i in range(len(path_latlon)):
        if i == len(path_latlon) - 1:
            # 最后一个点沿用前一个点的航向角
            headings.append(headings[-1])
        else:
            # 提取当前点和下一个点的经纬度（弧度）
            lon1, lat1 = path_latlon[i]
            lon2, lat2 = path_latlon[i+1]
            
            lat1_rad = math.radians(lat1)
            lat2_rad = math.radians(lat2)
            delta_lon_rad = math.radians(lon2 - lon1)
            
            # 计算方位角（基于球面三角公式）
            y = math.sin(delta_lon_rad) * math.cos(lat2_rad)
            x = math.cos(lat1_rad) * math.sin(lat2_rad) - \
                math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon_rad)
            heading_rad = math.atan2(y, x)  # 弧度，范围(-π, π)
            
            # 转换为度并归一化到0°~360°
            heading_deg = math.degrees(heading_rad)
            heading_deg = (heading_deg + 360) % 360  # 确保为正角度
            headings.append(heading_deg)
    
    return headings

def rotate_point(e, n, e0, n0, rotation_rad):
    """
    对UTM坐标点进行旋转（绕基准点(e0, n0)旋转）
    参数:
        e, n: 待旋转点的东向、北向坐标
        e0, n0: 旋转中心（基准点）
        rotation_rad: 旋转角度（弧度，顺时针为正）
    返回:
        e_rot, n_rot: 旋转后的东向、北向坐标
    """
    # 平移到原点（以基准点为中心）
    e_trans = e - e0
    n_trans = n - n0
    
    # 旋转矩阵（顺时针旋转，适配地理坐标系习惯）
    e_rot_trans = e_trans * math.cos(rotation_rad) - n_trans * math.sin(rotation_rad)
    n_rot_trans = e_trans * math.sin(rotation_rad) + n_trans * math.cos(rotation_rad)
    
    # 平移回原坐标系
    e_rot = e_rot_trans + e0
    n_rot = n_rot_trans + n0
    
    return e_rot, n_rot

def calculate_region_from_3points(point_a, point_b, point_c):
    """
    从3个经纬度点计算区域参数
    参数：
        point_a: (lon, lat) - 区域起始角点（对应start_corner）
        point_b: (lon, lat) - 与A点形成第一条边的点
        point_c: (lon, lat) - 确定第二条边的点
    返回：
        base_point: (lon, lat) - 基准点（原逻辑的top_left）
        width: 区域宽度（m）
        height: 区域高度（m）
        rotation_deg: 旋转角度（度）
        start_corner_utm: (e, n) - 起始角点的UTM坐标
    """
    # 转换3个点到UTM坐标
    a_lon, a_lat = point_a
    b_lon, b_lat = point_b
    c_lon, c_lat = point_c
    
    a_e, a_n, zone_num, zone_letter = get_utm_coords(a_lat, a_lon)
    b_e, b_n, _, _ = get_utm_coords(b_lat, b_lon)
    c_e, c_n, _, _ = get_utm_coords(c_lat, c_lon)
    
    # 计算AB向量（第一条边）
    ab_e = b_e - a_e
    ab_n = b_n - a_n
    ab_length = math.hypot(ab_e, ab_n)  # AB边长度
    
    # 计算AC向量在AB垂直方向的分量（第二条边）
    # 计算AB的单位法向量
    ab_unit_e = ab_e / ab_length
    ab_unit_n = ab_n / ab_length
    # 垂直方向向量（左转90度）
    perp_e = -ab_unit_n
    perp_n = ab_unit_e
    
    # AC向量
    ac_e = c_e - a_e
    ac_n = c_n - a_n
    # 计算AC在垂直方向的投影长度（第二条边长度）
    ac_perp_length = ac_e * perp_e + ac_n * perp_n
    
    # 确定宽度和高度（取较长的为height，较短的为width）
    if ab_length >= abs(ac_perp_length):
        height = ab_length
        width = abs(ac_perp_length)
    else:
        height = abs(ac_perp_length)
        width = ab_length
    
    # 计算旋转角度（AB向量与正北方向的夹角）
    # 正北方向向量是(0,1)
    angle_rad = math.atan2(ab_e, ab_n)  # 与正北的夹角（弧度）
    rotation_deg = radians_to_degrees(angle_rad)
    
    # 计算原逻辑的基准点（top_left）：从A点反向计算
    # 先确定top_left在UTM中的坐标
    if ac_perp_length > 0:
        # C点在AB左侧，top_left = A点 - 垂直方向向量 * width
        top_left_e = a_e - perp_e * width
        top_left_n = a_n - perp_n * width
    else:
        # C点在AB右侧，top_left = A点 + 垂直方向向量 * width
        top_left_e = a_e + perp_e * width
        top_left_n = a_n + perp_n * width
    
    # 转换top_left回经纬度作为base_point
    top_left_lat, top_left_lon = get_latlon_from_utm(top_left_e, top_left_n, zone_num, zone_letter)
    base_point = (top_left_lon, top_left_lat)
    
    return base_point, width, height, rotation_deg, (a_e, a_n), (zone_num, zone_letter)

def generate_cleaning_path_with_rotation_3points(point_a, point_b, point_c, start_corner, param):
    """
    基于3个标定点生成带旋转的清扫路径
    参数：
        point_a: (lon, lat) - 起始角点经纬度
        point_b: (lon, lat) - 第二标定点
        point_c: (lon, lat) - 第三标定点
        start_corner: 轨迹起始角点 ['top_left', 'top_right', 'bottom_right', 'bottom_left']
        param: 包含interval, edge_distance_lon, edge_distance_lat的参数字典
    返回：
        path_latlon, path_utm_rot, original_corners_utm, inner_corners_utm, utm_zone
    """
    # 从3个点计算区域参数
    base_point, width, height, rotation_deg, start_utm, utm_zone = calculate_region_from_3points(
        point_a, point_b, point_c
    )
    zone_num, zone_letter = utm_zone
    
    lon0, lat0 = base_point
    interval = param['interval']
    edge_lon = param['edge_distance_lon']
    edge_lat = param['edge_distance_lat']
    rotation_rad = degrees_to_radians(rotation_deg)
    
    # 基准点转UTM（冗余但保持原逻辑）
    e0, n0, _, _ = get_utm_coords(lat0, lon0)
    
    # 1. 生成未旋转的原始矩形四个角点（以基准点为top_left）
    orig_unrot = {
        'top_left': (e0, n0),
        'top_right': (e0 + width, n0),
        'bottom_right': (e0 + width, n0 - height),
        'bottom_left': (e0, n0 - height)
    }
    
    # 2. 旋转所有原始角点
    orig_rot = {}
    for corner_name, (e, n) in orig_unrot.items():
        e_rot, n_rot = rotate_point(e, n, e0, n0, rotation_rad)
        orig_rot[corner_name] = (e_rot, n_rot)
    original_corners_utm = list(orig_rot.values())
    
    # 3. 生成未旋转的内部矩形角点（边界偏移后）
    inner_unrot = {
        'top_left': (e0 + edge_lon, n0 - edge_lat),
        'top_right': (e0 + width - edge_lon, n0 - edge_lat),
        'bottom_right': (e0 + width - edge_lon, n0 - height + edge_lat),
        'bottom_left': (e0 + edge_lon, n0 - height + edge_lat)
    }
    
    # 安全检查：内部区域有效性
    inner_width = (e0 + width - edge_lon) - (e0 + edge_lon)
    inner_height = (n0 - edge_lat) - (n0 - height + edge_lat)
    if inner_width <= 0.1 or inner_height <= 0.1:
        raise ValueError(f"内部区域无效！宽度:{inner_width:.2f}m, 高度:{inner_height:.2f}m")
    
    # 4. 旋转内部矩形角点
    inner_rot = {}
    for corner_name, (e, n) in inner_unrot.items():
        e_rot, n_rot = rotate_point(e, n, e0, n0, rotation_rad)
        inner_rot[corner_name] = (e_rot, n_rot)
    inner_corners_utm = list(inner_rot.values())
    
    # 5. 根据起始角点确定路径生成方向（核心修改）
    # 解析起始角点的方位：left/right（水平方向）、top/bottom（垂直方向）
    hori_dir = 'left' if 'left' in start_corner else 'right'  # 水平起始方向
    vert_dir = 'top' if 'top' in start_corner else 'bottom'    # 垂直起始方向
    
    # 内部矩形边界（未旋转）
    inner_e_min = inner_unrot['top_left'][0]
    inner_e_max = inner_unrot['top_right'][0]
    inner_n_max = inner_unrot['top_left'][1]
    inner_n_min = inner_unrot['bottom_left'][1]
    
    # 生成未旋转的内部路径（根据起始角点调整方向）
    path_utm_unrot = []
    if inner_width >= inner_height:
        # 宽 >= 高：沿垂直方向分条（上下移动），水平方向交替
        # 垂直方向步长和点数
        num_strips = max(1, int(inner_height / interval) + 1)
        # 根据垂直起始方向确定n的遍历顺序
        if vert_dir == 'top':
            n_values = [inner_n_max - (inner_height) * (i / (num_strips - 1) if num_strips > 1 else 0) 
                        for i in range(num_strips)]
        else:  # bottom
            n_values = [inner_n_min + (inner_height) * (i / (num_strips - 1) if num_strips > 1 else 0) 
                        for i in range(num_strips)]
        
        for i, current_n_unrot in enumerate(n_values):
            # 根据水平起始方向和条带索引，确定水平遍历方向
            if (i % 2 == 0 and hori_dir == 'left') or (i % 2 == 1 and hori_dir == 'right'):
                # 偶数条带：左->右；奇数条带：右->左（适配left起始）
                path_utm_unrot.append((inner_e_min, current_n_unrot))
                path_utm_unrot.append((inner_e_max, current_n_unrot))
            else:
                # 偶数条带：右->左；奇数条带：左->右（适配right起始）
                path_utm_unrot.append((inner_e_max, current_n_unrot))
                path_utm_unrot.append((inner_e_min, current_n_unrot))
    else:
        # 宽 < 高：沿水平方向分条（左右移动），垂直方向交替
        # 水平方向步长和点数
        num_strips = max(1, int(inner_width / interval) + 1)
        # 根据水平起始方向确定e的遍历顺序
        if hori_dir == 'left':
            e_values = [inner_e_min + (inner_width) * (i / (num_strips - 1) if num_strips > 1 else 0) 
                        for i in range(num_strips)]
        else:  # right
            e_values = [inner_e_max - (inner_width) * (i / (num_strips - 1) if num_strips > 1 else 0) 
                        for i in range(num_strips)]
        
        for i, current_e_unrot in enumerate(e_values):
            # 根据垂直起始方向和条带索引，确定垂直遍历方向
            if (i % 2 == 0 and vert_dir == 'top') or (i % 2 == 1 and vert_dir == 'bottom'):
                # 偶数条带：上->下；奇数条带：下->上（适配top起始）
                path_utm_unrot.append((current_e_unrot, inner_n_max))
                path_utm_unrot.append((current_e_unrot, inner_n_min))
            else:
                # 偶数条带：下->上；奇数条带：上->下（适配bottom起始）
                path_utm_unrot.append((current_e_unrot, inner_n_min))
                path_utm_unrot.append((current_e_unrot, inner_n_max))
    
    # 6. 旋转路径点并转经纬度
    path_utm_rot = []
    path_latlon = []
    for (e_unrot, n_unrot) in path_utm_unrot:
        e_rot, n_rot = rotate_point(e_unrot, n_unrot, e0, n0, rotation_rad)
        path_utm_rot.append((e_rot, n_rot))
        lat, lon = get_latlon_from_utm(e_rot, n_rot, zone_num, zone_letter)
        path_latlon.append((lon, lat))
    
    # 7. 确保路径起点严格对应选择的起始角点（最终校验）
    target_start_utm = inner_rot[start_corner]
    # 计算路径第一个点与目标起始点的距离，若偏差过大则调整（防止生成逻辑偏差）
    first_point_dist = math.hypot(path_utm_rot[0][0] - target_start_utm[0], 
                                  path_utm_rot[0][1] - target_start_utm[1])
    if first_point_dist > 0.1:  # 偏差超过10cm则交换第一个点和第二个点
        path_utm_rot[0], path_utm_rot[1] = path_utm_rot[1], path_utm_rot[0]
        path_latlon[0], path_latlon[1] = path_latlon[1], path_latlon[0]
    
    return path_latlon, path_utm_rot, original_corners_utm, inner_corners_utm, utm_zone


def add_direction_arrows(ax, path_utm, arrow_interval=5):
    """在路径上添加方向箭头"""
    # 箭头间隔：每隔arrow_interval个点添加一个箭头
    for i in range(0, len(path_utm) - arrow_interval, arrow_interval):
        # 起点
        x1, y1 = path_utm[i]
        # 终点（箭头指向的位置）
        x2, y2 = path_utm[i + arrow_interval]
        
        # 计算箭头长度（根据实际距离调整箭头大小）
        dx = x2 - x1
        dy = y2 - y1
        length = math.sqrt(dx**2 + dy**2)
        
        # 添加箭头
        arrow = FancyArrowPatch(
            (x1, y1), (x2, y2),
            arrowstyle='->',  # 箭头样式
            mutation_scale=15,  # 箭头大小
            color='blue',       # 箭头颜色
            linewidth=1.5,      # 箭杆粗细
            alpha=0.7           # 透明度
        )
        ax.add_patch(arrow)

class CleaningPathPlanner(Node):
    def __init__(self):
        super().__init__('cleaning_path_planner')
        
        # 声明参数（ROS 2参数需要先声明）120.0711247716332,30.320803806689252
        self.declare_parameter('calib_point_a.lon', 120.0711247716332)
        self.declare_parameter('calib_point_a.lat', 30.320803806689252)
        self.declare_parameter('calib_point_b.lon', 120.0712247716332)
        self.declare_parameter('calib_point_b.lat', 30.320903806689252)
        self.declare_parameter('calib_point_c.lon', 120.0710247716332)
        self.declare_parameter('calib_point_c.lat', 30.320853806689252)
        self.declare_parameter('interval', 1.0)
        self.declare_parameter('start_corner', 'top_left')
        self.declare_parameter('edge_distance_lon', 0.5)
        self.declare_parameter('edge_distance_lat', 0.5)
        self.declare_parameter('headless', False)
        
        # 获取参数
        self.param = {
            # 3个标定点
            'calib_point_a': (
                self.get_parameter('calib_point_a.lon').value,
                self.get_parameter('calib_point_a.lat').value
            ),
            'calib_point_b': (
                self.get_parameter('calib_point_b.lon').value,
                self.get_parameter('calib_point_b.lat').value
            ),
            'calib_point_c': (
                self.get_parameter('calib_point_c.lon').value,
                self.get_parameter('calib_point_c.lat').value
            ),
            
            'interval': self.get_parameter('interval').value,
            'start_corner': self.get_parameter('start_corner').value,
            'edge_distance_lon': self.get_parameter('edge_distance_lon').value,
            'edge_distance_lat': self.get_parameter('edge_distance_lat').value,
            'headless': self.get_parameter('headless').value
        }
        
        # 校验 start_corner 的合法性
        valid_corners = ['top_left', 'top_right', 'bottom_right', 'bottom_left']
        if self.param['start_corner'] not in valid_corners:
            self.get_logger().error(
                f"无效的start_corner: {self.param['start_corner']}，必须是 {valid_corners}"
            )
            return
        
        # 执行路径规划
        self.plan_path()
    
    def plan_path(self):
        # 获取当前时间戳（用于文件名）
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        
        try:
            # 生成带旋转角度的清扫路径
            path_latlon, path_utm, original_corners_utm, inner_corners_utm, utm_zone = generate_cleaning_path_with_rotation_3points(
                self.param['calib_point_a'], 
                self.param['calib_point_b'], 
                self.param['calib_point_c'], 
                self.param['start_corner'],
                self.param
            )
            zone_num, zone_letter = utm_zone
            
            # 打印参数信息
            self.get_logger().info("参数配置:")
            self.get_logger().info(f"  路径间隔: {self.param['interval']}米")
            self.get_logger().info(f"  起始点: {self.param['start_corner']}")
            self.get_logger().info(f"  经度方向边缘距离: {self.param['edge_distance_lon']}米")
            self.get_logger().info(f"  纬度方向边缘距离: {self.param['edge_distance_lat']}米")
            self.get_logger().info(f"\n生成的路径点数量: {len(path_latlon)}")
            
            # 确保保存目录存在
            # save_dir = os.path.expanduser("/home/forlinx/robot_cleaning/src/rtk_nav/rtk_nav/cleaning_path/")
            save_dir = os.path.expanduser("/home/ubuntu/robot_cleaning/src/rtk_nav/rtk_nav/cleaning_path/")
            os.makedirs(save_dir, exist_ok=True)
            
            # 计算航向角并保存路径点
            points_filename = os.path.join(save_dir, f"cleaning_path_{timestamp}.txt")
            headings = calculate_heading_angles(path_latlon)
            
            with open(points_filename, "w", encoding="utf-8") as f:
                f.write("#序号,经度,纬度,航向角(度)\n")
                for i in range(len(path_latlon)):
                    lon, lat = path_latlon[i]
                    heading = headings[i]
                    f.write(f"{i+1},{lon:.8f},{lat:.8f},{heading:.2f}\n")
            self.get_logger().info(f"所有路径点及航向角已保存到 {points_filename} 文件")
            
            # 绘制清扫路径
            fig, ax = plt.subplots(figsize=(10, 8))
            
            # 绘制原始矩形边界
            orig_e = [corner[0] for corner in original_corners_utm] + [original_corners_utm[0][0]]
            orig_n = [corner[1] for corner in original_corners_utm] + [original_corners_utm[0][1]]
            ax.plot(orig_e, orig_n, 'b-', label='origin')
            
            # 绘制内部矩形边界
            inner_e = [corner[0] for corner in inner_corners_utm] + [inner_corners_utm[0][0]]
            inner_n = [corner[1] for corner in inner_corners_utm] + [inner_corners_utm[0][1]]
            ax.plot(inner_e, inner_n, 'g--', label='inner')
            
            # 绘制清扫路径
            path_e = [p[0] for p in path_utm]
            path_n = [p[1] for p in path_utm]
            ax.plot(path_e, path_n, 'r-', linewidth=1, label='cleaning path')
            
            # 添加方向箭头
            add_direction_arrows(ax, path_utm, arrow_interval=1)
            
            # 标记起点和终点
            ax.scatter(path_e[0], path_n[0], c='green', s=100, marker='o', label='start')
            ax.scatter(path_e[-1], path_n[-1], c='purple', s=100, marker='x', label='end')
            
            ax.set_xlabel(f'UTM east m - zone {zone_num}{zone_letter}')
            ax.set_ylabel(f'UTM north m - zone {zone_num}{zone_letter}')
            ax.set_title(
                f'robot cleaning path planner\n'
                f'path interval: {self.param["interval"]}m'
            )
            ax.legend()
            ax.grid(True)
            ax.axis('equal')
            plt.tight_layout()
            
            # 保存带时间戳的图片
            img_filename = os.path.join(save_dir, f'cleaning_path_{timestamp}.png')
            plt.savefig(img_filename, dpi=300)
            self.get_logger().info(f"路径图已保存到 {img_filename}")
            
            # 若不是在headless模式，显示图像
            if not self.param['headless']:
                plt.show()
            
        except ValueError as e:
            self.get_logger().error(f"错误: {e}")
        except Exception as e:
            self.get_logger().error(f"发生异常: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = CleaningPathPlanner()
    
    # 保持节点运行（ROS 2不需要spin()来维持简单节点的运行，除非有回调）
    # 这里使用try-except来捕获退出信号
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()