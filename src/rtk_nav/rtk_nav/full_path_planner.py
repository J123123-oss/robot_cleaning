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
import yaml
import numpy as np

# 设置中文字体 + 解决负号显示问题
matplotlib.rcParams['axes.unicode_minus'] = False
# try:
#     matplotlib.rcParams["font.family"] = ["WenQuanYi Micro Hei", "Heiti TC", "SimHei", "Arial Unicode MS"]
# except:
#     pass  # 无中文字体则使用默认，不影响运行

# ---------------------- 原有工具函数 完全不变 ----------------------
def degrees_to_radians(degrees):
    return degrees * math.pi / 180.0

def radians_to_degrees(radians):
    return radians * 180.0 / math.pi

def get_utm_coords(lat, lon):
    return utm.from_latlon(lat, lon)

def get_latlon_from_utm(easting, northing, zone_number, zone_letter):
    return utm.to_latlon(easting, northing, zone_number, zone_letter)

def calculate_heading_angles(path_latlon):
    headings = []
    if len(path_latlon) <= 1:
        return [0.0] * len(path_latlon)
    
    for i in range(len(path_latlon)):
        if i == len(path_latlon) - 1:
            headings.append(headings[-1])
        else:
            lon1, lat1 = path_latlon[i]
            lon2, lat2 = path_latlon[i+1]
            
            lat1_rad = math.radians(lat1)
            lat2_rad = math.radians(lat2)
            delta_lon_rad = math.radians(lon2 - lon1)
            
            y = math.sin(delta_lon_rad) * math.cos(lat2_rad)
            x = math.cos(lat1_rad) * math.sin(lat2_rad) - \
                math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon_rad)
            heading_rad = math.atan2(y, x)
            heading_deg = math.degrees(heading_rad)
            heading_deg = (heading_deg + 360) % 360
            headings.append(heading_deg)
    
    return headings

def rotate_point(e, n, e0, n0, rotation_rad):
    e_trans = e - e0
    n_trans = n - n0
    
    # e_rot_trans = e_trans * math.cos(rotation_rad) - n_trans * math.sin(rotation_rad)
    # n_rot_trans = e_trans * math.sin(rotation_rad) + n_trans * math.cos(rotation_rad)
    # 逆时针旋转（如需逆时针旋转，使用以下公式）
    e_rot_trans = e_trans * math.cos(rotation_rad) + n_trans * math.sin(rotation_rad)  # θ为逆时针旋转角
    n_rot_trans = -e_trans * math.sin(rotation_rad) + n_trans * math.cos(rotation_rad)
    
    e_rot = e_rot_trans + e0
    n_rot = n_rot_trans + n0
    
    return e_rot, n_rot

def calculate_region_from_3points(point_a, point_b, point_c):
    a_lon, a_lat = point_a
    b_lon, b_lat = point_b
    c_lon, c_lat = point_c
    
    a_e, a_n, zone_num, zone_letter = get_utm_coords(a_lat, a_lon)
    b_e, b_n, _, _ = get_utm_coords(b_lat, b_lon)
    c_e, c_n, _, _ = get_utm_coords(c_lat, c_lon)
    
    ab_e = b_e - a_e
    ab_n = b_n - a_n
    ab_length = math.hypot(ab_e, ab_n)
    
    ab_unit_e = ab_e / ab_length
    ab_unit_n = ab_n / ab_length
    perp_e = -ab_unit_n
    perp_n = ab_unit_e
    
    ac_e = c_e - a_e
    ac_n = c_n - a_n
    ac_perp_length = ac_e * perp_e + ac_n * perp_n
    
    # ========== 修复点1：宽高赋值逻辑，还原业务语义 ==========
    height = ab_length  # AB为主边，固定为高度
    width_origin = ac_perp_length  # 带符号的真实宽度，用于计算基准点
    
    # ========== 修复点2：旋转角度计算，原始逻辑正确，保留 ==========
    angle_rad = math.atan2(ab_e, ab_n)  # 与正北的夹角（弧度）
    angle_rad = angle_rad - math.pi    # change angle + PI

    rotation_deg = radians_to_degrees(angle_rad)
    
    # ========== 修复点3：基准点UTM坐标 核心修正公式【彻底解决偏移】 ==========
    top_left_e = a_e + perp_e * width_origin
    top_left_n = a_n + perp_n * width_origin

    
    # 转换top_left回经纬度作为base_point
    top_left_lat, top_left_lon = get_latlon_from_utm(top_left_e, top_left_n, zone_num, zone_letter)
    base_point = (top_left_lon, top_left_lat)
    
    # 对外返回的宽高取绝对值（物理尺寸为正数）
    width = abs(width_origin)
    print("计算区域参数：base_point={}, width={}, height={}, rotation_deg={}".format(base_point, width, height, rotation_deg))
    
    return base_point, width_origin, height, rotation_deg, (a_e, a_n), (zone_num, zone_letter)

def generate_cleaning_path_with_rotation_3points(point_a, point_b, point_c, start_corner, param):
    """
    最终修正版：
    1. 严格以传入的start_corner作为唯一基准，控制路径生成方向+起始点，支持随时修改生效
    2. 保留正负宽度统一适配逻辑、A点为旋转中心、边界固定的全部原有逻辑
    3. 修复轨迹斜线跳转BUG，改为90度垂直转向衔接，无斜线跨区域
    4. 路径起始点完全由start_corner决定，不再强制关联A点
    """
    # 1. 从3个点计算区域参数（保留原逻辑）
    base_point, width, height, rotation_deg, start_utm, utm_zone = calculate_region_from_3points(
        point_a, point_b, point_c
    )
    zone_num, zone_letter = utm_zone
    a_e, a_n = start_utm  # A点UTM坐标（固定为旋转中心，保留）
    
    lon0, lat0 = base_point
    interval = param['interval']
    edge_lon = param['edge_distance_lon']
    edge_lat = param['edge_distance_lat']
    rotation_rad = degrees_to_radians(rotation_deg)
    
    # 核心保留：统一旋转中心为A点（无论宽度正负）
    e0, n0 = a_e, a_n
    
    # 2. 生成未旋转的原始矩形四个角点（适配宽度正负，保留）
    orig_unrot = {}
    width_sign = 1 if width >= 0 else -1
    width_abs = abs(width)
    
    orig_unrot = {
        'top_left': (e0, n0),
        'top_right': (e0 + width_sign * width_abs, n0),
        'bottom_right': (e0 + width_sign * width_abs, n0 - height),
        'bottom_left': (e0, n0 - height)
    }
    
    # 3. 旋转原始矩形角点（以A点为中心，保留）
    orig_rot = {}
    for corner_name, (e, n) in orig_unrot.items():
        e_rot, n_rot = rotate_point(e, n, e0, n0, rotation_rad)
        orig_rot[corner_name] = (e_rot, n_rot)
    original_corners_utm = list(orig_rot.values())
    
    # 4. 生成未旋转的内部矩形角点（统一逻辑，适配宽度正负，保留）
    inner_unrot = {}
    inner_top_right_e = e0 + width_sign * (width_abs - edge_lon)
    inner_top_left_e = e0 + width_sign * edge_lon

    inner_unrot = {
        'top_left': (inner_top_left_e, n0 - edge_lat),
        'top_right': (inner_top_right_e, n0 - edge_lat),
        'bottom_right': (inner_top_right_e, n0 - height + edge_lat),
        'bottom_left': (inner_top_left_e, n0 - height + edge_lat)
    }
    
    # 5. 安全检查：内部区域有效性（统一计算逻辑，保留）
    inner_e_list = [inner_unrot[corner][0] for corner in inner_unrot]
    inner_n_list = [inner_unrot[corner][1] for corner in inner_unrot]
    inner_e_min = min(inner_e_list)
    inner_e_max = max(inner_e_list)
    inner_n_min = min(inner_n_list)
    inner_n_max = max(inner_n_list)
    inner_width = inner_e_max - inner_e_min
    inner_height = inner_n_max - inner_n_min
    
    if inner_width <= 0.3 or inner_height <= 0.3:
        # raise ValueError(f"内部区域无效！宽度:{inner_width:.2f}m, 高度:{inner_height:.2f}m")
        # 获取A点和B点的UTM坐标（这里需要根据实际的经纬度转UTM逻辑调整）
        # 假设point_a和point_b是经纬度坐标，先转换为UTM
                # 正确转换A/B点到UTM坐标
        a_lon, a_lat = point_a
        b_lon, b_lat = point_b
        
        # 经纬度转UTM（使用正确的函数）
        a_utm_e, a_utm_n, _, _ = get_utm_coords(a_lat, a_lon)
        b_utm_e, b_utm_n, _, _ = get_utm_coords(b_lat, b_lon)
        
        # 生成AB直线路径（UTM坐标）
        path_utm_rot = [(a_utm_e, a_utm_n), (b_utm_e, b_utm_n)]
        # 经纬度路径（保持原格式：(lon, lat)）
        path_latlon = [point_a, point_b]
        
        # 关键修复：给inner_corners_utm赋默认值，避免绘图时索引越界
        # 使用原始矩形角点作为兜底，保证绘图代码能正常运行
        inner_corners_utm = original_corners_utm.copy()
        
        return path_latlon, path_utm_rot, original_corners_utm, inner_corners_utm, utm_zone
    
    # 6. 旋转内部矩形角点（以A点为中心，保留）
    inner_rot = {}
    for corner_name, (e, n) in inner_unrot.items():
        e_rot, n_rot = rotate_point(e, n, e0, n0, rotation_rad)
        inner_rot[corner_name] = (e_rot, n_rot)
    inner_corners_utm = list(inner_rot.values())

    # 路径生成方向基于传入的start_corner
    hori_dir = 'left' if 'left' in start_corner else 'right'
    vert_dir = 'top' if 'top' in start_corner else 'bottom'
    
    # 8. 生成未旋转的内部路径【核心修复3：轨迹斜线跳转BUG + 保留原有逻辑】
    swap_wh_select = param['swap_wh_select']
    default_direction= inner_width >= inner_height if not swap_wh_select else inner_width <= inner_height
    path_utm_unrot = []
    if default_direction:
        n_values = []
        current_n = inner_n_min
        while current_n <= inner_n_max + 1e-6:
            n_values.append(current_n)
            current_n += interval
        if abs(n_values[-1] - inner_n_max) > 0.6:
            n_values.append(inner_n_max)
        num_strips = len(n_values)

        if vert_dir == 'bottom':
            n_values = n_values[::-1]

        for i, current_n_unrot in enumerate(n_values):
            left_first = (i % 2 == 0 and hori_dir == 'left') or (i % 2 == 1 and hori_dir == 'right')

            if left_first:
                path_utm_unrot.append((inner_e_min, current_n_unrot))
                path_utm_unrot.append((inner_e_max, current_n_unrot))
            else:
                path_utm_unrot.append((inner_e_max, current_n_unrot))
                path_utm_unrot.append((inner_e_min, current_n_unrot))
                
    else:
        e_values = []
        current_e = inner_e_min
        while current_e <= inner_e_max + 1e-6:
            e_values.append(current_e)
            current_e += interval
        if abs(e_values[-1] - inner_e_max) > 0.6:
            e_values.append(inner_e_max)
        num_strips = len(e_values)

        if hori_dir == 'right':
            e_values = e_values[::-1]

        for i, current_e_unrot in enumerate(e_values):
            top_first = (i % 2 == 0 and vert_dir == 'top') or (i % 2 == 1 and vert_dir == 'bottom')

            if top_first:
                path_utm_unrot.append((current_e_unrot, inner_n_max))
                path_utm_unrot.append((current_e_unrot, inner_n_min))
            else:
                path_utm_unrot.append((current_e_unrot, inner_n_min))
                path_utm_unrot.append((current_e_unrot, inner_n_max))
        # 日志优化：打印实际条带数和间隔
    if default_direction:
        print(f"垂直分条：实际条带数={num_strips}，设置interval={interval}m，实际间隔：{[round(n_values[i+1]-n_values[i],2) for i in range(len(n_values)-1)]}m")
    else:
        print(f"水平分条：实际条带数={num_strips}，设置interval={interval}m，实际间隔：{[round(e_values[i+1]-e_values[i],2) for i in range(len(e_values)-1)]}m")

    # 9. 旋转路径点（以A点为中心，保留原有逻辑）
    path_utm_rot = []
    path_latlon = []
    for (e_unrot, n_unrot) in path_utm_unrot:
        e_rot, n_rot = rotate_point(e_unrot, n_unrot, e0, n0, rotation_rad)
        path_utm_rot.append((e_rot, n_rot))
        lat, lon = get_latlon_from_utm(e_rot, n_rot, zone_num, zone_letter)
        path_latlon.append((lon, lat))
    
    # 路径方向日志
    print(f"路径生成方向：水平={hori_dir}，垂直={vert_dir}")
    print(f"宽度处理：原始宽度={width:.2f}m → 延伸方向={'东向' if width_sign ==1 else '西向'}，内部宽度={inner_width:.2f}m")

    # ========================= 【修复1：根据实际坐标重新确定角点名称】 =========================
    # 旋转后，原来的top_left/top_right等名称可能不再对应实际位置
    # 需要根据实际UTM坐标找到真正的四个角
    
    # 使用旋转后的实际坐标边界
    rot_e_list = [inner_rot[c][0] for c in inner_rot]
    rot_n_list = [inner_rot[c][1] for c in inner_rot]
    rot_e_min = min(rot_e_list)
    rot_e_max = max(rot_e_list)
    rot_n_min = min(rot_n_list)
    rot_n_max = max(rot_n_list)
    
    # 根据旋转后的实际坐标确定真正的角点位置
    actual_corners = {}
    for name, (e, n) in inner_rot.items():
        if n >= (rot_n_max + rot_n_min) / 2:
            actual_corners['top'] = actual_corners.get('top', []) + [(name, e, n)]
        else:
            actual_corners['bottom'] = actual_corners.get('bottom', []) + [(name, e, n)]
        if e <= (rot_e_max + rot_e_min) / 2:
            actual_corners['left'] = actual_corners.get('left', []) + [(name, e, n)]
        else:
            actual_corners['right'] = actual_corners.get('right', []) + [(name, e, n)]
    
    # 确定四个实际角点
    top_left_name = min(actual_corners['left'], key=lambda x: x[2])[0]
    top_right_name = min(actual_corners['right'], key=lambda x: x[2])[0]
    bottom_left_name = max(actual_corners['left'], key=lambda x: x[2])[0]
    bottom_right_name = max(actual_corners['right'], key=lambda x: x[2])[0]
    
    actual_corner_map = {
        'top_left': inner_rot[top_left_name],
        'top_right': inner_rot[top_right_name],
        'bottom_left': inner_rot[bottom_left_name],
        'bottom_right': inner_rot[bottom_right_name]
    }
    
    print(f"实际角点坐标: top_left={actual_corner_map['top_left']}, top_right={actual_corner_map['top_right']}")
    print(f"实际角点坐标: bottom_left={actual_corner_map['bottom_left']}, bottom_right={actual_corner_map['bottom_right']}")
    
    # 使用实际角点坐标来确定起始点
    target_start_utm = actual_corner_map[start_corner]
    print(f"配置起始角点: {start_corner}，目标起始点UTM坐标: {target_start_utm}")

    # ========================= 【修复2：使用实际角点坐标计算结束点】 =========================
    end_corner_mode = param.get('end_corner_mode', 'diagonal')
    reverse_end_point = param.get('reverse_end_point', False)  # 新增：控制是否执行反转逻辑
    print(f"end_corner_mode: {end_corner_mode}")
    print(f"reverse_end_point: {reverse_end_point}")
    tl = actual_corner_map['top_left']
    tr = actual_corner_map['top_right']
    bl = actual_corner_map['bottom_left']
    br = actual_corner_map['bottom_right']

    start_point = path_utm_rot[0]
    current_end = path_utm_rot[-1]

    corner_list = [tl, tr, bl, br]
    start = min(corner_list, key=lambda c: math.hypot(start_point[0]-c[0], start_point[1]-c[1]))

    # 对角/对边映射（基于实际角点位置）
    corner_name_map = {
        tuple(tl): 'top_left',
        tuple(tr): 'top_right',
        tuple(bl): 'bottom_left',
        tuple(br): 'bottom_right'
    }
    start_name = corner_name_map[tuple(start)]
    
    if end_corner_mode == 'diagonal':
        diagonal_map = {
            'top_left': actual_corner_map['bottom_right'],
            'top_right': actual_corner_map['bottom_left'],
            'bottom_left': actual_corner_map['top_right'],
            'bottom_right': actual_corner_map['top_left']
        }
        target = diagonal_map[start_name]
    else:
        opposite_map = {
            # 'top_left': actual_corner_map['top_right'],
            # 'top_right': actual_corner_map['top_left'],# swap_wh_select: False  ---bottom_right
            # 'bottom_left': actual_corner_map['bottom_right'], # swap_wh_select: False ---top_left
            # 'bottom_right': actual_corner_map['bottom_left']
        
            'top_left': actual_corner_map['top_right'],
            'top_right': actual_corner_map['bottom_right'] if not swap_wh_select else actual_corner_map['top_left'],
            'bottom_left': actual_corner_map['top_left'] if not swap_wh_select else actual_corner_map['bottom_right'],
            'bottom_right': actual_corner_map['bottom_left']
        }
        target = opposite_map[start_name]
    
    print(f"起点名称: {start_name}, 目标结束点: {target}")
    
    # 通过reverse_end_point配置来决定是否执行反转逻辑
    print(f"当前结束点: {current_end}")
    print(f"目标结束点: {target}")
    print(f"距离: {math.hypot(current_end[0]-target[0], current_end[1]-target[1])}")
    print(f"路径点数量: {len(path_utm_rot)}")
    
    if reverse_end_point:
        print("执行反转180度返回逻辑")
        if len(path_utm_rot) >= 2:
            p1 = path_utm_rot[-2]
            p2 = path_utm_rot[-1]
            dx = p2[0] - p1[0]
            dy = p2[1] - p1[1]
            back_x = p2[0] - dx
            back_y = p2[1] - dy
            print(f"最后两个点: {p1}, {p2}")
            print(f"计算的返回点: {back_x}, {back_y}")

            path_utm_rot.append( (back_x, back_y) )
            lat, lon = get_latlon_from_utm(back_x, back_y, zone_num, zone_letter)
            path_latlon.append( (lon, lat) )
            print(f"添加返回点: ({lon}, {lat})")
        else:
            print("路径点数量不足2，无法执行反转逻辑")
    
    return path_latlon, path_utm_rot, original_corners_utm, inner_corners_utm, utm_zone

def add_direction_arrows(ax, path_utm, arrow_interval=5):
    for i in range(0, len(path_utm) - arrow_interval, arrow_interval):
        x1, y1 = path_utm[i]
        x2, y2 = path_utm[i + arrow_interval]
        dx = x2 - x1
        dy = y2 - y1
        length = math.sqrt(dx**2 + dy**2)
        arrow = FancyArrowPatch(
            (x1, y1), (x2, y2),
            arrowstyle='->',
            mutation_scale=15,
            color='blue',
            linewidth=1.5,
            alpha=0.7
        )
        ax.add_patch(arrow)

# ---------------------- 多区域路径规划节点（核心修复） ----------------------
class MultiAreaCleaningPathPlanner(Node):
    def __init__(self):
        super().__init__('full_path')
        self.seq_num = 0
        
        # 声明配置文件路径参数和 headless 参数
        self.declare_parameter('config_file', '/home/ubuntu/robot_cleaning/src/rtk_nav/rtk_nav/config/areas_south4-8.yaml')
        self.declare_parameter('headless', False)
        
        # 尝试从 YAML 配置文件加载
        config_file = self.get_parameter('config_file').value
        self.get_logger().info(f"尝试加载配置文件: {config_file}")
        
        if config_file and isinstance(config_file, str) and os.path.exists(config_file):
            self.get_logger().info(f"从配置文件加载区域参数: {config_file}")
            self.all_areas = self._load_areas_from_yaml(config_file)
            if self.all_areas:
                self.plan_multi_area_path()
                return
        
        # 如果YAML加载失败或文件不存在，使用launch参数方式
        self.get_logger().info("YAML配置不可用，使用 launch 参数方式加载区域")
        self._init_from_launch_params()
    
    def _init_from_launch_params(self):
        # 声明多区域参数（支持动态配置）
        self.declare_parameter('area_count', 2)
        self.declare_parameter('default.interval', 1.0)
        self.declare_parameter('default.start_corner', 'top_left')
        self.declare_parameter('default.swap_wh_select', False)
        self.declare_parameter('default.edge_distance_lon', 0.5)
        self.declare_parameter('default.edge_distance_lat', 0.5)
        self.declare_parameter('default.end_corner_mode', 'diagonal') # diagonal / opposite
        self.declare_parameter('default.reverse_end_point', False)  # 新增：控制是否执行反转逻辑
        
        self.area_count = self.get_parameter('area_count').value
        for i in range(self.area_count):
            self.declare_parameter(f'area_{i}.calib_point_a.lon', 0.0)
            self.declare_parameter(f'area_{i}.calib_point_a.lat', 0.0)
            self.declare_parameter(f'area_{i}.calib_point_b.lon', 0.0)
            self.declare_parameter(f'area_{i}.calib_point_b.lat', 0.0)
            self.declare_parameter(f'area_{i}.calib_point_c.lon', 0.0)
            self.declare_parameter(f'area_{i}.calib_point_c.lat', 0.0)
            self.declare_parameter(f'area_{i}.interval', None)
            self.declare_parameter(f'area_{i}.start_corner', None)
            self.declare_parameter(f'area_{i}.swap_wh_select', None)
            self.declare_parameter(f'area_{i}.edge_distance_lon', None)
            self.declare_parameter(f'area_{i}.edge_distance_lat', None)
            self.declare_parameter(f'area_{i}.end_corner_mode', None)
            self.declare_parameter(f'area_{i}.reverse_end_point', None)
            
        
        self.all_areas = self._parse_area_parameters()
        if not self.all_areas:
            self.get_logger().error("区域参数解析失败，退出程序")
            return
        
        self.plan_multi_area_path()
    
    def _parse_area_parameters(self):
        all_areas = []
        valid_corners = ['top_left', 'top_right', 'bottom_right', 'bottom_left']
        
        default_params = {
            'interval': self.get_parameter('default.interval').value,
            'start_corner': self.get_parameter('default.start_corner').value,
            'swap_wh_select': self.get_parameter('default.swap_wh_select').value,
            'edge_distance_lon': self.get_parameter('default.edge_distance_lon').value,
            'edge_distance_lat': self.get_parameter('default.edge_distance_lat').value,
            'end_corner_mode': self.get_parameter('default.end_corner_mode').value,
            'reverse_end_point': self.get_parameter('default.reverse_end_point').value
        }
        
        for i in range(self.area_count):
            area_params = default_params.copy()
            
            calib_a = (
                self.get_parameter(f'area_{i}.calib_point_a.lon').value,
                self.get_parameter(f'area_{i}.calib_point_a.lat').value
            )
            calib_b = (
                self.get_parameter(f'area_{i}.calib_point_b.lon').value,
                self.get_parameter(f'area_{i}.calib_point_b.lat').value
            )
            calib_c = (
                self.get_parameter(f'area_{i}.calib_point_c.lon').value,
                self.get_parameter(f'area_{i}.calib_point_c.lat').value
            )
            
            # 校验标定点有效性（经纬度不能为0，需根据实际场景调整范围）
            if calib_a[0] == 0.0 or calib_a[1] == 0.0:
                self.get_logger().error(f"区域{i}的calib_point_a未配置（经纬度不能为0）")
                return []
            
            # 解析区域专属参数（若配置则覆盖默认值）
            area_interval = self.get_parameter(f'area_{i}.interval').value
            if area_interval is not None:
                area_params['interval'] = area_interval
                
            area_swap_wh_select = self.get_parameter(f'area_{i}.swap_wh_select').value
            if area_swap_wh_select is not None:
                area_params['swap_wh_select'] = area_swap_wh_select

            area_corner = self.get_parameter(f'area_{i}.start_corner').value
            if area_corner is not None:
                if area_corner not in valid_corners:
                    self.get_logger().error(f"区域{i}的start_corner无效，必须是{valid_corners}")
                    return []
                area_params['start_corner'] = area_corner
            
            area_edge_lon = self.get_parameter(f'area_{i}.edge_distance_lon').value
            if area_edge_lon is not None:
                area_params['edge_distance_lon'] = area_edge_lon
            
            area_edge_lat = self.get_parameter(f'area_{i}.edge_distance_lat').value
            if area_edge_lat is not None:
                area_params['edge_distance_lat'] = area_edge_lat
            
            area_end_corner = self.get_parameter(f'area_{i}.end_corner_mode').value
            if area_end_corner is not None:
                area_params['end_corner_mode'] = area_end_corner
            
            area_reverse_end_point = self.get_parameter(f'area_{i}.reverse_end_point').value
            if area_reverse_end_point is not None:
                area_params['reverse_end_point'] = area_reverse_end_point
            
            # 封装当前区域参数
            area_name = f'area_{i}'
            all_areas.append({
                'index': i,
                'name': area_name,
                'calib_point_a': calib_a,
                'calib_point_b': calib_b,
                'calib_point_c': calib_c,
                'param': area_params
            })
        
        return all_areas

    def _load_areas_from_yaml(self, config_file):
        """从 YAML 配置文件加载区域参数"""
        all_areas = []
        valid_corners = ['top_left', 'top_right', 'bottom_right', 'bottom_left']
        
        try:
            with open(config_file, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f"无法读取配置文件 {config_file}: {e}")
            return []
        
        if not config or 'areas' not in config:
            self.get_logger().error("配置文件缺少 'areas' 键")
            return []
        
        default_params = config.get('default', {})
        areas_list = config['areas']
        
        for i, area in enumerate(areas_list):
            area_params = default_params.copy()
            
            # 获取标定点
            try:
                calib_a = (area['calib_point_a']['lon'], area['calib_point_a']['lat'])
                calib_b = (area['calib_point_b']['lon'], area['calib_point_b']['lat'])
                calib_c = (area['calib_point_c']['lon'], area['calib_point_c']['lat'])
            except KeyError as e:
                self.get_logger().error(f"区域{i}缺少必要的标定点配置: {e}")
                return []
            
            # 校验标定点有效性
            if calib_a[0] == 0.0 or calib_a[1] == 0.0:
                self.get_logger().error(f"区域{i}的calib_point_a未配置（经纬度不能为0）")
                return []
            
            # 解析可选参数
            if 'interval' in area:
                area_params['interval'] = area['interval']
            if 'start_corner' in area:
                corner = area['start_corner']
                if corner not in valid_corners:
                    self.get_logger().error(f"区域{i}的start_corner无效: {corner}，必须是{valid_corners}")
                    return []
                area_params['start_corner'] = corner
            if 'swap_wh_select' in area:
                area_params['swap_wh_select'] = area['swap_wh_select']
            if 'edge_distance_lon' in area:
                area_params['edge_distance_lon'] = area['edge_distance_lon']
            if 'edge_distance_lat' in area:
                area_params['edge_distance_lat'] = area['edge_distance_lat']
            if 'end_corner_mode' in area:
                area_params['end_corner_mode'] = area['end_corner_mode']
            if 'reverse_end_point' in area:
                area_params['reverse_end_point'] = area['reverse_end_point']
            
            area_name = area.get('name', f'area_{i}')
            self.get_logger().info(f"加载区域 {i}: {area_name}")
            
            all_areas.append({
                'index': i,
                'name': area_name,
                'calib_point_a': calib_a,
                'calib_point_b': calib_b,
                'calib_point_c': calib_c,
                'param': area_params
            })
        
        self.get_logger().info(f"共加载 {len(all_areas)} 个区域")
        return all_areas

    def get_next_sequence(self, save_dir):
        """
        自动读取文件夹里已有的 001_、002_、003_... 文件
        返回下一个 3 位序号，例如：005
        """
        max_num = 0
        # 遍历文件夹所有文件
        for filename in os.listdir(save_dir):
            # 只匹配 3 位数字开头的文件（和你的正则一致）
            if filename[:3].isdigit() and filename[3] == '_':
                try:
                    num = int(filename[:3])
                    if num > max_num:
                        max_num = num
                except:
                    continue
        # 下一个序号 +1，并自动补 0 成 3 位
        next_num = max_num + 5
        return f"{next_num:03d}"
    
    def plan_multi_area_path(self):
        """生成多区域连续路径"""
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        save_dir = os.path.expanduser("/home/ubuntu/robot_cleaning/src/rtk_nav/rtk_nav/cleaning_path/")
        os.makedirs(save_dir, exist_ok=True)
        
        merged_path_latlon = []
        merged_path_utm = []
        all_original_corners = []  # 所有区域的原始矩形边界
        all_inner_corners = []     # 所有区域的内部矩形边界
        utm_zone = None            # 假设所有区域在同一UTM zone（实际场景需校验）
        # 新增：存储每个区域的ABC标定点UTM坐标，供绘图使用
        all_calib_points_utm = []
        # 新增：存储每个区域的路径点数量，用于保存文件时区分区域
        area_path_counts = []
        # 新增：存储每个区域的名称
        area_names = []
        
        try:
            # ---------------------- 循环生成每个区域的路径 ----------------------
            for area in self.all_areas:
                i = area['index']
                name = area['name']
                calib_a = area['calib_point_a']
                calib_b = area['calib_point_b']
                calib_c = area['calib_point_c']
                param = area['param']
                
                self.get_logger().info(f"\n正在生成区域{i}的路径...")
                self.get_logger().info(f"  标定点A: {calib_a}")
                self.get_logger().info(f"  标定点B: {calib_b}")
                self.get_logger().info(f"  标定点C: {calib_c}")
                self.get_logger().info(f"  参数: {param}")
                
                # 生成当前区域路径
                path_latlon, path_utm, orig_corners, inner_corners, zone = generate_cleaning_path_with_rotation_3points(
                    calib_a, calib_b, calib_c, param['start_corner'], param
                )
                
                if utm_zone is None:
                    utm_zone = zone
                else:
                    if zone != utm_zone:
                        self.get_logger().warning(f"区域{i}的UTM zone({zone})与其他区域不一致，可能导致路径偏移")
                
                # ---------------------- 路径连续性处理 ----------------------
                if merged_path_latlon:
                    # 上一个区域的终点
                    last_lon, last_lat = merged_path_latlon[-1]
                    # 当前区域的起点
                    curr_lon, curr_lat = path_latlon[0]
                    # 计算两点距离（若距离过远，可添加过渡点，这里简化处理）
                    dist = math.hypot(
                        (curr_lon - last_lon) * 111319.9,  # 经度差转米（近似）
                        (curr_lat - last_lat) * 111319.9   # 纬度差转米（近似）
                    )
                    if dist > 0.5:  # 若两区域起点终点距离>0.5米，打印警告
                        self.get_logger().warning(f"区域{i-1}终点与区域{i}起点距离{dist:.2f}m，可能存在断裂")
                
                # 合并当前区域数据
                merged_path_latlon.extend(path_latlon)
                merged_path_utm.extend(path_utm)
                all_original_corners.append(orig_corners)
                all_inner_corners.append(inner_corners)
                # 新增：转换当前区域ABC点为UTM并存入列表，供绘图使用
                a_lon, a_lat = calib_a
                b_lon, b_lat = calib_b
                c_lon, c_lat = calib_c
                a_e, a_n, _, _ = get_utm_coords(a_lat, a_lon)
                b_e, b_n, _, _ = get_utm_coords(b_lat, b_lon)
                c_e, c_n, _, _ = get_utm_coords(c_lat, c_lon)
                all_calib_points_utm.append( (a_e,a_n, b_e,b_n, c_e,c_n) )
                # 记录当前区域的路径点数量
                area_path_counts.append(len(path_latlon))
                # 记录当前区域的名称
                area_names.append(area['name'])
                
                self.get_logger().info(f"区域{i}路径生成完成，包含{len(path_latlon)}个点")
            
            # ---------------------- 计算全局航向角 ----------------------
            self.get_logger().info(f"\n所有区域路径合并完成，总点数：{len(merged_path_latlon)}")
            merged_headings = calculate_heading_angles(merged_path_latlon)
            
            # ---------------------- 保存合并后的路径文件（带区域区分） ----------------------
            self.seq_num = self.get_next_sequence(save_dir)
            points_filename = os.path.join(save_dir, f"{self.seq_num}_ser_south_{timestamp}.txt")
            with open(points_filename, "w", encoding="utf-8") as f:
                f.write("#序号,经度,纬度,航向角(度)\n")
                
                # 按区域写入路径点
                global_idx = 0
                for area_idx, (path_count, area_name) in enumerate(zip(area_path_counts, area_names)):
                    # 写入区域名称注释
                    f.write(f"#{area_name}\n")
                    
                    # 写入当前区域的所有点
                    for _ in range(path_count):
                        lon, lat = merged_path_latlon[global_idx]
                        heading = merged_headings[global_idx]
                        f.write(f"{global_idx+1},{lon:.8f},{lat:.8f},{heading:.2f}\n")
                        global_idx += 1
            
            self.get_logger().info(f"多区域路径文件已保存到：{points_filename}")
            
            # ---------------------- 可视化所有区域路径 ----------------------
            self._plot_multi_area_path(
                merged_path_utm, all_original_corners, all_inner_corners, utm_zone,
                save_dir, timestamp, all_calib_points_utm, area_names
            )
        
        except ValueError as e:
            self.get_logger().error(f"区域路径生成错误：{e}")
        except Exception as e:
            self.get_logger().error(f"未知异常：{str(e)}")

    # ===== 核心修复+新增ABC标定点显示：_plot_multi_area_path 绘图函数 =====
    def _plot_multi_area_path(self, merged_path_utm, all_orig_corners, all_inner_corners, utm_zone, save_dir, timestamp, all_calib_points_utm, area_names):
        """绘制所有区域的路径可视化图 - 修复matplotlib格式错误+NameError+阻塞问题 + 新增每个区域ABC标定点标注"""
        fig, ax = plt.subplots(figsize=(12, 10))
        zone_num, zone_letter = utm_zone
        
        # ✅ 修复BUG1：使用matplotlib支持的【十六进制色值】+ 单独配置线型，放弃错误的fmt格式
        # 支持无限个区域，颜色循环使用，区分度高
        hex_colors = ["#1f77b4", '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b', '#e377c2', '#7f7f7f']
        line_styles = ['-', '--'] # 实线=原始边界，虚线=内部边界
        
        # 绘制每个区域的边界+内部矩形+ABC标定点
        for i, (orig_corners, inner_corners, calib_utm) in enumerate(zip(all_orig_corners, all_inner_corners, all_calib_points_utm)):
            color = hex_colors[i % len(hex_colors)]
            a_e,a_n, b_e,b_n, c_e,c_n = calib_utm
            area_name = area_names[i] if i < len(area_names) else f'area_{i}'
            
            # 绘制原始矩形边界 - 单独传 color + linestyle 参数，无格式错误
            orig_e = [c[0] for c in orig_corners] + [orig_corners[0][0]]
            orig_n = [c[1] for c in orig_corners] + [orig_corners[0][1]]
            ax.plot(orig_e, orig_n, color=color, linestyle=line_styles[0], linewidth=2, label=f'{i}-{area_name}-boundary')
            
            # 绘制内部矩形边界 - 同上
            inner_e = [c[0] for c in inner_corners] + [inner_corners[0][0]]
            inner_n = [c[1] for c in inner_corners] + [inner_corners[0][1]]
            ax.plot(inner_e, inner_n, color=color, linestyle=line_styles[1], linewidth=1.5, label=f'{i}-{area_name}-inner')
            
            # ✅ 新增：绘制当前区域的A/B/C标定点，样式完全按参考来，带黑色描边+文字标注
            # 只在第一个区域添加label，防止图例重复；后续区域只绘图不添加label
            if i == 0:
                ax.scatter(a_e, a_n, c='red', s=60, marker='s', label='Calib Point A', edgecolors='black', linewidth=1.5, zorder=6)
                ax.scatter(b_e, b_n, c='orange', s=60, marker='o', label='Calib Point B', edgecolors='black', linewidth=1.5, zorder=6)
                ax.scatter(c_e, c_n, c='purple', s=60, marker='^', label='Calib Point C', edgecolors='black', linewidth=1.5, zorder=6)
            else:
                ax.scatter(a_e, a_n, c='red', s=60, marker='s', edgecolors='black', linewidth=1.5, zorder=6)
                ax.scatter(b_e, b_n, c='orange', s=60, marker='o', edgecolors='black', linewidth=1.5, zorder=6)
                ax.scatter(c_e, c_n, c='purple', s=60, marker='^', edgecolors='black', linewidth=1.5, zorder=6)
            
            # ✅ 每个标定点都添加文字标注，永不重复
            ax.annotate(f'{i}-A-{area_name}', (a_e, a_n), xytext=(5, 5), textcoords='offset points', fontsize=9)
            ax.annotate(f'{i}-B-{area_name}', (b_e, b_n), xytext=(5, 5), textcoords='offset points', fontsize=9)
            ax.annotate(f'{i}-C-{area_name}', (c_e, c_n), xytext=(5, 5), textcoords='offset points', fontsize=9)
        
        # 绘制合并后的清扫路径
        path_e = [p[0] for p in merged_path_utm]
        path_n = [p[1] for p in merged_path_utm]
        ax.plot(path_e, path_n, color='#000000', linewidth=1.2, label='cleaning path', alpha=0.8)
        
        # 添加方向箭头
        add_direction_arrows(ax, merged_path_utm, arrow_interval=1)
        
        # 标记全局起点和终点
        ax.scatter(path_e[0], path_n[0], c='#2ca02c', s=150, marker='o', label='start', zorder=5)
        ax.scatter(path_e[-1], path_n[-1], c='#d62728', s=150, marker='x', label='end', zorder=5)
        
        # 图表配置
        ax.set_xlabel(f'UTM E/m - {zone_num}{zone_letter}')
        ax.set_ylabel(f'UTM N/m - {zone_num}{zone_letter}')
        ax.set_title(f'planner area counter {len(all_orig_corners)} | point counter {len(merged_path_utm)}')
        ax.legend(loc='upper right', bbox_to_anchor=(1.3, 1))
        ax.grid(True, alpha=0.3)
        ax.axis('equal')
        plt.tight_layout()
        
                # 保存图片
        img_filename = os.path.join(save_dir, f"{self.seq_num}_ser_south_{timestamp}.png")
        plt.savefig(img_filename, dpi=300, bbox_inches='tight')
        self.get_logger().info(f"多区域路径图已保存到：{img_filename}")
        
        # ✅ 修复BUG3：plt.show() 非阻塞显示 + 自动关闭，避免ROS2节点卡死
        if not self.get_parameter('headless').value:
            plt.show(block=False)  # 非阻塞
            plt.pause(3)           # 显示3秒
            plt.close(fig)         # 自动关闭图片窗口

# ---------------------- 主函数 ----------------------
def main(args=None):
    rclpy.init(args=args)
    node = MultiAreaCleaningPathPlanner()
    try:
        # 路径生成在节点初始化时完成，不需要进入事件循环
        pass
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()