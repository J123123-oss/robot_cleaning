#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
import matplotlib
import matplotlib.pyplot as plt
import utm
import rclpy
from rclpy.node import Node
import datetime
from matplotlib.patches import FancyArrowPatch
import os
import numpy as np

matplotlib.rcParams['axes.unicode_minus'] = False

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
            # 最后一个点，继承上一个有效航向角，不产生新角度
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
    """核心修改1：改为顺时针旋转，适配地理坐标系习惯"""
    e_trans = e - e0
    n_trans = n - n0
    
    # 顺时针旋转矩阵（原代码是逆时针，导致方向相反）
    # 旋转矩阵（顺时针旋转，适配地理坐标系习惯）
    # e_rot_trans = e_trans * math.cos(rotation_rad) - n_trans * math.sin(rotation_rad)
    # n_rot_trans = e_trans * math.sin(rotation_rad) + n_trans * math.cos(rotation_rad)
    # 逆时针旋转（如需逆时针旋转，使用以下公式）
    e_rot_trans = e_trans * math.cos(rotation_rad) + n_trans * math.sin(rotation_rad)  # θ为逆时针旋转角
    n_rot_trans = -e_trans * math.sin(rotation_rad) + n_trans * math.cos(rotation_rad)
    e_rot = e_rot_trans + e0
    n_rot = n_rot_trans + n0
    return e_rot, n_rot

def calculate_region_from_3points(point_a, point_b, point_c):
    """向量计算保留原逻辑（已确认正确）"""
    a_lon, a_lat = point_a
    b_lon, b_lat = point_b
    c_lon, c_lat = point_c
    
    a_e, a_n, zone_num, zone_letter = get_utm_coords(a_lat, a_lon)
    b_e, b_n, _, _ = get_utm_coords(b_lat, b_lon)
    c_e, c_n, _, _ = get_utm_coords(c_lat, c_lon)
    
    # AB向量（原计算正确，保留）
    ab_e = b_e - a_e
    ab_n = b_n - a_n
    ab_length = math.hypot(ab_e, ab_n)
    
    ab_unit_e = ab_e / ab_length
    ab_unit_n = ab_n / ab_length
    perp_e = -ab_unit_n
    perp_n = ab_unit_e
    
    # AC向量（原计算正确，保留）
    ac_e = c_e - a_e
    ac_n = c_n - a_n
    ac_perp_length = ac_e * perp_e + ac_n * perp_n
    
    height = ab_length
    width_origin = ac_perp_length
    
    # 旋转角度（基于AB向量，原计算正确）
    angle_rad = math.atan2(ab_e, ab_n)
    angle_rad = angle_rad - math.pi
    rotation_deg = radians_to_degrees(angle_rad)
    
    # 基准点计算（保留原逻辑，不影响方向）
    top_left_e = a_e + perp_e * width_origin
    top_left_n = a_n + perp_n * width_origin
    top_left_lat, top_left_lon = get_latlon_from_utm(top_left_e, top_left_n, zone_num, zone_letter)
    base_point = (top_left_lon, top_left_lat)
    
    width = abs(width_origin)
    print("计算区域参数：base_point={}, width={:.2f}m, height={:.2f}m, rotation_deg={:.2f}°".format(
        base_point, width_origin, height, rotation_deg
    ))
    
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
    # default swap_wh_select=False 
    inner_width = inner_e_max - inner_e_min
    inner_height = inner_n_max - inner_n_min
    
    if inner_width <= 0.1 or inner_height <= 0.1:
        raise ValueError(f"内部区域无效！宽度:{inner_width:.2f}m, 高度:{inner_height:.2f}m")
    
    # 6. 旋转内部矩形角点（以A点为中心，保留）
    inner_rot = {}
    for corner_name, (e, n) in inner_unrot.items():
        e_rot, n_rot = rotate_point(e, n, e0, n0, rotation_rad)
        inner_rot[corner_name] = (e_rot, n_rot)
    inner_corners_utm = list(inner_rot.values())

    # ========== 【核心修改1：删除自动匹配A点逻辑，严格使用传入的start_corner】 ==========
    # 100% 以外部传入的start_corner作为唯一基准，不再自动替换，支持随时修改
    target_start_utm = inner_rot[start_corner]
    print(f"配置起始角点: {start_corner}，目标起始点UTM坐标: {target_start_utm}")
    
    # ========== 【核心修改2：路径生成方向 严格基于传入的start_corner】 ==========
    # 从传入的start_corner中解析水平/垂直方向，无任何自动适配
    hori_dir = 'left' if 'left' in start_corner else 'right'
    vert_dir = 'top' if 'top' in start_corner else 'bottom'
    
    # 8. 生成未旋转的内部路径【核心修复3：轨迹斜线跳转BUG + 保留原有逻辑】
    swap_wh_select = param['swap_wh_select']
    default_direction= inner_width >= inner_height if not swap_wh_select else inner_width <= inner_height
    path_utm_unrot = []
    if default_direction:
        # 宽 >= 高：垂直分条（上下移动）
        num_strips = max(1, int(inner_height / interval) + 1)
        if vert_dir == 'top':
            n_values = [inner_n_max - (inner_height) * (i / (num_strips - 1) if num_strips > 1 else 0) 
                        for i in range(num_strips)]
        else:
            n_values = [inner_n_min + (inner_height) * (i / (num_strips - 1) if num_strips > 1 else 0) 
                        for i in range(num_strips)]
        
        for i, current_n_unrot in enumerate(n_values):
            if (i % 2 == 0 and hori_dir == 'left') or (i % 2 == 1 and hori_dir == 'right'):
                path_utm_unrot.append((inner_e_min, current_n_unrot))
                path_utm_unrot.append((inner_e_max, current_n_unrot))
            else:
                path_utm_unrot.append((inner_e_max, current_n_unrot))
                path_utm_unrot.append((inner_e_min, current_n_unrot))
            
            # ✅ 关键修复：条带衔接逻辑，走完当前条带后垂直90度移动到同侧下一条起点，无斜线
            # if i < num_strips - 1:
            #     current_end = path_utm_unrot[-1]
            #     next_n = n_values[i+1]
            #     next_start = (current_end[0], next_n)
            #     path_utm_unrot.append(next_start)
                
    else:
        # 宽 < 高：水平分条（左右移动）
        num_strips = max(1, int(inner_width / interval) + 1)
        if hori_dir == 'left':
            e_values = [inner_e_min + (inner_width) * (i / (num_strips - 1) if num_strips > 1 else 0) 
                        for i in range(num_strips)]
        else:
            e_values = [inner_e_max - (inner_width) * (i / (num_strips - 1) if num_strips > 1 else 0) 
                        for i in range(num_strips)]
        
        for i, current_e_unrot in enumerate(e_values):
            if (i % 2 == 0 and vert_dir == 'top') or (i % 2 == 1 and vert_dir == 'bottom'):
                path_utm_unrot.append((current_e_unrot, inner_n_max))
                path_utm_unrot.append((current_e_unrot, inner_n_min))
            else:
                path_utm_unrot.append((current_e_unrot, inner_n_min))
                path_utm_unrot.append((current_e_unrot, inner_n_max))
            
            # ✅ 关键修复：条带衔接逻辑，走完当前条带后水平90度移动到同侧下一条起点，无斜线
            # if i < num_strips - 1:
            #     current_end = path_utm_unrot[-1]
            #     next_e = e_values[i+1]
            #     next_start = (next_e, current_end[1])
            #     path_utm_unrot.append(next_start)
    
    # 9. 旋转路径点（以A点为中心，保留原有逻辑）
    path_utm_rot = []
    path_latlon = []
    for (e_unrot, n_unrot) in path_utm_unrot:
        e_rot, n_rot = rotate_point(e_unrot, n_unrot, e0, n0, rotation_rad)
        path_utm_rot.append((e_rot, n_rot))
        lat, lon = get_latlon_from_utm(e_rot, n_rot, zone_num, zone_letter)
        path_latlon.append((lon, lat))
    
    # ========== 【核心修改4：修正路径起点 严格基于传入的start_corner】 ==========
    # 仅用配置的start_corner对应角点做校准，不再关联A点，阈值合理0.5m，避免误交换
    first_point_dist = math.hypot(path_utm_rot[0][0] - target_start_utm[0], 
                                  path_utm_rot[0][1] - target_start_utm[1])
    # if first_point_dist > 0.5:
    #     path_utm_rot[0], path_utm_rot[1] = path_utm_rot[1], path_utm_rot[0]
    #     path_latlon[0], path_latlon[1] = path_latlon[1], path_latlon[0]
    #     print(f"路径起点调整：原起点与{start_corner}偏差{first_point_dist:.2f}m，已交换前两点")
    
    # 日志优化：打印配置的起始角点+关键参数，方便调试
    print(f"路径生成方向：水平={hori_dir}，垂直={vert_dir}")
    print(f"宽度处理：原始宽度={width:.2f}m → 延伸方向={'东向' if width_sign ==1 else '西向'}，内部宽度={inner_width:.2f}m")
    
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

class CleaningPathPlanner(Node):
    def __init__(self):
        super().__init__('three_point_planner')
        
        # 保留原参数配置
        # 声明参数（ROS 2参数需要先声明）120.0711247716332,30.320803806689252

        # 120.07113330166666,30.321393565666668
        # 120.07104722000001,30.321617500000002
        # 120.07112942683334


        # A:120.07149470883333,30.32131817333333
        # B:120.07142615983334,30.32123303533333
        # C:120.07151235049999,30.321311033166662

        # A:120.07129427383333,30.321726006333332
        # B:120.07130059466665,30.321665768833334
        # C:120.07132923066665,30.321670159499998
        self.declare_parameter('calib_point_c.lon', 120.07132923066665)
        self.declare_parameter('calib_point_c.lat', 30.321670159499998)
        self.declare_parameter('calib_point_b.lon', 120.07130059466665)
        self.declare_parameter('calib_point_b.lat', 30.321665768833334)
        self.declare_parameter('calib_point_a.lon', 120.07129427383333)
        self.declare_parameter('calib_point_a.lat', 30.321726006333332)

        # self.declare_parameter('calib_point_c.lon', 120.07149470883333)
        # self.declare_parameter('calib_point_c.lat', 30.32131817333333)
        # self.declare_parameter('calib_point_b.lon', 120.07142615983334)
        # self.declare_parameter('calib_point_b.lat', 30.32123303533333)
        # self.declare_parameter('calib_point_a.lon', 120.07151235049999)
        # self.declare_parameter('calib_point_a.lat', 30.321311033166662)
        # self.declare_parameter('calib_point_a.lon', 120.06893303174934)  # 第一个点 = start_corner
        # self.declare_parameter('calib_point_a.lat', 30.320515493992254)  # 120.06908157229124,30.320549326045818
        # self.declare_parameter('calib_point_b.lon', 120.0689008658952)   #120.06934719266512,30.319875087155783,up mirror:120.0689008658952,30.32109457743618
        # self.declare_parameter('calib_point_b.lat', 30.32109457743618)
        # self.declare_parameter('calib_point_c.lon', 120.06908157229124)
        # self.declare_parameter('calib_point_c.lat', 30.320549326045818)#120.06893303174934,30.320515493992254,miorror:120.0692708938497,30.320612377146574

        # self.declare_parameter('calib_point_a.lon', 120.06891577325935)#120.06891577325935,30.320537691107706
        # self.declare_parameter('calib_point_a.lat', 30.320537691107706)
        # self.declare_parameter('calib_point_b.lon', 120.0691364728377)#120.0691364728377,30.319852150388023
        # self.declare_parameter('calib_point_b.lat', 30.319852150388023)
        # self.declare_parameter('calib_point_c.lon', 120.06873704947856)#120.06873704947856,30.320508473942272
        # self.declare_parameter('calib_point_c.lat', 30.320508473942272)




        self.declare_parameter('interval', 1.0)
        self.declare_parameter('start_corner', 'top_left')
        self.declare_parameter('swap_wh_select', False)
        self.declare_parameter('edge_distance_lon', 0.0)
        self.declare_parameter('edge_distance_lat', 0.0)
        self.declare_parameter('headless', True)
        
        self.param = {
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
            'swap_wh_select': self.get_parameter('swap_wh_select').value,
            'edge_distance_lon': self.get_parameter('edge_distance_lon').value,
            'edge_distance_lat': self.get_parameter('edge_distance_lat').value,
            'headless': self.get_parameter('headless').value
        }
        
        valid_corners = ['top_left', 'top_right', 'bottom_right', 'bottom_left']
        if self.param['start_corner'] not in valid_corners:
            self.get_logger().error(
                f"无效的start_corner: {self.param['start_corner']}，必须是 {valid_corners}"
            )
            return
        
        self.plan_path()
    
    def plan_path(self):
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        
        try:
            path_latlon, path_utm, original_corners_utm, inner_corners_utm, utm_zone = generate_cleaning_path_with_rotation_3points(
                self.param['calib_point_a'], 
                self.param['calib_point_b'], 
                self.param['calib_point_c'], 
                self.param['start_corner'],
                self.param
            )
            zone_num, zone_letter = utm_zone
            
            self.get_logger().info("参数配置:")
            self.get_logger().info(f"  路径间隔: {self.param['interval']}米")
            self.get_logger().info(f"  起始点: {self.param['start_corner']}")
            self.get_logger().info(f"  经度方向边缘距离: {self.param['edge_distance_lon']}米")
            self.get_logger().info(f"  纬度方向边缘距离: {self.param['edge_distance_lat']}米")
            self.get_logger().info(f"\n生成的路径点数量: {len(path_latlon)}")
            
            save_dir = os.path.expanduser("/home/ubuntu/robot_cleaning/src/rtk_nav/rtk_nav/cleaning_path/")
            os.makedirs(save_dir, exist_ok=True)
            
            points_filename = os.path.join(save_dir, f"three_path_{timestamp}.txt")
            headings = calculate_heading_angles(path_latlon)
            
            with open(points_filename, "w", encoding="utf-8") as f:
                f.write("#序号,经度,纬度,航向角(度)\n")
                for i in range(len(path_latlon)):
                    lon, lat = path_latlon[i]
                    heading = headings[i]
                    f.write(f"{i+1},{lon:.8f},{lat:.8f},{heading:.2f}\n")
            self.get_logger().info(f"所有路径点及航向角已保存到 {points_filename} 文件")
            
            # 绘制图表（不变）
            fig, ax = plt.subplots(figsize=(10, 8))
            
            orig_e = [corner[0] for corner in original_corners_utm] + [original_corners_utm[0][0]]
            orig_n = [corner[1] for corner in original_corners_utm] + [original_corners_utm[0][1]]
            ax.plot(orig_e, orig_n, 'b-', label='origin')
            
            inner_e = [corner[0] for corner in inner_corners_utm] + [inner_corners_utm[0][0]]
            inner_n = [corner[1] for corner in inner_corners_utm] + [inner_corners_utm[0][1]]
            ax.plot(inner_e, inner_n, 'g--', label='inner')
            
            a_lon, a_lat = self.param['calib_point_a']
            b_lon, b_lat = self.param['calib_point_b']
            c_lon, c_lat = self.param['calib_point_c']
            a_e, a_n, _, _ = get_utm_coords(a_lat, a_lon)
            b_e, b_n, _, _ = get_utm_coords(b_lat, b_lon)
            c_e, c_n, _, _ = get_utm_coords(c_lat, c_lon)
            
            ax.scatter(a_e, a_n, c='red', s=120, marker='s', label='Calib Point A', edgecolors='black', linewidth=1.5)
            ax.annotate('Point A', (a_e, a_n), xytext=(5, 5), textcoords='offset points', fontsize=10, fontweight='bold')
            ax.scatter(b_e, b_n, c='orange', s=120, marker='o', label='Calib Point B', edgecolors='black', linewidth=1.5)
            ax.annotate('Point B', (b_e, b_n), xytext=(5, 5), textcoords='offset points', fontsize=10, fontweight='bold')
            ax.scatter(c_e, c_n, c='purple', s=120, marker='^', label='Calib Point C', edgecolors='black', linewidth=1.5)
            ax.annotate('Point C', (c_e, c_n), xytext=(5, 5), textcoords='offset points', fontsize=10, fontweight='bold')
            
            path_e = [p[0] for p in path_utm]
            path_n = [p[1] for p in path_utm]
            ax.plot(path_e, path_n, 'r-', linewidth=1, label='cleaning path')
            
            add_direction_arrows(ax, path_utm, arrow_interval=1)
            
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
            
            img_filename = os.path.join(save_dir, f'three_path_{timestamp}.png')
            plt.savefig(img_filename, dpi=300)
            self.get_logger().info(f"路径图已保存到 {img_filename}")
            
            if not self.param['headless']:
                plt.show()
            
        except ValueError as e:
            self.get_logger().error(f"错误: {e}")
        except Exception as e:
            self.get_logger().error(f"发生异常: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = CleaningPathPlanner()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()