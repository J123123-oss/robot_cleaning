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
    
    e_rot_trans = e_trans * math.cos(rotation_rad) - n_trans * math.sin(rotation_rad)
    n_rot_trans = e_trans * math.sin(rotation_rad) + n_trans * math.cos(rotation_rad)
    
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
    
    return base_point, width, height, rotation_deg, (a_e, a_n), (zone_num, zone_letter)

def generate_cleaning_path_with_rotation_3points(point_a, point_b, point_c, start_corner, param):
    base_point, width, height, rotation_deg, start_utm, utm_zone = calculate_region_from_3points(
        point_a, point_b, point_c
    )
    zone_num, zone_letter = utm_zone
    
    lon0, lat0 = base_point
    interval = param['interval']
    edge_lon = param['edge_distance_lon']
    edge_lat = param['edge_distance_lat']
    rotation_rad = degrees_to_radians(rotation_deg)
    
    e0, n0, _, _ = get_utm_coords(lat0, lon0)
    
    # 原始矩形
    orig_unrot = {
        'top_left': (e0, n0),
        'top_right': (e0 + width, n0),
        'bottom_right': (e0 + width, n0 - height),
        'bottom_left': (e0, n0 - height)
    }
    orig_rot = {}
    for corner_name, (e, n) in orig_unrot.items():
        e_rot, n_rot = rotate_point(e, n, e0, n0, rotation_rad)
        orig_rot[corner_name] = (e_rot, n_rot)
    original_corners_utm = list(orig_rot.values())
    
    # 内部矩形
    inner_unrot = {
        'top_left': (e0 + edge_lon, n0 - edge_lat),
        'top_right': (e0 + width - edge_lon, n0 - edge_lat),
        'bottom_right': (e0 + width - edge_lon, n0 - height + edge_lat),
        'bottom_left': (e0 + edge_lon, n0 - height + edge_lat)
    }
    inner_width = (e0 + width - edge_lon) - (e0 + edge_lon)
    inner_height = (n0 - edge_lat) - (n0 - height + edge_lat)
    if inner_width <= 0.1 or inner_height <= 0.1:
        raise ValueError(f"区域内部无效！宽度:{inner_width:.2f}m, 高度:{inner_height:.2f}m")
    
    inner_rot = {}
    for corner_name, (e, n) in inner_unrot.items():
        e_rot, n_rot = rotate_point(e, n, e0, n0, rotation_rad)
        inner_rot[corner_name] = (e_rot, n_rot)
    inner_corners_utm = list(inner_rot.values())
    
    # 生成路径
    path_utm_unrot = []
    # ===== 修复BUG2：提前定义 inner_n_max/inner_n_min，解决NameError =====
    inner_n_max = inner_unrot['top_left'][1]
    inner_n_min = inner_unrot['bottom_left'][1]
    if inner_width >= inner_height:
        num_strips = max(1, int(inner_height / interval) + 1)
        hori_dir = 'left' if 'left' in start_corner else 'right'
        vert_dir = 'top' if 'top' in start_corner else 'bottom'
        
        if vert_dir == 'top':
            n_values = [inner_n_max - (inner_height) * (i / (num_strips - 1) if num_strips > 1 else 0) 
                        for i in range(num_strips)]
        else:
            n_values = [inner_n_min + (inner_height) * (i / (num_strips - 1) if num_strips > 1 else 0) 
                        for i in range(num_strips)]
        
        for i, current_n_unrot in enumerate(n_values):
            if (i % 2 == 0 and hori_dir == 'left') or (i % 2 == 1 and hori_dir == 'right'):
                path_utm_unrot.append((inner_unrot['top_left'][0], current_n_unrot))
                path_utm_unrot.append((inner_unrot['top_right'][0], current_n_unrot))
            else:
                path_utm_unrot.append((inner_unrot['top_right'][0], current_n_unrot))
                path_utm_unrot.append((inner_unrot['top_left'][0], current_n_unrot))
    else:
        num_strips = max(1, int(inner_width / interval) + 1)
        hori_dir = 'left' if 'left' in start_corner else 'right'
        vert_dir = 'top' if 'top' in start_corner else 'bottom'
        
        if hori_dir == 'left':
            e_values = [inner_unrot['top_left'][0] + (inner_width) * (i / (num_strips - 1) if num_strips > 1 else 0) 
                        for i in range(num_strips)]
        else:
            e_values = [inner_unrot['top_right'][0] - (inner_width) * (i / (num_strips - 1) if num_strips > 1 else 0) 
                        for i in range(num_strips)]
        
        for i, current_e_unrot in enumerate(e_values):
            if (i % 2 == 0 and vert_dir == 'top') or (i % 2 == 1 and vert_dir == 'bottom'):
                path_utm_unrot.append((current_e_unrot, inner_n_max))
                path_utm_unrot.append((current_e_unrot, inner_n_min))
            else:
                path_utm_unrot.append((current_e_unrot, inner_n_min))
                path_utm_unrot.append((current_e_unrot, inner_n_max))
    
    # 旋转路径并转经纬度
    path_utm_rot = []
    path_latlon = []
    for (e_unrot, n_unrot) in path_utm_unrot:
        e_rot, n_rot = rotate_point(e_unrot, n_unrot, e0, n0, rotation_rad)
        path_utm_rot.append((e_rot, n_rot))
        lat, lon = get_latlon_from_utm(e_rot, n_rot, zone_num, zone_letter)
        path_latlon.append((lon, lat))
    
    # 校准起点
    target_start_utm = inner_rot[start_corner]
    first_point_dist = math.hypot(path_utm_rot[0][0] - target_start_utm[0], 
                                  path_utm_rot[0][1] - target_start_utm[1])
    if first_point_dist > 0.1:
        path_utm_rot[0], path_utm_rot[1] = path_utm_rot[1], path_utm_rot[0]
        path_latlon[0], path_latlon[1] = path_latlon[1], path_latlon[0]
    
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
        
        # 声明多区域参数（支持动态配置）
        self.declare_parameter('area_count', 2)
        self.declare_parameter('default.interval', 1.0)
        self.declare_parameter('default.start_corner', 'top_left')
        self.declare_parameter('default.edge_distance_lon', 0.5)
        self.declare_parameter('default.edge_distance_lat', 0.5)
        self.declare_parameter('headless', False)
        
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
            self.declare_parameter(f'area_{i}.edge_distance_lon', None)
            self.declare_parameter(f'area_{i}.edge_distance_lat', None)
        
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
            'edge_distance_lon': self.get_parameter('default.edge_distance_lon').value,
            'edge_distance_lat': self.get_parameter('default.edge_distance_lat').value
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
            
            # 封装当前区域参数
            all_areas.append({
                'index': i,
                'calib_point_a': calib_a,
                'calib_point_b': calib_b,
                'calib_point_c': calib_c,
                'param': area_params
            })
        
        return all_areas
    
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
        
        try:
            # ---------------------- 循环生成每个区域的路径 ----------------------
            for area in self.all_areas:
                i = area['index']
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
                
                self.get_logger().info(f"区域{i}路径生成完成，包含{len(path_latlon)}个点")
            
            # ---------------------- 计算全局航向角 ----------------------
            self.get_logger().info(f"\n所有区域路径合并完成，总点数：{len(merged_path_latlon)}")
            merged_headings = calculate_heading_angles(merged_path_latlon)
            
            # ---------------------- 保存合并后的路径文件 ----------------------
            points_filename = os.path.join(save_dir, f'fullpath_{timestamp}.txt')
            with open(points_filename, "w", encoding="utf-8") as f:
                f.write("#序号,经度,纬度,航向角(度)\n")
                for idx in range(len(merged_path_latlon)):
                    lon, lat = merged_path_latlon[idx]
                    heading = merged_headings[idx]
                    f.write(f"{idx+1},{lon:.8f},{lat:.8f},{heading:.2f}\n")
            self.get_logger().info(f"多区域路径文件已保存到：{points_filename}")
            
            # ---------------------- 可视化所有区域路径 ----------------------
            self._plot_multi_area_path(
                merged_path_utm, all_original_corners, all_inner_corners, utm_zone,
                save_dir, timestamp
            )
        
        except ValueError as e:
            self.get_logger().error(f"区域路径生成错误：{e}")
        except Exception as e:
            self.get_logger().error(f"未知异常：{str(e)}")

    # ===== 核心修复：_plot_multi_area_path 绘图函数 修复所有报错 =====
    def _plot_multi_area_path(self, merged_path_utm, all_orig_corners, all_inner_corners, utm_zone, save_dir, timestamp):
        """绘制所有区域的路径可视化图 - 修复matplotlib格式错误+NameError+阻塞问题"""
        fig, ax = plt.subplots(figsize=(12, 10))
        zone_num, zone_letter = utm_zone
        
        # ✅ 修复BUG1：使用matplotlib支持的【十六进制色值】+ 单独配置线型，放弃错误的fmt格式
        # 支持无限个区域，颜色循环使用，区分度高
        hex_colors = ["#1f77b4", '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b', '#e377c2', '#7f7f7f']
        line_styles = ['-', '--'] # 实线=原始边界，虚线=内部边界
        
        for i, (orig_corners, inner_corners) in enumerate(zip(all_orig_corners, all_inner_corners)):
            color = hex_colors[i % len(hex_colors)]
            
            # 绘制原始矩形边界 - 单独传 color + linestyle 参数，无格式错误
            orig_e = [c[0] for c in orig_corners] + [orig_corners[0][0]]
            orig_n = [c[1] for c in orig_corners] + [orig_corners[0][1]]
            ax.plot(orig_e, orig_n, color=color, linestyle=line_styles[0], linewidth=2, label=f'aera{i} - boundary')
            
            # 绘制内部矩形边界 - 同上
            inner_e = [c[0] for c in inner_corners] + [inner_corners[0][0]]
            inner_n = [c[1] for c in inner_corners] + [inner_corners[0][1]]
            ax.plot(inner_e, inner_n, color=color, linestyle=line_styles[1], linewidth=1.5, label=f'aera{i} - inner')
        
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
        img_filename = os.path.join(save_dir, f'fullpath_{timestamp}.png')
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
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()