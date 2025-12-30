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

def generate_cleaning_path_with_rotation(base_point, width, height, rotation_deg, start_corner, param):
    """
    生成带旋转角度且支持自定义起点角点的清扫路径
    参数新增：
        start_corner: 轨迹起始角点，可选值 ['top_left', 'top_right', 'bottom_right', 'bottom_left']
    """
    lon0, lat0 = base_point
    interval = param['interval']
    edge_lon = param['edge_distance_lon']
    edge_lat = param['edge_distance_lat']
    rotation_rad = degrees_to_radians(rotation_deg)

    # 基准点转UTM
    e0, n0, zone_num, zone_letter = get_utm_coords(lat0, lon0)
    utm_zone = (zone_num, zone_letter)

    # 1. 生成未旋转的原始矩形四个角点（以基准点为top_left）
    orig_unrot = {
        'top_left': (e0, n0),
        'top_right': (e0 + width, n0),
        'bottom_right': (e0 + width, n0 - height),
        'bottom_left': (e0, n0 - height)
    }

    # 2. 旋转所有原始角点，并保持命名
    orig_rot = {}
    for corner_name, (e, n) in orig_unrot.items():
        e_rot, n_rot = rotate_point(e, n, e0, n0, rotation_rad)
        orig_rot[corner_name] = (e_rot, n_rot)
    original_corners_utm = list(orig_rot.values())  # 兼容原有返回格式

    # 3. 生成未旋转的内部矩形角点（同样命名）
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

    # 4. 旋转内部矩形角点，并保持命名
    inner_rot = {}
    for corner_name, (e, n) in inner_unrot.items():
        e_rot, n_rot = rotate_point(e, n, e0, n0, rotation_rad)
        inner_rot[corner_name] = (e_rot, n_rot)
    inner_corners_utm = list(inner_rot.values())  # 兼容原有返回格式

    # 5. 生成未旋转的内部路径（沿宽度/高度方向，和之前逻辑一致）
    path_utm_unrot = []
    inner_e0_unrot = e0 + edge_lon
    inner_n0_unrot = n0 - edge_lat
    inner_width_unrot = width - 2 * edge_lon
    inner_height_unrot = height - 2 * edge_lat

    if inner_width_unrot >= inner_height_unrot:
        num_strips = max(1, int(inner_height_unrot / interval) + 1)
        for i in range(num_strips):
            current_n_unrot = inner_n0_unrot - (inner_height_unrot) * (i / (num_strips - 1) if num_strips > 1 else 0)
            if i % 2 == 0:
                path_utm_unrot.append((inner_e0_unrot, current_n_unrot))
                path_utm_unrot.append((inner_e0_unrot + inner_width_unrot, current_n_unrot))
            else:
                path_utm_unrot.append((inner_e0_unrot + inner_width_unrot, current_n_unrot))
                path_utm_unrot.append((inner_e0_unrot, current_n_unrot))
    else:
        num_strips = max(1, int(inner_width_unrot / interval) + 1)
        for i in range(num_strips):
            current_e_unrot = inner_e0_unrot + (inner_width_unrot) * (i / (num_strips - 1) if num_strips > 1 else 0)
            if i % 2 == 0:
                path_utm_unrot.append((current_e_unrot, inner_n0_unrot))
                path_utm_unrot.append((current_e_unrot, inner_n0_unrot - inner_height_unrot))
            else:
                path_utm_unrot.append((current_e_unrot, inner_n0_unrot - inner_height_unrot))
                path_utm_unrot.append((current_e_unrot, inner_n0_unrot))

    # 6. 旋转路径点并转经纬度
    path_utm_rot = []
    path_latlon = []
    for (e_unrot, n_unrot) in path_utm_unrot:
        e_rot, n_rot = rotate_point(e_unrot, n_unrot, e0, n0, rotation_rad)
        path_utm_rot.append((e_rot, n_rot))
        lat, lon = get_latlon_from_utm(e_rot, n_rot, zone_num, zone_letter)
        path_latlon.append((lon, lat))

    # 7. 根据选定的起点角点，调整路径顺序
    # 步骤1：找到旋转后内部矩形的起点角点坐标
    inner_start_utm = inner_rot[start_corner]
    # 步骤2：找到路径中距离起点角点最近的点（作为新路径的起点）
    min_dist = float('inf')
    start_idx = 0
    for i, (e, n) in enumerate(path_utm_rot):
        dist = math.hypot(e - inner_start_utm[0], n - inner_start_utm[1])
        if dist < min_dist:
            min_dist = dist
            start_idx = i
    # 步骤3：重排路径（从起点索引开始，循环拼接）
    path_utm = path_utm_rot[start_idx:] + path_utm_rot[:start_idx]
    path_latlon = path_latlon[start_idx:] + path_latlon[:start_idx]

    return path_latlon, path_utm, original_corners_utm, inner_corners_utm, utm_zone

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
        
        # 声明参数（ROS 2参数需要先声明）
        self.declare_parameter('base_point.lon', 120.0712028)
        self.declare_parameter('base_point.lat', 30.32071848)
        self.declare_parameter('rect_width', 6.0)
        self.declare_parameter('rect_height', 15.0)
        self.declare_parameter('rotation_deg', 20.0)
        self.declare_parameter('interval', 1.0)
        self.declare_parameter('start_corner', 'bottom_right')
        self.declare_parameter('edge_distance_lon', 0.5)
        self.declare_parameter('edge_distance_lat', 0.5)
        self.declare_parameter('headless', False)
        
        # 获取参数
        self.param = {
            'base_point': (
                self.get_parameter('base_point.lon').value,
                self.get_parameter('base_point.lat').value
            ),
            'rect_width': self.get_parameter('rect_width').value,
            'rect_height': self.get_parameter('rect_height').value,
            'rotation_deg': self.get_parameter('rotation_deg').value,
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
            path_latlon, path_utm, original_corners_utm, inner_corners_utm, utm_zone = generate_cleaning_path_with_rotation(
                self.param['base_point'], 
                self.param['rect_width'], 
                self.param['rect_height'], 
                self.param['rotation_deg'], 
                self.param['start_corner'],
                self.param
            )
            zone_num, zone_letter = utm_zone
            
            # 打印参数信息
            self.get_logger().info("参数配置:")
            self.get_logger().info(f"  基准点: {self.param['base_point']}")
            self.get_logger().info(f"  矩形宽度: {self.param['rect_width']}米")
            self.get_logger().info(f"  矩形高度: {self.param['rect_height']}米")
            self.get_logger().info(f"  旋转角度: {self.param['rotation_deg']}度")
            self.get_logger().info(f"  路径间隔: {self.param['interval']}米")
            self.get_logger().info(f"  经度方向边缘距离: {self.param['edge_distance_lon']}米")
            self.get_logger().info(f"  纬度方向边缘距离: {self.param['edge_distance_lat']}米")
            self.get_logger().info(f"\n生成的路径点数量: {len(path_latlon)}")
            
            # 确保保存目录存在
            save_dir = os.path.expanduser("/home/ubuntu/robot_cleaning/src/rtk_nav/rtk_nav/cleaning_path/")
            os.makedirs(save_dir, exist_ok=True)
            
            # 计算航向角并保存路径点
            points_filename = os.path.join(save_dir, f"cleaning_path_{timestamp}.txt")
            headings = calculate_heading_angles(path_latlon)
            
            with open(points_filename, "w", encoding="utf-8") as f:
                f.write("序号,经度,纬度,航向角(度)\n")
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
            ax.plot(orig_e, orig_n, 'b-', label='原始区域边界')
            
            # 绘制内部矩形边界
            inner_e = [corner[0] for corner in inner_corners_utm] + [inner_corners_utm[0][0]]
            inner_n = [corner[1] for corner in inner_corners_utm] + [inner_corners_utm[0][1]]
            ax.plot(inner_e, inner_n, 'g--', label='内部清扫边界')
            
            # 绘制清扫路径
            path_e = [p[0] for p in path_utm]
            path_n = [p[1] for p in path_utm]
            ax.plot(path_e, path_n, 'r-', linewidth=1, label='清扫路径')
            
            # 添加方向箭头
            add_direction_arrows(ax, path_utm, arrow_interval=1)
            
            # 标记起点和终点
            ax.scatter(path_e[0], path_n[0], c='green', s=100, marker='o', label='起点')
            ax.scatter(path_e[-1], path_n[-1], c='purple', s=100, marker='x', label='终点')
            
            ax.set_xlabel(f'UTM东向 (米) - 区域 {zone_num}{zone_letter}')
            ax.set_ylabel(f'UTM北向 (米) - 区域 {zone_num}{zone_letter}')
            ax.set_title(
                f'机器人清扫路径规划\n'
                f'旋转角度: {self.param["rotation_deg"]}°, 路径间隔: {self.param["interval"]}m'
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