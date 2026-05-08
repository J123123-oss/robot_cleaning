#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Utility to generate dense test points from a cleaning path.
This script imports `generate_cleaning_path_with_rotation_3points` from
`three_point_planner.py`, interpolates along each path segment using a
`dense_spacing` parameter (meters), computes headings for dense points,
and saves them to a file alongside the regular path file.

Usage: run as a ROS2 node or standalone script. It writes a file
`three_path_<timestamp>_dense.txt` into the same `cleaning_path` directory.
"""
import math
import datetime
import os
import utm
import rclpy
from rclpy.node import Node
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch

from three_point_planner import (
    generate_cleaning_path_with_rotation_3points,
    calculate_heading_angles,
)


def interpolate_segment(a, b, spacing):
    """在两个UTM点之间按spacing插值，返回包含起点但不包含终点的点列表"""
    ax, ay = a
    bx, by = b
    dx = bx - ax
    dy = by - ay
    dist = math.hypot(dx, dy)
    if dist <= 0 or spacing <= 0:
        return [a]
    n = max(1, int(math.floor(dist / spacing)))
    points = []
    for i in range(n):
        t = i * spacing / dist
        x = ax + dx * t
        y = ay + dy * t
        points.append((x, y))
    return points


def generate_dense_from_utm(path_utm, spacing):
    """对UTM路径点按每段间隔spacing插值，返回dense点列表（UTM）"""
    if len(path_utm) < 2:
        return list(path_utm)
    dense = []
    for i in range(len(path_utm) - 1):
        a = path_utm[i]
        b = path_utm[i + 1]
        seg_pts = interpolate_segment(a, b, spacing)
        if i > 0 and seg_pts:
            # 如果上一段已经添加了终点，避免重复起点
            pass
        dense.extend(seg_pts)
    # 最后加上终点
    dense.append(path_utm[-1])
    return dense


def utm_to_latlon_list(path_utm, utm_zone):
    zone_num, zone_letter = utm_zone
    latlon = []
    for e, n in path_utm:
        lat, lon = utm.to_latlon(e, n, zone_num, zone_letter)
        # 返回 (lon, lat) 与 three_point_planner 一致
        latlon.append((lon, lat))
    return latlon


def compute_headings_for_latlon(latlon_points):
    return calculate_heading_angles(latlon_points)


class DensePointGenerator(Node):
    def __init__(self):
        super().__init__('latlon_test_point')
        # 声明参数
        # A:120.07129427383333,30.321726006333332
        # B:120.07130059466665,30.321665768833334
        # C:120.07132923066665,30.321670159499998
        self.declare_parameter('calib_point_a.lon', 120.07129427383333)
        self.declare_parameter('calib_point_a.lat', 30.321726006333332)
        self.declare_parameter('calib_point_b.lon', 120.07130059466665)
        self.declare_parameter('calib_point_b.lat', 30.321665768833334)
        self.declare_parameter('calib_point_c.lon', 120.07132923066665)
        self.declare_parameter('calib_point_c.lat', 30.321670159499998)
        self.declare_parameter('interval', 0.8)
        self.declare_parameter('dense_spacing', 0.4)
        self.declare_parameter('start_corner', 'top_left')
        self.declare_parameter('swap_wh_select', False)
        self.declare_parameter('edge_distance_lon', 0.0)
        self.declare_parameter('edge_distance_lat', 0.0)

        self.param = {
            'calib_point_a': (
                self.get_parameter('calib_point_a.lon').value,
                self.get_parameter('calib_point_a.lat').value,
            ),
            'calib_point_b': (
                self.get_parameter('calib_point_b.lon').value,
                self.get_parameter('calib_point_b.lat').value,
            ),
            'calib_point_c': (
                self.get_parameter('calib_point_c.lon').value,
                self.get_parameter('calib_point_c.lat').value,
            ),
            'interval': self.get_parameter('interval').value,
            'start_corner': self.get_parameter('start_corner').value,
            'swap_wh_select': self.get_parameter('swap_wh_select').value,
            'edge_distance_lon': self.get_parameter('edge_distance_lon').value,
            'edge_distance_lat': self.get_parameter('edge_distance_lat').value,
            'dense_spacing': self.get_parameter('dense_spacing').value,
        }

        self.generate_and_save()

    def generate_and_save(self):
        timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        try:
            path_latlon, path_utm, original_corners_utm, inner_corners_utm, utm_zone = \
                generate_cleaning_path_with_rotation_3points(
                    self.param['calib_point_a'],
                    self.param['calib_point_b'],
                    self.param['calib_point_c'],
                    self.param['start_corner'],
                    self.param,
                )

            # 生成密集UTM点
            dense_utm = generate_dense_from_utm(path_utm, self.param['dense_spacing'])
            dense_latlon = utm_to_latlon_list(dense_utm, utm_zone)
            dense_headings = compute_headings_for_latlon(dense_latlon)

            save_dir = os.path.expanduser('/home/ztl/robot_cleaning/src/rtk_nav/rtk_nav/cleaning_path/')
            os.makedirs(save_dir, exist_ok=True)

            dense_filename = os.path.join(save_dir, f'three_path_{timestamp}_dense.txt')
            with open(dense_filename, 'w', encoding='utf-8') as f:
                f.write('#idx,lon,lat,heading_deg\n')
                for i, (lonlat, hdg) in enumerate(zip(dense_latlon, dense_headings)):
                    lon, lat = lonlat
                    f.write(f"{i+1},{lon:.8f},{lat:.8f},{hdg:.2f}\n")

            self.get_logger().info(f'Dense points: {len(dense_latlon)} saved to {dense_filename}')

            # 绘图：在原路径图上叠加密集点并保存为图片
            try:
                matplotlib.rcParams['axes.unicode_minus'] = False
                fig, ax = plt.subplots(figsize=(10, 8))

                # 原始外接与内部角点
                if original_corners_utm:
                    orig_e = [corner[0] for corner in original_corners_utm] + [original_corners_utm[0][0]]
                    orig_n = [corner[1] for corner in original_corners_utm] + [original_corners_utm[0][1]]
                    ax.plot(orig_e, orig_n, 'b-', label='origin')

                if inner_corners_utm:
                    inner_e = [corner[0] for corner in inner_corners_utm] + [inner_corners_utm[0][0]]
                    inner_n = [corner[1] for corner in inner_corners_utm] + [inner_corners_utm[0][1]]
                    ax.plot(inner_e, inner_n, 'g--', label='inner')

                # 清扫路径
                if path_utm:
                    path_e = [p[0] for p in path_utm]
                    path_n = [p[1] for p in path_utm]
                    ax.plot(path_e, path_n, 'r-', linewidth=1, label='cleaning path')
                    ax.scatter(path_e[0], path_n[0], c='green', s=100, marker='o', label='start')
                    ax.scatter(path_e[-1], path_n[-1], c='purple', s=100, marker='x', label='end')

                # 密集点
                if dense_utm:
                    dense_e = [p[0] for p in dense_utm]
                    dense_n = [p[1] for p in dense_utm]
                    ax.scatter(dense_e, dense_n, c='blue', s=10, label='dense points', alpha=0.6)

                # 标注校准点 A/B/C
                try:
                    a_lon, a_lat = self.param['calib_point_a']
                    b_lon, b_lat = self.param['calib_point_b']
                    c_lon, c_lat = self.param['calib_point_c']
                    a_e, a_n, _, _ = utm.from_latlon(a_lat, a_lon)
                    b_e, b_n, _, _ = utm.from_latlon(b_lat, b_lon)
                    c_e, c_n, _, _ = utm.from_latlon(c_lat, c_lon)

                    ax.scatter(a_e, a_n, c='red', s=120, marker='s', label='Calib Point A', edgecolors='black', linewidth=1.0)
                    ax.annotate('Point A', (a_e, a_n), xytext=(5, 5), textcoords='offset points')
                    ax.scatter(b_e, b_n, c='orange', s=120, marker='o', label='Calib Point B', edgecolors='black', linewidth=1.0)
                    ax.annotate('Point B', (b_e, b_n), xytext=(5, 5), textcoords='offset points')
                    ax.scatter(c_e, c_n, c='purple', s=120, marker='^', label='Calib Point C', edgecolors='black', linewidth=1.0)
                    ax.annotate('Point C', (c_e, c_n), xytext=(5, 5), textcoords='offset points')
                except Exception:
                    pass

                zone_num, zone_letter = utm_zone
                ax.set_xlabel(f'UTM east m - zone {zone_num}{zone_letter}')
                ax.set_ylabel(f'UTM north m - zone {zone_num}{zone_letter}')
                ax.set_title(f'three_point dense points - {timestamp}')
                ax.legend()
                ax.grid(True)
                ax.axis('equal')
                plt.tight_layout()

                img_filename = os.path.join(save_dir, f'three_path_{timestamp}_dense.png')
                plt.savefig(img_filename, dpi=300)
                plt.close(fig)
                self.get_logger().info(f'Dense path image saved to {img_filename}')
            except Exception as e:
                self.get_logger().error(f'绘图失败: {e}')
        except Exception as e:
            self.get_logger().error(f'生成密集点失败: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = DensePointGenerator()
    try:
        # 生成完毕后不持续spin，直接退出
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
