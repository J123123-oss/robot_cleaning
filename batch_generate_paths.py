#!/usr/bin/env python3
"""批量生成清扫路径文件 —— 纯 Python 版本，不依赖 ROS2 环境。

直接读取 YAML 配置，调用 full_path_planner_dense 中的核心函数生成路径。
无需 ros2 run，适合在开发机上快速预览和调试。
"""

import sys
import os
import math
import glob
import yaml
import datetime
# import matplotlib
# import matplotlib.pyplot as plt
# from matplotlib.patches import FancyArrowPatch

# 添加项目路径以便导入
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(SCRIPT_DIR, "src", "rtk_nav", "rtk_nav"))

from full_path_planner_dense import (
    generate_cleaning_path_with_rotation_3points,
    calculate_heading_angles,
    generate_dense_from_utm,
    get_dense_spacing,
    get_utm_coords,
    get_latlon_from_utm,
    # add_direction_arrows,
)

# matplotlib.rcParams['axes.unicode_minus'] = False

CONFIG_DIR = os.path.join(SCRIPT_DIR, "src", "rtk_nav", "rtk_nav", "config")
OUTPUT_DIR = os.path.join(SCRIPT_DIR, "src", "rtk_nav", "rtk_nav", "cleaning_path")

VALID_CORNERS = ['top_left', 'top_right', 'bottom_right', 'bottom_left']


def load_areas_from_yaml(config_file):
    """从 YAML 配置文件加载区域参数（与 ROS 节点的 _load_areas_from_yaml 等价）"""
    with open(config_file, 'r', encoding='utf-8') as f:
        config = yaml.safe_load(f)

    if not config or 'areas' not in config:
        raise ValueError(f"配置文件缺少 'areas' 键: {config_file}")

    default_params = config.get('default', {})
    areas_list = config['areas']
    all_areas = []

    for i, area in enumerate(areas_list):
        area_params = default_params.copy()

        calib_a = (area['calib_point_a']['lon'], area['calib_point_a']['lat'])
        calib_b = (area['calib_point_b']['lon'], area['calib_point_b']['lat'])
        calib_c = (area['calib_point_c']['lon'], area['calib_point_c']['lat'])

        if calib_a[0] == 0.0 or calib_a[1] == 0.0:
            raise ValueError(f"区域{i}的calib_point_a未配置（经纬度不能为0）")

        if 'interval' in area:
            area_params['interval'] = area['interval']
        if 'start_corner' in area:
            corner = area['start_corner']
            if corner not in VALID_CORNERS:
                raise ValueError(f"区域{i}的start_corner无效: {corner}，必须是{VALID_CORNERS}")
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
        area_params['dense_spacing'] = get_dense_spacing(area)

        area_name = area.get('name', f'area_{i}')
        print(f"  加载区域 {i}: {area_name}")

        all_areas.append({
            'index': i,
            'name': area_name,
            'calib_point_a': calib_a,
            'calib_point_b': calib_b,
            'calib_point_c': calib_c,
            'param': area_params
        })

    print(f"  共加载 {len(all_areas)} 个区域")
    return all_areas, default_params


# def plot_multi_area_path(merged_path_utm, all_orig_corners, all_inner_corners,
#                           utm_zone, save_dir, file_prefix, all_calib_points_utm,
#                           area_names, dense_utm=None, area_index_ranges=None,
#                           headless=True):
#     """绘制所有区域的路径可视化图"""
#     fig, ax = plt.subplots(figsize=(24, 20))
#     zone_num, zone_letter = utm_zone
#
#     hex_colors = ["#1f77b4", '#ff7f0e', '#2ca02c', '#d62728', '#9467bd',
#                   '#8c564b', '#e377c2', '#7f7f7f']
#     line_styles = ['-', '--']
#
#     for i, (orig_corners, inner_corners, calib_utm) in enumerate(
#             zip(all_orig_corners, all_inner_corners, all_calib_points_utm)):
#         color = hex_colors[i % len(hex_colors)]
#         a_e, a_n, b_e, b_n, c_e, c_n = calib_utm
#         area_name = area_names[i] if i < len(area_names) else f'area_{i}'
#
#         inner_e = [c[0] for c in inner_corners] + [inner_corners[0][0]]
#         inner_n = [c[1] for c in inner_corners] + [inner_corners[0][1]]
#         ax.plot(inner_e, inner_n, color=color, linestyle=line_styles[1],
#                 linewidth=1.5, label=f'{i}-{area_name}-inner')
#
#         if i == 0:
#             ax.scatter(a_e, a_n, c='red', s=60, marker='s',
#                        label='Calib Point A', edgecolors='black', linewidth=1.5, zorder=6)
#             ax.scatter(b_e, b_n, c='orange', s=60, marker='o',
#                        label='Calib Point B', edgecolors='black', linewidth=1.5, zorder=6)
#             ax.scatter(c_e, c_n, c='purple', s=60, marker='^',
#                        label='Calib Point C', edgecolors='black', linewidth=1.5, zorder=6)
#         else:
#             ax.scatter(a_e, a_n, c='red', s=60, marker='s',
#                        edgecolors='black', linewidth=1.5, zorder=6)
#             ax.scatter(b_e, b_n, c='orange', s=60, marker='o',
#                        edgecolors='black', linewidth=1.5, zorder=6)
#             ax.scatter(c_e, c_n, c='purple', s=60, marker='^',
#                        edgecolors='black', linewidth=1.5, zorder=6)
#
#         ax.annotate(f'{i}-A-{area_name}', (a_e, a_n), xytext=(5, 5),
#                     textcoords='offset points', fontsize=9)
#         ax.annotate(f'{i}-B-{area_name}', (b_e, b_n), xytext=(5, 5),
#                     textcoords='offset points', fontsize=9)
#         ax.annotate(f'{i}-C-{area_name}', (c_e, c_n), xytext=(5, 5),
#                     textcoords='offset points', fontsize=9)
#
#     path_e = [p[0] for p in merged_path_utm]
#     path_n = [p[1] for p in merged_path_utm]
#     ax.plot(path_e, path_n, color='#000000', linewidth=1.2,
#             label='cleaning path', alpha=0.8)
#
#     add_direction_arrows(ax, merged_path_utm, arrow_interval=1)
#
#     ax.scatter(path_e[0], path_n[0], c='#2ca02c', s=150, marker='o',
#                label='start', zorder=5)
#     ax.scatter(path_e[-1], path_n[-1], c='#d62728', s=150, marker='x',
#                label='end', zorder=5)
#
#     ax.set_xlabel(f'UTM E/m - {zone_num}{zone_letter}')
#     ax.set_ylabel(f'UTM N/m - {zone_num}{zone_letter}')
#     ax.set_title(f'planner area counter {len(all_orig_corners)} | point counter {len(merged_path_utm)}')
#     ax.legend(loc='upper right', bbox_to_anchor=(1.3, 1))
#     ax.grid(True, alpha=0.3)
#     ax.axis('equal')
#     plt.tight_layout()
#
#     # 保存带密集点的图片
#     if dense_utm is not None:
#         dense_e = [p[0] for p in dense_utm]
#         dense_n = [p[1] for p in dense_utm]
#         ax.scatter(dense_e, dense_n, c='blue', s=10, label='dense points', alpha=0.6)
#         ax.set_title(f'planner area counter {len(all_orig_corners)} | point counter {len(merged_path_utm)} | dense point counter {len(dense_utm)}')
#     img_filename = os.path.join(save_dir, f"{file_prefix}.png")
#     plt.savefig(img_filename, dpi=300, bbox_inches='tight')
#     print(f"  路径图已保存: {img_filename}")
#
#     if not headless:
#         plt.show(block=False)
#         plt.pause(3)
#         plt.close(fig)


def process_config(config_file, output_dir):
    """处理单个配置文件，生成路径文件和可视化"""
    basename = os.path.splitext(os.path.basename(config_file))[0]
    print(f"\n{'='*60}")
    print(f"Processing: {basename}")
    print(f"Config: {config_file}")
    print(f"{'='*60}")

    all_areas, default_params = load_areas_from_yaml(config_file)
    # # 批量模式下始终 headless，忽略 YAML 中的 headless 设置
    # headless = True

    merged_path_latlon = []
    merged_path_utm = []
    all_original_corners = []
    all_inner_corners = []
    utm_zone = None
    all_calib_points_utm = []
    area_path_counts = []
    area_names = []
    area_index_ranges = []

    for area in all_areas:
        i = area['index']
        name = area['name']
        calib_a = area['calib_point_a']
        calib_b = area['calib_point_b']
        calib_c = area['calib_point_c']
        param = area['param']

        print(f"\n  生成区域{i} ({name}) 的路径...")
        print(f"    标定点A: {calib_a}")
        print(f"    标定点B: {calib_b}")
        print(f"    标定点C: {calib_c}")
        print(f"    参数: start_corner={param.get('start_corner')}, "
              f"swap_wh_select={param.get('swap_wh_select')}, "
              f"reverse_end_point={param.get('reverse_end_point')}")

        path_latlon, path_utm, orig_corners, inner_corners, zone = \
            generate_cleaning_path_with_rotation_3points(
                calib_a, calib_b, calib_c, param['start_corner'], param)

        if utm_zone is None:
            utm_zone = zone

        merged_path_latlon.extend(path_latlon)
        merged_path_utm.extend(path_utm)
        all_original_corners.append(orig_corners)
        all_inner_corners.append(inner_corners)

        area_start_idx = len(merged_path_latlon) - len(path_latlon)
        area_end_idx = len(merged_path_latlon) - 1
        area_index_ranges.append((
            area_start_idx,
            area_end_idx,
            name,
            param['dense_spacing'],
        ))

        a_lon, a_lat = calib_a
        b_lon, b_lat = calib_b
        c_lon, c_lat = calib_c
        a_e, a_n, _, _ = get_utm_coords(a_lat, a_lon)
        b_e, b_n, _, _ = get_utm_coords(b_lat, b_lon)
        c_e, c_n, _, _ = get_utm_coords(c_lat, c_lon)
        all_calib_points_utm.append((a_e, a_n, b_e, b_n, c_e, c_n))
        area_path_counts.append(len(path_latlon))
        area_names.append(name)

        print(f"  区域{i} 路径生成完成，包含 {len(path_latlon)} 个点")

    print(f"\n所有区域路径合并完成，总点数: {len(merged_path_latlon)}")

    # 生成并保存密集点路径文件
    dense_latlon = []
    dense_area_names = []

    for area_idx, (start_idx, end_idx, area_name, dense_spacing) in enumerate(area_index_ranges):
        area_path_utm = merged_path_utm[start_idx:end_idx + 1]
        area_dense_utm = generate_dense_from_utm(area_path_utm, dense_spacing)

        for e, n in area_dense_utm:
            lat, lon = get_latlon_from_utm(e, n, utm_zone[0], utm_zone[1])
            dense_latlon.append((lon, lat))
            dense_area_names.append(area_name)

        if area_idx < len(area_index_ranges) - 1:
            next_start_idx = area_index_ranges[area_idx + 1][0]
            if end_idx < len(merged_path_utm) and next_start_idx < len(merged_path_utm):
                bridge_utm = [merged_path_utm[end_idx], merged_path_utm[next_start_idx]]
                bridge_dense_utm = generate_dense_from_utm(bridge_utm, dense_spacing)
                for e, n in bridge_dense_utm[1:-1]:
                    lat, lon = get_latlon_from_utm(e, n, utm_zone[0], utm_zone[1])
                    dense_latlon.append((lon, lat))
                    dense_area_names.append(f"{area_name}_mid")

    dense_headings = calculate_heading_angles(dense_latlon)

    dense_filename = os.path.join(output_dir, f"{basename}.txt")
    with open(dense_filename, "w", encoding="utf-8") as f:
        f.write("序号,经度,纬度,航向角(度)\n")
        current_area_name = None
        for global_idx, ((lon, lat), hdg, area_name) in enumerate(
                zip(dense_latlon, dense_headings, dense_area_names)):
            if area_name != current_area_name:
                f.write(f"#{area_name}\n")
                current_area_name = area_name
            f.write(f"{global_idx + 1},{lon:.8f},{lat:.8f},{hdg:.2f}\n")
    print(f"  密集点路径文件已保存: {dense_filename}，共 {len(dense_latlon)} 个点")

    # # 生成密集点UTM用于绘图
    # all_dense_utm = []
    # for lon, lat in dense_latlon:
    #     e, n, _, _ = get_utm_coords(lat, lon)
    #     all_dense_utm.append((e, n))
    #
    # # 可视化
    # plot_multi_area_path(
    #     merged_path_utm, all_original_corners, all_inner_corners, utm_zone,
    #     output_dir, basename, all_calib_points_utm, area_names,
    #     all_dense_utm, area_index_ranges, headless=headless)

    print(f"  Done: {basename}")
    return True


def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    config_files = sorted(glob.glob(os.path.join(CONFIG_DIR, "*.yaml")))
    if not config_files:
        print(f"ERROR: No config files found in {CONFIG_DIR}")
        sys.exit(1)

    print(f"Found {len(config_files)} config files")
    print(f"Output directory: {OUTPUT_DIR}")

    success = 0
    for i, config_file in enumerate(config_files):
        try:
            process_config(config_file, OUTPUT_DIR)
            success += 1
        except Exception as e:
            print(f"  FAILED: {os.path.basename(config_file)} — {e}")
            import traceback
            traceback.print_exc()

    print(f"\n{'='*60}")
    print(f"Done: {success}/{len(config_files)} succeeded")
    print(f"Output: {OUTPUT_DIR}")
    print(f"{'='*60}")


if __name__ == "__main__":
    main()
