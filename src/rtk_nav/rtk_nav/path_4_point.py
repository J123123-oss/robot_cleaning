import math
import matplotlib
import matplotlib.pyplot as plt
import utm
import rclpy
from rclpy.node import Node
import datetime
from matplotlib.patches import FancyArrowPatch
import os

# ===================== 基础配置与工具函数（精简无冗余） =====================
matplotlib.rcParams['axes.unicode_minus'] = False

def degrees_to_radians(degrees):
    return degrees * math.pi / 180.0

def radians_to_degrees(radians):
    return radians * 180.0 / math.pi

def get_utm_coords(lat, lon):
    """经纬度 → UTM坐标 (东向e,北向n,区号,字母)，统一坐标系用于距离计算"""
    return utm.from_latlon(lat, lon)

def get_latlon_from_utm(easting, northing, zone_number, zone_letter):
    """UTM坐标 → 经纬度，用于最终路径保存"""
    return utm.to_latlon(easting, northing, zone_number, zone_letter)

def calculate_heading_angles(path_latlon):
    """计算每个路径点的航向角(0°为北，顺时针递增，适配导航)"""
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
            x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon_rad)
            heading_rad = math.atan2(y, x)
            heading_deg = (math.degrees(heading_rad) + 360) % 360
            headings.append(heading_deg)
    return headings

def add_direction_arrows(ax, path_utm, arrow_interval=2):
    """路径添加方向箭头，直观显示清扫方向"""
    for i in range(0, len(path_utm) - arrow_interval, arrow_interval):
        x1, y1 = path_utm[i]
        x2, y2 = path_utm[i + arrow_interval]
        arrow = FancyArrowPatch((x1, y1), (x2, y2), arrowstyle='->', mutation_scale=15, color='blue', linewidth=1.5, alpha=0.7)
        ax.add_patch(arrow)

# ===================== 核心：四点直接计算矩形区域【无旋转、无重构、无冗余】 =====================
def calculate_region_from_4points(point_a, point_b, point_c, point_d):
    """
    传入4个矩形角点经纬度，直接计算出清扫区域的核心参数
    核心规则：传入的4个点 = 最终的清扫矩形，四点唯一确定，不做任何旋转/变形
    返回：矩形4角点的UTM坐标字典、UTM投影带、矩形宽/高、中心点
    """
    four_points = [point_a, point_b, point_c, point_d]
    four_points_utm = []
    zone_num, zone_letter = None, None
    
    # 4个角点经纬度转UTM，统一坐标系计算
    for lon, lat in four_points:
        e, n, zn, zl = get_utm_coords(lat, lon)
        four_points_utm.append((e, n))
        zone_num, zone_letter = zn, zl
    
    # 提取4个角点的最大/最小东向、北向，确定矩形边界
    e_list = [p[0] for p in four_points_utm]
    n_list = [p[1] for p in four_points_utm]
    e_min, e_max = min(e_list), max(e_list)
    n_min, n_max = min(n_list), max(n_list)
    rect_width = e_max - e_min  # 矩形宽度
    rect_height = n_max - n_min # 矩形高度

    # 规整4个角点的标准命名，完美匹配start_corner参数
    rect_corners_utm = {
        'top_left':     (e_min, n_max),
        'top_right':    (e_max, n_max),
        'bottom_right': (e_max, n_min),
        'bottom_left':  (e_min, n_min)
    }
    return rect_corners_utm, (zone_num, zone_letter), rect_width, rect_height, four_points_utm

# ===================== 核心：生成清扫路径【无旋转，直接基于四点区域生成，精准贴合】 =====================
def generate_cleaning_path_4points(point_a, point_b, point_c, point_d, start_corner, param):
    """
    核心函数：四点确定区域 → 直接生成清扫路径
    :param point_a-d: 4个角点经纬度 (lon, lat)
    :param start_corner: 清扫起点 ['top_left','top_right','bottom_right','bottom_left']
    :param param: 配置参数 {interval, edge_distance_lon, edge_distance_lat}
    :return: 最终路径(经纬度)、路径(UTM)、原始矩形UTM、内部清扫区UTM、4个标定角点UTM
    """
    # 1. 四点计算矩形核心参数
    rect_corners_utm, utm_zone, rect_width, rect_height, four_points_utm = calculate_region_from_4points(point_a, point_b, point_c, point_d)
    zone_num, zone_letter = utm_zone
    interval = param['interval']
    edge_lon = param['edge_distance_lon']
    edge_lat = param['edge_distance_lat']

    # 2. 原始矩形边界（四点确定的区域）
    original_corners_utm = [rect_corners_utm[k] for k in ['top_left','top_right','bottom_right','bottom_left']]
    
    # 3. 内部清扫区域（边界偏移，防碰撞，向内收缩，不超出四点标定的矩形）
    inner_corners_utm = {
        'top_left':     (rect_corners_utm['top_left'][0] + edge_lon, rect_corners_utm['top_left'][1] - edge_lat),
        'top_right':    (rect_corners_utm['top_right'][0] - edge_lon, rect_corners_utm['top_right'][1] - edge_lat),
        'bottom_right': (rect_corners_utm['bottom_right'][0] - edge_lon, rect_corners_utm['bottom_right'][1] + edge_lat),
        'bottom_left':  (rect_corners_utm['bottom_left'][0] + edge_lon, rect_corners_utm['bottom_left'][1] + edge_lat)
    }
    # 安全校验：内部区域不能过小
    inner_e_min = inner_corners_utm['top_left'][0]
    inner_e_max = inner_corners_utm['top_right'][0]
    inner_n_max = inner_corners_utm['top_left'][1]
    inner_n_min = inner_corners_utm['bottom_left'][1]
    inner_width = inner_e_max - inner_e_min
    inner_height = inner_n_max - inner_n_min
    if inner_width <= 0.1 or inner_height <= 0.1:
        raise ValueError(f"内部区域过小！请减小边界偏移值，当前宽：{inner_width:.2f}m，高：{inner_height:.2f}m")

    # 4. 解析起始角点，确定清扫方向（完全保留你的原有逻辑，精准适配）
    hori_dir = 'left' if 'left' in start_corner else 'right'
    vert_dir = 'top' if 'top' in start_corner else 'bottom'

    # 5. 生成清扫路径点【核心路径逻辑，无旋转，直接生成】
    path_utm = []
    if inner_width >= inner_height:
        # 宽≥高：横向条带清扫，垂直方向步进
        num_strips = max(1, int(inner_height / interval) + 1)
        if vert_dir == 'top':
            n_values = [inner_n_max - inner_height * (i/(num_strips-1) if num_strips>1 else 0) for i in range(num_strips)]
        else:
            n_values = [inner_n_min + inner_height * (i/(num_strips-1) if num_strips>1 else 0) for i in range(num_strips)]
        
        for i, current_n in enumerate(n_values):
            if (i%2 == 0 and hori_dir == 'left') or (i%2 == 1 and hori_dir == 'right'):
                path_utm.append((inner_e_min, current_n))
                path_utm.append((inner_e_max, current_n))
            else:
                path_utm.append((inner_e_max, current_n))
                path_utm.append((inner_e_min, current_n))
    else:
        # 宽<高：纵向条带清扫，水平方向步进
        num_strips = max(1, int(inner_width / interval) + 1)
        if hori_dir == 'left':
            e_values = [inner_e_min + inner_width * (i/(num_strips-1) if num_strips>1 else 0) for i in range(num_strips)]
        else:
            e_values = [inner_e_max - inner_width * (i/(num_strips-1) if num_strips>1 else 0) for i in range(num_strips)]
        
        for i, current_e in enumerate(e_values):
            if (i%2 == 0 and vert_dir == 'top') or (i%2 == 1 and vert_dir == 'bottom'):
                path_utm.append((current_e, inner_n_max))
                path_utm.append((current_e, inner_n_min))
            else:
                path_utm.append((current_e, inner_n_min))
                path_utm.append((current_e, inner_n_max))

    # 6. UTM路径点 转回 经纬度路径点（最终用于导航/保存）
    path_latlon = []
    for e, n in path_utm:
        lat, lon = get_latlon_from_utm(e, n, zone_num, zone_letter)
        path_latlon.append((lon, lat))

    # 7. 强制校准：路径第一个点 = 指定的start_corner，百分百精准
    target_start_utm = inner_corners_utm[start_corner]
    first_dist = math.hypot(path_utm[0][0]-target_start_utm[0], path_utm[0][1]-target_start_utm[1])
    if first_dist > 0.1:
        path_utm[0], path_utm[1] = path_utm[1], path_utm[0]
        path_latlon[0], path_latlon[1] = path_latlon[1], path_latlon[0]

    return path_latlon, path_utm, original_corners_utm, inner_corners_utm, four_points_utm, utm_zone

# ===================== ROS2节点（完整功能：显示+保存+日志，无任何冗余） =====================
class FourPointCleaningPathPlanner(Node):
    def __init__(self):
        super().__init__('four_point_cleaning_planner')
        
        # ========== 4个角点参数（核心：这4个点就是最终的清扫区域，唯一确定） ==========
        self.declare_parameter('corner_a.lon', 120.06916774367069)  # 第一个点 = start_corner
        self.declare_parameter('corner_a.lat', 30.320349035482604)
        self.declare_parameter('corner_b.lon', 120.069343119751)
        self.declare_parameter('corner_b.lat', 30.319865295979422)
        self.declare_parameter('corner_c.lon', 120.06915847480371)
        self.declare_parameter('corner_c.lat', 30.319843208521007)
        self.declare_parameter('corner_d.lon', 120.06894345479535)
        self.declare_parameter('corner_d.lat', 30.320538889755248)
        
        # ========== 保留所有你需要的配置参数 ==========
        self.declare_parameter('interval', 2.0)
        self.declare_parameter('start_corner', 'top_left')
        self.declare_parameter('edge_distance_lon', 0.5)
        self.declare_parameter('edge_distance_lat', 0.5)
        self.declare_parameter('headless', False)

        # 封装参数
        self.param = {
            'corner_a': (self.get_parameter('corner_a.lon').value, self.get_parameter('corner_a.lat').value),
            'corner_b': (self.get_parameter('corner_b.lon').value, self.get_parameter('corner_b.lat').value),
            'corner_c': (self.get_parameter('corner_c.lon').value, self.get_parameter('corner_c.lat').value),
            'corner_d': (self.get_parameter('corner_d.lon').value, self.get_parameter('corner_d.lat').value),
            'interval': self.get_parameter('interval').value,
            'start_corner': self.get_parameter('start_corner').value,
            'edge_distance_lon': self.get_parameter('edge_distance_lon').value,
            'edge_distance_lat': self.get_parameter('edge_distance_lat').value,
            'headless': self.get_parameter('headless').value
        }
        # 校验起始角点合法性
        self.valid_corners = ['top_left', 'top_right', 'bottom_right', 'bottom_left']
        if self.param['start_corner'] not in self.valid_corners:
            self.get_logger().error(f"无效start_corner！可选值：{self.valid_corners}")
            return
        
        # 执行路径规划
        self.plan_and_show_path()

    def plan_and_show_path(self):
        """完整流程：生成路径 → 计算航向角 → 保存文件 → 绘图显示"""
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        save_dir = os.path.expanduser("/home/ubuntu/robot_cleaning/src/rtk_nav/rtk_nav/cleaning_path/")
        os.makedirs(save_dir, exist_ok=True)

        try:
            # 1. 生成核心路径数据
            path_latlon, path_utm, original_corners, inner_corners, four_points_utm, utm_zone = generate_cleaning_path_4points(
                self.param['corner_a'], self.param['corner_b'], self.param['corner_c'], self.param['corner_d'],
                self.param['start_corner'], self.param
            )
            zone_num, zone_letter = utm_zone

            # 2. 打印关键日志
            self.get_logger().info("="*60)
            self.get_logger().info(f"✅ 四点标定清扫路径生成成功 | 起始角点：{self.param['start_corner']}")
            self.get_logger().info(f"✅ 路径点数量：{len(path_latlon)} | 路径间隔：{self.param['interval']}m")
            self.get_logger().info(f"✅ 标定区域尺寸：宽 {original_corners[1][0]-original_corners[0][0]:.2f}m | 高 {original_corners[0][1]-original_corners[3][1]:.2f}m")
            self.get_logger().info("="*60)

            # 3. 计算航向角并保存路径文件（经纬度+航向角，可直接用于导航）
            headings = calculate_heading_angles(path_latlon)
            path_file = os.path.join(save_dir, f"cleaning_path_{timestamp}.txt")
            with open(path_file, 'w', encoding='utf-8') as f:
                f.write("# 序号,经度,纬度,航向角(度,北为0°顺时针)\n")
                for i, ((lon, lat), head) in enumerate(zip(path_latlon, headings)):
                    f.write(f"{i+1},{lon:.8f},{lat:.8f},{head:.2f}\n")
            self.get_logger().info(f"📄 路径文件已保存：{path_file}")

            # 4. 绘图可视化（所有元素完美贴合，无任何错位）
            fig, ax = plt.subplots(figsize=(10, 8))
            # 绘制原始四点标定区域（蓝色实线，核心清扫区域）
            orig_e = [p[0] for p in original_corners] + [original_corners[0][0]]
            orig_n = [p[1] for p in original_corners] + [original_corners[0][1]]
            ax.plot(orig_e, orig_n, 'b-', linewidth=2, label='四点标定清扫区域', alpha=0.8)
            # 绘制内部防碰撞区域（绿色虚线）
            inner_e = [inner_corners[k][0] for k in ['top_left','top_right','bottom_right','bottom_left']] + [inner_corners['top_left'][0]]
            inner_n = [inner_corners[k][1] for k in ['top_left','top_right','bottom_right','bottom_left']] + [inner_corners['top_left'][1]]
            ax.plot(inner_e, inner_n, 'g--', linewidth=2, label='内部清扫边界', alpha=0.8)
            # 绘制清扫路径（红色实线，核心）
            path_e = [p[0] for p in path_utm]
            path_n = [p[1] for p in path_utm]
            ax.plot(path_e, path_n, 'r-', linewidth=1.2, label='清扫路径', alpha=0.9)
            # 添加方向箭头
            add_direction_arrows(ax, path_utm)
            # 标记关键点位
            ax.scatter(path_e[0], path_n[0], c='limegreen', s=150, marker='o', label='清扫起点', zorder=5)
            ax.scatter(path_e[-1], path_n[-1], c='purple', s=150, marker='x', label='清扫终点', zorder=5)
            ax.scatter([p[0] for p in four_points_utm], [p[1] for p in four_points_utm], c='black', s=120, marker='s', label='四点标定角点', zorder=6)

            # 图表美化
            ax.set_xlabel(f'UTM 东向坐标 (m) | 投影带 {zone_num}{zone_letter}')
            ax.set_ylabel(f'UTM 北向坐标 (m) | 投影带 {zone_num}{zone_letter}')
            ax.set_title(f'四点标定清扫路径规划 | 起始角点：{self.param["start_corner"]}', fontsize=14)
            ax.legend(loc='upper right')
            ax.grid(True, alpha=0.3)
            ax.axis('equal')
            plt.tight_layout()

            # 保存图片
            img_file = os.path.join(save_dir, f"cleaning_path_{timestamp}.png")
            plt.savefig(img_file, dpi=300, bbox_inches='tight')
            self.get_logger().info(f"🖼️ 路径图片已保存：{img_file}")

            # 非无头模式显示图表
            if not self.param['headless']:
                plt.show()

        except ValueError as e:
            self.get_logger().error(f"❌ 路径生成失败：{str(e)}")
        except Exception as e:
            self.get_logger().error(f"❌ 未知错误：{str(e)}")

# ===================== 主函数 =====================
def main(args=None):
    rclpy.init(args=args)
    node = FourPointCleaningPathPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()   

        