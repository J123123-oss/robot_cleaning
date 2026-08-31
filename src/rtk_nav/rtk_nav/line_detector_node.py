#!/usr/bin/env python3
"""RTK 路径感知的栅格线视觉纠偏节点。"""

import json
import math
import threading
import time
from queue import Empty, Full, Queue

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool, Float32, String


WAYPOINT_MOVE_STATE = 'WAYPOINT_MOVE'


def wrap180(angle_deg):
    """将有向角归一化到 [-180, 180)。"""
    return (float(angle_deg) + 180.0) % 360.0 - 180.0


def parse_nav_state_message(payload):
    """解析 ``/rtk/nav_state`` 的 JSON 或纯文本状态名称。"""
    if payload is None:
        return None
    text = str(payload).strip()
    if not text:
        return None
    try:
        decoded = json.loads(text)
    except (TypeError, ValueError, json.JSONDecodeError):
        decoded = text
    if isinstance(decoded, dict):
        decoded = decoded.get('nav_state')
    if not isinstance(decoded, str):
        return None
    state = decoded.strip().upper()
    return state or None


def nav_state_allows_lateral_output(nav_state):
    """仅在 RTK 正处于航点移动状态时允许横向偏移参与控制。"""
    return str(nav_state).strip().upper() == WAYPOINT_MOVE_STATE


def resolve_effective_path_axis_image(
    path_context_valid,
    last_path_context_time,
    path_context_timeout_sec,
    path_direction_deg,
    vehicle_heading_deg,
    camera_angle_offset,
    fallback_path_axis_image_deg,
    now=None,
):
    """解析图像中的运行方向，并返回方向角及其数据来源。"""
    if now is None:
        now = time.monotonic()
    context_age = float(now) - float(last_path_context_time)
    context_fresh = (
        bool(path_context_valid)
        and 0.0 <= context_age <= float(path_context_timeout_sec)
    )
    if context_fresh:
        relative_path_heading = wrap180(
            float(path_direction_deg) - float(vehicle_heading_deg)
        )
        return (
            wrap180(90.0 - relative_path_heading + float(camera_angle_offset)),
            'rtk',
        )
    return wrap180(float(fallback_path_axis_image_deg)), 'fallback'


def format_run_axis_debug(
    axis_label,
    image_axis_deg,
    source,
    path_direction_deg=None,
    vehicle_heading_deg=None,
):
    """格式化运行方向诊断，明确区分 RTK 航向和图像轴角度。"""
    debug = f'axis={axis_label} image_axis={float(image_axis_deg):.1f}deg'
    if source == 'rtk':
        try:
            path_heading = float(path_direction_deg)
            vehicle_heading = float(vehicle_heading_deg)
        except (TypeError, ValueError):
            path_heading = float('nan')
            vehicle_heading = float('nan')
        if math.isfinite(path_heading) and math.isfinite(vehicle_heading):
            relative_heading = wrap180(path_heading - vehicle_heading)
            debug += (
                f' path_heading={path_heading:.1f}deg'
                f' vehicle_heading={vehicle_heading:.1f}deg'
                f' relative_heading={relative_heading:.1f}deg'
            )
    return f'{debug} source={source}'


def undirected_angle(angle_deg):
    """将无向直线角归一化到 [-90, 90)。"""
    return (float(angle_deg) + 90.0) % 180.0 - 90.0


def undirected_angle_distance(angle_a, angle_b):
    """返回两条无向直线的最小夹角。"""
    return abs(undirected_angle(float(angle_a) - float(angle_b)))


def weighted_line_angle(lines):
    """使用线段长度权重计算无向线组的平均方向。"""
    if not lines:
        return None

    sin_sum = 0.0
    cos_sum = 0.0
    for line in lines:
        length = float(line[4])
        angle = float(line[5])
        if not math.isfinite(length) or not math.isfinite(angle):
            return None
        doubled = math.radians(angle * 2.0)
        sin_sum += length * math.sin(doubled)
        cos_sum += length * math.cos(doubled)

    if abs(sin_sum) < 1e-9 and abs(cos_sum) < 1e-9:
        return None
    return undirected_angle(math.degrees(0.5 * math.atan2(sin_sum, cos_sum)))


def line_salience_score(line):
    """根据线段长度、粗细和白色支持度计算单线选择分数。"""
    if len(line) < 10:
        return float('-inf')
    try:
        length = float(line[4])
        width = float(line[8])
        support = float(line[9])
    except (IndexError, TypeError, ValueError):
        return float('-inf')
    if not all(math.isfinite(value) for value in (length, width, support)):
        return float('-inf')
    if length <= 0.0 or width <= 0.0 or support < 0.0:
        return float('-inf')
    score = length * width * min(1.0, support)
    return score if math.isfinite(score) else float('-inf')


def select_most_salient_line(lines):
    """从已按运行方向筛选的线组中选择最明显的一条线。"""
    best_line = None
    best_key = None
    for line in lines:
        score = line_salience_score(line)
        if not math.isfinite(score):
            continue
        try:
            support = float(line[9])
            length = float(line[4])
            width = float(line[8])
        except (IndexError, TypeError, ValueError):
            continue
        key = (score, min(1.0, support), length, width)
        # Preserve the first line on an exact tie so the result is stable.
        if best_key is None or key > best_key:
            best_line = line
            best_key = key
    return best_line


def select_coarse_white_lines(
    lines, min_length_px, min_width_px, min_white_support
):
    """保留长度、白色带宽度和白色支持度都足够的线段。"""
    selected = []
    for line in lines:
        if len(line) < 10:
            continue
        try:
            length = float(line[4])
            width = float(line[8])
            support = float(line[9])
        except (TypeError, ValueError):
            continue
        if not all(math.isfinite(value) for value in (length, width, support)):
            continue
        if (
            length >= float(min_length_px)
            and width >= float(min_width_px)
            and support >= float(min_white_support)
        ):
            selected.append(line)
    return selected


def merge_nearby_line_records(lines, axis_angle_deg, max_normal_gap_px):
    """沿路径轴法向合并同一粗白带的两条边缘线。"""
    if not lines:
        return []
    if not math.isfinite(float(axis_angle_deg)) or not math.isfinite(
        float(max_normal_gap_px)
    ) or float(max_normal_gap_px) < 0.0:
        return []

    valid_lines = []
    for line in lines:
        try:
            values = [float(line[index]) for index in range(10)]
        except (IndexError, TypeError, ValueError):
            continue
        if all(math.isfinite(value) for value in values):
            valid_lines.append(line)
    if not valid_lines:
        return []

    axis_rad = math.radians(float(axis_angle_deg))
    axis_x = math.cos(axis_rad)
    axis_y = math.sin(axis_rad)
    normal_x = -math.sin(axis_rad)
    normal_y = math.cos(axis_rad)

    def normalize_record(record):
        """按运行方向重新排列线段端点，保证轴向顺序一致。"""
        x1, y1, x2, y2 = [float(record[index]) for index in range(4)]
        first_axis_projection = x1 * axis_x + y1 * axis_y
        second_axis_projection = x2 * axis_x + y2 * axis_y
        if second_axis_projection < first_axis_projection:
            x1, y1, x2, y2 = x2, y2, x1, y1
        return (
            x1,
            y1,
            x2,
            y2,
            float(record[4]),
            float(record[5]),
            float(record[6]),
            float(record[7]),
            float(record[8]),
            float(record[9]),
        )

    def axis_interval(record):
        """计算线段在运行方向轴上的投影区间。"""
        first = float(record[0]) * axis_x + float(record[1]) * axis_y
        second = float(record[2]) * axis_x + float(record[3]) * axis_y
        return min(first, second), max(first, second)

    ordered = sorted(
        (normalize_record(line) for line in valid_lines),
        key=lambda line: float(line[6]) * normal_x + float(line[7]) * normal_y,
    )
    merged = []
    cluster = [ordered[0]]
    previous_projection = (
        float(ordered[0][6]) * normal_x + float(ordered[0][7]) * normal_y
    )
    cluster_axis_start, cluster_axis_end = axis_interval(ordered[0])

    def make_record(records):
        """将同一粗线的多条边缘记录合并为一条加权记录。"""
        total_length = sum(float(record[4]) for record in records)
        if total_length <= 0.0 or not math.isfinite(total_length):
            return records[0]
        weights = [float(record[4]) / total_length for record in records]
        endpoints = [
            sum(weight * float(record[index]) for weight, record in zip(weights, records))
            for index in range(4)
        ]
        x1, y1, x2, y2 = endpoints
        length = math.hypot(x2 - x1, y2 - y1)
        angle = (
            math.degrees(math.atan2(y2 - y1, x2 - x1)) + 90.0
        ) % 180.0 - 90.0
        center_x = sum(weight * float(record[6]) for weight, record in zip(weights, records))
        center_y = sum(weight * float(record[7]) for weight, record in zip(weights, records))
        width = max(float(record[8]) for record in records)
        support = sum(weight * float(record[9]) for weight, record in zip(weights, records))
        return (
            int(round(x1)),
            int(round(y1)),
            int(round(x2)),
            int(round(y2)),
            length,
            angle,
            center_x,
            center_y,
            width,
            support,
        )

    for line in ordered[1:]:
        projection = float(line[6]) * normal_x + float(line[7]) * normal_y
        line_axis_start, line_axis_end = axis_interval(line)
        axis_intervals_join = (
            line_axis_start < cluster_axis_end
            and cluster_axis_start < line_axis_end
        )
        if (
            projection - previous_projection <= float(max_normal_gap_px)
            and axis_intervals_join
        ):
            cluster.append(line)
            cluster_axis_start = min(cluster_axis_start, line_axis_start)
            cluster_axis_end = max(cluster_axis_end, line_axis_end)
        else:
            merged.append(make_record(cluster))
            cluster = [line]
            cluster_axis_start, cluster_axis_end = line_axis_start, line_axis_end
        previous_projection = projection
    merged.append(make_record(cluster))
    return merged


def select_boundary_pair(lines, axis_angle_deg, width, height, max_gap_px):
    """选择图像中心两侧最近的路径边界线，并返回投影信息。"""
    if len(lines) < 2:
        return None
    if not all(
        math.isfinite(float(value))
        for value in (axis_angle_deg, width, height, max_gap_px)
    ):
        return None

    axis_rad = math.radians(float(axis_angle_deg))
    normal_x = -math.sin(axis_rad)
    normal_y = math.cos(axis_rad)
    center_projection = (
        (float(width) / 2.0) * normal_x
        + (float(height) / 2.0) * normal_y
    )

    left_candidates = []
    right_candidates = []
    for line in lines:
        projection = float(line[6]) * normal_x + float(line[7]) * normal_y
        if not math.isfinite(projection):
            continue
        if projection < center_projection:
            left_candidates.append((projection, line))
        elif projection > center_projection:
            right_candidates.append((projection, line))

    if not left_candidates or not right_candidates:
        return None

    left_projection, left_line = max(left_candidates, key=lambda item: item[0])
    right_projection, right_line = min(right_candidates, key=lambda item: item[0])
    gap = right_projection - left_projection
    if gap <= 0.0 or gap > float(max_gap_px):
        return None
    return left_line, right_line, left_projection, right_projection, center_projection


def line_normal_offset_at_reference(
    line, axis_angle_deg, width, height, axis_offset_px=0.0
):
    """计算无限拟合直线在固定路径截面上的法向偏移。"""
    try:
        x1, y1, x2, y2 = [float(line[index]) for index in range(4)]
        values = [
            float(value)
            for value in (axis_angle_deg, width, height, axis_offset_px)
        ]
    except (IndexError, TypeError, ValueError):
        return float('nan')
    if not all(math.isfinite(value) for value in (x1, y1, x2, y2, *values)):
        return float('nan')

    axis_rad = math.radians(float(axis_angle_deg))
    axis_x = math.cos(axis_rad)
    axis_y = math.sin(axis_rad)
    normal_x = -axis_y
    normal_y = axis_x
    center_x = float(width) / 2.0
    center_y = float(height) / 2.0
    reference_x = center_x + axis_x * float(axis_offset_px)
    reference_y = center_y + axis_y * float(axis_offset_px)
    line_dx = x2 - x1
    line_dy = y2 - y1
    denominator = normal_x * line_dy - normal_y * line_dx
    if abs(denominator) < 1e-9:
        return float('nan')

    numerator = (
        (x1 - reference_x) * line_dy
        - (y1 - reference_y) * line_dx
    )
    offset = numerator / denominator
    return offset if math.isfinite(offset) else float('nan')


def select_line_for_tracking(
    lines,
    axis_angle_deg,
    width,
    height,
    previous_offset_px=None,
    max_jump_px=float('inf'),
):
    """按法向位置关联当前候选线，防止切换到相邻平行线。"""
    if not lines:
        return None
    if previous_offset_px is not None:
        try:
            previous_offset_px = float(previous_offset_px)
            max_jump_px = float(max_jump_px)
        except (TypeError, ValueError):
            return None
        if (
            not math.isfinite(previous_offset_px)
            or not math.isfinite(max_jump_px)
            or max_jump_px < 0.0
        ):
            return None

    candidates = []
    for line in lines:
        offset = line_normal_offset_at_reference(
            line,
            axis_angle_deg,
            width,
            height,
            0.0,
        )
        score = line_salience_score(line)
        if math.isfinite(offset) and math.isfinite(score):
            candidates.append((line, offset, score))
    if not candidates:
        return None

    if previous_offset_px is None:
        line, offset, _ = max(
            candidates,
            key=lambda item: item[2],
        )
        return line, offset, 0.0

    line, offset, _ = min(
        candidates,
        key=lambda item: (
            abs(item[1] - previous_offset_px),
            -item[2],
        ),
    )
    jump_px = abs(offset - previous_offset_px)
    if jump_px > max_jump_px:
        return None
    return line, offset, jump_px


def update_line_tracking_state(
    lines,
    axis_angle_deg,
    width,
    height,
    previous_offset_px=None,
    missed_frames=0,
    max_jump_px=float('inf'),
    max_missed_frames=2,
):
    """更新单线跟踪状态，并拒绝跳到相邻平行线的候选结果。

    返回 ``(line, offset, missed_frames, status)``。没有历史锚点时只允许
    选择最明显线；有历史锚点时无论是否已经进入重获状态，都必须通过同一
    法向跳变门限，因此短时遮挡或重新看到画面时不会改跟踪另一条栅格线。
    """
    try:
        missed_frames = max(0, int(missed_frames))
        max_missed_frames = max(0, int(max_missed_frames))
    except (TypeError, ValueError):
        return None, previous_offset_px, 0, 'invalid'

    if previous_offset_px is None:
        selected = select_most_salient_line(lines)
        if selected is None:
            missed_frames += 1
            return None, None, missed_frames, 'unlocked'
        offset = line_normal_offset_at_reference(
            selected, axis_angle_deg, width, height, 0.0
        )
        if not math.isfinite(offset):
            missed_frames += 1
            return None, None, missed_frames, 'unlocked'
        return selected, offset, 0, 'acquired'

    selected = select_line_for_tracking(
        lines,
        axis_angle_deg,
        width,
        height,
        previous_offset_px=previous_offset_px,
        max_jump_px=max_jump_px,
    )
    if selected is None:
        missed_frames += 1
        if lines:
            # Candidates exist but none is close enough to the locked line.
            status = 'rejected'
        else:
            status = 'reacquire' if missed_frames >= max_missed_frames else 'lost'
        return None, previous_offset_px, missed_frames, status

    line, offset, _ = selected
    status = 'reacquired' if missed_frames > 0 else 'locked'
    return line, offset, 0, status


def select_reference_line(
    lines,
    axis_angle_deg,
    width,
    height,
    target_offset_px,
    max_error_px,
    axis_offset_px=0.0,
):
    """选择最接近标定路径参考位置的单条平行线。

    ``target_offset_px`` 是目标线路相对图像中心的法向投影，不能假定
    目标路线位于两条检测线的中点。
    """
    if not lines:
        return None
    if not all(
        math.isfinite(float(value))
        for value in (
            axis_angle_deg,
            width,
            height,
            target_offset_px,
            max_error_px,
            axis_offset_px,
        )
    ) or float(max_error_px) < 0.0:
        return None

    candidates = []
    for line in lines:
        projection = line_normal_offset_at_reference(
            line,
            axis_angle_deg,
            width,
            height,
            axis_offset_px,
        )
        if math.isfinite(projection):
            candidates.append((abs(projection - target_offset_px), line, projection))
    if not candidates:
        return None

    distance, line, projection = min(candidates, key=lambda item: item[0])
    if distance > float(max_error_px):
        return None
    return line, projection, float(target_offset_px)


class GridLineDetector(Node):
    """根据 RTK 或回退图像方向选择一条最明显平行线并纠偏。"""

    def __init__(self):
        """初始化订阅、发布、检测参数、最新帧缓存和检测定时器。"""
        super().__init__('grid_line_detector')

        self.bridge = CvBridge()
        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.image_sub = self.create_subscription(
            CompressedImage,
            '/camera/color/image_compressed',
            self.image_callback,
            image_qos,
        )
        self.path_context_sub = self.create_subscription(
            Vector3,
            '/rtk/visual_path_context',
            self.path_context_callback,
            10,
        )
        self.nav_state_sub = self.create_subscription(
            String,
            '/rtk/nav_state',
            self.nav_state_callback,
            10,
        )
        self.path_reference_sub = self.create_subscription(
            Vector3,
            '/rtk/visual_path_reference',
            self.path_reference_callback,
            10,
        )

        self.angle_pub = self.create_publisher(
            Vector3, '/grid_line/angle_deviation', 10
        )
        self.confidence_pub = self.create_publisher(
            Float32, '/grid_line/detection_confidence', 10
        )
        self.heading_valid_pub = self.create_publisher(
            Bool, '/grid_line/heading_valid', 10
        )
        self.lateral_valid_pub = self.create_publisher(
            Bool, '/grid_line/lateral_valid', 10
        )
        self.heading_confidence_pub = self.create_publisher(
            Float32, '/grid_line/heading_confidence', 10
        )
        self.lateral_confidence_pub = self.create_publisher(
            Float32, '/grid_line/lateral_confidence', 10
        )
        self.run_axis_pub = self.create_publisher(
            String, '/grid_line/run_axis', 10
        )
        self.run_axis_debug_pub = self.create_publisher(
            String, '/grid_line/run_axis_debug', 10
        )
        debug_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.image_pub = self.create_publisher(
            Image, '/grid_line/detected_image', debug_qos
        )
        self.gray_pub = self.create_publisher(
            Image, '/grid_line/gray_image', debug_qos
        )
        self.binary_pub = self.create_publisher(
            Image, '/grid_line/binary_image', debug_qos
        )
        self.edges_pub = self.create_publisher(
            Image, '/grid_line/edges_image', debug_qos
        )

        # 相机安装和像素到地面距离换算参数。
        # 相机光轴相对车体/图像坐标的角度修正，单位为度。
        self.declare_parameter('camera_angle_offset', 0.0)
        # 相机离地高度，单位为米。
        self.declare_parameter('camera_height', 0.5)
        # 相机俯视角，单位为度，取值范围为 (0, 90]。
        self.declare_parameter('camera_pitch_deg', 90.0)
        # 相机等效焦距，单位为像素。
        self.declare_parameter('focal_length_px', 132.0)
        # 兼容旧配置的最小线段数量；当前单线模式不以线段数量作为有效条件。
        self.declare_parameter('min_line_count', 2)
        # 是否启用视觉纠偏；关闭时发布无效结果。
        self.declare_parameter('enable_visual_correction', True)
        # 是否跳过 RTK 路径方向有效性门控，便于无 RTK 调试。
        self.declare_parameter('bypass_path_context_gate', False)
        # 没有新鲜 RTK 方向时使用的图像运行轴，单位为度。
        # 图像坐标约定为 0 度向右、90 度向下。
        self.declare_parameter('fallback_path_axis_image_deg', -5.0)
        # 已停用的旧参数声明，仅保留以兼容历史配置文件。
        # self.declare_parameter('target_line_offset_m', float('nan'))
        # 目标参考线相对图像中心的横向偏移，单位为米；当前单线模式以中心为零点。
        self.declare_parameter('target_line_offset_m', 0.0)
        # 目标参考线匹配容差，单位为米；保留用于兼容历史配置。
        self.declare_parameter('target_line_match_tolerance_m', 0.5)
        # 测量截面沿运行轴的偏移，单位为像素；0 表示图像中心截面。
        self.declare_parameter('reference_axis_offset_px', 0.0)
        # 检测线段与运行轴的最大允许夹角，单位为度。
        self.declare_parameter('line_angle_tolerance_deg', 8.0)
        # RTK 路径方向消息的有效保持时间，单位为秒。
        self.declare_parameter('path_context_timeout_sec', 0.5)
        # 兼容旧双边界模式的最大法向间距，单位为像素。
        self.declare_parameter('boundary_pair_max_gap_px', 1200.0)
        # 连续检测到有效线段的帧数，达到后才确认检测结果。
        self.declare_parameter('reacquire_frames', 3)
        # 是否启用单线位置跟踪；启用后按上一帧法向位置关联候选线。
        self.declare_parameter('line_tracking_enabled', True)
        # 相邻两帧允许的最大法向跳变，单位为像素；超出即拒绝本帧。
        self.declare_parameter('max_line_tracking_jump_px', 30.0)
        # 连续丢失达到该帧数后进入受限重获状态，仍以最后有效位置为锚点。
        self.declare_parameter('max_line_tracking_missed_frames', 2)
        # HSV 中白色线条的最小明度阈值，取值范围为 0 到 255。
        self.declare_parameter('white_line_value_threshold', 170.0)
        # HSV 中白色线条的最大饱和度阈值，取值范围为 0 到 255。
        self.declare_parameter('white_line_saturation_max', 100.0)
        # 粗白线候选的最小线段长度，单位为像素。
        self.declare_parameter('coarse_line_min_length_px', 50.0)
        # 粗白线候选的最小估计宽度，单位为像素。
        self.declare_parameter('coarse_line_min_width_px', 3.0)
        # 粗白线候选沿线被白色掩膜支持的最小比例，范围为 0 到 1。
        self.declare_parameter('coarse_line_min_support', 0.3)
        # 同一粗白线边缘合并时允许的法向间隙，单位为像素。
        self.declare_parameter('coarse_line_merge_gap_px', 100.0)
        # 沿候选线法向扫描白色带宽度时的半窗口宽度，单位为像素。
        self.declare_parameter('white_line_scan_half_width_px', 28.0)
        # 检测定时器频率，单位为 FPS；只处理最新压缩图像帧。
        self.declare_parameter('detection_fps', 30.0)
        # 是否发布检测标注图、灰度图、二值图和边缘图调试话题。
        self.declare_parameter('publish_debug_images', True)

        self.camera_angle_offset = float(
            self.get_parameter('camera_angle_offset').value
        )
        self.camera_height = float(self.get_parameter('camera_height').value)
        self.camera_pitch_deg = float(
            self.get_parameter('camera_pitch_deg').value
        )
        self.focal_length_px = float(
            self.get_parameter('focal_length_px').value
        )
        self.min_line_count = int(self.get_parameter('min_line_count').value)
        self.enable_visual_correction = bool(
            self.get_parameter('enable_visual_correction').value
        )
        self.bypass_path_context_gate = bool(
            self.get_parameter('bypass_path_context_gate').value
        )
        self.fallback_path_axis_image_deg = float(
            self.get_parameter('fallback_path_axis_image_deg').value
        )
        self.target_line_offset_m = float(
            self.get_parameter('target_line_offset_m').value
        )
        self.target_line_match_tolerance_m = float(
            self.get_parameter('target_line_match_tolerance_m').value
        )
        self.reference_axis_offset_px = float(
            self.get_parameter('reference_axis_offset_px').value
        )
        self.line_angle_tolerance_deg = float(
            self.get_parameter('line_angle_tolerance_deg').value
        )
        self.path_context_timeout_sec = float(
            self.get_parameter('path_context_timeout_sec').value
        )
        self.boundary_pair_max_gap_px = float(
            self.get_parameter('boundary_pair_max_gap_px').value
        )
        self.reacquire_frames = max(
            1, int(self.get_parameter('reacquire_frames').value)
        )
        self.line_tracking_enabled = bool(
            self.get_parameter('line_tracking_enabled').value
        )
        self.max_line_tracking_jump_px = float(
            self.get_parameter('max_line_tracking_jump_px').value
        )
        self.max_line_tracking_missed_frames = max(
            0,
            int(self.get_parameter('max_line_tracking_missed_frames').value),
        )
        self.white_line_value_threshold = float(
            self.get_parameter('white_line_value_threshold').value
        )
        self.white_line_saturation_max = float(
            self.get_parameter('white_line_saturation_max').value
        )
        self.coarse_line_min_length_px = float(
            self.get_parameter('coarse_line_min_length_px').value
        )
        self.coarse_line_min_width_px = float(
            self.get_parameter('coarse_line_min_width_px').value
        )
        self.coarse_line_min_support = float(
            self.get_parameter('coarse_line_min_support').value
        )
        self.coarse_line_merge_gap_px = float(
            self.get_parameter('coarse_line_merge_gap_px').value
        )
        self.white_line_scan_half_width_px = float(
            self.get_parameter('white_line_scan_half_width_px').value
        )
        self.detection_fps = float(self.get_parameter('detection_fps').value)
        self.publish_debug_images_enabled = bool(
            self.get_parameter('publish_debug_images').value
        )

        if (
            not math.isfinite(self.path_context_timeout_sec)
            or self.path_context_timeout_sec <= 0.0
        ):
            raise ValueError('path_context_timeout_sec must be finite and > 0')
        if (
            not math.isfinite(self.target_line_match_tolerance_m)
            or self.target_line_match_tolerance_m <= 0.0
        ):
            raise ValueError(
                'target_line_match_tolerance_m must be finite and > 0'
            )
        if not math.isfinite(self.reference_axis_offset_px):
            raise ValueError('reference_axis_offset_px must be finite')
        if (
            not math.isfinite(self.max_line_tracking_jump_px)
            or self.max_line_tracking_jump_px < 0.0
        ):
            raise ValueError(
                'max_line_tracking_jump_px must be finite and >= 0'
            )
        if (
            not math.isfinite(self.camera_height)
            or self.camera_height <= 0.0
        ):
            raise ValueError('camera_height must be finite and > 0')
        if (
            not math.isfinite(self.focal_length_px)
            or self.focal_length_px <= 0.0
        ):
            raise ValueError('focal_length_px must be finite and > 0')
        if (
            not math.isfinite(self.camera_pitch_deg)
            or not 0.0 < self.camera_pitch_deg <= 90.0
        ):
            raise ValueError(
                'camera_pitch_deg must be finite and in (0, 90]'
            )
        if (
            not math.isfinite(self.white_line_value_threshold)
            or not 0.0 <= self.white_line_value_threshold <= 255.0
        ):
            raise ValueError('white_line_value_threshold must be in [0, 255]')
        if (
            not math.isfinite(self.white_line_saturation_max)
            or not 0.0 <= self.white_line_saturation_max <= 255.0
        ):
            raise ValueError('white_line_saturation_max must be in [0, 255]')
        if (
            not math.isfinite(self.coarse_line_min_length_px)
            or self.coarse_line_min_length_px <= 0.0
        ):
            raise ValueError('coarse_line_min_length_px must be finite and > 0')
        if (
            not math.isfinite(self.coarse_line_min_width_px)
            or self.coarse_line_min_width_px <= 0.0
        ):
            raise ValueError('coarse_line_min_width_px must be finite and > 0')
        if (
            not math.isfinite(self.coarse_line_min_support)
            or not 0.0 <= self.coarse_line_min_support <= 1.0
        ):
            raise ValueError('coarse_line_min_support must be finite in [0, 1]')
        if (
            not math.isfinite(self.coarse_line_merge_gap_px)
            or self.coarse_line_merge_gap_px < 0.0
        ):
            raise ValueError('coarse_line_merge_gap_px must be finite and >= 0')
        if (
            not math.isfinite(self.white_line_scan_half_width_px)
            or self.white_line_scan_half_width_px <= 0.0
        ):
            raise ValueError(
                'white_line_scan_half_width_px must be finite and > 0'
            )
        if not math.isfinite(self.detection_fps) or self.detection_fps <= 0.0:
            raise ValueError('detection_fps must be finite and > 0')
        if not math.isfinite(self.fallback_path_axis_image_deg):
            raise ValueError('fallback_path_axis_image_deg must be finite')
        self.fallback_path_axis_image_deg = wrap180(
            self.fallback_path_axis_image_deg
        )

        self.path_direction_deg = 0.0
        self.vehicle_heading_deg = 0.0
        self.path_context_valid = False
        self.last_path_context_time = 0.0
        self.last_path_direction_deg = None
        self.last_path_relative_heading_deg = None
        # 横向纠偏只允许在 RTK 的实际航点移动状态中生效。
        self.nav_state = None
        self.path_reference_lateral_m = 0.0
        self.path_reference_projection_ratio = 0.0
        self.path_reference_valid = False
        self.last_path_reference_time = 0.0
        self.valid_streak = 0
        self.heading_valid_streak = 0
        self.lateral_valid_streak = 0
        # Tracking uses a signed normal offset in the directed image axis.
        # Keep the last valid anchor across short occlusions, but never use a
        # rejected candidate to move that anchor.
        self.tracked_line_offset_px = None
        self.last_valid_line_offset_px = None
        self.line_tracking_missed_frames = 0
        self.line_tracking_reacquire = False
        self.line_tracking_status = 'unlocked'
        self.last_tracking_axis_image_deg = None
        self.last_tracking_axis_source = None

        self.latest_frame_lock = threading.Lock()
        self.latest_compressed_data = None
        self.debug_frame_queue = Queue(maxsize=1)
        self.debug_worker_stop = threading.Event()
        self.debug_worker = None
        if self.publish_debug_images_enabled:
            self.debug_worker = threading.Thread(
                target=self.debug_image_worker,
                name='grid-line-debug-publisher',
                daemon=True,
            )
            self.debug_worker.start()
        self.timer = self.create_timer(1.0 / self.detection_fps, self.timer_callback)

    def timer_callback(self):
        """取出最新 JPEG 帧，完成解码、检测并发布全部视觉结果。"""
        with self.latest_frame_lock:
            compressed_data = self.latest_compressed_data
            self.latest_compressed_data = None
        if compressed_data is None:
            return

        try:
            encoded = np.frombuffer(compressed_data, dtype=np.uint8)
            image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
            if image is None:
                raise ValueError('JPEG 解码失败')
            (
                angle,
                lateral_m,
                detected,
                confidence,
                heading_validity,
                lateral_validity,
                heading_confidence_value,
                lateral_confidence_value,
            ) = (
                self.detect_and_draw_grid_lines(image)
            )

            result = Vector3()
            result.x = float(angle)
            result.y = float(lateral_m)
            result.z = 1.0 if detected else 0.0
            self.angle_pub.publish(result)

            confidence_msg = Float32()
            confidence_msg.data = float(confidence)
            self.confidence_pub.publish(confidence_msg)

            heading_valid_msg = Bool()
            heading_valid_msg.data = bool(heading_validity)
            self.heading_valid_pub.publish(heading_valid_msg)

            lateral_valid_msg = Bool()
            lateral_valid_msg.data = bool(lateral_validity)
            self.lateral_valid_pub.publish(lateral_valid_msg)

            heading_confidence_msg = Float32()
            heading_confidence_msg.data = float(heading_confidence_value)
            self.heading_confidence_pub.publish(heading_confidence_msg)

            lateral_confidence_msg = Float32()
            lateral_confidence_msg.data = float(lateral_confidence_value)
            self.lateral_confidence_pub.publish(lateral_confidence_msg)

            if (
                lateral_validity
                and self.path_reference_valid
                and time.monotonic() - self.last_path_reference_time
                <= self.path_context_timeout_sec
            ):
                delta = lateral_m - self.path_reference_lateral_m
                self.get_logger().debug(
                    f'RTK reference lateral={self.path_reference_lateral_m:.3f}m '
                    f'projection={self.path_reference_projection_ratio:.3f} '
                    f'visual lateral={lateral_m:.3f}m delta={delta:.3f}m',
                    throttle_duration_sec=1.0,
                )
        except Exception as exc:
            self.get_logger().error(
                f'压缩图像处理错误: {exc}',
                throttle_duration_sec=1.0,
            )
            self.reset_line_tracking()
            self.publish_invalid()

    def path_context_callback(self, msg):
        """接收 RTK 当前路径方向和车体航向。"""
        try:
            path_direction = float(msg.x)
            vehicle_heading = float(msg.y)
            context_validity = float(msg.z)
            valid = (
                math.isfinite(path_direction)
                and math.isfinite(vehicle_heading)
                and math.isfinite(context_validity)
                and context_validity >= 0.5
            )
        except (AttributeError, TypeError, ValueError):
            valid = False

        if not valid:
            self.path_context_valid = False
            self.last_path_direction_deg = None
            self.last_path_relative_heading_deg = None
            self.reset_line_tracking()
            return

        relative_path_heading = wrap180(path_direction - vehicle_heading)
        if (
            self.last_path_direction_deg is not None
            and abs(wrap180(path_direction - self.last_path_direction_deg)) > 20.0
        ):
            self.reset_line_tracking()
        if (
            self.last_path_relative_heading_deg is not None
            and abs(
                wrap180(
                    relative_path_heading
                    - self.last_path_relative_heading_deg
                )
            )
            > 20.0
        ):
            self.reset_line_tracking()

        self.path_direction_deg = path_direction
        self.vehicle_heading_deg = vehicle_heading
        self.last_path_direction_deg = path_direction
        self.last_path_relative_heading_deg = relative_path_heading
        self.path_context_valid = True
        self.last_path_context_time = time.monotonic()

    def nav_state_callback(self, msg):
        """接收 RTK 导航状态，并在状态切换时清除视觉跟踪锚点。"""
        state = parse_nav_state_message(getattr(msg, 'data', None))
        if state is None:
            self.nav_state = None
            self.reset_line_tracking()
            return
        if state != self.nav_state:
            self.reset_line_tracking()
        self.nav_state = state

    def path_reference_callback(self, msg):
        """接收 RTK 当前路径段的横向参考，仅用于视觉结果评估。"""
        try:
            lateral_m = float(msg.x)
            projection_ratio = float(msg.y)
            validity = float(msg.z)
            valid = (
                math.isfinite(lateral_m)
                and math.isfinite(projection_ratio)
                and math.isfinite(validity)
                and validity >= 0.5
            )
        except (AttributeError, TypeError, ValueError):
            valid = False

        if not valid:
            self.path_reference_valid = False
            return

        self.path_reference_lateral_m = lateral_m
        self.path_reference_projection_ratio = projection_ratio
        self.path_reference_valid = True
        self.last_path_reference_time = time.monotonic()

    def reset_reacquisition(self):
        """清除当前连续有效帧，避免沿用旧方向检测结果。"""
        self.valid_streak = 0
        self.heading_valid_streak = 0
        self.lateral_valid_streak = 0

    def reset_line_tracking(self):
        """清除单线位置锚点和连续有效帧，等待新方向下重新锁定。"""
        self.reset_reacquisition()
        self.tracked_line_offset_px = None
        self.last_valid_line_offset_px = None
        self.line_tracking_missed_frames = 0
        self.line_tracking_reacquire = False
        self.line_tracking_status = 'unlocked'
        self.last_tracking_axis_image_deg = None
        self.last_tracking_axis_source = None

    def update_tracking_axis(self, directed_path_axis_image, source):
        """检测运行方向来源或角度突变，并在变化时清除旧线锚点。"""
        axis = wrap180(float(directed_path_axis_image))
        source_changed = (
            self.last_tracking_axis_source is not None
            and source != self.last_tracking_axis_source
        )
        angle_changed = (
            self.last_tracking_axis_image_deg is not None
            and abs(
                wrap180(axis - self.last_tracking_axis_image_deg)
            )
            > 20.0
        )
        if source_changed or angle_changed:
            self.reset_line_tracking()
        self.last_tracking_axis_image_deg = axis
        self.last_tracking_axis_source = source

    def choose_parallel_line(self, lines, axis_angle_deg, width, height):
        """选择当前单线并更新跟踪锚点，返回线段、偏移和跟踪状态。"""
        if not self.line_tracking_enabled:
            selected = select_most_salient_line(lines)
            if selected is None:
                return None, None, 'disabled'
            offset = line_normal_offset_at_reference(
                selected, axis_angle_deg, width, height, 0.0
            )
            if not math.isfinite(offset):
                return None, None, 'disabled'
            return selected, offset, 'disabled'

        selected, offset, missed_frames, status = update_line_tracking_state(
            lines,
            axis_angle_deg,
            width,
            height,
            previous_offset_px=self.last_valid_line_offset_px,
            missed_frames=self.line_tracking_missed_frames,
            max_jump_px=self.max_line_tracking_jump_px,
            max_missed_frames=self.max_line_tracking_missed_frames,
        )
        self.line_tracking_missed_frames = missed_frames
        self.line_tracking_status = status
        self.line_tracking_reacquire = (
            selected is None
            and self.last_valid_line_offset_px is not None
            and missed_frames >= self.max_line_tracking_missed_frames
        )
        if selected is not None and offset is not None:
            self.tracked_line_offset_px = offset
            self.last_valid_line_offset_px = offset
        else:
            # A rejected or missing candidate must never move the anchor.
            self.tracked_line_offset_px = None
        return selected, offset, status

    def publish_invalid(self):
        """发布明确的无效视觉结果。"""
        result = Vector3()
        result.z = 0.0
        self.angle_pub.publish(result)

        confidence = Float32()
        confidence.data = 0.0
        self.confidence_pub.publish(confidence)

        heading_valid = Bool()
        heading_valid.data = False
        self.heading_valid_pub.publish(heading_valid)

        lateral_valid = Bool()
        lateral_valid.data = False
        self.lateral_valid_pub.publish(lateral_valid)

        heading_confidence = Float32()
        heading_confidence.data = 0.0
        self.heading_confidence_pub.publish(heading_confidence)

        lateral_confidence = Float32()
        lateral_confidence.data = 0.0
        self.lateral_confidence_pub.publish(lateral_confidence)

        axis = String()
        axis.data = 'invalid'
        self.run_axis_pub.publish(axis)
        axis_debug = String()
        axis_debug.data = 'axis=invalid angle=nan deg source=none'
        self.run_axis_debug_pub.publish(axis_debug)

    def publish_run_axis(
        self,
        path_axis_image,
        directed_path_axis_image,
        source,
        path_direction_deg=None,
        vehicle_heading_deg=None,
    ):
        """发布方向分类和带来源的运行方向诊断。"""
        axis_label = (
            'vertical' if abs(path_axis_image) >= 45.0 else 'horizontal'
        )
        axis = String()
        axis.data = axis_label
        self.run_axis_pub.publish(axis)
        axis_debug = String()
        axis_debug.data = format_run_axis_debug(
            axis_label,
            directed_path_axis_image,
            source,
            path_direction_deg,
            vehicle_heading_deg,
        )
        self.run_axis_debug_pub.publish(axis_debug)
        self.get_logger().info(
            f'运行方向: {axis_debug.data}',
            throttle_duration_sec=1.0,
        )

    def image_callback(self, msg):
        """仅缓存最新 JPEG 数据，由检测定时器异步完成图像处理。"""
        if not self.enable_visual_correction:
            self.reset_line_tracking()
            self.publish_invalid()
            return

        if (
            not self.bypass_path_context_gate
            and (
                not self.path_context_valid
                or time.monotonic() - self.last_path_context_time
                > self.path_context_timeout_sec
            )
        ):
            self.path_context_valid = False
            self.reset_line_tracking()
            self.publish_invalid()
            return
        payload = bytes(msg.data)
        if payload:
            with self.latest_frame_lock:
                self.latest_compressed_data = payload

    def pixels_to_lateral_meters(self, lateral_pixel_error):
        """将路径法向像素误差换算为米制横向误差。"""
        lateral_pixel_error = float(lateral_pixel_error)
        camera_height = float(self.camera_height)
        camera_pitch_deg = float(self.camera_pitch_deg)
        focal_length_px = float(self.focal_length_px)
        if not all(
            math.isfinite(value)
            for value in (
                lateral_pixel_error,
                camera_height,
                camera_pitch_deg,
                focal_length_px,
            )
        ):
            return float('nan')
        if (
            camera_height <= 0.0
            or focal_length_px <= 0.0
            or not 0.0 < camera_pitch_deg <= 90.0
        ):
            return float('nan')

        pitch_rad = math.radians(float(self.camera_pitch_deg))
        sin_pitch = math.sin(pitch_rad)
        if not math.isfinite(sin_pitch) or sin_pitch <= 0.0:
            return float('nan')
        lateral_m = lateral_pixel_error * camera_height / (
            sin_pitch * focal_length_px
        )
        return lateral_m if math.isfinite(lateral_m) else float('nan')

    def lateral_meters_to_pixels(self, lateral_m):
        """将标定的地面横向位置转换为法向像素偏移。"""
        lateral_m = float(lateral_m)
        camera_height = float(self.camera_height)
        camera_pitch_deg = float(self.camera_pitch_deg)
        focal_length_px = float(self.focal_length_px)
        if not all(
            math.isfinite(value)
            for value in (
                lateral_m,
                camera_height,
                camera_pitch_deg,
                focal_length_px,
            )
        ):
            return float('nan')
        if (
            camera_height <= 0.0
            or focal_length_px <= 0.0
            or not 0.0 < camera_pitch_deg <= 90.0
        ):
            return float('nan')
        sin_pitch = math.sin(math.radians(camera_pitch_deg))
        if sin_pitch <= 0.0 or not math.isfinite(sin_pitch):
            return float('nan')
        pixels = lateral_m * sin_pitch * focal_length_px / camera_height
        return pixels if math.isfinite(pixels) else float('nan')

    def detect_and_draw_grid_lines(self, image):
        """检测粗白光伏结构线并计算路径感知纠偏量。"""
        height, width = image.shape[:2]
        # Generate debug frames at the compressed input processing rate.
        # Publication is asynchronous and never blocks the detector timer.
        debug_due = (
            self.publish_debug_images_enabled
            and self.has_debug_subscribers()
        )
        gray_blur = None
        if debug_due:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            gray_blur = cv2.GaussianBlur(gray, (5, 5), 1.0)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        white_mask = cv2.inRange(
            hsv,
            np.array(
                [0, 0, int(self.white_line_value_threshold)],
                dtype=np.uint8,
            ),
            np.array(
                [180, int(self.white_line_saturation_max), 255],
                dtype=np.uint8,
            ),
        )
        white_mask = cv2.morphologyEx(
            white_mask,
            cv2.MORPH_OPEN,
            np.ones((3, 3), dtype=np.uint8),
        )
        white_mask = cv2.morphologyEx(
            white_mask,
            cv2.MORPH_CLOSE,
            np.ones((5, 5), dtype=np.uint8),
        )
        white_edges = cv2.Canny(white_mask, 50, 150, apertureSize=3)
        lines = cv2.HoughLinesP(
            white_edges,
            rho=1,
            theta=np.pi / 180,
            threshold=40,
            minLineLength=max(1, int(self.coarse_line_min_length_px)),
            maxLineGap=15,
        )
        display = image.copy() if debug_due else None

        directed_path_axis_image, path_axis_source = (
            resolve_effective_path_axis_image(
                self.path_context_valid,
                self.last_path_context_time,
                self.path_context_timeout_sec,
                self.path_direction_deg,
                self.vehicle_heading_deg,
                self.camera_angle_offset,
                self.fallback_path_axis_image_deg,
            )
        )
        path_axis_image = undirected_angle(directed_path_axis_image)
        cross_axis_image = undirected_angle(directed_path_axis_image + 90.0)
        self.publish_run_axis(
            path_axis_image,
            directed_path_axis_image,
            path_axis_source,
            path_direction_deg=(
                self.path_direction_deg
                if path_axis_source == 'rtk'
                else None
            ),
            vehicle_heading_deg=(
                self.vehicle_heading_deg
                if path_axis_source == 'rtk'
                else None
            ),
        )
        lateral_state_valid = nav_state_allows_lateral_output(self.nav_state)
        # The normal-offset coordinate is only comparable while the active
        # image axis is stable. A stale RTK axis or a sharp turn therefore
        # starts a new line lock instead of reusing an old anchor.
        self.update_tracking_axis(
            directed_path_axis_image, path_axis_source
        )
        if display is not None:
            center = (width // 2, height // 2)
            arrow_length = max(40, int(min(width, height) * 0.25))
            axis_radians = math.radians(directed_path_axis_image)
            endpoint = (
                int(round(center[0] + arrow_length * math.cos(axis_radians))),
                int(round(center[1] + arrow_length * math.sin(axis_radians))),
            )
            cv2.arrowedLine(
                display, center, endpoint, (0, 165, 255), 4, tipLength=0.2
            )
            cv2.putText(
                display,
                f'Image axis: {directed_path_axis_image:.1f} deg '
                f'[{path_axis_source}]',
                (10, 100),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 165, 255),
                2,
            )
            if path_axis_source == 'rtk':
                relative_heading = wrap180(
                    self.path_direction_deg - self.vehicle_heading_deg
                )
                cv2.putText(
                    display,
                    f'RTK path: {self.path_direction_deg:.1f} deg',
                    (10, 125),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 165, 255),
                    2,
                )
                cv2.putText(
                    display,
                    f'Vehicle: {self.vehicle_heading_deg:.1f}, '
                    f'Relative: {relative_heading:.1f}',
                    (10, 150),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 165, 255),
                    2,
                )

        def estimate_line_metrics(x1, y1, x2, y2):
            """沿线段法向扫描白色带宽度和支持度。"""
            segment_length = math.hypot(x2 - x1, y2 - y1)
            if segment_length <= 0.0 or not math.isfinite(segment_length):
                return None
            tangent_x = (x2 - x1) / segment_length
            tangent_y = (y2 - y1) / segment_length
            normal_x = -tangent_y
            normal_y = tangent_x
            sample_count = max(2, int(segment_length / 8.0))
            half_width = int(round(self.white_line_scan_half_width_px))
            fractions = np.linspace(0.0, 1.0, sample_count, dtype=np.float32)
            offsets = np.arange(
                -half_width, half_width + 1, dtype=np.float32
            )
            sample_x = x1 + (x2 - x1) * fractions[:, None]
            sample_y = y1 + (y2 - y1) * fractions[:, None]
            pixel_x = np.rint(sample_x + normal_x * offsets[None, :]).astype(int)
            pixel_y = np.rint(sample_y + normal_y * offsets[None, :]).astype(int)
            valid = (
                (pixel_x >= 0)
                & (pixel_x < width)
                & (pixel_y >= 0)
                & (pixel_y < height)
            )
            safe_x = np.clip(pixel_x, 0, width - 1)
            safe_y = np.clip(pixel_y, 0, height - 1)
            scan = white_mask[safe_y, safe_x] != 0
            scan &= valid
            supported = int(np.count_nonzero(np.any(scan, axis=1)))

            # The scan is narrow, so a column-wise NumPy run length is cheaper
            # than nested Python loops for every Hough segment.
            current_run = np.zeros(sample_count, dtype=np.int16)
            longest_run = np.zeros(sample_count, dtype=np.int16)
            for column in range(scan.shape[1]):
                current_run = np.where(scan[:, column], current_run + 1, 0)
                longest_run = np.maximum(longest_run, current_run)
            if sample_count <= 0:
                return None
            return float(np.mean(longest_run)), supported / float(sample_count)

        coarse_lines = []
        if lines is not None:
            for raw_line in lines:
                x1, y1, x2, y2 = [int(value) for value in raw_line[0]]
                length = math.hypot(x2 - x1, y2 - y1)
                metrics = estimate_line_metrics(x1, y1, x2, y2)
                if metrics is None:
                    continue
                width_px, white_support = metrics
                line_angle = undirected_angle(
                    math.degrees(math.atan2(y2 - y1, x2 - x1))
                )
                record = (
                    x1,
                    y1,
                    x2,
                    y2,
                    length,
                    line_angle,
                    (x1 + x2) / 2.0,
                    (y1 + y2) / 2.0,
                    width_px,
                    white_support,
                )
                coarse_lines.append(record)

        coarse_lines = select_coarse_white_lines(
            coarse_lines,
            self.coarse_line_min_length_px,
            self.coarse_line_min_width_px,
            self.coarse_line_min_support,
        )
        if not coarse_lines:
            _, _, tracking_status = self.choose_parallel_line(
                [], directed_path_axis_image, width, height
            )
            self.get_logger().info(
                f'P:0 C:0 line=False track={tracking_status} '
                f'missed={self.line_tracking_missed_frames}',
                throttle_duration_sec=1.0,
            )
            self.reset_reacquisition()
            self.queue_debug_images(
                display, gray_blur, white_mask, white_edges
            )
            return 0.0, 0.0, False, 0.0, False, False, 0.0, 0.0

        parallel_group = []
        perpendicular_group = []
        for record in coarse_lines:
            line_angle = float(record[5])
            parallel_error = undirected_angle_distance(
                line_angle, path_axis_image
            )
            perpendicular_error = undirected_angle_distance(
                line_angle, cross_axis_image
            )
            if (
                parallel_error <= self.line_angle_tolerance_deg
                and parallel_error <= perpendicular_error
            ):
                parallel_group.append(record)
            elif perpendicular_error <= self.line_angle_tolerance_deg:
                perpendicular_group.append(record)

        parallel_group = merge_nearby_line_records(
            parallel_group,
            path_axis_image,
            self.coarse_line_merge_gap_px,
        )
        perpendicular_group = merge_nearby_line_records(
            perpendicular_group,
            cross_axis_image,
            self.coarse_line_merge_gap_px,
        )
        self.get_logger().debug(
            f'P:{len(parallel_group)} C:{len(perpendicular_group)}',
            throttle_duration_sec=1.0,
        )

        selected_parallel_line, selected_line_offset, tracking_status = (
            self.choose_parallel_line(
                parallel_group,
                directed_path_axis_image,
                width,
                height,
            )
        )
        selected_line_score = (
            line_salience_score(selected_parallel_line)
            if selected_parallel_line is not None
            else 0.0
        )

        if display is not None:
            for line in parallel_group:
                cv2.line(
                    display,
                    (line[0], line[1]),
                    (line[2], line[3]),
                    (0, 255, 0),
                    2,
                )
            for line in perpendicular_group:
                cv2.line(
                    display,
                    (line[0], line[1]),
                    (line[2], line[3]),
                    (255, 0, 0),
                    2,
                )
            if selected_parallel_line is not None:
                cv2.line(
                    display,
                    (selected_parallel_line[0], selected_parallel_line[1]),
                    (selected_parallel_line[2], selected_parallel_line[3]),
                    (0, 0, 255),
                    4,
                )

        # Both corrections must come from the same line.  This is important
        # when the camera sees only one physical reference line.
        parallel_angle = (
            undirected_angle(float(selected_parallel_line[5]))
            if selected_parallel_line is not None
            else None
        )
        heading_geometry = (
            selected_parallel_line is not None
            and parallel_angle is not None
            and math.isfinite(float(parallel_angle))
        )
        lateral_pixel_error = float('nan')
        if heading_geometry:
            # Reuse the selector's offset so association and correction use
            # exactly the same image-center reference.
            lateral_pixel_error = selected_line_offset
        lateral_geometry = heading_geometry and math.isfinite(
            lateral_pixel_error
        )
        valid_geometry = lateral_geometry

        if not heading_geometry:
            self.get_logger().info(
                f'P:{len(parallel_group)} C:{len(perpendicular_group)} '
                f'line={selected_parallel_line is not None} '
                f'track={tracking_status} '
                f'geometry={valid_geometry} '
                f'streak=0/{self.reacquire_frames}'
            )
            self.reset_reacquisition()
            self.queue_debug_images(
                display, gray_blur, white_mask, white_edges
            )
            return 0.0, 0.0, False, 0.0, False, False, 0.0, 0.0

        self.heading_valid_streak += 1
        if lateral_geometry and lateral_state_valid:
            self.lateral_valid_streak += 1
        else:
            self.lateral_valid_streak = 0
        self.valid_streak = self.lateral_valid_streak
        heading_error = undirected_angle(parallel_angle - path_axis_image)
        lateral_m = 0.0
        if lateral_geometry:
            lateral_m = self.pixels_to_lateral_meters(lateral_pixel_error)
        if lateral_geometry and not math.isfinite(lateral_m):
            self.get_logger().info(
                f'P:{len(parallel_group)} C:{len(perpendicular_group)} '
                f'line={selected_parallel_line is not None} '
                f'track={tracking_status} '
                f'geometry={valid_geometry} '
                f'streak={self.valid_streak}/{self.reacquire_frames} '
                f'lateral_px={lateral_pixel_error:.1f} lateral_m=invalid'
            )
            self.reset_reacquisition()
            self.queue_debug_images(
                display, gray_blur, white_mask, white_edges
            )
            return 0.0, 0.0, False, 0.0, False, False, 0.0, 0.0

        # Apply the navigation-state gate to every correction output so a
        # stale streak cannot expose angle or lateral data outside motion.
        heading_valid = (
            lateral_state_valid
            and self.heading_valid_streak >= self.reacquire_frames
        )
        lateral_valid = (
            lateral_state_valid
            and self.lateral_valid_streak >= self.reacquire_frames
        )
        detected = heading_valid
        output_angle = heading_error if heading_valid else 0.0
        output_lateral = lateral_m if lateral_valid else 0.0
        heading_confidence = 0.0
        lateral_confidence = 0.0
        if heading_valid:
            support_score = float(selected_parallel_line[9])
            heading_confidence = max(
                0.0,
                min(1.0, support_score),
            )
        if lateral_valid:
            lateral_confidence = heading_confidence
        confidence = max(heading_confidence, lateral_confidence)
        self.get_logger().info(
            f'P:{len(parallel_group)} C:{len(perpendicular_group)} '
            f'line={selected_parallel_line is not None} '
            f'score={selected_line_score:.1f} '
            f'track={tracking_status} '
            f'nav_state={self.nav_state or "unknown"} '
            f'lateral_state={lateral_state_valid} '
            f'geometry={valid_geometry} '
            f'streak={self.valid_streak}/{self.reacquire_frames} '
            f'detected={detected} lateral_px={lateral_pixel_error:.1f} '
            f'lateral_m={lateral_m:.3f} '
            f'heading_valid={heading_valid} lateral_valid={lateral_valid}'
        )

        if display is not None:
            cv2.putText(
                display,
                f'Angle: {output_angle:.2f} deg',
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
            )
            cv2.putText(
                display,
                f'Lat Dev: {output_lateral:.3f} m',
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 0, 165),
                2,
            )
            cv2.putText(
                display,
                f'P:{len(parallel_group)} C:{len(perpendicular_group)} '
                f'line={selected_parallel_line is not None} '
                f'track={tracking_status}',
                (10, 185),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )
        self.queue_debug_images(
            display, gray_blur, white_mask, white_edges
        )

        if not detected:
            return (
                0.0, 0.0, False, 0.0,
                False, False, heading_confidence, lateral_confidence,
            )
        return (
            output_angle, output_lateral, True, confidence,
            heading_valid, lateral_valid, heading_confidence, lateral_confidence,
        )

    def has_debug_subscribers(self):
        """检查调试图像话题是否存在订阅者，以决定是否生成调试图。"""
        return any(
            publisher.get_subscription_count() > 0
            for publisher in (
                self.image_pub,
                self.gray_pub,
                self.binary_pub,
                self.edges_pub,
            )
        )

    def queue_debug_images(self, display, gray, binary, edges):
        """以非阻塞方式只保留最新调试帧，避免拖慢检测循环。"""
        if not self.publish_debug_images_enabled or display is None:
            return
        payload = (display, gray, binary, edges)
        try:
            self.debug_frame_queue.put_nowait(payload)
            return
        except Full:
            pass

        try:
            self.debug_frame_queue.get_nowait()
        except Empty:
            return
        try:
            self.debug_frame_queue.put_nowait(payload)
        except Full:
            pass

    def debug_image_worker(self):
        """在独立线程中发布调试图像，避免阻塞检测定时器。"""
        while not self.debug_worker_stop.is_set():
            try:
                payload = self.debug_frame_queue.get(timeout=0.1)
            except Empty:
                continue
            try:
                self.publish_debug_images(*payload)
            except Exception as exc:
                if not self.debug_worker_stop.is_set():
                    self.get_logger().warning(
                        f'调试图像发布错误: {exc}',
                        throttle_duration_sec=1.0,
                    )

    def stop_debug_image_worker(self):
        """在 ROS 节点销毁前停止异步调试图像发布线程。"""
        self.debug_worker_stop.set()
        if (
            self.debug_worker is not None
            and self.debug_worker.is_alive()
            and threading.current_thread() is not self.debug_worker
        ):
            self.debug_worker.join(timeout=1.0)

    def destroy_node(self):
        """停止后台发布线程，并释放 ROS 发布器和订阅器资源。"""
        self.stop_debug_image_worker()
        super().destroy_node()

    def publish_debug_images(self, display, gray, binary, edges):
        """发布检测标注图和中间处理图。"""
        if self.image_pub.get_subscription_count() > 0:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(display, 'bgr8'))
        if self.gray_pub.get_subscription_count() > 0:
            self.gray_pub.publish(self.bridge.cv2_to_imgmsg(gray, 'mono8'))
        if self.binary_pub.get_subscription_count() > 0:
            self.binary_pub.publish(self.bridge.cv2_to_imgmsg(binary, 'mono8'))
        if self.edges_pub.get_subscription_count() > 0:
            self.edges_pub.publish(self.bridge.cv2_to_imgmsg(edges, 'mono8'))


def main(args=None):
    """初始化 ROS 2，运行检测节点，并在退出时释放节点资源。"""
    rclpy.init(args=args)
    node = GridLineDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
