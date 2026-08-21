#!/usr/bin/env python3
"""RTK 路径感知的栅格线视觉纠偏节点。"""

import math
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String


def wrap180(angle_deg):
    """将有向角归一化到 [-180, 180)。"""
    return (float(angle_deg) + 180.0) % 360.0 - 180.0


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

    axis_rad = math.radians(float(axis_angle_deg))
    axis_x = math.cos(axis_rad)
    axis_y = math.sin(axis_rad)
    normal_x = -math.sin(axis_rad)
    normal_y = math.cos(axis_rad)

    def normalize_record(record):
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
        first = float(record[0]) * axis_x + float(record[1]) * axis_y
        second = float(record[2]) * axis_x + float(record[3]) * axis_y
        return min(first, second), max(first, second)

    ordered = sorted(
        (normalize_record(line) for line in lines),
        key=lambda line: float(line[6]) * normal_x + float(line[7]) * normal_y,
    )
    merged = []
    cluster = [ordered[0]]
    previous_projection = (
        float(ordered[0][6]) * normal_x + float(ordered[0][7]) * normal_y
    )
    cluster_axis_start, cluster_axis_end = axis_interval(ordered[0])

    def make_record(records):
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
            line_axis_start <= cluster_axis_end
            and cluster_axis_start <= line_axis_end
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


class GridLineDetector(Node):
    """根据 RTK 当前路径段选择栅格线组并计算视觉纠偏量。"""

    def __init__(self):
        super().__init__('grid_line_detector')

        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10,
        )
        self.path_context_sub = self.create_subscription(
            Vector3,
            '/rtk/visual_path_context',
            self.path_context_callback,
            10,
        )

        self.angle_pub = self.create_publisher(
            Vector3, '/grid_line/angle_deviation', 10
        )
        self.confidence_pub = self.create_publisher(
            Float32, '/grid_line/detection_confidence', 10
        )
        self.run_axis_pub = self.create_publisher(
            String, '/grid_line/run_axis', 10
        )
        self.image_pub = self.create_publisher(
            Image, '/grid_line/detected_image', 10
        )
        self.gray_pub = self.create_publisher(Image, '/grid_line/gray_image', 10)
        self.binary_pub = self.create_publisher(
            Image, '/grid_line/binary_image', 10
        )
        self.edges_pub = self.create_publisher(Image, '/grid_line/edges_image', 10)

        # These five defaults are calibrated for the current camera installation.
        self.declare_parameter('camera_angle_offset', 0.0)
        self.declare_parameter('camera_height', 0.5)
        self.declare_parameter('camera_pitch_deg', 90.0)
        self.declare_parameter('focal_length_px', 132.0)
        self.declare_parameter('min_line_count', 2)
        self.declare_parameter('enable_visual_correction', False)
        self.declare_parameter('line_angle_tolerance_deg', 15.0)
        self.declare_parameter('path_context_timeout_sec', 0.5)
        self.declare_parameter('boundary_pair_max_gap_px', 1200.0)
        self.declare_parameter('reacquire_frames', 3)
        self.declare_parameter('white_line_value_threshold', 170.0)
        self.declare_parameter('white_line_saturation_max', 100.0)
        self.declare_parameter('coarse_line_min_length_px', 80.0)
        self.declare_parameter('coarse_line_min_width_px', 3.0)
        self.declare_parameter('coarse_line_min_support', 0.55)
        self.declare_parameter('coarse_line_merge_gap_px', 10.0)
        self.declare_parameter('white_line_scan_half_width_px', 8.0)

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

        if (
            not math.isfinite(self.path_context_timeout_sec)
            or self.path_context_timeout_sec <= 0.0
        ):
            raise ValueError('path_context_timeout_sec must be finite and > 0')
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

        self.path_direction_deg = 0.0
        self.vehicle_heading_deg = 0.0
        self.path_context_valid = False
        self.last_path_context_time = 0.0
        self.last_path_direction_deg = None
        self.valid_streak = 0

        self.last_process_time = time.monotonic()
        self.min_process_interval = 0.1
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        """保留定时器接口，图像回调负责实际处理。"""

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
            self.reset_reacquisition()
            return

        if (
            self.last_path_direction_deg is not None
            and abs(wrap180(path_direction - self.last_path_direction_deg)) > 20.0
        ):
            self.reset_reacquisition()

        self.path_direction_deg = path_direction
        self.vehicle_heading_deg = vehicle_heading
        self.last_path_direction_deg = path_direction
        self.path_context_valid = True
        self.last_path_context_time = time.monotonic()

    def reset_reacquisition(self):
        """清除当前连续有效帧，避免沿用旧方向检测结果。"""
        self.valid_streak = 0

    def publish_invalid(self):
        """发布明确的无效视觉结果。"""
        result = Vector3()
        result.z = 0.0
        self.angle_pub.publish(result)

        confidence = Float32()
        confidence.data = 0.0
        self.confidence_pub.publish(confidence)

        axis = String()
        axis.data = 'invalid'
        self.run_axis_pub.publish(axis)

    def publish_run_axis(self, path_axis_image):
        """发布当前路径轴相对图像的方向诊断。"""
        axis = String()
        axis.data = 'vertical' if abs(path_axis_image) >= 45.0 else 'horizontal'
        self.run_axis_pub.publish(axis)

    def image_callback(self, msg):
        """处理图像并发布路径感知的视觉纠偏结果。"""
        if not self.enable_visual_correction:
            self.reset_reacquisition()
            self.publish_invalid()
            return

        if (
            not self.path_context_valid
            or time.monotonic() - self.last_path_context_time
            > self.path_context_timeout_sec
        ):
            self.path_context_valid = False
            self.reset_reacquisition()
            self.publish_invalid()
            return

        current_time = time.monotonic()
        if current_time - self.last_process_time < self.min_process_interval:
            return

        self.last_process_time = current_time
        try:
            image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            angle, lateral_m, detected, confidence = (
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
        except Exception as exc:
            self.last_process_time = time.monotonic()
            self.get_logger().error(f'图像处理错误: {exc}')
            self.reset_reacquisition()
            self.publish_invalid()

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

    def detect_and_draw_grid_lines(self, image):
        """检测粗白光伏结构线并计算路径感知纠偏量。"""
        height, width = image.shape[:2]
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
        display = image.copy()

        relative_path_heading = wrap180(
            self.path_direction_deg - self.vehicle_heading_deg
        )
        directed_path_axis_image = wrap180(
            90.0 - relative_path_heading + self.camera_angle_offset
        )
        path_axis_image = undirected_angle(directed_path_axis_image)
        cross_axis_image = undirected_angle(directed_path_axis_image + 90.0)
        self.publish_run_axis(path_axis_image)

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
            widths = []
            supported = 0
            for sample_index in range(sample_count):
                fraction = sample_index / float(sample_count - 1)
                sample_x = x1 + (x2 - x1) * fraction
                sample_y = y1 + (y2 - y1) * fraction
                scan = []
                for offset in range(-half_width, half_width + 1):
                    pixel_x = int(round(sample_x + normal_x * offset))
                    pixel_y = int(round(sample_y + normal_y * offset))
                    if (
                        0 <= pixel_x < width
                        and 0 <= pixel_y < height
                    ):
                        scan.append(bool(white_mask[pixel_y, pixel_x]))
                    else:
                        scan.append(False)
                if any(scan):
                    supported += 1
                longest_run = 0
                current_run = 0
                for is_white in scan:
                    if is_white:
                        current_run += 1
                        longest_run = max(longest_run, current_run)
                    else:
                        current_run = 0
                widths.append(float(longest_run))
            if not widths:
                return None
            return float(np.mean(widths)), supported / float(sample_count)

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
            self.reset_reacquisition()
            self.publish_debug_images(
                display, gray_blur, white_mask, white_edges
            )
            return 0.0, 0.0, False, 0.0

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

        pair = select_boundary_pair(
            parallel_group,
            directed_path_axis_image,
            width,
            height,
            self.boundary_pair_max_gap_px,
        )
        parallel_angle = weighted_line_angle(parallel_group)
        valid_geometry = (
            len(parallel_group) >= self.min_line_count
            and pair is not None
            and parallel_angle is not None
            and math.isfinite(float(parallel_angle))
        )

        if not valid_geometry:
            self.reset_reacquisition()
            self.publish_debug_images(
                display, gray_blur, white_mask, white_edges
            )
            return 0.0, 0.0, False, 0.0

        self.valid_streak += 1
        heading_error = undirected_angle(parallel_angle - path_axis_image)
        _, _, left_projection, right_projection, center_projection = pair
        lateral_pixel_error = (
            (left_projection + right_projection) / 2.0 - center_projection
        )
        lateral_m = self.pixels_to_lateral_meters(lateral_pixel_error)
        if not math.isfinite(lateral_m):
            self.reset_reacquisition()
            self.publish_debug_images(
                display, gray_blur, white_mask, white_edges
            )
            return 0.0, 0.0, False, 0.0

        detected = self.valid_streak >= self.reacquire_frames
        output_angle = heading_error if detected else 0.0
        output_lateral = lateral_m if detected else 0.0
        confidence = 0.0
        if detected:
            count_score = min(1.0, len(parallel_group) / 6.0)
            support_score = sum(
                float(line[9]) for line in parallel_group
            ) / float(len(parallel_group))
            confidence = max(
                0.0,
                min(1.0, 0.5 * count_score + 0.5 * support_score),
            )

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
            (255, 0, 0),
            2,
        )
        cv2.putText(
            display,
            f'P:{len(parallel_group)} C:{len(perpendicular_group)}',
            (10, 90),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
        )
        self.publish_debug_images(
            display, gray_blur, white_mask, white_edges
        )

        if not detected:
            return 0.0, 0.0, False, 0.0
        return output_angle, output_lateral, True, confidence

    def publish_debug_images(self, display, gray, binary, edges):
        """发布检测标注图和中间处理图。"""
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(display, 'bgr8'))
        self.gray_pub.publish(self.bridge.cv2_to_imgmsg(gray, 'mono8'))
        self.binary_pub.publish(self.bridge.cv2_to_imgmsg(binary, 'mono8'))
        self.edges_pub.publish(self.bridge.cv2_to_imgmsg(edges, 'mono8'))


def main(args=None):
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
