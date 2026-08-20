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
    """将角度归一化到 [-180, 180)。"""
    return (float(angle_deg) + 180.0) % 360.0 - 180.0


def undirected_angle(angle_deg):
    """将无向直线角度归一化到 [-90, 90)。"""
    return (float(angle_deg) + 90.0) % 180.0 - 90.0


def undirected_angle_distance(angle_a, angle_b):
    """返回两条无向直线的最小夹角。"""
    return abs(undirected_angle(float(angle_a) - float(angle_b)))


def weighted_line_angle(lines):
    """使用长度权重计算无向线段的平均方向。"""
    if not lines:
        return None
    weights = np.asarray([line[4] for line in lines], dtype=float)
    angles = np.radians(np.asarray([line[5] for line in lines], dtype=float) * 2.0)
    sin_sum = float(np.sum(weights * np.sin(angles)))
    cos_sum = float(np.sum(weights * np.cos(angles)))
    if abs(sin_sum) < 1e-9 and abs(cos_sum) < 1e-9:
        return None
    return undirected_angle(math.degrees(0.5 * math.atan2(sin_sum, cos_sum)))


def select_boundary_pair(lines, axis_angle_deg, width, height, max_gap_px):
    """在路径轴的法向投影上选择图像中心左右的边界线。"""
    if len(lines) < 2:
        return None
    axis_rad = math.radians(axis_angle_deg)
    normal_x = -math.sin(axis_rad)
    normal_y = math.cos(axis_rad)
    center_x = width / 2.0
    center_y = height / 2.0
    center_projection = center_x * normal_x + center_y * normal_y

    left_candidates = []
    right_candidates = []
    for line in lines:
        projection = line[6] * normal_x + line[7] * normal_y
        if projection < center_projection:
            left_candidates.append((projection, line))
        elif projection > center_projection:
            right_candidates.append((projection, line))
    if not left_candidates or not right_candidates:
        return None

    left_projection, left_line = max(left_candidates, key=lambda item: item[0])
    right_projection, right_line = min(right_candidates, key=lambda item: item[0])
    gap = right_projection - left_projection
    if gap <= 0.0 or gap > max_gap_px:
        return None
    return left_line, right_line, left_projection, right_projection, center_projection


class GridLineDetector(Node):
    """根据 RTK 当前路径段选择栅格线组并计算视觉纠偏量。"""

    def __init__(self):
        super().__init__('grid_line_detector')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, 10
        )
        self.path_context_sub = self.create_subscription(
            Vector3, '/rtk/visual_path_context', self.path_context_callback, 10
        )
        self.angle_pub = self.create_publisher(
            Vector3, '/grid_line/angle_deviation', 10
        )
        self.confidence_pub = self.create_publisher(
            Float32, '/grid_line/detection_confidence', 10
        )
        self.run_axis_pub = self.create_publisher(String, '/grid_line/run_axis', 10)
        self.image_pub = self.create_publisher(
            Image, '/grid_line/detected_image', 10
        )
        self.gray_pub = self.create_publisher(Image, '/grid_line/gray_image', 10)
        self.binary_pub = self.create_publisher(Image, '/grid_line/binary_image', 10)
        self.edges_pub = self.create_publisher(Image, '/grid_line/edges_image', 10)

        self.declare_parameter('camera_angle_offset', 0.8)
        self.declare_parameter('camera_height', 0.5)
        self.declare_parameter('camera_pitch_deg', 30.0)
        self.declare_parameter('focal_length_px', 600.0)
        self.declare_parameter('min_line_count', 2)
        self.declare_parameter('line_angle_tolerance_deg', 15.0)
        self.declare_parameter('path_context_timeout_sec', 0.5)
        self.declare_parameter('boundary_pair_max_gap_px', 1200.0)
        self.declare_parameter('reacquire_frames', 3)
        self.declare_parameter('enable_visual_correction', False)

        self.camera_angle_offset = self.get_parameter('camera_angle_offset').value
        self.camera_height = self.get_parameter('camera_height').value
        self.camera_pitch_deg = self.get_parameter('camera_pitch_deg').value
        self.focal_length_px = self.get_parameter('focal_length_px').value
        self.min_line_count = self.get_parameter('min_line_count').value
        self.line_angle_tolerance_deg = self.get_parameter(
            'line_angle_tolerance_deg'
        ).value
        self.path_context_timeout_sec = self.get_parameter(
            'path_context_timeout_sec'
        ).value
        self.boundary_pair_max_gap_px = self.get_parameter(
            'boundary_pair_max_gap_px'
        ).value
        self.reacquire_frames = max(
            1, int(self.get_parameter('reacquire_frames').value)
        )
        self.enable_visual_correction = self.get_parameter(
            'enable_visual_correction'
        ).get_parameter_value().bool_value

        self.path_direction_deg = 0.0
        self.vehicle_heading_deg = 0.0
        self.path_context_valid = False
        self.last_path_context_time = 0.0
        self.valid_streak = 0
        self.last_path_direction_deg = None
        self.get_logger().info(
            '视觉纠偏节点已启动，enable_visual_correction=%s',
            self.enable_visual_correction,
        )

    def path_context_callback(self, msg):
        """接收 RTK 当前路径段和车体航向上下文。"""
        now = time.monotonic()
        new_valid = bool(msg.z >= 0.5) and all(
            math.isfinite(float(value)) for value in (msg.x, msg.y)
        )
        if new_valid:
            path_direction = float(msg.x)
            if (
                self.last_path_direction_deg is not None
                and abs(wrap180(path_direction - self.last_path_direction_deg)) > 20.0
            ):
                self.reset_reacquisition()
            self.last_path_direction_deg = path_direction
            self.path_direction_deg = path_direction
            self.vehicle_heading_deg = float(msg.y)
        else:
            self.reset_reacquisition()
        self.path_context_valid = new_valid
        self.last_path_context_time = now

    def reset_reacquisition(self):
        """换向或上下文失效时清除旧方向检测状态。"""
        self.valid_streak = 0

    def publish_invalid(self):
        """发布无效视觉纠偏，避免无效被误认为零偏差。"""
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
        """发布当前路径轴相对图像的水平/竖直诊断。"""
        axis = String()
        axis.data = 'vertical' if abs(path_axis_image) >= 45.0 else 'horizontal'
        self.run_axis_pub.publish(axis)

    def image_callback(self, msg):
        if not self.enable_visual_correction:
            self.publish_invalid()
            return
        if (
            not self.path_context_valid
            or time.monotonic() - self.last_path_context_time
            > self.path_context_timeout_sec
        ):
            self.reset_reacquisition()
            self.publish_invalid()
            return

        try:
            image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            angle, lateral_m, detected, confidence = self.detect_and_draw_grid_lines(image)
            result = Vector3()
            result.x = float(angle)
            result.y = float(lateral_m)
            result.z = 1.0 if detected else 0.0
            self.angle_pub.publish(result)
            confidence_msg = Float32()
            confidence_msg.data = float(confidence)
            self.confidence_pub.publish(confidence_msg)
        except Exception as exc:
            self.get_logger().error('图像处理错误: %s', exc)
            self.reset_reacquisition()
            self.publish_invalid()

    def detect_and_draw_grid_lines(self, image):
        """检测与当前 RTK 路径轴平行/垂直的线组。"""
        height, width = image.shape[:2]
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray_blur = cv2.GaussianBlur(gray, (5, 5), 1.0)
        edges = cv2.Canny(gray_blur, 50, 150, apertureSize=3)
        _, binary = cv2.threshold(
            gray_blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU
        )
        lines = cv2.HoughLinesP(
            edges,
            rho=1,
            theta=np.pi / 180,
            threshold=80,
            minLineLength=60,
            maxLineGap=15,
        )
        display = image.copy()
        if lines is None:
            self.reset_reacquisition()
            self.publish_debug_images(display, gray_blur, binary, edges)
            return 0.0, 0.0, False, 0.0

        relative_path_heading = wrap180(
            self.path_direction_deg - self.vehicle_heading_deg + self.camera_angle_offset
        )
        # 图像 X 轴为0°；车体前进方向投影到图像中接近竖直方向。
        directed_path_axis_image = wrap180(90.0 - relative_path_heading)
        path_axis_image = undirected_angle(directed_path_axis_image)
        cross_axis_image = undirected_angle(directed_path_axis_image + 90.0)
        self.publish_run_axis(path_axis_image)
        parallel_group = []
        perpendicular_group = []
        for raw_line in lines:
            x1, y1, x2, y2 = [int(value) for value in raw_line[0]]
            length = math.hypot(x2 - x1, y2 - y1)
            line_angle = undirected_angle(
                math.degrees(math.atan2(y2 - y1, x2 - x1))
            )
            center_x = (x1 + x2) / 2.0
            center_y = (y1 + y2) / 2.0
            record = (x1, y1, x2, y2, length, line_angle, center_x, center_y)
            parallel_error = undirected_angle_distance(line_angle, path_axis_image)
            perpendicular_error = undirected_angle_distance(line_angle, cross_axis_image)
            if parallel_error <= self.line_angle_tolerance_deg:
                parallel_group.append(record)
            elif perpendicular_error <= self.line_angle_tolerance_deg:
                perpendicular_group.append(record)

        pair = None
        if len(parallel_group) >= self.min_line_count:
            pair = select_boundary_pair(
                parallel_group,
                directed_path_axis_image,
                width,
                height,
                self.boundary_pair_max_gap_px,
            )
        cross_angle = weighted_line_angle(perpendicular_group)
        parallel_angle = weighted_line_angle(parallel_group)
        valid_geometry = (
            len(parallel_group) >= self.min_line_count
            and len(perpendicular_group) >= self.min_line_count
            and pair is not None
            and cross_angle is not None
            and parallel_angle is not None
        )
        if not valid_geometry:
            self.reset_reacquisition()
            self.publish_debug_images(display, gray_blur, binary, edges)
            return 0.0, 0.0, False, 0.0

        self.valid_streak += 1
        heading_error = wrap180(cross_angle - cross_axis_image)
        if heading_error > 90.0:
            heading_error -= 180.0
        elif heading_error < -90.0:
            heading_error += 180.0

        _, _, left_projection, right_projection, center_projection = pair
        lateral_pixel_error = (left_projection + right_projection) / 2.0 - center_projection
        lateral_m = self.pixels_to_lateral_meters(lateral_pixel_error)

        for line in parallel_group:
            cv2.line(display, (line[0], line[1]), (line[2], line[3]), (0, 255, 0), 2)
        for line in perpendicular_group:
            cv2.line(display, (line[0], line[1]), (line[2], line[3]), (255, 0, 0), 2)

        detected = self.valid_streak >= self.reacquire_frames
        confidence = 0.0
        if detected:
            group_score = min(
                1.0, (len(parallel_group) + len(perpendicular_group)) / 10.0
            )
            confidence = max(0.0, min(1.0, 0.5 * group_score + 0.5))
        cv2.putText(
            display, f'Angle: {heading_error:.2f} deg', (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2
        )
        cv2.putText(
            display, f'Lat Dev: {lateral_m:.3f} m', (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2
        )
        cv2.putText(
            display, f'P:{len(parallel_group)} C:{len(perpendicular_group)}', (10, 90),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
        )
        self.publish_debug_images(display, gray_blur, binary, edges)
        if not detected:
            return 0.0, 0.0, False, 0.0
        return heading_error, lateral_m, True, confidence

    def pixels_to_lateral_meters(self, lateral_pixel_error):
        pitch_rad = math.radians(float(self.camera_pitch_deg))
        z_dist = float(self.camera_height) / max(math.sin(pitch_rad), 1e-6)
        return float(lateral_pixel_error) * z_dist / max(float(self.focal_length_px), 1e-6)

    def publish_debug_images(self, display, gray, binary, edges):
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
