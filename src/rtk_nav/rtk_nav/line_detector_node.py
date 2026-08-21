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

        self.path_direction_deg = 0.0
        self.vehicle_heading_deg = 0.0
        self.path_context_valid = False
        self.last_path_context_time = 0.0
        self.last_path_direction_deg = None
        self.valid_streak = 0

        self.last_process_time = self.get_clock().now()
        self.min_process_interval = 0.1
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        """保留定时器接口，图像回调负责实际处理。"""

    def path_context_callback(self, msg):
        """接收 RTK 当前路径方向和车体航向。"""
        try:
            path_direction = float(msg.x)
            vehicle_heading = float(msg.y)
            valid = (
                math.isfinite(path_direction)
                and math.isfinite(vehicle_heading)
                and float(msg.z) >= 0.5
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

        current_time = self.get_clock().now()
        if (
            current_time - self.last_process_time
        ).nanoseconds / 1e9 < self.min_process_interval:
            return

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
            self.last_process_time = current_time
        except Exception as exc:
            self.get_logger().error(f'图像处理错误: {exc}')
            self.reset_reacquisition()
            self.publish_invalid()

    def pixels_to_lateral_meters(self, lateral_pixel_error):
        """将路径法向像素误差换算为米制横向误差。"""
        pitch_rad = math.radians(float(self.camera_pitch_deg))
        z_dist = float(self.camera_height) / max(math.sin(pitch_rad), 1e-6)
        return float(lateral_pixel_error) * z_dist / max(
            float(self.focal_length_px), 1e-6
        )

    def detect_and_draw_grid_lines(self, image):
        """检测路径平行/垂直线组并计算纠偏量。"""
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
            self.path_direction_deg - self.vehicle_heading_deg
        )
        directed_path_axis_image = wrap180(
            90.0 - relative_path_heading + self.camera_angle_offset
        )
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
            record = (
                x1,
                y1,
                x2,
                y2,
                length,
                line_angle,
                center_x,
                center_y,
            )
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

        pair = None
        if len(parallel_group) >= self.min_line_count:
            pair = select_boundary_pair(
                parallel_group,
                directed_path_axis_image,
                width,
                height,
                self.boundary_pair_max_gap_px,
            )

        parallel_angle = weighted_line_angle(parallel_group)
        cross_angle = weighted_line_angle(perpendicular_group)
        valid_geometry = (
            len(parallel_group) >= self.min_line_count
            and len(perpendicular_group) >= self.min_line_count
            and pair is not None
            and parallel_angle is not None
            and cross_angle is not None
            and math.isfinite(float(parallel_angle))
            and math.isfinite(float(cross_angle))
        )

        if not valid_geometry:
            self.reset_reacquisition()
            self.publish_debug_images(display, gray_blur, binary, edges)
            return 0.0, 0.0, False, 0.0

        self.valid_streak += 1
        heading_error = undirected_angle(cross_angle - cross_axis_image)
        _, _, left_projection, right_projection, center_projection = pair
        lateral_pixel_error = (
            (left_projection + right_projection) / 2.0 - center_projection
        )
        lateral_m = self.pixels_to_lateral_meters(lateral_pixel_error)

        detected = self.valid_streak >= self.reacquire_frames
        output_angle = heading_error if detected else 0.0
        output_lateral = lateral_m if detected else 0.0
        confidence = 0.0
        if detected:
            group_score = min(
                1.0,
                (len(parallel_group) + len(perpendicular_group)) / 10.0,
            )
            confidence = max(0.0, min(1.0, 0.5 + 0.5 * group_score))

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
        self.publish_debug_images(display, gray_blur, binary, edges)

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
