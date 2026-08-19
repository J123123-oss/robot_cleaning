#!/usr/bin/env python3
"""
栅格线检测节点：订阅相机RGB图像，检测栅格线并发布角度偏差。

输出（供 RTK Stanley 控制器做视觉纠偏）：
  /grid_line/angle_deviation  (Vector3)
      x = 航向偏差（度，已减安装偏置）
      y = 横向偏差（米，板缝线相对车体中线的横向偏移）
      z = 检测有效性标志（1.0=有效，0.0=未检测到线）
  /grid_line/detection_confidence (Float32)
      置信度 [0,1]，消费端可据此做慢速偏置积分/门控

标定参数（launch 或 YAML 覆盖，替代原硬编码 ANGLE_OFFSET）：
  camera_angle_offset  相机安装航向偏置（度）
  camera_height        相机距板面高度（米）
  camera_pitch_deg     相机俯角（度，相对水平面向下）
  focal_length_px      相机焦距（像素）
  min_line_count       判定"检测到线"所需的最少线段数
"""
import math
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32
import cv2
import numpy as np

class GridLineDetector(Node):
    def __init__(self):
        super().__init__('grid_line_detector')

        # 创建CvBridge对象用于ROS图像和OpenCV图像之间的转换
        self.bridge = CvBridge()

        # 订阅相机RGB图像话题
        self.image_sub = self.create_subscription(
            Image,
            "/camera/color/image_raw",
            self.image_callback,
            10)

        # 发布角度偏差话题（x=航向偏差°, y=横向偏差m, z=detected 0/1）
        self.angle_pub = self.create_publisher(Vector3, "/grid_line/angle_deviation", 10)

        # 发布检测置信度
        self.confidence_pub = self.create_publisher(Float32, "/grid_line/detection_confidence", 10)

        # 发布带标注的图像话题
        self.image_pub = self.create_publisher(Image, "/grid_line/detected_image", 10)

        # 发布灰度图像话题
        self.gray_pub = self.create_publisher(Image, "/grid_line/gray_image", 10)

        # 发布二值化图像话题
        self.binary_pub = self.create_publisher(Image, "/grid_line/binary_image", 10)

        # 发布边缘检测图像话题
        self.edges_pub = self.create_publisher(Image, "/grid_line/edges_image", 10)

        # 标定参数（launch/YAML 可覆盖，默认值为占位，需实测标定）
        self.declare_parameter('camera_angle_offset', 0.8)   # 相机安装航向偏置（度），原 ANGLE_OFFSET
        self.declare_parameter('camera_height', 0.5)          # 相机距板面高度（米）
        self.declare_parameter('camera_pitch_deg', 30.0)      # 相机俯角（度）
        self.declare_parameter('focal_length_px', 600.0)      # 相机焦距（像素）
        self.declare_parameter('min_line_count', 2)           # 判定检测有效的最少线段数
        self.declare_parameter('enable_visual_correction', False)  # 视觉纠偏总开关

        self.camera_angle_offset = self.get_parameter('camera_angle_offset').get_parameter_value().double_value
        self.camera_height = self.get_parameter('camera_height').get_parameter_value().double_value
        self.camera_pitch_deg = self.get_parameter('camera_pitch_deg').get_parameter_value().double_value
        self.focal_length_px = self.get_parameter('focal_length_px').get_parameter_value().double_value
        self.min_line_count = self.get_parameter('min_line_count').get_parameter_value().integer_value
        self.enable_visual_correction = self.get_parameter(
            'enable_visual_correction'
        ).get_parameter_value().bool_value

        # 存储角度偏差信息
        self.angle_deviation = Vector3()
        self.confidence = Float32()

        # 控制处理频率，避免占用过高CPU
        self.timer = self.create_timer(0.1, self.timer_callback)  # 限制为10Hz

        # 添加时间戳用于跳过处理
        self.last_process_time = self.get_clock().now()
        self.min_process_interval = 0.1  # 最小处理间隔为100ms

    def timer_callback(self):
        """定时器回调函数"""
        # 此函数为空，仅用于控制处理频率
        pass

    def image_callback(self, msg):
        """图像回调函数"""
        if not self.enable_visual_correction:
            self.angle_deviation.x = 0.0
            self.angle_deviation.y = 0.0
            self.angle_deviation.z = 0.0
            self.confidence.data = 0.0
            self.angle_pub.publish(self.angle_deviation)
            self.confidence_pub.publish(self.confidence)
            return

        # 检查是否需要处理此帧
        current_time = self.get_clock().now()
        if (current_time - self.last_process_time).nanoseconds / 1e9 < self.min_process_interval:
            return

        try:
            # 将ROS图像消息转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # 检测栅格线并计算角度偏差、横向偏差（米）、检测有效性、置信度
            angle, lateral_m, detected, confidence = self.detect_and_draw_grid_lines(cv_image)

            # 应用安装偏置修正（航向偏差减去安装角度误差）
            corrected_angle = float(angle) - self.camera_angle_offset

            self.angle_deviation.x = corrected_angle   # 航向偏差（度）
            self.angle_deviation.y = float(lateral_m)  # 横向偏差（米）
            self.angle_deviation.z = 1.0 if detected else 0.0  # 检测有效性标志

            # 发布角度偏差
            self.angle_pub.publish(self.angle_deviation)

            # 发布置信度
            self.confidence.data = float(confidence)
            self.confidence_pub.publish(self.confidence)

            # 更新处理时间
            self.last_process_time = current_time

        except Exception as e:
            self.get_logger().error(f"图像处理错误: {str(e)}")

    def pixels_to_lateral_meters(self, pos_dev_x, width):
        """将归一化横向偏移换算为米制横向偏差（针孔模型近似）

        lateral_m = (x_px - cx) * Z / fx
        pos_dev_x 已归一化到 [-1,1]，对应 x_px - cx = pos_dev_x * width/2
        Z = 相机到板面的斜距 ≈ camera_height / sin(pitch)
        """
        dx_px = pos_dev_x * (width / 2.0)
        pitch_rad = math.radians(self.camera_pitch_deg)
        z_dist = self.camera_height / max(math.sin(pitch_rad), 1e-6)
        lateral_m = dx_px * z_dist / max(self.focal_length_px, 1e-6)
        return lateral_m

    def detect_and_draw_grid_lines(self, image):
        """检测栅格线并计算与前进方向的角度偏差，同时在图像上绘制检测结果

        返回 (angle_deg, lateral_m, detected, confidence)
        """
        if not self.enable_visual_correction:
            return 0.0, 0.0, False, 0.0

        height, width = image.shape[:2]

        # 使用整个图像进行处理
        roi = image

        # 创建遮罩以屏蔽图像中心的8个亮斑（照明灯）
        mask = np.ones((height, width), dtype=np.uint8) * 255
        center_x, center_y = width // 2, height // 2
        # 在图像中心创建一个圆形遮罩区域，半径约为图像宽度的1/6
        radius = min(width, height) // 6
        cv2.circle(mask, (center_x, center_y), radius, 0, -1)

        # 直接使用原始图像，取消对比度增强处理
        # 转换为灰度图像
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

        # 应用遮罩以屏蔽中心亮斑
        # gray = cv2.bitwise_and(gray, gray, mask=mask)

        # 添加高斯模糊以降低背景噪声 - 使用新参数
        gray_blur = cv2.GaussianBlur(gray, (5, 5), 1.0)  # (3,3)核大小, σ=1.0

        # 使用形态学操作进一步降低噪声
        kernel = np.ones((3, 3), np.uint8)
        gray_morph = cv2.morphologyEx(gray_blur, cv2.MORPH_OPEN, kernel)

        # 应用更严格的二值化处理
        # 使用全局阈值结合自适应阈值来获得更好的效果
        _, binary_global = cv2.threshold(gray_morph, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        binary_adaptive = cv2.adaptiveThreshold(gray_morph, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)

        # 结合两种二值化结果
        binary = cv2.bitwise_and(binary_global, binary_adaptive)

        # 使用形态学闭运算连接断开的线条
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=1)

        # 使用优化的边缘检测参数 - 使用新参数
        edges = cv2.Canny(gray_blur, 50, 150, apertureSize=3)  # low=50, high=150, apertureSize=3

        # 使用优化的霍夫变换参数 - 使用新参数
        lines = cv2.HoughLinesP(
            edges,
            rho=1,              # rho=1
            theta=np.pi/180,    # theta=π/180
            threshold=100,      # threshold=100
            minLineLength=80,   # minLineLength=80
            maxLineGap=15       # maxLineGap=15
        )

        # 创建用于显示的图像副本
        display_image = roi.copy()

        if lines is None:
            self.get_logger().warn("未检测到直线")
            # 在图像上显示角度信息
            cv2.putText(display_image, "Angle: 0.00 degrees", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            # 发布带标注的图像
            try:
                annotated_image_msg = self.bridge.cv2_to_imgmsg(display_image, "bgr8")
                self.image_pub.publish(annotated_image_msg)

                # 发布灰度图像
                gray_image_msg = self.bridge.cv2_to_imgmsg(gray_blur, "mono8")
                self.gray_pub.publish(gray_image_msg)

                # 发布二值化图像
                binary_image_msg = self.bridge.cv2_to_imgmsg(binary, "mono8")
                self.binary_pub.publish(binary_image_msg)

                # 发布边缘检测图像
                edges_image_msg = self.bridge.cv2_to_imgmsg(edges, "mono8")
                self.edges_pub.publish(edges_image_msg)
            except Exception as e:
                self.get_logger().error(f"发布图像时出错: {str(e)}")
            return 0.0, 0.0, False, 0.0  # 无检测：角度0、横向0、无效、置信度0

        # 分析直线的角度分布
        angle_groups = {}
        line_info = []

        for line in lines:
            for x1, y1, x2, y2 in line:
                # 计算直线长度
                length = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)

                # 计算直线角度（相对于x轴）
                angle = np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi

                # 计算直线的中心点（用于位置偏差计算）
                line_center_x = (x1 + x2) / 2
                line_center_y = (y1 + y2) / 2

                # 将角度归一化到[-90, 90]范围
                normalized_angle = angle
                if normalized_angle < -45:
                    normalized_angle += 90
                elif normalized_angle > 45:
                    normalized_angle -= 90

                # 按角度分组（以5度为间隔）
                angle_group = round(normalized_angle / 5) * 5

                if angle_group not in angle_groups:
                    angle_groups[angle_group] = []
                angle_groups[angle_group].append((x1, y1, x2, y2, length, normalized_angle))
                line_info.append((x1, y1, x2, y2, length, normalized_angle, angle_group, line_center_x, line_center_y))

        # 找到包含最多线条的角度组
        max_group = None
        max_count = 0
        for group, lines_in_group in angle_groups.items():
            if len(lines_in_group) > max_count:
                max_count = len(lines_in_group)
                max_group = group

        # 只保留主要角度组的线条（允许±10度的偏差，比之前更严格）
        filtered_lines = []
        if max_group is not None:
            angle_tolerance = 10
            for x1, y1, x2, y2, length, normalized_angle, angle_group, line_center_x, line_center_y in line_info:
                if abs(normalized_angle - max_group) <= angle_tolerance:
                    filtered_lines.append((x1, y1, x2, y2, length, normalized_angle, line_center_x, line_center_y))

        # 如果过滤后线条太少，使用较宽松的条件
        if len(filtered_lines) < 2:  # 降低最低要求从3到2
            filtered_lines = []
            for x1, y1, x2, y2, length, normalized_angle, angle_group, line_center_x, line_center_y in line_info:
                # 提高长度要求，从70提高到90
                if length > 90:
                    filtered_lines.append((x1, y1, x2, y2, length, normalized_angle, line_center_x, line_center_y))

            # 如果还是太少，就使用所有线条
            if len(filtered_lines) < 2:  # 降低最低要求从3到2
                for x1, y1, x2, y2, length, normalized_angle, angle_group, line_center_x, line_center_y in line_info:
                    filtered_lines.append((x1, y1, x2, y2, length, normalized_angle, line_center_x, line_center_y))

        # 在图像上绘制过滤后的线条（需要调整坐标到完整图像）
        for x1, y1, x2, y2, length, angle, line_center_x, line_center_y in filtered_lines:
            # 使用不同颜色绘制线条，便于区分
            cv2.line(display_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            # 不在黑白图上绘制线条

        # 计算平均角度作为最终的角度偏差
        angles = [angle for x1, y1, x2, y2, length, angle, line_center_x, line_center_y in filtered_lines]
        mean_angle = np.mean(angles) if angles else 0.0

        # 计算位置偏差（基于检测到的直线中心点与图像中心的偏差）
        if filtered_lines:
            # 计算所有检测直线的中心点的平均位置
            avg_line_center_x = np.mean([line_center_x for x1, y1, x2, y2, length, angle, line_center_x, line_center_y in filtered_lines])
            avg_line_center_y = np.mean([line_center_y for x1, y1, x2, y2, length, angle, line_center_x, line_center_y in filtered_lines])

            # 计算与图像中心的偏差
            image_center_x = width / 2
            image_center_y = height / 2

            # 归一化位置偏差到[-1, 1]范围
            pos_dev_x = (avg_line_center_x - image_center_x) / (width / 2)
            pos_dev_y = (avg_line_center_y - image_center_y) / (height / 2)

            # 使用x方向的位置偏差换算成米制横向偏差
            lateral_m = self.pixels_to_lateral_meters(pos_dev_x, width)
        else:
            lateral_m = 0.0

        # 假设前进方向是水平的（0度），计算与前进方向的偏差
        # 如果相机和栅格面互相垂直，则检测到的直线角度即为偏差角度
        deviation_angle = float(mean_angle)

        # 检测有效性：过滤后的线段数达到阈值
        detected = len(filtered_lines) >= self.min_line_count

        # 置信度：主组线条占比 + 数量饱和度
        if detected and line_info:
            n_main = len(filtered_lines)
            ratio = n_main / float(len(line_info))
            count_score = min(1.0, n_main / 5.0)  # 5条以上视为饱和
            confidence = 0.5 * ratio + 0.5 * count_score
            confidence = max(0.0, min(1.0, confidence))
        else:
            confidence = 0.0

        # 在图像上显示角度信息
        cv2.putText(display_image, "Angle: {:.2f} degrees".format(deviation_angle), (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        # 显示检测到的线段数量
        cv2.putText(display_image, "Lines: {}".format(len(filtered_lines)), (10, 70),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        # 显示横向偏差信息
        cv2.putText(display_image, "Lat Dev: {:.3f} m".format(lateral_m), (10, 110),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

        # 发布带标注的图像
        try:
            annotated_image_msg = self.bridge.cv2_to_imgmsg(display_image, "bgr8")
            self.image_pub.publish(annotated_image_msg)

            # 发布灰度图像
            gray_image_msg = self.bridge.cv2_to_imgmsg(gray_blur, "mono8")
            self.gray_pub.publish(gray_image_msg)

            # 发布二值化图像
            binary_image_msg = self.bridge.cv2_to_imgmsg(binary, "mono8")
            self.binary_pub.publish(binary_image_msg)

            # 发布边缘检测图像
            edges_image_msg = self.bridge.cv2_to_imgmsg(edges, "mono8")
            self.edges_pub.publish(edges_image_msg)
        except Exception as e:
            self.get_logger().error(f"发布图像时出错: {str(e)}")

        self.get_logger().debug(
            f"检测到的角度偏差: {deviation_angle:.2f}度，横向偏差: {lateral_m:.3f}米，"
            f"线段数量: {len(filtered_lines)}，主要角度组: {str(max_group)}，置信度: {confidence:.2f}"
        )

        return deviation_angle, lateral_m, detected, confidence

def main(args=None):
    rclpy.init(args=args)

    try:
        detector = GridLineDetector()
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            detector.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
