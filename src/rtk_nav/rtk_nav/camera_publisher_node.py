#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
图像发布节点：从 /dev/video0 或静态图片读取图像并发布到 /camera/color/image_raw
"""

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2


def load_static_image(image_path):
    """Load a static image as a BGR OpenCV frame."""
    frame = cv2.imread(image_path, cv2.IMREAD_COLOR)
    if frame is None:
        raise RuntimeError(f"无法读取图片文件: {image_path}")
    return frame


class CameraPublisherNode(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        # 获取参数
        self.declare_parameter('device_id', 0)  # 默认设备ID为0 (/dev/video0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('image_path', '')
        self.declare_parameter('pixel_format', '')
        self.declare_parameter('use_gstreamer', True)
        self.declare_parameter('io_mode', 0)

        self.device_id = self.get_parameter('device_id').get_parameter_value().integer_value
        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.height = self.get_parameter('height').get_parameter_value().integer_value
        self.fps = self.get_parameter('fps').get_parameter_value().integer_value
        self.image_path = self.get_parameter('image_path').get_parameter_value().string_value
        self.pixel_format = self.get_parameter('pixel_format').get_parameter_value().string_value
        self.use_gstreamer = self.get_parameter('use_gstreamer').get_parameter_value().bool_value
        self.io_mode = self.get_parameter('io_mode').get_parameter_value().integer_value

        # 创建CvBridge对象用于ROS图像和OpenCV图像之间的转换
        self.bridge = CvBridge()

        self.cap = None
        self.static_frame = None

        if self.image_path:
            self.static_frame = load_static_image(self.image_path)
            actual_height, actual_width = self.static_frame.shape[:2]
            actual_fps = self.fps
            self.get_logger().info(f"使用静态图片: {self.image_path}")
        else:
            # 打开摄像头设备
            self.get_logger().info(f"正在打开摄像头设备: /dev/video{self.device_id}")

            if self.use_gstreamer:
                # 使用GStreamer管道，参考系统中的test_camera.sh
                if self.pixel_format:
                    gst_str = f"v4l2src device=/dev/video{self.device_id} ! video/x-raw,format={self.pixel_format},width={self.width},height={self.height},framerate={self.fps}/1 ! videoconvert ! appsink"
                else:
                    gst_str = f"v4l2src device=/dev/video{self.device_id} ! video/x-raw,width={self.width},height={self.height},framerate={self.fps}/1 ! videoconvert ! appsink"
                self.get_logger().info(f"使用GStreamer管道: {gst_str}")
                self.cap = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)
            else:
                # 使用默认方式
                self.cap = cv2.VideoCapture(self.device_id)

            # 检查摄像头是否成功打开
            if not self.cap.isOpened():
                self.get_logger().error(f"无法打开摄像头设备: /dev/video{self.device_id}")
                # 尝试其他设备ID（参考test_camera.sh中的设备ID）
                test_devices = [69, 51, 42, 33, 31, 22, 11, 0]
                for device_id in test_devices:
                    if device_id == self.device_id:
                        continue
                    self.get_logger().info(f"尝试打开摄像头设备: /dev/video{device_id}")
                    if self.use_gstreamer:
                        if self.pixel_format:
                            gst_str = f"v4l2src device=/dev/video{device_id} ! video/x-raw,format={self.pixel_format},width={self.width},height={self.height},framerate={self.fps}/1 ! videoconvert ! appsink"
                        else:
                            gst_str = f"v4l2src device=/dev/video{device_id} ! video/x-raw,width={self.width},height={self.height},framerate={self.fps}/1 ! videoconvert ! appsink"
                        cap = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)
                    else:
                        cap = cv2.VideoCapture(device_id)

                    if cap.isOpened():
                        self.get_logger().info(f"成功打开摄像头设备: /dev/video{device_id}")
                        self.cap = cap
                        self.device_id = device_id
                        break
                    cap.release()
                else:
                    raise RuntimeError("无法打开任何摄像头设备")

            # 设置摄像头参数
            if self.pixel_format and not self.use_gstreamer:
                try:
                    self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.pixel_format))
                    self.get_logger().info(f"已设置像素格式为: {self.pixel_format}")
                except Exception as e:
                    self.get_logger().warn(f"设置像素格式失败: {str(e)}")

            if not self.use_gstreamer:
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                self.cap.set(cv2.CAP_PROP_FPS, self.fps)

            actual_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            actual_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            actual_fps = self.cap.get(cv2.CAP_PROP_FPS)

        # 打印实际设置的参数
        self.get_logger().info(
            f"摄像头参数: {self.width}x{self.height} @ {self.fps} fps "
            f"(实际: {int(actual_width)}x{int(actual_height)} @ {int(actual_fps)} fps)"
        )

        # 发布图像话题
        self.image_pub = self.create_publisher(Image, "/camera/color/image_raw", 10)

        # 创建定时器
        self.timer = self.create_timer(1.0/self.fps, self.timer_callback)

        self.frame_count = 0

    def timer_callback(self):
        """定时器回调函数"""
        if self.static_frame is not None:
            frame = self.static_frame
        else:
            # 读取摄像头帧
            ret, frame = self.cap.read()

            if not ret:
                self.get_logger().warn("无法从摄像头读取帧", throttle_duration_sec=1)
                return

        # 每30帧记录一次日志
        self.frame_count += 1
        if self.frame_count % 30 == 0:
            self.get_logger().debug(f"成功读取帧: {frame.shape[1]}x{frame.shape[0]}")

        try:
            # 将OpenCV图像转换为ROS图像消息
            image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")

            # 添加时间戳
            image_msg.header.stamp = self.get_clock().now().to_msg()
            image_msg.header.frame_id = "camera_frame"

            # 发布图像消息
            self.image_pub.publish(image_msg)

        except Exception as e:
            self.get_logger().error(f"图像转换或发布过程中出现错误: {str(e)}")

    def destroy_node(self):
        # 释放摄像头资源
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = CameraPublisherNode()
        rclpy.spin(node)
    except Exception as e:
        print(f"节点运行出错: {str(e)}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
