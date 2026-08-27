#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Read /dev/video0 and publish JPEG frames for the vision pipeline."""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
import cv2


def load_static_image(image_path):
    """Load a static image as a BGR OpenCV frame."""
    frame = cv2.imread(image_path, cv2.IMREAD_COLOR)
    if frame is None:
        raise RuntimeError(f"无法读取图片文件: {image_path}")
    return frame


def resize_frame_to_output(frame, width, height):
    """Resize a captured BGR frame without cropping or adding borders."""
    target_width = int(width)
    target_height = int(height)
    if target_width <= 0 or target_height <= 0:
        raise ValueError("output image dimensions must be positive")
    if frame.shape[1] == target_width and frame.shape[0] == target_height:
        return frame
    return cv2.resize(
        frame,
        (target_width, target_height),
        interpolation=cv2.INTER_AREA,
    )


def extract_translated_roi(
    source,
    crop_x,
    crop_y,
    output_width,
    output_height,
    translate_x,
    translate_y,
):
    """Extract a fixed-size ROI using only valid pixels from the source image."""
    source_height, source_width = source.shape[:2]
    output_width = int(output_width)
    output_height = int(output_height)
    if output_width <= 0 or output_height <= 0:
        raise ValueError("output image dimensions must be positive")
    if output_width > source_width or output_height > source_height:
        raise ValueError("source image is smaller than the requested output")

    max_x = source_width - output_width
    max_y = source_height - output_height
    origin_x = int(round(float(crop_x) + float(translate_x)))
    origin_y = int(round(float(crop_y) + float(translate_y)))
    origin_x = max(0, min(max_x, origin_x))
    origin_y = max(0, min(max_y, origin_y))
    return source[
        origin_y:origin_y + output_height,
        origin_x:origin_x + output_width,
    ].copy()


def build_gstreamer_pipeline(device_id, pixel_format=''):
    """Read the device's native frame size and resize after capture."""
    source_caps = 'video/x-raw'
    if pixel_format:
        source_caps += f',format={pixel_format}'
    return (
        f'v4l2src device=/dev/video{int(device_id)} ! {source_caps} ! '
        'videoconvert ! video/x-raw,format=BGR ! '
        'appsink max-buffers=1 drop=true sync=false'
    )


class CameraPublisherNode(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        # 获取参数
        self.declare_parameter('device_id', 0)  # 默认设备ID为0 (/dev/video0)
        self.declare_parameter('width', 360)
        self.declare_parameter('height', 640)
        self.declare_parameter('fps', 30)
        self.declare_parameter('image_path', '')
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('crop_x', 0)
        self.declare_parameter('crop_y', 0)
        self.declare_parameter('translate_x', 0)
        self.declare_parameter('translate_y', 0)
        self.declare_parameter('pixel_format', '')
        self.declare_parameter('use_gstreamer', True)
        self.declare_parameter('io_mode', 0)

        self.device_id = self.get_parameter('device_id').get_parameter_value().integer_value
        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.height = self.get_parameter('height').get_parameter_value().integer_value
        self.fps = self.get_parameter('fps').get_parameter_value().integer_value
        self.image_path = self.get_parameter('image_path').get_parameter_value().string_value
        self.jpeg_quality = self.get_parameter('jpeg_quality').get_parameter_value().integer_value
        self.crop_x = self.get_parameter('crop_x').get_parameter_value().integer_value
        self.crop_y = self.get_parameter('crop_y').get_parameter_value().integer_value
        self.translate_x = self.get_parameter('translate_x').get_parameter_value().integer_value
        self.translate_y = self.get_parameter('translate_y').get_parameter_value().integer_value
        self.pixel_format = self.get_parameter('pixel_format').get_parameter_value().string_value
        self.use_gstreamer = self.get_parameter('use_gstreamer').get_parameter_value().bool_value
        self.io_mode = self.get_parameter('io_mode').get_parameter_value().integer_value

        if not 0 < self.jpeg_quality <= 100:
            raise ValueError('jpeg_quality must be in [1, 100]')

        self.cap = None
        self.static_frame = None

        if self.image_path:
            self.static_frame = load_static_image(self.image_path)
            extract_translated_roi(
                self.static_frame,
                self.crop_x,
                self.crop_y,
                self.width,
                self.height,
                self.translate_x,
                self.translate_y,
            )
            actual_height, actual_width = self.static_frame.shape[:2]
            actual_fps = self.fps
            self.get_logger().info(f"使用静态图片: {self.image_path}")
        else:
            # 打开摄像头设备
            self.get_logger().info(f"正在打开摄像头设备: /dev/video{self.device_id}")

            if self.use_gstreamer:
                # 使用GStreamer管道，参考系统中的test_camera.sh
                gst_str = build_gstreamer_pipeline(
                    self.device_id, self.pixel_format
                )
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
                        gst_str = build_gstreamer_pipeline(
                            device_id, self.pixel_format
                        )
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
                try:
                    self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                except Exception as exc:
                    self.get_logger().debug(f"设置相机缓冲区深度失败: {exc}")
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

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.image_pub = self.create_publisher(
            CompressedImage,
            "/camera/color/image_compressed",
            image_qos,
        )

        # 创建定时器
        self.timer = self.create_timer(1.0/self.fps, self.timer_callback)

        self.frame_count = 0

    def timer_callback(self):
        """定时器回调函数"""
        if self.static_frame is not None:
            frame = extract_translated_roi(
                self.static_frame,
                self.crop_x,
                self.crop_y,
                self.width,
                self.height,
                self.translate_x,
                self.translate_y,
            )
        else:
            # 读取摄像头帧
            ret, frame = self.cap.read()

            if not ret:
                self.get_logger().warn("无法从摄像头读取帧", throttle_duration_sec=1)
                return

        frame = resize_frame_to_output(frame, self.width, self.height)

        # 每30帧记录一次日志
        self.frame_count += 1
        if self.frame_count % 30 == 0:
            self.get_logger().debug(f"成功读取帧: {frame.shape[1]}x{frame.shape[0]}")

        try:
            encoded_ok, encoded = cv2.imencode(
                '.jpg',
                frame,
                [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality],
            )
            if not encoded_ok:
                raise RuntimeError('JPEG 编码失败')

            image_msg = CompressedImage()
            image_msg.header.stamp = self.get_clock().now().to_msg()
            image_msg.header.frame_id = "camera_frame"
            image_msg.format = 'jpeg'
            image_msg.data = encoded.tobytes()
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
