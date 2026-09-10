#!/usr/bin/env python3
"""将栅格线检测标注图编码并推送到 RTSP 媒体服务器。"""

import math
import subprocess
import threading
import time
from urllib.parse import urlparse

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image


def validate_rtsp_url(rtsp_url):
    """校验 RTSP 发布地址，避免启动后才发现配置错误。"""
    value = str(rtsp_url or '').strip()
    parsed = urlparse(value)
    if parsed.scheme.lower() != 'rtsp' or not parsed.netloc:
        raise ValueError(
            'rtsp_url must be a non-empty URL such as '
            'rtsp://media-server:8554/live/grid_line'
        )
    return value


def build_ffmpeg_command(
    width,
    height,
    fps,
    bitrate,
    preset,
    rtsp_url,
    ffmpeg_binary='ffmpeg',
):
    """构造无 shell 的 FFmpeg 原始 BGR 到 RTSP 命令。"""
    width = int(width)
    height = int(height)
    fps = float(fps)
    bitrate = str(bitrate).strip()
    preset = str(preset).strip()
    rtsp_url = validate_rtsp_url(rtsp_url)
    ffmpeg_binary = str(ffmpeg_binary).strip()
    if width <= 0 or height <= 0:
        raise ValueError('image width and height must be positive')
    if not math.isfinite(fps) or fps <= 0.0:
        raise ValueError('fps must be finite and positive')
    if not bitrate:
        raise ValueError('bitrate must not be empty')
    if not preset:
        raise ValueError('preset must not be empty')
    if not ffmpeg_binary:
        raise ValueError('ffmpeg_path must not be empty')
    return [
        str(ffmpeg_binary),
        '-hide_banner',
        '-loglevel',
        'warning',
        '-f',
        'rawvideo',
        '-pix_fmt',
        'bgr24',
        '-s',
        f'{width}x{height}',
        '-r',
        f'{fps:g}',
        '-i',
        'pipe:0',
        '-an',
        '-c:v',
        'libx264',
        '-preset',
        preset,
        '-tune',
        'zerolatency',
        '-pix_fmt',
        'yuv420p',
        '-b:v',
        bitrate,
        '-f',
        'rtsp',
        '-rtsp_transport',
        'tcp',
        rtsp_url,
    ]


class GridLineStreamer(Node):
    """订阅最新检测图像，并在独立线程中维护 FFmpeg 推流进程。"""

    def __init__(self):
        super().__init__('grid_line_streamer')

        self.declare_parameter('image_topic', '/grid_line/detected_image')
        self.declare_parameter('rtsp_url', '')
        self.declare_parameter('fps', 10.0)
        self.declare_parameter('bitrate', '800k')
        self.declare_parameter('preset', 'veryfast')
        self.declare_parameter('reconnect_sec', 2.0)
        self.declare_parameter('ffmpeg_path', 'ffmpeg')

        self.image_topic = str(self.get_parameter('image_topic').value)
        self.rtsp_url = validate_rtsp_url(self.get_parameter('rtsp_url').value)
        self.fps = float(self.get_parameter('fps').value)
        self.bitrate = str(self.get_parameter('bitrate').value)
        self.preset = str(self.get_parameter('preset').value)
        self.reconnect_sec = float(self.get_parameter('reconnect_sec').value)
        self.ffmpeg_path = str(self.get_parameter('ffmpeg_path').value)
        if not math.isfinite(self.fps) or self.fps <= 0.0:
            raise ValueError('fps must be finite and positive')
        if not math.isfinite(self.reconnect_sec) or self.reconnect_sec <= 0.0:
            raise ValueError('reconnect_sec must be finite and positive')

        self.bridge = CvBridge()
        self.frame_condition = threading.Condition()
        self.latest_message = None
        self.latest_generation = 0
        self.stop_event = threading.Event()
        self.process_lock = threading.Lock()
        self.process = None
        self.worker = threading.Thread(
            target=self.stream_worker,
            name='grid-line-rtsp-streamer',
            daemon=True,
        )
        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            image_qos,
        )
        self.worker.start()

    def image_callback(self, message):
        """仅替换待处理帧，不在 ROS 回调中执行转换或网络 I/O。"""
        with self.frame_condition:
            self.latest_message = message
            self.latest_generation += 1
            self.frame_condition.notify()

    def stream_worker(self):
        """按目标帧率发送最新帧，并在连接失败后自动重连。"""
        last_frame = None
        last_generation = 0
        frame_size = None
        retry_at = 0.0
        next_send_at = 0.0
        frame_interval = 1.0 / self.fps

        while not self.stop_event.is_set():
            now = time.monotonic()
            with self.frame_condition:
                if self.latest_message is None and last_frame is None:
                    self.frame_condition.wait(timeout=0.5)
                    continue
                if now < retry_at:
                    self.frame_condition.wait(timeout=retry_at - now)
                    continue
                message = None
                if self.latest_generation != last_generation:
                    message = self.latest_message
                    last_generation = self.latest_generation

            if message is not None:
                try:
                    frame = self.bridge.imgmsg_to_cv2(
                        message, desired_encoding='bgr8'
                    )
                    frame = np.asarray(frame)
                    if (
                        frame.ndim != 3
                        or frame.shape[2] != 3
                        or frame.dtype != np.uint8
                    ):
                        raise ValueError('converted image is not an 8-bit BGR frame')
                    last_frame = np.ascontiguousarray(frame)
                    new_frame_size = (last_frame.shape[1], last_frame.shape[0])
                    if frame_size != new_frame_size:
                        self.stop_process()
                        frame_size = new_frame_size
                        next_send_at = 0.0
                except Exception as exc:
                    self.get_logger().warning(
                        f'检测图像转换失败，丢弃当前帧: {exc}',
                        throttle_duration_sec=2.0,
                    )
                    continue

            if last_frame is None:
                continue

            process = self.get_process()
            if process is None:
                if time.monotonic() < retry_at:
                    continue
                try:
                    process = self.start_process(*frame_size)
                    if self.stop_event.is_set():
                        self.stop_process(process)
                        break
                    retry_at = 0.0
                    next_send_at = time.monotonic()
                    self.get_logger().info(
                        f'栅格线检测 RTSP 推流已启动: {frame_size[0]}x{frame_size[1]} '
                        f'@ {self.fps:g} FPS'
                    )
                except OSError as exc:
                    retry_at = time.monotonic() + self.reconnect_sec
                    self.get_logger().error(
                        f'无法启动 FFmpeg，请确认已安装 ffmpeg: {exc}',
                        throttle_duration_sec=5.0,
                    )
                    continue
                except ValueError as exc:
                    self.get_logger().error(str(exc), throttle_duration_sec=5.0)
                    self.stop_event.set()
                    continue

            return_code = process.poll()
            if return_code is not None:
                self.stop_process(process)
                retry_at = time.monotonic() + self.reconnect_sec
                self.get_logger().warning(
                    f'FFmpeg 推流进程已退出(code={return_code})，将在 '
                    f'{self.reconnect_sec:g} 秒后重连',
                    throttle_duration_sec=2.0,
                )
                continue

            wait_time = next_send_at - time.monotonic()
            if wait_time > 0.0:
                self.stop_event.wait(wait_time)
                continue
            try:
                process.stdin.write(last_frame.tobytes())
                process.stdin.flush()
                next_send_at = time.monotonic() + frame_interval
            except (BrokenPipeError, OSError, ValueError) as exc:
                self.stop_process(process)
                retry_at = time.monotonic() + self.reconnect_sec
                self.get_logger().warning(
                    f'RTSP 推流连接断开，将在 {self.reconnect_sec:g} 秒后重连: {exc}',
                    throttle_duration_sec=2.0,
                )

        self.stop_process()

    def start_process(self, width, height):
        """启动一条与当前图像尺寸匹配的 FFmpeg 子进程。"""
        command = build_ffmpeg_command(
            width,
            height,
            self.fps,
            self.bitrate,
            self.preset,
            self.rtsp_url,
            ffmpeg_binary=self.ffmpeg_path,
        )
        process = subprocess.Popen(
            command,
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            shell=False,
        )
        with self.process_lock:
            if self.stop_event.is_set():
                process.terminate()
                return process
            self.process = process
        return process

    def get_process(self):
        """读取当前 FFmpeg 进程引用。"""
        with self.process_lock:
            return self.process

    def stop_process(self, process=None):
        """关闭当前或指定的 FFmpeg 子进程，允许重复调用。"""
        if process is None:
            process = self.get_process()
        if process is None:
            return
        try:
            if process.stdin is not None:
                process.stdin.close()
        except (BrokenPipeError, OSError, ValueError):
            pass
        try:
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=1.0)
        except (OSError, ValueError, subprocess.TimeoutExpired):
            try:
                process.kill()
                process.wait(timeout=1.0)
            except (OSError, ValueError, subprocess.TimeoutExpired):
                pass
        with self.process_lock:
            if self.process is process:
                self.process = None

    def destroy_node(self):
        """停止工作线程和 FFmpeg 子进程后释放 ROS 资源。"""
        self.stop_event.set()
        with self.frame_condition:
            self.frame_condition.notify_all()
        self.stop_process()
        if self.worker.is_alive() and threading.current_thread() is not self.worker:
            self.worker.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    """启动主机端栅格线 RTSP 推流节点。"""
    rclpy.init(args=args)
    node = None
    try:
        node = GridLineStreamer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
