#!/usr/bin/env python3
"""Read JPEG frames from OpenMV USB serial and publish them to ROS 2."""

import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

try:
    import serial
except ImportError:  # pragma: no cover - exercised on an unconfigured host
    serial = None

if __package__:
    from .openmv_serial_protocol import FRAME_PACKET_TYPE, FrameStreamDecoder
else:  # Support direct execution of this file during hardware bring-up.
    from openmv_serial_protocol import FRAME_PACKET_TYPE, FrameStreamDecoder


def is_jpeg_payload(payload):
    """Return whether a payload has the expected JPEG start/end markers."""
    return (
        len(payload) >= 4
        and payload[:2] == b"\xff\xd8"
        and payload[-2:] == b"\xff\xd9"
    )


class OpenMVSerialPublisherNode(Node):
    """Bridge OpenMV framed JPEG packets to a compressed ROS image topic."""

    def __init__(self):
        super().__init__("openmv_serial_publisher")

        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 921600)
        self.declare_parameter("read_timeout_sec", 0.2)
        self.declare_parameter("no_data_timeout_sec", 5.0)
        self.declare_parameter("reconnect_interval_sec", 1.0)
        self.declare_parameter("max_frame_bytes", 2 * 1024 * 1024)
        self.declare_parameter("read_chunk_bytes", 4096)
        self.declare_parameter("frame_id", "camera_frame")
        self.declare_parameter("topic", "/camera/color/image_compressed")

        self.serial_port = self.get_parameter("serial_port").value
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.read_timeout_sec = float(self.get_parameter("read_timeout_sec").value)
        self.no_data_timeout_sec = float(
            self.get_parameter("no_data_timeout_sec").value
        )
        self.reconnect_interval_sec = float(
            self.get_parameter("reconnect_interval_sec").value
        )
        self.max_frame_bytes = int(self.get_parameter("max_frame_bytes").value)
        self.read_chunk_bytes = int(self.get_parameter("read_chunk_bytes").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        topic = str(self.get_parameter("topic").value)

        if serial is None:
            raise RuntimeError("python3-serial is required for OpenMV serial input")
        if not self.serial_port:
            raise ValueError("serial_port must not be empty")
        if self.baudrate <= 0:
            raise ValueError("baudrate must be positive")
        if self.read_timeout_sec <= 0.0:
            raise ValueError("read_timeout_sec must be positive")
        if self.no_data_timeout_sec <= self.read_timeout_sec:
            raise ValueError(
                "no_data_timeout_sec must be greater than read_timeout_sec"
            )
        if self.reconnect_interval_sec <= 0.0:
            raise ValueError("reconnect_interval_sec must be positive")
        if self.max_frame_bytes <= 0:
            raise ValueError("max_frame_bytes must be positive")
        if self.read_chunk_bytes <= 0:
            raise ValueError("read_chunk_bytes must be positive")

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.image_pub = self.create_publisher(CompressedImage, topic, image_qos)

        self._decoder = FrameStreamDecoder(self.max_frame_bytes)
        self._serial = None
        self._last_data_time = 0.0
        self._stop_event = threading.Event()
        self._latest_lock = threading.Lock()
        self._latest_packet = None
        self._published_count = 0
        self._reader_thread = threading.Thread(
            target=self._read_loop,
            name="openmv-serial-reader",
            daemon=True,
        )
        self._reader_thread.start()
        self._publish_timer = self.create_timer(0.01, self._publish_latest_frame)

    def _try_open_serial(self):
        try:
            connection = serial.Serial(
                port=self.serial_port,
                baudrate=self.baudrate,
                timeout=self.read_timeout_sec,
            )
        except (serial.SerialException, OSError) as exc:
            self.get_logger().warning(
                f"无法打开 OpenMV 串口 {self.serial_port}: {exc}"
            )
            return False

        if self._stop_event.is_set():
            connection.close()
            return False

        self._serial = connection
        self._decoder.reset()
        self._last_data_time = time.monotonic()
        self.get_logger().info(
            f"已连接 OpenMV 串口: {self.serial_port} @ {self.baudrate}"
        )
        return True

    def _close_serial(self):
        connection = self._serial
        self._serial = None
        if connection is not None:
            try:
                connection.close()
            except (OSError, serial.SerialException):
                pass
        self._decoder.reset()

    def _read_loop(self):
        while not self._stop_event.is_set():
            if self._serial is None:
                self._try_open_serial()
                if self._serial is None:
                    self._stop_event.wait(self.reconnect_interval_sec)
                continue

            try:
                data = self._serial.read(self.read_chunk_bytes)
                if not data:
                    if (
                        time.monotonic() - self._last_data_time
                        >= self.no_data_timeout_sec
                    ):
                        self.get_logger().warning(
                            "OpenMV 串口连续无数据，将关闭并重连"
                        )
                        self._close_serial()
                        self._stop_event.wait(self.reconnect_interval_sec)
                    continue
                self._last_data_time = time.monotonic()
                for packet in self._decoder.feed(data):
                    if packet.packet_type != FRAME_PACKET_TYPE:
                        continue
                    if not is_jpeg_payload(packet.payload):
                        self.get_logger().warning(
                            "收到的 OpenMV 帧不是完整 JPEG，已丢弃"
                        )
                        continue
                    with self._latest_lock:
                        self._latest_packet = packet
            except (serial.SerialException, OSError) as exc:
                self.get_logger().warning(f"OpenMV 串口读取失败，将重连: {exc}")
                self._close_serial()
                self._stop_event.wait(self.reconnect_interval_sec)

    def _publish_latest_frame(self):
        with self._latest_lock:
            packet = self._latest_packet
            self._latest_packet = None
        if packet is None:
            return

        image_msg = CompressedImage()
        image_msg.header.stamp = self.get_clock().now().to_msg()
        image_msg.header.frame_id = self.frame_id
        image_msg.format = "jpeg"
        image_msg.data = packet.payload
        self.image_pub.publish(image_msg)

        self._published_count += 1
        if self._published_count % 30 == 0:
            self.get_logger().debug(
                f"已发布 OpenMV JPEG 帧: seq={packet.sequence}, "
                f"bytes={len(packet.payload)}"
            )

    def destroy_node(self):
        """Stop the reader and release the USB serial device."""
        self._stop_event.set()
        self._close_serial()
        if self._reader_thread.is_alive():
            self._reader_thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    """Run the OpenMV serial image publisher."""
    node = None
    exit_code = 0
    try:
        rclpy.init(args=args)
        node = OpenMVSerialPublisherNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        exit_code = 1
        if node is not None:
            node.get_logger().error(f"OpenMV 串口节点运行失败: {exc}")
        else:
            print(f"OpenMV 串口节点启动失败: {exc}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
