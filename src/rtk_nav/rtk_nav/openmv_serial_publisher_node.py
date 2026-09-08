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


LIGHT_ON_COMMAND = b"LIGHT_ON\n"
LIGHT_OFF_COMMAND = b"LIGHT_OFF\n"
LIGHT_HEARTBEAT_COMMAND = b"LIGHT_HEARTBEAT\n"
LIGHT_HEARTBEAT_INTERVAL_SEC = 0.5
LIGHT_STATUS_PACKET_TYPE = 2
LIGHT_STATUS_ON = b"LIGHT_ON"
LIGHT_STATUS_OFF = b"LIGHT_OFF"

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

        self.declare_parameter("serial_port", "/dev/OpenMV_Cam_H7_Plus")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("read_timeout_sec", 0.2)
        self.declare_parameter("no_data_timeout_sec", 5.0)
        self.declare_parameter("reconnect_interval_sec", 1.0)
        self.declare_parameter("max_frame_bytes", 2 * 1024 * 1024)
        self.declare_parameter("read_chunk_bytes", 4096)
        self.declare_parameter("frame_id", "camera_frame")
        # Publish on the standard image_transport compressed suffix.  Consumers
        # can select the base topic /camera/color/image with transport=compressed.
        self.declare_parameter("topic", "/camera/color/image/compressed")

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
        self._serial_lock = threading.Lock()
        self._last_data_time = 0.0
        self._stop_event = threading.Event()
        self._latest_lock = threading.Lock()
        self._latest_packet = None
        self._published_count = 0
        self._light_heartbeat_count = 0
        self._light_status_count = 0
        self._last_light_status = None
        self._reader_thread = threading.Thread(
            target=self._read_loop,
            name="openmv-serial-reader",
            daemon=True,
        )
        self._reader_thread.start()
        self._publish_timer = self.create_timer(0.01, self._publish_latest_frame)
        self._light_timer = self.create_timer(
            LIGHT_HEARTBEAT_INTERVAL_SEC, self._send_light_heartbeat
        )

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

        with self._serial_lock:
            if self._stop_event.is_set():
                connection.close()
                return False
            self._serial = connection
        self._light_heartbeat_count = 0
        self._light_status_count = 0
        self._last_light_status = None
        self._decoder.reset()
        self._last_data_time = time.monotonic()
        self.get_logger().info(
            f"已连接 OpenMV 串口: {self.serial_port} @ {self.baudrate}"
        )

        if not self._send_light_command(LIGHT_ON_COMMAND):
            self.get_logger().warning("OpenMV 补光灯开启命令发送失败")
            self._close_serial()
            return False
        self.get_logger().info("已发送 OpenMV 补光灯开启命令")
        return True

    def _close_serial(self):
        with self._serial_lock:
            connection = self._serial
            self._serial = None
            if connection is not None:
                try:
                    connection.close()
                except (OSError, serial.SerialException):
                    pass
        self._decoder.reset()

    def _send_light_command(self, command):
        """Send a light command while serial close/write operations are serialized."""
        connection = self._serial
        if connection is None:
            return False

        try:
            with self._serial_lock:
                if self._serial is not connection:
                    return False
                if self._stop_event.is_set() and command != LIGHT_OFF_COMMAND:
                    return False
                connection.write(command)
                connection.flush()
        except (serial.SerialException, OSError, TypeError):
            return False
        return True

    def _send_light_heartbeat(self):
        """Keep the camera-side fill-light lease alive while this node runs."""
        if self._stop_event.is_set() or self._serial is None:
            return
        if not self._send_light_command(LIGHT_HEARTBEAT_COMMAND):
            self.get_logger().warning("OpenMV 补光灯心跳发送失败，将重连")
            self._close_serial()
            return

        self._light_heartbeat_count += 1
        if self._light_heartbeat_count == 1:
            self.get_logger().info("已发送 OpenMV 补光灯首个心跳")

    def _handle_light_status(self, payload):
        """Log the camera confirmation without treating it as an image frame."""
        if self._stop_event.is_set():
            return

        if payload == LIGHT_STATUS_ON:
            self._light_status_count += 1
            if self._last_light_status != "ON":
                self.get_logger().info("OpenMV 已确认补光灯开启")
                self._last_light_status = "ON"
        elif payload == LIGHT_STATUS_OFF:
            if self._last_light_status != "OFF":
                self.get_logger().info("OpenMV 补光灯状态: OFF")
                self._last_light_status = "OFF"

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
                    if packet.packet_type == LIGHT_STATUS_PACKET_TYPE:
                        self._handle_light_status(packet.payload)
                        continue
                    if packet.packet_type != FRAME_PACKET_TYPE:
                        continue
                    if not is_jpeg_payload(packet.payload):
                        self.get_logger().warning(
                            "收到的 OpenMV 帧不是完整 JPEG，已丢弃"
                        )
                        continue
                    with self._latest_lock:
                        self._latest_packet = packet
            except (serial.SerialException, OSError, TypeError) as exc:
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
        if self._light_timer is not None:
            self._light_timer.cancel()
        self._send_light_command(LIGHT_OFF_COMMAND)
        if self._reader_thread.is_alive():
            # 先让 read() 因有限超时返回，再关闭串口，避免并发 close 导致
            # serialposix.read() 使用 None 文件描述符。
            self._reader_thread.join(timeout=max(2.0, self.read_timeout_sec * 2.0))
        self._close_serial()
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
