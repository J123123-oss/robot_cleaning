#!/usr/bin/env python3
"""Read JPEG frames from OpenMV USB serial and publish them to ROS 2."""

import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import CompressedImage

if __package__:
    from .openmv_image_transform import transform_jpeg_payload
else:  # Support direct execution of this file during hardware bring-up.
    from openmv_image_transform import transform_jpeg_payload

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
LIGHT_BRIGHTNESS_COMMAND_PREFIX = b"LIGHT_BRIGHTNESS="
LIGHT_BRIGHTNESS_STATUS_PREFIX = LIGHT_BRIGHTNESS_COMMAND_PREFIX
LIGHT_BRIGHTNESS_ERROR = b"LIGHT_BRIGHTNESS_ERROR"
LIGHT_BRIGHTNESS_MIN = 0
LIGHT_BRIGHTNESS_MAX = 100

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
        self.declare_parameter("light_brightness", 50)
        self.declare_parameter("read_timeout_sec", 0.2)
        self.declare_parameter("no_data_timeout_sec", 5.0)
        self.declare_parameter("reconnect_interval_sec", 1.0)
        self.declare_parameter("max_frame_bytes", 2 * 1024 * 1024)
        self.declare_parameter("read_chunk_bytes", 4096)
        self.declare_parameter("frame_id", "camera_frame")
        # Rotate the decoded image before republishing so camera mounting can
        # be corrected without changing the OpenMV firmware.
        self.declare_parameter("image_rotation_deg", 0)
        # Publish on the standard image_transport compressed suffix.  Consumers
        # can select the base topic /camera/color/image with transport=compressed.
        self.declare_parameter("topic", "/camera/color/image/compressed")

        self.serial_port = self.get_parameter("serial_port").value
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.light_brightness = int(self.get_parameter("light_brightness").value)
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
        self.image_rotation_deg = int(
            self.get_parameter("image_rotation_deg").value
        )
        topic = str(self.get_parameter("topic").value)

        if serial is None:
            raise RuntimeError("python3-serial is required for OpenMV serial input")
        if not self.serial_port:
            raise ValueError("serial_port must not be empty")
        if self.baudrate <= 0:
            raise ValueError("baudrate must be positive")
        if not LIGHT_BRIGHTNESS_MIN <= self.light_brightness <= LIGHT_BRIGHTNESS_MAX:
            raise ValueError("light_brightness must be between 0 and 100")
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
        if self.image_rotation_deg not in (0, 90, 180, 270):
            raise ValueError("image_rotation_deg must be one of 0, 90, 180, 270")

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
        self._light_brightness_status_count = 0
        self._last_light_brightness = None
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
        self.add_on_set_parameters_callback(self._on_parameter_set)

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
        self._light_brightness_status_count = 0
        self._last_light_brightness = None
        self._decoder.reset()
        self._last_data_time = time.monotonic()
        self.get_logger().info(
            f"已连接 OpenMV 串口: {self.serial_port} @ {self.baudrate}"
        )

        if not self._send_light_brightness(self.light_brightness):
            self.get_logger().warning("OpenMV 补光灯亮度命令发送失败")
            self._close_serial()
            return False
        self.get_logger().info(
            f"已发送 OpenMV 补光灯亮度: {self.light_brightness}%"
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

    def _send_light_brightness(self, brightness):
        """Send a validated runtime brightness command to the camera."""
        if not LIGHT_BRIGHTNESS_MIN <= brightness <= LIGHT_BRIGHTNESS_MAX:
            return False
        command = (
            LIGHT_BRIGHTNESS_COMMAND_PREFIX
            + str(brightness).encode()
            + b"\n"
        )
        return self._send_light_command(command)

    def _on_parameter_set(self, params):
        """Apply a brightness parameter change immediately or queue it for reconnect."""
        requested_brightness = None
        for parameter in params:
            if parameter.name != "light_brightness":
                continue
            try:
                requested_brightness = int(parameter.value)
            except (TypeError, ValueError, OverflowError):
                return SetParametersResult(
                    successful=False, reason="light_brightness must be an integer"
                )
            if not (
                LIGHT_BRIGHTNESS_MIN
                <= requested_brightness
                <= LIGHT_BRIGHTNESS_MAX
            ):
                return SetParametersResult(
                    successful=False,
                    reason="light_brightness must be between 0 and 100",
                )

        if requested_brightness is None:
            return SetParametersResult(successful=True)

        if self._serial is not None and not self._stop_event.is_set():
            if not self._send_light_brightness(requested_brightness):
                return SetParametersResult(
                    successful=False,
                    reason="failed to send brightness command to OpenMV",
                )
            self.get_logger().info(
                f"已下发 OpenMV 补光灯亮度: {requested_brightness}%"
            )
        else:
            self.get_logger().info(
                f"OpenMV 未连接，补光灯亮度 {requested_brightness}% 将在重连时下发"
            )

        self.light_brightness = requested_brightness
        return SetParametersResult(successful=True)

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
        elif payload.startswith(LIGHT_BRIGHTNESS_STATUS_PREFIX):
            try:
                brightness = int(
                    payload[len(LIGHT_BRIGHTNESS_STATUS_PREFIX):].decode()
                )
            except (TypeError, ValueError):
                return
            if not LIGHT_BRIGHTNESS_MIN <= brightness <= LIGHT_BRIGHTNESS_MAX:
                return
            self._light_brightness_status_count += 1
            if self._last_light_brightness != brightness:
                self.get_logger().info(
                    f"OpenMV 已确认补光灯亮度: {brightness}%"
                )
                self._last_light_brightness = brightness
        elif payload == LIGHT_BRIGHTNESS_ERROR:
            self.get_logger().warning("OpenMV 拒绝了补光灯亮度命令")

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
        transformed_payload = transform_jpeg_payload(
            packet.payload, self.image_rotation_deg
        )
        if transformed_payload is None:
            self.get_logger().warning(
                "OpenMV JPEG 解码或旋转失败，已丢弃当前帧"
            )
            return
        image_msg.data = transformed_payload
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
