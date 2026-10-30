"""W200B-8 magnetic navigation sensor driver and geometry helpers."""
from __future__ import annotations
from typing import Optional
try:
    import serial
except ImportError:  # Allows protocol helpers to run on development hosts without pyserial.
    serial = None
try:
    from rclpy.node import Node
    from std_msgs.msg import Bool, Float32, UInt16
except ImportError:  # pragma: no cover - ROS runtime supplies these.
    Node = object
    Bool = Float32 = UInt16 = object

def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc

def build_read_frame(device_id: int = 0x08) -> bytes:
    body = bytes((device_id, 0x03, 0x00, 0x01, 0x00, 0x01))
    crc = crc16_modbus(body)
    return body + bytes((crc & 0xFF, (crc >> 8) & 0xFF))

def decode_response(frame: bytes, device_id: int = 0x08) -> Optional[int]:
    if len(frame) != 7 or frame[0] != device_id or frame[1] != 0x03 or frame[2] != 0x02:
        return None
    if crc16_modbus(frame[:-2]) != frame[-2] | (frame[-1] << 8):
        return None
    return (frame[3] << 8) | frame[4]

def calculate_magnetic_error(mask: int) -> Optional[float]:
    active = [index for index in range(8) if mask & (1 << index)]
    if not active:
        return None
    return sum(index - 3.5 for index in active) / len(active)

class MagneticNavigationNode(Node):
    def __init__(self):
        super().__init__("magnetic_navigation")
        self.declare_parameter("serial_port", "/dev/magnetic_nav")
        self.declare_parameter("baud_rate", 9600)
        self.declare_parameter("device_id", 8)
        self.declare_parameter("poll_interval", 0.05)
        self.declare_parameter("read_timeout", 0.05)
        self.port = self.get_parameter("serial_port").value
        self.device_id = int(self.get_parameter("device_id").value)
        self.serial = None
        self.error_pub = self.create_publisher(Float32, "/magnetic/error", 10)
        self.valid_pub = self.create_publisher(Bool, "/magnetic/valid", 10)
        self.mask_pub = self.create_publisher(UInt16, "/magnetic/mask", 10)
        self._open_serial()
        self.create_timer(float(self.get_parameter("poll_interval").value), self._poll)

    def _open_serial(self):
        try:
            if serial is None:
                raise RuntimeError("pyserial is required for the magnetic navigation node")
            self.serial = serial.Serial(self.port, int(self.get_parameter("baud_rate").value), timeout=float(self.get_parameter("read_timeout").value))
            self.get_logger().info(f"磁导航串口已打开: {self.port}")
        except (getattr(serial, "SerialException", OSError), OSError) as exc:
            self.get_logger().error(f"磁导航串口打开失败: {self.port}: {exc}")
            self.serial = None

    def _publish_invalid(self):
        valid = Bool(); valid.data = False; self.valid_pub.publish(valid)

    def _poll(self):
        if self.serial is None or not self.serial.is_open:
            self._publish_invalid(); return
        try:
            self.serial.reset_input_buffer(); self.serial.write(build_read_frame(self.device_id)); register = decode_response(self.serial.read(7), self.device_id)
        except (getattr(serial, "SerialException", OSError), OSError) as exc:
            self.get_logger().error(f"磁导航读取失败: {exc}"); self._publish_invalid(); return
        if register is None:
            self._publish_invalid(); return
        mask = register & 0xFF; error = calculate_magnetic_error(mask)
        mask_msg = UInt16(); mask_msg.data = register; self.mask_pub.publish(mask_msg)
        valid = Bool(); valid.data = error is not None; self.valid_pub.publish(valid)
        if error is not None:
            msg = Float32(); msg.data = float(error); self.error_pub.publish(msg)

def run_standalone():
    """Poll the sensor without ROS, for bring-up and serial diagnostics."""
    import argparse
    import time
    parser = argparse.ArgumentParser()
    parser.add_argument("--serial-port", default="/dev/magnetic_nav")
    parser.add_argument("--baud-rate", type=int, default=9600)
    parser.add_argument("--device-id", type=int, default=8)
    args = parser.parse_args()
    if serial is None:
        print("pyserial 未安装，无法访问磁导航串口")
        return 2
    try:
        stream = serial.Serial(args.serial_port, args.baud_rate, timeout=0.2)
    except Exception as exc:
        print(f"磁导航串口打开失败: {args.serial_port}: {exc}")
        return 1
    print(f"磁导航独立模式: {args.serial_port} @ {args.baud_rate}", flush=True)
    try:
        while True:
            stream.reset_input_buffer(); stream.write(build_read_frame(args.device_id)); stream.flush()
            register = decode_response(stream.read(7), args.device_id)
            if register is None:
                print("无有效响应", flush=True)
            else:
                print(f"register=0x{register:04x} mask=0x{register & 0xff:02x} error={calculate_magnetic_error(register & 0xff)}", flush=True)
            time.sleep(0.5)
    except KeyboardInterrupt:
        return 0
    finally:
        stream.close()

def main(args=None):
    try:
        import rclpy
    except ImportError:
        return run_standalone()
    rclpy.init(args=args); node = MagneticNavigationNode()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    raise SystemExit(main())
