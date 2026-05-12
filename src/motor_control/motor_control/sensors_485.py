#!/usr/bin/env python3
import struct

import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import UInt8


class Sensors485(Node):
    def __init__(self):
        super().__init__('sensors_485_node')

        self.declare_parameter('serial_port', '/dev/ttyS2')
        self.declare_parameter('io_baudrate', 9600)
        self.declare_parameter('poll_interval', 0.1)
        self.declare_parameter('timeout', 0.1)
        self.declare_parameter('device_addr', 0x01)
        self.declare_parameter('io_channel_count', 8)

        self.port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.io_baudrate = self.get_parameter('io_baudrate').get_parameter_value().integer_value
        self.poll_interval = self.get_parameter('poll_interval').value
        self.timeout = self.get_parameter('timeout').value
        self.device_addr = self.get_parameter('device_addr').get_parameter_value().integer_value
        self.io_channel_count = self.get_parameter('io_channel_count').get_parameter_value().integer_value

        self.io_channel_count = max(1, min(self.io_channel_count, 8))
        self.io_func_code = 0x04
        self.io_cmd = self.build_modbus_request(
            self.device_addr,
            self.io_func_code,
            0x0000,
            self.io_channel_count,
        )
        self.ser = None
        self.last_io_poll_time = 0.0
        self.last_reconnect_time = 0.0
        self.reconnect_interval = 1.0
        self.buffer = bytearray()

        self.io_pub = self.create_publisher(UInt8, '/io_data', 1)

        self.init_serial()
        self.polling_timer = self.create_timer(0.01, self.polling_callback)

    def init_serial(self):
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.io_baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout,
            )
            self.buffer.clear()
            self.get_logger().info(
                f"Connected IO serial: {self.port}, baudrate={self.io_baudrate}"
            )
            return True
        except Exception as exc:
            self.get_logger().error(f"IO serial init failed: {exc}")
            return False

    def polling_callback(self):
        now = self.get_clock().now().nanoseconds / 1e9

        if (not self.ser or not self.ser.is_open) and now - self.last_reconnect_time >= self.reconnect_interval:
            self.last_reconnect_time = now
            self.init_serial()
            return

        if not self.ser or not self.ser.is_open:
            return

        try:
            if now - self.last_io_poll_time >= self.poll_interval:
                self.ser.write(self.io_cmd)
                self.last_io_poll_time = now

            bytes_available = self.ser.in_waiting
            if bytes_available > 0:
                self.buffer += self.ser.read(min(bytes_available, 1024))
                self.process_buffer()
        except Exception as exc:
            self.get_logger().error(f"IO polling failed: {exc}")
            self.init_serial()

    def process_buffer(self):
        while len(self.buffer) >= 5:
            header_pos = self.find_frame_header()
            if header_pos < 0:
                self.buffer.clear()
                return

            if header_pos > 0:
                del self.buffer[:header_pos]

            if len(self.buffer) < 5:
                return

            if self.buffer[1] != self.io_func_code:
                del self.buffer[0]
                continue

            expected_length = 5 + self.buffer[2]
            if len(self.buffer) < expected_length:
                return

            frame = bytes(self.buffer[:expected_length])
            del self.buffer[:expected_length]

            parsed = self.parse_io_response(frame)
            if parsed is not None:
                msg = UInt8()
                msg.data = parsed
                self.io_pub.publish(msg)

    def find_frame_header(self):
        for idx, value in enumerate(self.buffer):
            if value == self.device_addr:
                return idx
        return -1

    def parse_io_response(self, data):
        if len(data) < 5 or data[1] != self.io_func_code:
            return None

        expected_length = 5 + data[2]
        if len(data) != expected_length:
            return None

        expected_byte_count = self.io_channel_count * 2
        if data[2] != expected_byte_count:
            self.get_logger().warn(
                f"Unexpected IO payload length: expected={expected_byte_count}, actual={data[2]}"
            )
            return None

        recv_crc = data[-2:]
        calc_crc = self.calculate_modbus_crc(data[:-2])
        if recv_crc != calc_crc:
            self.get_logger().warn("IO CRC check failed")
            return None

        io_bitmap = 0
        for channel in range(self.io_channel_count):
            reg_offset = 3 + channel * 2
            register_value = (data[reg_offset] << 8) | data[reg_offset + 1]
            if register_value != 0x0000:
                io_bitmap |= 1 << channel

        return io_bitmap

    @classmethod
    def build_modbus_request(cls, device_addr, func_code, start_addr, quantity):
        payload = bytes((
            device_addr & 0xFF,
            func_code & 0xFF,
            (start_addr >> 8) & 0xFF,
            start_addr & 0xFF,
            (quantity >> 8) & 0xFF,
            quantity & 0xFF,
        ))
        return payload + cls.calculate_modbus_crc(payload)

    @staticmethod
    def calculate_modbus_crc(data):
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return struct.pack('<H', crc)

    def destroy_node(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("IO serial connection closed.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Sensors485()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
