#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import threading
import time

import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int16
from std_srvs.srv import Trigger

from custom_msgs.srv import ChargeControl


def crc16_modbus(data: bytes) -> bytes:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte & 0xFF
        for _ in range(8):
            if crc & 0x0001:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return bytes([crc & 0xFF, (crc >> 8) & 0xFF])


FAULT_CODE_MAP = {
    0x00: "无故障",  # 修复 KeyError:0
    0x01: "预留",
    0x02: "恒流区输出过流保护",
    0x03: "恒流区输出欠流保护",
    0x04: "VRECT过压硬件保护（全程）",
    0x05: "VRECT欠压软件保护",
    0x06: "输出过压软件保护",
    0x07: "电池异常保护，电池电压不在正常区间",
    0x08: "接收端过温保护",
    0x09: "接收端满充保护",
    0x0A: "接收端零距离启动软件保护(开继电器之前)",
    0x0B: "接收端远距离启动软件保护(开继电器之前)",
    0x0C: "预留",
    0x0D: "接收端过温不启动保护",
    0x0E: "输出过压硬件保护",
    0x0F: "接收端短路保护",
    0x10: "接收端VRECT稳压失败保护",
    0x11: "事中FOD保护",
    0x12: "接收端自动配对失败",
    0x13: "接收端远距离启动软件保护",
    0x14: "拨号无响应保护",
    0x15: "拨号启动失败保护",
    0x16: "预充区输出过流保护",
    0x17: "预充区输出欠流保护",
    0x18: "VRECT欠压硬件保护(开继电器之前)",
    0x19: "接收端欠温保护",
    0x1A: "抛负载保护",
    0x1B: "电池故障保护（BMS）",
    0x1C: "电池满充保护（BMS）",
    0x1D: "空载启动保护",
    0x1E: "预留",
    0x1F: "VRECT过压硬件保护(开继电器之后)",
    0x20: "VRECT欠压硬件保护(开继电器之后)",
    0x21: "接收端远距离启动软件保护(开继电器之前)",
    0x22: "接收端欠温不启动",
    0x23: "进恒压区保护",
    0x24: "模拟pin配地址失败保护",
    0x25: "Vrect过压软件保护（全程）",
    0x26: "接收端模拟PIN未收到地址保护",
}


class Charging485Node(Node):
    def __init__(self):
        super().__init__('charging_485_node')

        self.declare_parameter('serial_port', '/dev/ttyS4')
        self.declare_parameter('charge_baud_rate', 19200)
        self.declare_parameter('battery_baud_rate', 4800)
        self.declare_parameter('slave_addr', 0x01)
        self.declare_parameter('battery_addr', 0x0B)
        self.declare_parameter('timeout', 0.5)
        self.declare_parameter('battery_timeout', 0.2)
        self.declare_parameter('battery_base_poll_interval', 10.0)
        self.declare_parameter('battery_temp_poll_interval', 20.0)
        self.declare_parameter('charge_poll_interval', 5.0)

        self.serial_port = self.get_parameter('serial_port').value
        self.charge_baud_rate = self.get_parameter('charge_baud_rate').value
        self.battery_baud_rate = self.get_parameter('battery_baud_rate').value
        self.slave_addr = self.get_parameter('slave_addr').value
        self.battery_addr = self.get_parameter('battery_addr').value
        self.timeout = self.get_parameter('timeout').value
        self.battery_timeout = self.get_parameter('battery_timeout').value
        self.battery_base_poll_interval = self.get_parameter('battery_base_poll_interval').value
        self.battery_temp_poll_interval = self.get_parameter('battery_temp_poll_interval').value
        self.charge_poll_interval = self.get_parameter('charge_poll_interval').value

        self.THREE_BYTE_ADDR = [0x10, 0x04, 0x6B]

        self.ser = None
        self.mutex = threading.Lock()
        self.current_baudrate = None
        self.battery_buffer = bytearray()
        self.latest_battery_temp = 0.0
        self.last_fault_code = 0
        self.last_charge_poll = 0.0
        self.last_battery_base_poll = 0.0
        self.last_battery_temp_poll = 0.0

        self.battery_base_cmd = bytes.fromhex(f"{self.battery_addr:02x} 04 00 00 00 03 B0 A1")
        self.battery_temp_cmd = bytes.fromhex(f"{self.battery_addr:02x} 03 00 50 00 01 84 B1")

        self.volt_curr_pub = self.create_publisher(Float32MultiArray, 'charging_volt_curr', 10)
        self.fault_code_pub = self.create_publisher(Int16, 'charging_fault_code', 10)
        self.battery_pub = self.create_publisher(Float32MultiArray, '/battery_data', 10)

        self.srv_start_charge = self.create_service(ChargeControl, 'start_charging', self._start_charge_cb)
        self.srv_stop_charge = self.create_service(ChargeControl, 'stop_charging', self._stop_charge_cb)
        self.srv_query_volt_curr = self.create_service(Trigger, 'query_volt_curr', self._query_volt_curr_cb)
        self.srv_query_fault = self.create_service(Trigger, 'query_fault_code', self._query_fault_code_cb)

        self._init_serial(self.charge_baud_rate)
        self.timer = self.create_timer(0.2, self._poll_bus)

        self.get_logger().info(
            f"Shared bus enabled on {self.serial_port}, charge_baud={self.charge_baud_rate}, battery_baud={self.battery_baud_rate}"
        )

    def _init_serial(self, baudrate: int):
        try:
            with self.mutex:
                if self.ser and self.ser.is_open:
                    self.ser.close()
                self.ser = serial.Serial(
                    port=self.serial_port,
                    baudrate=baudrate,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=self.timeout,
                    write_timeout=self.timeout,
                )
                self.current_baudrate = baudrate
                self.battery_buffer.clear()
        except serial.SerialException as exc:
            self.get_logger().error(f"Serial init failed: {exc}")
            self.ser = None

    def _switch_baudrate(self, baudrate: int, timeout: float | None = None):
        if self.current_baudrate == baudrate and self.ser and self.ser.is_open:
            return True

        target_timeout = self.timeout if timeout is None else timeout
        try:
            with self.mutex:
                if self.ser and self.ser.is_open:
                    self.ser.close()
                self.ser = serial.Serial(
                    port=self.serial_port,
                    baudrate=baudrate,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=target_timeout,
                    write_timeout=target_timeout,
                )
                self.current_baudrate = baudrate
                self.battery_buffer.clear()
            time.sleep(0.01)
            return True
        except serial.SerialException as exc:
            self.get_logger().error(f"Switch baudrate to {baudrate} failed: {exc}")
            self.ser = None
            return False

    def _send_raw(self, payload: bytes, read_size: int, baudrate: int, timeout: float | None = None) -> bytes:
        if not self._switch_baudrate(baudrate, timeout=timeout):
            return b''
        try:
            with self.mutex:
                self.ser.reset_input_buffer()
                self.ser.write(payload)
                return self.ser.read(read_size)
        except Exception as exc:
            self.get_logger().error(f"Serial transaction failed: {exc}")
            self.ser = None
            return b''

    def _send_charge_cmd(self, cmd: list) -> bytes:
        payload = bytes(cmd) + crc16_modbus(bytes(cmd))
        return self._send_raw(payload, read_size=16, baudrate=self.charge_baud_rate, timeout=self.timeout)

    def _send_battery_cmd(self, payload: bytes):
        if not self._switch_baudrate(self.battery_baud_rate, timeout=self.battery_timeout):
            return False
        try:
            with self.mutex:
                self.ser.write(payload)
            return True
        except Exception as exc:
            self.get_logger().error(f"Battery write failed: {exc}")
            self.ser = None
            return False

    def _read_battery_frames(self):
        if not self.ser or not self.ser.is_open:
            return
        try:
            with self.mutex:
                waiting = self.ser.in_waiting
                if waiting > 0:
                    self.battery_buffer += self.ser.read(min(waiting, 1024))
        except Exception as exc:
            self.get_logger().error(f"Battery read failed: {exc}")
            self.ser = None

        self._process_battery_buffer()

    def _process_battery_buffer(self):
        while len(self.battery_buffer) >= 3:
            if self.battery_buffer[0] != self.battery_addr:
                del self.battery_buffer[0]
                continue

            func_code = self.battery_buffer[1]
            if func_code == 0x04:
                frame_length = 11
            elif func_code == 0x03:
                frame_length = 7
            else:
                del self.battery_buffer[0]
                continue

            if len(self.battery_buffer) < frame_length:
                return

            frame = bytes(self.battery_buffer[:frame_length])
            del self.battery_buffer[:frame_length]

            if func_code == 0x04:
                battery_data = self._parse_battery_base_response(frame)
                if battery_data:
                    self._publish_battery_data(battery_data)
            else:
                self._parse_battery_temp_response(frame)

    def _parse_battery_base_response(self, data: bytes):
        if len(data) != 11 or data[0] != self.battery_addr or data[1] != 0x04 or data[2] != 0x06:
            return None
        if crc16_modbus(data[:-2]) != data[-2:]:
            self.get_logger().warn("Battery base CRC check failed")
            return None

        capacity_raw = (data[3] << 8) | data[4]
        current_raw = (data[5] << 8) | data[6]
        voltage_raw = (data[7] << 8) | data[8]

        if current_raw > 0x7FFF:
            current_raw -= 0x10000

        battery_data = {
            'capacity_percent': capacity_raw * 0.01,
            'total_current': current_raw * 0.01,
            'total_voltage': voltage_raw * 0.01,
            'temperature': self.latest_battery_temp,
        }

        if battery_data['total_voltage'] < 5.0 or battery_data['total_voltage'] > 60.0:
            self.get_logger().warn(f"Battery voltage out of range: {battery_data['total_voltage']:.2f}V")
            return None
        return battery_data

    def _parse_battery_temp_response(self, data: bytes):
        if len(data) != 7 or data[0] != self.battery_addr or data[1] != 0x03 or data[2] != 0x02:
            return None
        if crc16_modbus(data[:-2]) != data[-2:]:
            self.get_logger().warn("Battery temp CRC check failed")
            return None

        temp_raw = (data[3] << 8) | data[4]
        if temp_raw & 0x8000:
            temp_raw -= 0x10000
        self.latest_battery_temp = temp_raw * 0.1
        return self.latest_battery_temp

    def _publish_battery_data(self, battery_data):
        msg = Float32MultiArray()
        msg.data = [
            battery_data['capacity_percent'],
            battery_data['total_current'],
            battery_data['total_voltage'],
            battery_data['temperature'],
        ]
        self.battery_pub.publish(msg)

    def _parse_volt_curr(self, resp: bytes) -> tuple[float, float]:
        volt, curr = 0.0, 0.0
        if len(resp) >= 8 and resp[0] == self.slave_addr and resp[1] == 0x03 and resp[2] == 0x04:
            if crc16_modbus(resp[:-2]) == resp[-2:]:
                volt = ((resp[3] << 8) | resp[4]) * 0.01
                curr = ((resp[5] << 8) | resp[6]) * 0.01
        return volt, curr

    def _parse_fault_code(self, resp: bytes) -> tuple[int, str]:
        fault_code = 0
        fault_msg = FAULT_CODE_MAP.get(0, "No fault")
        if len(resp) >= 7 and resp[0] == self.slave_addr and resp[1] == 0x03 and resp[2] == 0x02:
            if crc16_modbus(resp[:-2]) == resp[-2:]:
                fault_code = (resp[3] << 8) | resp[4]
                fault_msg = FAULT_CODE_MAP.get(fault_code, f"Unknown fault code: 0x{fault_code:04X}")
                self.last_fault_code = fault_code
        return fault_code, fault_msg

    def _poll_bus(self):
        now = self.get_clock().now().nanoseconds / 1e9

        if now - self.last_charge_poll >= self.charge_poll_interval:
            cmd_volt_curr = [self.slave_addr, 0x03, 0x00, 0x01, 0x00, 0x02]
            volt, curr = self._parse_volt_curr(self._send_charge_cmd(cmd_volt_curr))
            volt_msg = Float32MultiArray()
            volt_msg.data = [volt, curr]
            self.volt_curr_pub.publish(volt_msg)

            cmd_fault = [self.slave_addr, 0x03, 0x00, 0x05, 0x00, 0x01]
            fault_code, _ = self._parse_fault_code(self._send_charge_cmd(cmd_fault))
            fault_msg = Int16()
            fault_msg.data = fault_code
            self.fault_code_pub.publish(fault_msg)
            self.last_charge_poll = now

        if now - self.last_battery_temp_poll >= self.battery_temp_poll_interval:
            if self._send_battery_cmd(self.battery_temp_cmd):
                time.sleep(0.05)
                self._read_battery_frames()
            self.last_battery_temp_poll = now

        if now - self.last_battery_base_poll >= self.battery_base_poll_interval:
            if self._send_battery_cmd(self.battery_base_cmd):
                time.sleep(0.05)
                self._read_battery_frames()
            self.last_battery_base_poll = now

        self._read_battery_frames()

    def _start_charge_cb(self, req, res):
        try:
            cmd = [
                self.slave_addr, 0x10, 0x30, 0x01,
                0x00, 0x02, 0x04,
                *self.THREE_BYTE_ADDR,
                0xB1,
            ]
            resp = self._send_charge_cmd(cmd)
            if resp and resp[:6] == bytes(cmd[:6]):
                res.success = True
                res.message = "Start charging command sent successfully"
            else:
                res.success = False
                res.message = f"Start charging failed, resp={list(resp)}"
        except Exception as exc:
            res.success = False
            res.message = f"Start charging error: {exc}"
        return res

    def _stop_charge_cb(self, req, res):
        try:
            cmd = [
                self.slave_addr, 0x10, 0x30, 0x01,
                0x00, 0x02, 0x04,
                *self.THREE_BYTE_ADDR,
                0xB2,
            ]
            resp = self._send_charge_cmd(cmd)
            if resp and resp[:6] == bytes(cmd[:6]):
                res.success = True
                res.message = "Stop charging command sent successfully"
            else:
                res.success = False
                res.message = f"Stop charging failed, resp={list(resp)}"
        except Exception as exc:
            res.success = False
            res.message = f"Stop charging error: {exc}"
        return res

    def _query_volt_curr_cb(self, req, res):
        cmd = [self.slave_addr, 0x03, 0x00, 0x01, 0x00, 0x02]
        volt, curr = self._parse_volt_curr(self._send_charge_cmd(cmd))
        if volt > 0 or curr > 0:
            res.success = True
            res.message = f"Voltage/current query success | {volt:.1f}V | {curr:.2f}A"
        else:
            res.success = False
            res.message = "Voltage/current query failed"
        return res

    def _query_fault_code_cb(self, req, res):
        cmd = [self.slave_addr, 0x03, 0x00, 0x05, 0x00, 0x01]
        fault_code, fault_msg = self._parse_fault_code(self._send_charge_cmd(cmd))
        res.success = True
        res.message = f"Fault query success | 0x{fault_code:04X} | {fault_msg}"
        return res

    def destroy_node(self):
        with self.mutex:
            if self.ser and self.ser.is_open:
                self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Charging485Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
