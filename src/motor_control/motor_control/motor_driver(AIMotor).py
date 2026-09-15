#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""AIMOTOR CANopen/CiA402 driver.

The public driver methods intentionally keep the legacy motor-control API so
the upper layer can switch drivers without changing its calls.  The wire
protocol in this file is the AIMOTOR manual's standard 11-bit CANopen
protocol, not the previous 29-bit vendor protocol.
"""

import math
import struct
import subprocess
import threading
import time
from typing import Optional

import can
import rclpy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


def build_sdo_frame(command: int, index: int, subindex: int,
                    value: bytes = b"") -> bytes:
    """Build an expedited, 8-byte CANopen SDO frame."""
    if not 0 <= command <= 0xFF:
        raise ValueError("SDO command must fit in one byte")
    if not 0 <= index <= 0xFFFF:
        raise ValueError("SDO index must fit in two bytes")
    if not 0 <= subindex <= 0xFF:
        raise ValueError("SDO subindex must fit in one byte")
    if len(value) > 4:
        raise ValueError("expedited SDO value cannot exceed four bytes")
    return bytes((command, index & 0xFF, index >> 8, subindex)) + value.ljust(4, b"\x00")


class CanMotorDriver(Node):
    """ROS2 node and AIMOTOR CANopen master-side adapter."""

    SDO_RX_BASE = 0x600
    SDO_TX_BASE = 0x580
    EMCY_BASE = 0x080
    NMT_COB_ID = 0x000

    CONTROLWORD_INDEX = 0x6040
    STATUSWORD_INDEX = 0x6041
    MODES_OF_OPERATION_INDEX = 0x6060
    MODES_OF_OPERATION_DISPLAY_INDEX = 0x6061
    POSITION_ACTUAL_INDEX = 0x6064
    VELOCITY_ACTUAL_INDEX = 0x606C
    TORQUE_ACTUAL_INDEX = 0x6077
    TARGET_VELOCITY_INDEX = 0x60FF

    SDO_READ = 0x40
    SDO_WRITE_1 = 0x2F
    SDO_WRITE_2 = 0x2B
    SDO_WRITE_4 = 0x23
    SDO_READ_2_RESPONSE = 0x4B
    SDO_READ_4_RESPONSE = 0x43
    SDO_ABORT = 0x80

    CANOPEN_VELOCITY_MODE = 0x03
    CONTROL_SHUTDOWN = 0x0006
    CONTROL_SWITCH_ON = 0x0007
    CONTROL_ENABLE_OPERATION = 0x000F
    CONTROL_DISABLE_VOLTAGE = 0x0000
    CONTROL_QUICK_STOP = 0x0002
    CONTROL_FAULT_RESET = 0x0080

    def __init__(self, node_name="can_motor_driver", channel="can0",
                 interface="socketcan", baudrate=500000, motor_ids=None,
                 velocity_ratio=10000.0, encoder_pulses_per_rev=1000):
        super().__init__(node_name)

        self.can_interface = channel
        self.can_bus_interface = interface
        self.can_bitrate = int(baudrate)
        self.bus: Optional[can.Bus] = None
        self.can_initialized = False

        self.declare_parameter("auto_enable", False)
        self.declare_parameter("command_timeout_sec", 0.0)
        self.auto_enable = bool(self.get_parameter("auto_enable").value)
        self.command_timeout_sec = float(
            self.get_parameter("command_timeout_sec").value
        )
        if not math.isfinite(self.command_timeout_sec) or self.command_timeout_sec < 0:
            raise ValueError("command_timeout_sec must be finite and >= 0")

        self.velocity_ratio = float(velocity_ratio)
        if not math.isfinite(self.velocity_ratio) or self.velocity_ratio <= 0:
            raise ValueError("velocity_ratio must be greater than zero")
        self.encoder_pulses_per_rev = int(encoder_pulses_per_rev)
        if self.encoder_pulses_per_rev <= 0:
            raise ValueError("encoder_pulses_per_rev must be greater than zero")

        if motor_ids is None:
            motor_ids = (1, 2, 3)
        try:
            self.motor_ids = tuple(int(motor_id) for motor_id in motor_ids)
        except (TypeError, ValueError):
            raise ValueError("motor_ids must be an iterable of CANopen node IDs")
        if (
            not self.motor_ids
            or len(set(self.motor_ids)) != len(self.motor_ids)
            or any(not 1 <= motor_id <= 0x7F for motor_id in self.motor_ids)
        ):
            raise ValueError("motor_ids must contain unique CANopen IDs in range 1..127")

        self.motors = [
            {
                "id": motor_id,
                "velocity": 0.0,
                "actual_velocity": 0.0,
                "actual_position": 0.0,
                "actual_torque": 0.0,
                "actual_temperature": 0.0,
                "status_word": 0,
                "mode_display": 0,
                "fault_code": 0,
                "send_errors": 0,
                "online": True,
            }
            for motor_id in self.motor_ids
        ]
        self._send_tick = 0
        self._feedback_tick = 0
        self.last_speed_command_time = time.monotonic()
        self._stopping = False

        self.wheel_radius = 0.05
        self.wheel_base = 0.3
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.BASE_SPEED = 2.0
        self.last_time = self.get_clock().now()

        if not self.create_can_bus():
            self.get_logger().warn("Failed to initialize CAN bus, will retry periodically")

        if self.auto_enable:
            self.initialize_motors()

        self.subscription = self.create_subscription(
            Float32MultiArray, "motor_speed_commands",
            self.speed_command_callback, 10
        )
        self.velocity_publisher = self.create_publisher(
            Float32MultiArray, "motor_velocities", 10
        )
        self.motor_feedback_publisher = self.create_publisher(
            Float32MultiArray, "motor_feedback", 10
        )
        self.motor_fault_publisher = self.create_publisher(
            Float32MultiArray, "motor_fault_codes", 10
        )
        self.odom_publisher = self.create_publisher(Odometry, "odom", 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.receive_thread = None
        self.running = True
        self.start_receive_thread()
        self.get_logger().info(
            f"AIMOTOR CANopen driver started: channel={self.can_interface}, "
            f"bitrate={self.can_bitrate}, nodes={self.motor_ids}"
        )

    def _get_motor(self, motor_id: int):
        for motor in self.motors:
            if motor["id"] == motor_id:
                return motor
        return None

    def _validate_motor_id(self, motor_id: int) -> bool:
        if not isinstance(motor_id, int) or not 1 <= motor_id <= 0x7F:
            self.get_logger().error(f"Invalid CANopen motor ID: {motor_id}")
            return False
        return True

    def create_can_bus(self) -> bool:
        """Create the SocketCAN bus using the AIMOTOR default of 500 kbit/s."""
        for attempt in range(1, 4):
            try:
                self.bus = can.Bus(
                    interface=self.can_bus_interface,
                    channel=self.can_interface,
                    bitrate=self.can_bitrate,
                )
                self.can_initialized = True
                self.get_logger().info(
                    f"CAN bus {self.can_interface} initialized at {self.can_bitrate} bit/s"
                )
                return True
            except Exception as exc:
                self.get_logger().error(
                    f"Failed to initialize CAN bus (attempt {attempt}/3): {exc}"
                )
                if attempt < 3:
                    time.sleep(0.5)
        self.can_initialized = False
        return False

    def reconnect_can_bus(self):
        if self.can_initialized:
            return
        self.get_logger().info("Retrying CAN bus initialization...")
        if self.bus is not None:
            try:
                self.bus.shutdown()
            except Exception as exc:
                self.get_logger().warn(f"Error shutting down old CAN bus: {exc}")
            self.bus = None

        try:
            subprocess.run(
                ["ip", "link", "set", self.can_interface, "down"],
                capture_output=True, timeout=5.0
            )
            subprocess.run(
                ["ip", "link", "set", self.can_interface, "up", "type", "can",
                 "bitrate", str(self.can_bitrate)],
                capture_output=True, timeout=5.0
            )
        except Exception as exc:
            self.get_logger().warn(f"Failed to reset CAN interface via ip link: {exc}")
        time.sleep(0.1)
        self.create_can_bus()

    def send_can_frame(self, can_id: int, data: bytes) -> bool:
        """Send one standard 11-bit CAN frame."""
        if self.bus is None:
            self.get_logger().error("CAN bus not initialized")
            return False
        try:
            if not 0 <= can_id <= 0x7FF:
                raise ValueError(f"CANopen COB-ID out of range: 0x{can_id:X}")
            payload = bytes(data[:8]).ljust(8, b"\x00")
            message = can.Message(
                arbitration_id=can_id,
                data=payload,
                is_extended_id=False,
            )
            self.bus.send(message)
            time.sleep(0.01)
            return True
        except Exception as exc:
            self.get_logger().error(
                f"Failed to send CAN frame (ID=0x{can_id:03X}): {exc}"
            )
            return False

    def _sdo_write(self, motor_id: int, index: int, subindex: int,
                   command: int, value: bytes = b"") -> bool:
        if not self._validate_motor_id(motor_id):
            return False
        try:
            frame = build_sdo_frame(command, index, subindex, value)
        except ValueError as exc:
            self.get_logger().error(f"Invalid SDO write: {exc}")
            return False
        return self.send_can_frame(self.SDO_RX_BASE + motor_id, frame)

    def _sdo_read(self, motor_id: int, index: int, subindex: int) -> bool:
        return self._sdo_write(motor_id, index, subindex, self.SDO_READ)

    def _write_u16(self, motor_id: int, index: int, value: int) -> bool:
        return self._sdo_write(
            motor_id, index, 0, self.SDO_WRITE_2,
            int(value).to_bytes(2, "little", signed=False)
        )

    def _write_i32(self, motor_id: int, index: int, value: int) -> bool:
        if not -0x80000000 <= value <= 0x7FFFFFFF:
            self.get_logger().error(f"INT32 value out of range: {value}")
            return False
        return self._sdo_write(
            motor_id, index, 0, self.SDO_WRITE_4,
            int(value).to_bytes(4, "little", signed=True)
        )

    def send_nmt_command(self, command: int, motor_id: int = 0) -> bool:
        """Send an NMT command; node 0 addresses all nodes."""
        if not 0 <= command <= 0xFF or not 0 <= motor_id <= 0x7F:
            return False
        return self.send_can_frame(self.NMT_COB_ID, bytes((command, motor_id)))

    def motor_clear_fault(self, motor_id: int) -> bool:
        """Reset a CiA402 fault with controlword 0x0080."""
        return self._write_u16(motor_id, self.CONTROLWORD_INDEX, self.CONTROL_FAULT_RESET)

    def parse_motor_fault(self, can_id: int, data: bytes):
        """Parse a CANopen EMCY frame (0x080 + node ID)."""
        motor_id = can_id - self.EMCY_BASE
        motor = self._get_motor(motor_id)
        if motor is None or len(data) < 3:
            return
        emergency_code = int.from_bytes(data[0:2], "little", signed=False)
        error_register = data[2]
        motor["fault_code"] = emergency_code
        self.get_logger().error(
            f"Motor {motor_id} EMCY: code=0x{emergency_code:04X}, "
            f"error_register=0x{error_register:02X}"
        )
        self.publish_motor_fault_codes()

    def publish_motor_fault_codes(self):
        message = Float32MultiArray()
        message.data = [float(motor["fault_code"]) for motor in self.motors]
        self.motor_fault_publisher.publish(message)

    def motor_set_mode(self, motor_id: int, mode: int) -> bool:
        """Set AIMOTOR profile velocity mode (6060h = 3).

        The old upper layer used ``2`` for speed mode, so both 2 and the
        CANopen value 3 are accepted and normalized to 3 on the wire.
        """
        if mode not in (2, self.CANOPEN_VELOCITY_MODE):
            self.get_logger().error(f"Unsupported AIMOTOR mode: {mode}")
            return False
        return self._sdo_write(
            motor_id, self.MODES_OF_OPERATION_INDEX, 0,
            self.SDO_WRITE_1, bytes((self.CANOPEN_VELOCITY_MODE,))
        )

    def motor_set_current_limit(self, motor_id: int, current_limit: float) -> bool:
        """Keep the legacy API; AIMOTOR manual exposes no generic current-limit object."""
        self.get_logger().warn(
            f"AIMOTOR CANopen does not define a portable current-limit object; "
            f"ignoring motor {motor_id} value {current_limit}"
        )
        return False

    def motor_set_other_param(self, motor_id: int, param_value: float) -> bool:
        """Keep the legacy API without sending the obsolete 0x7022 frame."""
        self.get_logger().warn(
            f"AIMOTOR CANopen does not define the legacy vendor parameter; "
            f"ignoring motor {motor_id} value {param_value}"
        )
        return False

    def motor_set_speed(self, motor_id: int, speed: float) -> bool:
        """Write 60FFh target velocity as signed INT32 Pul/s."""
        try:
            numeric_speed = float(speed)
        except (TypeError, ValueError):
            self.get_logger().error(f"Invalid target velocity: {speed!r}")
            return False
        if not math.isfinite(numeric_speed):
            self.get_logger().error(f"Invalid target velocity: {speed!r}")
            return False
        target_pulses = int(round(numeric_speed * self.velocity_ratio))
        return self._write_i32(motor_id, self.TARGET_VELOCITY_INDEX, target_pulses)

    def motor_query_feedback(self, motor_id: int) -> bool:
        """Request status, actual position, actual velocity and torque via SDO."""
        if not self._validate_motor_id(motor_id):
            return False
        results = [
            self._sdo_read(motor_id, self.STATUSWORD_INDEX, 0),
            self._sdo_read(motor_id, self.POSITION_ACTUAL_INDEX, 0),
            self._sdo_read(motor_id, self.VELOCITY_ACTUAL_INDEX, 0),
            self._sdo_read(motor_id, self.TORQUE_ACTUAL_INDEX, 0),
        ]
        return all(results)

    def motor_enable(self, motor_id: int) -> bool:
        """Run the CiA402 enable sequence 06h -> 07h -> 0Fh."""
        if not self._validate_motor_id(motor_id):
            return False
        results = []
        for controlword in (
            self.CONTROL_SHUTDOWN,
            self.CONTROL_SWITCH_ON,
            self.CONTROL_ENABLE_OPERATION,
        ):
            results.append(self._write_u16(motor_id, self.CONTROLWORD_INDEX, controlword))
        return all(results)

    def motor_disable(self, motor_id: int) -> bool:
        """Stop normal operation by returning CiA402 to switch-on-disabled."""
        return self._write_u16(motor_id, self.CONTROLWORD_INDEX, self.CONTROL_SWITCH_ON)

    def initialize_motors(self):
        """Start CANopen nodes, select profile velocity mode and enable them."""
        self.get_logger().info("Initializing AIMOTOR CANopen nodes...")
        time.sleep(0.2)
        for motor in self.motors:
            motor_id = motor["id"]
            self.send_nmt_command(0x01, motor_id)  # Start remote node.
            time.sleep(0.01)
            self.motor_set_mode(motor_id, self.CANOPEN_VELOCITY_MODE)
            time.sleep(0.01)
            if not self.motor_enable(motor_id):
                self.get_logger().error(f"Failed to enable AIMOTOR node {motor_id}")
            time.sleep(0.01)

    def speed_command_callback(self, msg: Float32MultiArray):
        expected_length = len(self.motors)
        if len(msg.data) not in (3, expected_length):
            self.get_logger().warn(
                f"Received speed command with incorrect length: {len(msg.data)}, "
                f"expected {expected_length} or legacy 3"
            )
            return
        try:
            speeds = [float(value) for value in msg.data]
        except (TypeError, ValueError):
            self.get_logger().warn("Received non-numeric motor speed command")
            return
        if not all(math.isfinite(speed) for speed in speeds):
            self.get_logger().warn("Received non-finite motor speed command")
            return
        for motor in self.motors:
            motor["velocity"] = 0.0
        for index, speed in enumerate(speeds[:len(self.motors)]):
            self.motors[index]["velocity"] = speed
        self.last_speed_command_time = time.monotonic()

    def send_speed_commands(self):
        send_error_threshold = 3
        retry_interval_ticks = 50
        self._send_tick += 1
        for motor in self.motors:
            if not motor["online"]:
                if self._send_tick % retry_interval_ticks != 0:
                    continue
                motor["velocity"] = 0.0
            result = self.motor_set_speed(motor["id"], motor["velocity"])
            if result:
                motor["send_errors"] = 0
                motor["online"] = True
            else:
                motor["send_errors"] += 1
                if motor["send_errors"] >= send_error_threshold:
                    motor["online"] = False
                    self.get_logger().error(
                        f"Motor {motor['id']} failed {send_error_threshold} speed writes; marked offline"
                    )

    def query_motor_feedback(self):
        for motor in self.motors:
            if not self.motor_query_feedback(motor["id"]):
                self.get_logger().warn(f"Failed to query motor {motor['id']} feedback")

    def _parse_sdo_feedback(self, motor_id: int, data: bytes):
        if len(data) < 8 or data[0] in (self.SDO_ABORT,):
            return
        index = data[1] | (data[2] << 8)
        value = data[4:8]
        motor = self._get_motor(motor_id)
        if motor is None:
            return
        if index == self.STATUSWORD_INDEX:
            motor["status_word"] = int.from_bytes(value[:2], "little")
        elif index == self.MODES_OF_OPERATION_DISPLAY_INDEX:
            motor["mode_display"] = int.from_bytes(value[:1], "little", signed=True)
        elif index == self.POSITION_ACTUAL_INDEX:
            pulses = int.from_bytes(value, "little", signed=True)
            motor["actual_position"] = (
                pulses * 2.0 * math.pi / self.encoder_pulses_per_rev
            )
        elif index == self.VELOCITY_ACTUAL_INDEX:
            pulses_per_sec = int.from_bytes(value, "little", signed=True)
            motor["actual_velocity"] = pulses_per_sec / self.velocity_ratio
        elif index == self.TORQUE_ACTUAL_INDEX:
            torque_raw = int.from_bytes(value[:2], "little", signed=True)
            motor["actual_torque"] = torque_raw / 10.0

    def _parse_tpdo_feedback(self, can_id: int, data: bytes):
        node_id = can_id & 0x7F
        motor = self._get_motor(node_id)
        if motor is None:
            return
        if 0x180 <= can_id <= 0x1FF and len(data) >= 7:
            motor["status_word"] = int.from_bytes(data[0:2], "little")
            position = int.from_bytes(data[3:7], "little", signed=True)
            motor["actual_position"] = (
                position * 2.0 * math.pi / self.encoder_pulses_per_rev
            )
        elif 0x280 <= can_id <= 0x2FF and len(data) >= 4:
            velocity = int.from_bytes(data[0:4], "little", signed=True)
            motor["actual_velocity"] = velocity / self.velocity_ratio
            if len(data) >= 6:
                motor["actual_torque"] = int.from_bytes(
                    data[4:6], "little", signed=True
                ) / 10.0

    def parse_motor_feedback(self, can_id: int, data: bytearray):
        """Parse SDO responses and the manual's default TPDO mappings."""
        if self.SDO_TX_BASE <= can_id <= self.SDO_TX_BASE + 0x7F:
            self._parse_sdo_feedback(can_id - self.SDO_TX_BASE, bytes(data))
        elif 0x180 <= can_id <= 0x2FF:
            self._parse_tpdo_feedback(can_id, bytes(data))

    def uint16_to_float(self, x, x_min, x_max, bits):
        span = (1 << bits) - 1
        return (x_max - x_min) * x / span + x_min

    def update_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time.nanoseconds - self.last_time.nanoseconds) / 1e9
        self.last_time = current_time
        if dt <= 0 or len(self.motors) < 2:
            return
        left_vel = self.motors[0]["actual_velocity"]
        right_vel = self.motors[1]["actual_velocity"]
        left_linear = left_vel * self.wheel_radius
        right_linear = right_vel * self.wheel_radius
        linear_velocity = (right_linear + left_linear) / 2.0
        angular_velocity = (right_linear - left_linear) / self.wheel_base
        self.x += linear_velocity * dt * math.cos(self.th)
        self.y += linear_velocity * dt * math.sin(self.th)
        self.th += angular_velocity * dt
        self.publish_odometry(linear_velocity, angular_velocity)

    def publish_odometry(self, linear_velocity, angular_velocity):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = self.quaternion_from_euler(0, 0, self.th)
        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = angular_velocity
        odom.pose.covariance = [0.0] * 36
        odom.twist.covariance = [0.0] * 36
        self.odom_publisher.publish(odom)

    def quaternion_from_euler(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        quaternion = Quaternion()
        quaternion.w = cr * cp * cy + sr * sp * sy
        quaternion.x = sr * cp * cy - cr * sp * sy
        quaternion.y = cr * sp * cy + sr * cp * sy
        quaternion.z = cr * cp * sy - sr * sp * cy
        return quaternion

    def receive_can_frames(self):
        self.get_logger().info("Starting AIMOTOR CANopen receive thread...")
        consecutive_errors = 0
        while self.running:
            try:
                if not self.can_initialized:
                    self.reconnect_can_bus()
                    time.sleep(1.0)
                    continue
                if self.bus is None:
                    time.sleep(0.1)
                    continue
                message = self.bus.recv(timeout=0.1)
                if message is None:
                    continue
                consecutive_errors = 0
                can_id = message.arbitration_id
                if self.EMCY_BASE <= can_id < self.EMCY_BASE + 0x80:
                    self.parse_motor_fault(can_id, message.data)
                elif self.SDO_TX_BASE <= can_id < self.SDO_TX_BASE + 0x80:
                    self.parse_motor_feedback(can_id, message.data)
                elif 0x180 <= can_id < 0x300:
                    self.parse_motor_feedback(can_id, message.data)
                elif 0x700 <= can_id < 0x780:
                    motor = self._get_motor(can_id - 0x700)
                    if motor is not None and message.data:
                        motor["online"] = message.data[0] in (0x05, 0x7F)
            except Exception as exc:
                consecutive_errors += 1
                self.get_logger().error(
                    f"CAN receive error ({consecutive_errors}/5): {exc}"
                )
                if consecutive_errors >= 5:
                    self.can_initialized = False
                    if self.bus is not None:
                        try:
                            self.bus.shutdown()
                        except Exception:
                            pass
                        self.bus = None
                    consecutive_errors = 0
                time.sleep(1.0)
        self.get_logger().info("Stopped CANopen receive thread")

    def start_receive_thread(self):
        self.receive_thread = threading.Thread(
            target=self.receive_can_frames, daemon=True
        )
        self.receive_thread.start()

    def timer_callback(self):
        if (
            self.command_timeout_sec > 0
            and time.monotonic() - self.last_speed_command_time > self.command_timeout_sec
        ):
            for motor in self.motors:
                motor["velocity"] = 0.0

        self.send_speed_commands()
        self._feedback_tick += 1
        if self._feedback_tick >= 5:
            self._feedback_tick = 0
            self.query_motor_feedback()
        self.update_odometry()

        velocity_msg = Float32MultiArray()
        velocity_msg.data = [float(motor["actual_velocity"]) for motor in self.motors]
        self.velocity_publisher.publish(velocity_msg)

        feedback_msg = Float32MultiArray()
        feedback_data = []
        for motor in self.motors:
            feedback_data.extend([
                float(motor["id"]),
                motor["actual_position"],
                motor["actual_velocity"],
                motor["actual_torque"],
                motor["actual_temperature"],
            ])
        feedback_msg.data = feedback_data
        self.motor_feedback_publisher.publish(feedback_msg)

    def stop_all_motors(self):
        if self._stopping:
            return
        self._stopping = True
        self.running = False
        for motor in self.motors:
            motor["velocity"] = 0.0
            self.motor_set_speed(motor["id"], 0.0)
            time.sleep(0.01)
        for motor in self.motors:
            self.motor_disable(motor["id"])
            time.sleep(0.01)

    def destroy_node(self):
        self.stop_all_motors()
        if self.receive_thread:
            self.receive_thread.join(timeout=1.0)
        if self.bus is not None:
            try:
                self.bus.shutdown()
            except Exception as exc:
                self.get_logger().warn(f"CAN bus shutdown failed: {exc}")
            self.bus = None
        if rclpy.ok():
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    motor_driver = None
    try:
        motor_driver = CanMotorDriver()
        rclpy.spin(motor_driver)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"Unexpected error: {exc}")
    finally:
        if motor_driver is not None:
            motor_driver.stop_all_motors()
            if rclpy.ok():
                motor_driver.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
