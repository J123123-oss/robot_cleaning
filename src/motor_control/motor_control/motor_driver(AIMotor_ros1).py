#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""ROS1 AIMOTOR CANopen/CiA402 motor driver.

This is the ROS1 counterpart of ``motor_driver(AIMotor).py``.  The public
class and method names remain compatible with the existing motor-control
layer, while the node lifecycle uses rospy instead of rclpy.

The AIMOTOR manual specifies standard 11-bit CANopen frames, 500 kbit/s by
default, profile velocity mode (6060h = 3), and target velocity in 60FFh as
signed INT32 pulses per second.
"""

import math
import subprocess
import threading
import time

import can
import rospy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray


def build_sdo_frame(command, index, subindex, value=b""):
    """Build an expedited eight-byte CANopen SDO frame."""
    if not 0 <= command <= 0xFF:
        raise ValueError("SDO command must fit in one byte")
    if not 0 <= index <= 0xFFFF:
        raise ValueError("SDO index must fit in two bytes")
    if not 0 <= subindex <= 0xFF:
        raise ValueError("SDO subindex must fit in one byte")
    if len(value) > 4:
        raise ValueError("expedited SDO value cannot exceed four bytes")
    return bytes((command, index & 0xFF, index >> 8, subindex)) + value.ljust(
        4, b"\x00"
    )


class CanMotorDriver(object):
    """ROS1 node and AIMOTOR CANopen master-side adapter."""

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
    PROFILE_ACCELERATION_INDEX = 0x6083
    PROFILE_DECELERATION_INDEX = 0x6084

    DEFAULT_PROFILE_ACCELERATION = 16666
    DEFAULT_PROFILE_DECELERATION = 11111
    DEFAULT_PULSES_PER_MOTOR_REV = 1000
    DEFAULT_MECHANICAL_REDUCTION_RATIO = 40.0
    DEFAULT_MECHANICAL_REDUCTION_RATIOS = {
        1: 50.0,
        2: 50.0,
        3: 40.0,
        4: 40.0,
    }

    SDO_READ = 0x40
    SDO_WRITE_1 = 0x2F
    SDO_WRITE_2 = 0x2B
    SDO_WRITE_4 = 0x23
    SDO_ABORT = 0x80

    CANOPEN_VELOCITY_MODE = 0x03
    CONTROL_SHUTDOWN = 0x0006
    CONTROL_SWITCH_ON = 0x0007
    CONTROL_ENABLE_OPERATION = 0x000F
    CONTROL_DISABLE_VOLTAGE = 0x0000
    CONTROL_QUICK_STOP = 0x0002
    CONTROL_FAULT_RESET = 0x0080

    def __init__(
        self,
        node_name="can_motor_driver",
        channel="can1",
        interface="socketcan",
        baudrate=500000,
        motor_ids=None,
        pulses_per_motor_rev=DEFAULT_PULSES_PER_MOTOR_REV,
        mechanical_reduction_ratio=DEFAULT_MECHANICAL_REDUCTION_RATIO,
        mechanical_reduction_ratios=None,
        profile_acceleration=DEFAULT_PROFILE_ACCELERATION,
        profile_deceleration=DEFAULT_PROFILE_DECELERATION,
    ):
        """Create the driver, publishers, subscriber, timer, and CAN thread.

        ROS1 node initialization is intentionally kept in ``main`` so this
        class can also be constructed by an existing ROS1 control node after
        it has called ``rospy.init_node``.
        """
        del node_name  # ROS1 names the process in rospy.init_node().

        self.can_interface = rospy.get_param("~channel", channel)
        self.can_bus_interface = rospy.get_param("~interface", interface)
        self.can_bitrate = int(rospy.get_param("~baudrate", baudrate))
        self.bus = None
        self.can_initialized = False

        self.auto_enable = bool(rospy.get_param("~auto_enable", False))
        self.command_timeout_sec = float(
            rospy.get_param("~command_timeout_sec", 0.0)
        )
        if (
            not math.isfinite(self.command_timeout_sec)
            or self.command_timeout_sec < 0
        ):
            raise ValueError("command_timeout_sec must be finite and >= 0")

        self.profile_acceleration = self._validate_profile_parameter(
            rospy.get_param("~profile_acceleration", profile_acceleration),
            "profile_acceleration",
        )
        self.profile_deceleration = self._validate_profile_parameter(
            rospy.get_param("~profile_deceleration", profile_deceleration),
            "profile_deceleration",
        )

        if motor_ids is None:
            motor_ids = (1, 2, 3)
        configured_motor_ids = rospy.get_param("~motor_ids", list(motor_ids))
        try:
            self.motor_ids = tuple(int(motor_id) for motor_id in configured_motor_ids)
        except (TypeError, ValueError):
            raise ValueError("motor_ids must be an iterable of CANopen node IDs")
        if (
            not self.motor_ids
            or len(set(self.motor_ids)) != len(self.motor_ids)
            or any(not 1 <= motor_id <= 0x7F for motor_id in self.motor_ids)
        ):
            raise ValueError(
                "motor_ids must contain unique CANopen IDs in range 1..127"
            )

        self.pulses_per_motor_rev = self._validate_positive_int_parameter(
            rospy.get_param("~pulses_per_motor_rev", pulses_per_motor_rev),
            "pulses_per_motor_rev",
        )
        self.encoder_pulses_per_rev = self.pulses_per_motor_rev
        default_reduction_ratio = self._validate_positive_float_parameter(
            rospy.get_param(
                "~mechanical_reduction_ratio", mechanical_reduction_ratio
            ),
            "mechanical_reduction_ratio",
        )
        configured_reduction_ratios = (
            self.DEFAULT_MECHANICAL_REDUCTION_RATIOS
            if mechanical_reduction_ratios is None
            else mechanical_reduction_ratios
        )
        if not isinstance(configured_reduction_ratios, dict):
            raise ValueError("mechanical_reduction_ratios must be a dict keyed by motor ID")
        self.mechanical_reduction_ratios = {}
        for motor_id in self.motor_ids:
            parameter_name = f"mechanical_reduction_ratio_{motor_id}"
            self.mechanical_reduction_ratios[motor_id] = (
                self._validate_positive_float_parameter(
                    rospy.get_param(
                        f"~{parameter_name}",
                        configured_reduction_ratios.get(
                            motor_id, default_reduction_ratio
                        ),
                    ),
                    parameter_name,
                )
            )

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

        self._state_lock = threading.RLock()
        self._send_tick = 0
        self._feedback_tick = 0
        self.last_speed_command_time = time.monotonic()
        self._stopping = False

        self.wheel_radius = float(rospy.get_param("~wheel_radius", 0.05))
        self.wheel_base = float(rospy.get_param("~wheel_base", 0.3))
        if self.wheel_radius <= 0 or self.wheel_base <= 0:
            raise ValueError("wheel_radius and wheel_base must be greater than zero")
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = rospy.Time.now()

        self.velocity_publisher = rospy.Publisher(
            "motor_velocities", Float32MultiArray, queue_size=10
        )
        self.motor_feedback_publisher = rospy.Publisher(
            "motor_feedback", Float32MultiArray, queue_size=10
        )
        self.motor_fault_publisher = rospy.Publisher(
            "motor_fault_codes", Float32MultiArray, queue_size=10
        )
        self.odom_publisher = rospy.Publisher("odom", Odometry, queue_size=10)
        self.subscription = rospy.Subscriber(
            "motor_speed_commands",
            Float32MultiArray,
            self.speed_command_callback,
            queue_size=10,
        )
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

        if not self.create_can_bus():
            rospy.logwarn(
                "Failed to initialize CAN bus, will retry periodically"
            )

        self.running = True
        self.receive_thread = None
        rospy.on_shutdown(self.stop_all_motors)
        self.start_receive_thread()

        if self.auto_enable:
            self.initialize_motors()

        rospy.loginfo(
            "AIMOTOR ROS1 CANopen driver started: channel=%s, bitrate=%s, nodes=%s",
            self.can_interface,
            self.can_bitrate,
            self.motor_ids,
        )

    def _get_motor(self, motor_id):
        """Return the state dictionary for a configured node ID."""
        for motor in self.motors:
            if motor["id"] == motor_id:
                return motor
        return None

    def _validate_motor_id(self, motor_id):
        """Check that a value is a valid CANopen node ID."""
        if not isinstance(motor_id, int) or not 1 <= motor_id <= 0x7F:
            rospy.logerr("Invalid CANopen motor ID: %r", motor_id)
            return False
        return True

    @staticmethod
    def _validate_profile_parameter(value, name):
        """Validate a positive INT32 profile acceleration/deceleration value."""
        try:
            numeric_value = float(value)
        except (TypeError, ValueError):
            raise ValueError(f"{name} must be a finite positive integer")
        if (
            not math.isfinite(numeric_value)
            or numeric_value <= 0
            or not numeric_value.is_integer()
            or numeric_value > 0x7FFFFFFF
        ):
            raise ValueError(f"{name} must be a finite positive INT32 value")
        return int(numeric_value)

    @staticmethod
    def _validate_positive_float_parameter(value, name):
        """Validate a finite positive scalar used for unit conversion."""
        try:
            numeric_value = float(value)
        except (TypeError, ValueError):
            raise ValueError(f"{name} must be a finite positive number")
        if not math.isfinite(numeric_value) or numeric_value <= 0:
            raise ValueError(f"{name} must be a finite positive number")
        return numeric_value

    @staticmethod
    def _validate_positive_int_parameter(value, name):
        """Validate a finite positive integer used as pulses per revolution."""
        try:
            numeric_value = float(value)
        except (TypeError, ValueError):
            raise ValueError(f"{name} must be a finite positive integer")
        if (
            not math.isfinite(numeric_value)
            or numeric_value <= 0
            or not numeric_value.is_integer()
            or numeric_value > 0x7FFFFFFF
        ):
            raise ValueError(f"{name} must be a finite positive integer")
        return int(numeric_value)

    def _get_mechanical_reduction_ratio(self, motor_id):
        """Return the configured motor-specific reduction ratio."""
        if not self._validate_motor_id(motor_id):
            return None
        return self.mechanical_reduction_ratios.get(motor_id)

    def create_can_bus(self):
        """Open a python-can bus, supporting python-can 3.x and 4.x APIs."""
        for attempt in range(1, 4):
            try:
                try:
                    self.bus = can.Bus(
                        interface=self.can_bus_interface,
                        channel=self.can_interface,
                        bitrate=self.can_bitrate,
                    )
                except (AttributeError, TypeError):
                    # ROS1 distributions commonly ship python-can 3.x, whose
                    # constructor uses bustype instead of interface.
                    self.bus = can.interface.Bus(
                        channel=self.can_interface,
                        bustype=self.can_bus_interface,
                        bitrate=self.can_bitrate,
                    )
                self.can_initialized = True
                rospy.loginfo(
                    "CAN bus %s initialized at %s bit/s",
                    self.can_interface,
                    self.can_bitrate,
                )
                return True
            except Exception as exc:
                self.can_initialized = False
                rospy.logerr(
                    "Failed to initialize CAN bus (attempt %s/3): %s",
                    attempt,
                    exc,
                )
                if attempt < 3:
                    time.sleep(0.5)
        return False

    def reconnect_can_bus(self):
        """Reset SocketCAN and retry bus creation after receive failures."""
        if self.can_initialized:
            return
        rospy.loginfo("Retrying CAN bus initialization...")
        if self.bus is not None:
            try:
                self.bus.shutdown()
            except Exception as exc:
                rospy.logwarn("Error shutting down old CAN bus: %s", exc)
            self.bus = None

        try:
            subprocess.run(
                ["ip", "link", "set", self.can_interface, "down"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=5.0,
            )
            subprocess.run(
                [
                    "ip",
                    "link",
                    "set",
                    self.can_interface,
                    "up",
                    "type",
                    "can",
                    "bitrate",
                    str(self.can_bitrate),
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=5.0,
            )
        except Exception as exc:
            rospy.logwarn("Failed to reset CAN interface via ip link: %s", exc)
        time.sleep(0.1)
        self.create_can_bus()

    def send_can_frame(self, can_id, data):
        """Send one standard 11-bit CAN frame."""
        if self.bus is None:
            rospy.logerr("CAN bus not initialized")
            return False
        try:
            if not 0 <= can_id <= 0x7FF:
                raise ValueError("CANopen COB-ID out of range: 0x%X" % can_id)
            payload = bytes(data[:8]).ljust(8, b"\x00")
            try:
                message = can.Message(
                    arbitration_id=can_id,
                    data=payload,
                    is_extended_id=False,
                )
            except TypeError:
                # python-can 3.x calls this field extended_id.
                message = can.Message(
                    arbitration_id=can_id,
                    data=payload,
                    extended_id=False,
                )
            self.bus.send(message)
            time.sleep(0.01)
            return True
        except Exception as exc:
            rospy.logerr(
                "Failed to send CAN frame (ID=0x%03X): %s", can_id, exc
            )
            return False

    def _sdo_write(self, motor_id, index, subindex, command, value=b""):
        """Send an expedited SDO write to one node."""
        if not self._validate_motor_id(motor_id):
            return False
        try:
            frame = build_sdo_frame(command, index, subindex, value)
        except ValueError as exc:
            rospy.logerr("Invalid SDO write: %s", exc)
            return False
        return self.send_can_frame(self.SDO_RX_BASE + motor_id, frame)

    def _sdo_read(self, motor_id, index, subindex):
        """Request an SDO object from one node."""
        return self._sdo_write(motor_id, index, subindex, self.SDO_READ)

    def _write_u16(self, motor_id, index, value):
        """Write an unsigned 16-bit SDO object."""
        try:
            raw_value = int(value).to_bytes(2, "little", signed=False)
        except (OverflowError, ValueError):
            rospy.logerr("UINT16 value out of range: %r", value)
            return False
        return self._sdo_write(
            motor_id, index, 0, self.SDO_WRITE_2, raw_value
        )

    def _write_i32(self, motor_id, index, value):
        """Write a signed 32-bit SDO object."""
        try:
            raw_value = int(value).to_bytes(4, "little", signed=True)
        except (OverflowError, ValueError):
            rospy.logerr("INT32 value out of range: %r", value)
            return False
        return self._sdo_write(
            motor_id, index, 0, self.SDO_WRITE_4, raw_value
        )

    def send_nmt_command(self, command, motor_id=0):
        """Send an NMT command; node 0 addresses all nodes."""
        if not 0 <= command <= 0xFF or not 0 <= motor_id <= 0x7F:
            rospy.logerr("Invalid NMT command or node ID")
            return False
        return self.send_can_frame(
            self.NMT_COB_ID, bytes((command, motor_id))
        )

    def motor_clear_fault(self, motor_id):
        """Reset a CiA402 fault with controlword 0x0080."""
        return self._write_u16(
            motor_id, self.CONTROLWORD_INDEX, self.CONTROL_FAULT_RESET
        )

    def parse_motor_fault(self, can_id, data):
        """Parse an EMCY frame with COB-ID 0x080 plus the node ID."""
        motor_id = can_id - self.EMCY_BASE
        motor = self._get_motor(motor_id)
        if motor is None or len(data) < 3:
            return
        emergency_code = int.from_bytes(data[0:2], "little", signed=False)
        error_register = data[2]
        motor["fault_code"] = emergency_code
        rospy.logerr(
            "Motor %s EMCY: code=0x%04X, error_register=0x%02X",
            motor_id,
            emergency_code,
            error_register,
        )
        self.publish_motor_fault_codes()

    def publish_motor_fault_codes(self):
        """Publish one fault code per configured motor, in motor order."""
        message = Float32MultiArray()
        message.data = [float(motor["fault_code"]) for motor in self.motors]
        self.motor_fault_publisher.publish(message)

    def motor_set_mode(self, motor_id, mode):
        """Set AIMOTOR profile velocity mode (6060h = 3)."""
        # The previous upper layer used 2 for speed mode.  Accept it for API
        # compatibility, but always send the AIMOTOR/CiA402 value 3.
        if mode not in (2, self.CANOPEN_VELOCITY_MODE):
            rospy.logerr("Unsupported AIMOTOR mode: %r", mode)
            return False
        return self._sdo_write(
            motor_id,
            self.MODES_OF_OPERATION_INDEX,
            0,
            self.SDO_WRITE_1,
            bytes((self.CANOPEN_VELOCITY_MODE,)),
        )

    def motor_set_acceleration(self, motor_id, acceleration=None):
        """Write PV profile acceleration to 6083h in Pul/s^2."""
        if acceleration is None:
            acceleration = self.profile_acceleration
        try:
            acceleration = self._validate_profile_parameter(
                acceleration, "profile_acceleration"
            )
        except ValueError as exc:
            rospy.logerr("%s", exc)
            return False
        return self._write_i32(
            motor_id, self.PROFILE_ACCELERATION_INDEX, acceleration
        )

    def motor_set_deceleration(self, motor_id, deceleration=None):
        """Write PV profile deceleration to 6084h in Pul/s^2."""
        if deceleration is None:
            deceleration = self.profile_deceleration
        try:
            deceleration = self._validate_profile_parameter(
                deceleration, "profile_deceleration"
            )
        except ValueError as exc:
            rospy.logerr("%s", exc)
            return False
        return self._write_i32(
            motor_id, self.PROFILE_DECELERATION_INDEX, deceleration
        )

    def motor_set_current_limit(self, motor_id, current_limit):
        """Keep the legacy API without sending an obsolete vendor frame."""
        rospy.logwarn(
            "AIMOTOR CANopen has no portable current-limit object; "
            "ignoring motor %s value %s",
            motor_id,
            current_limit,
        )
        return False

    def motor_set_other_param(self, motor_id, param_value):
        """Keep the legacy API without sending the old 0x7022 frame."""
        rospy.logwarn(
            "AIMOTOR CANopen has no legacy vendor parameter; "
            "ignoring motor %s value %s",
            motor_id,
            param_value,
        )
        return False

    def motor_set_speed(self, motor_id, speed):
        """Write output-shaft speed in r/min to 60FFh as signed INT32 Pul/s."""
        reduction_ratio = self._get_mechanical_reduction_ratio(motor_id)
        if reduction_ratio is None:
            rospy.logerr(
                "No mechanical reduction ratio configured for motor %s", motor_id
            )
            return False
        try:
            numeric_speed = float(speed)
        except (TypeError, ValueError):
            rospy.logerr("Invalid target velocity: %r", speed)
            return False
        if not math.isfinite(numeric_speed):
            rospy.logerr("Invalid target velocity: %r", speed)
            return False
        target_pulses = int(round(
            numeric_speed * reduction_ratio / 60.0 * self.pulses_per_motor_rev
        ))
        return self._write_i32(
            motor_id, self.TARGET_VELOCITY_INDEX, target_pulses
        )

    def motor_query_feedback(self, motor_id):
        """Request status, position, velocity, and torque through SDO."""
        if not self._validate_motor_id(motor_id):
            return False
        results = [
            self._sdo_read(motor_id, self.STATUSWORD_INDEX, 0),
            self._sdo_read(motor_id, self.POSITION_ACTUAL_INDEX, 0),
            self._sdo_read(motor_id, self.VELOCITY_ACTUAL_INDEX, 0),
            self._sdo_read(motor_id, self.TORQUE_ACTUAL_INDEX, 0),
        ]
        return all(results)

    def motor_enable(self, motor_id):
        """Run the CiA402 enable sequence 06h -> 07h -> 0Fh."""
        if not self._validate_motor_id(motor_id):
            return False
        results = []
        for controlword in (
            self.CONTROL_SHUTDOWN,
            self.CONTROL_SWITCH_ON,
            self.CONTROL_ENABLE_OPERATION,
        ):
            results.append(
                self._write_u16(motor_id, self.CONTROLWORD_INDEX, controlword)
            )
        return all(results)

    def motor_disable(self, motor_id):
        """Disable operation with CiA402 controlword 0x0007."""
        # 0x0007 clears Enable Operation while retaining the switch-on state;
        # the next motor_enable() can therefore repeat the documented sequence.
        return self._write_u16(
            motor_id, self.CONTROLWORD_INDEX, self.CONTROL_SWITCH_ON
        )

    def initialize_motors(self):
        """Start nodes, set PV mode and ramps, then enable each motor."""
        rospy.loginfo("Initializing AIMOTOR CANopen nodes...")
        time.sleep(0.2)
        for motor in self.motors:
            motor_id = motor["id"]
            self.send_nmt_command(0x01, motor_id)
            time.sleep(0.01)
            self.motor_set_mode(motor_id, self.CANOPEN_VELOCITY_MODE)
            time.sleep(0.01)
            if not self.motor_set_acceleration(motor_id):
                rospy.logerr(
                    "Failed to set acceleration for AIMOTOR node %s", motor_id
                )
            time.sleep(0.01)
            if not self.motor_set_deceleration(motor_id):
                rospy.logerr(
                    "Failed to set deceleration for AIMOTOR node %s", motor_id
                )
            time.sleep(0.01)
            if not self.motor_enable(motor_id):
                rospy.logerr("Failed to enable AIMOTOR node %s", motor_id)
            time.sleep(0.01)

    def speed_command_callback(self, msg):
        """Store a three- or configured-count speed command from ROS."""
        expected_length = len(self.motors)
        if len(msg.data) not in (3, expected_length):
            rospy.logwarn(
                "Received speed command with incorrect length: %s, "
                "expected %s or legacy 3",
                len(msg.data),
                expected_length,
            )
            return
        try:
            speeds = [float(value) for value in msg.data]
        except (TypeError, ValueError):
            rospy.logwarn("Received non-numeric motor speed command")
            return
        if not all(math.isfinite(speed) for speed in speeds):
            rospy.logwarn("Received non-finite motor speed command")
            return

        with self._state_lock:
            for motor in self.motors:
                motor["velocity"] = 0.0
            for index, speed in enumerate(speeds[: len(self.motors)]):
                self.motors[index]["velocity"] = speed
            self.last_speed_command_time = time.monotonic()

    def send_speed_commands(self):
        """Send targets to all online motors and retry offline motors slowly."""
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
                    rospy.logerr(
                        "Motor %s failed %s speed writes; marked offline",
                        motor["id"],
                        send_error_threshold,
                    )

    def query_motor_feedback(self):
        """Request feedback from every configured motor."""
        for motor in self.motors:
            if not self.motor_query_feedback(motor["id"]):
                rospy.logwarn("Failed to query motor %s feedback", motor["id"])

    def _parse_sdo_feedback(self, motor_id, data):
        """Decode SDO responses for the feedback objects used by the node."""
        if len(data) < 8 or data[0] == self.SDO_ABORT:
            return
        index = data[1] | (data[2] << 8)
        value = data[4:8]
        motor = self._get_motor(motor_id)
        if motor is None:
            return
        if index == self.STATUSWORD_INDEX:
            motor["status_word"] = int.from_bytes(value[:2], "little")
        elif index == self.MODES_OF_OPERATION_DISPLAY_INDEX:
            motor["mode_display"] = int.from_bytes(
                value[:1], "little", signed=True
            )
        elif index == self.POSITION_ACTUAL_INDEX:
            pulses = int.from_bytes(value, "little", signed=True)
            motor["actual_position"] = (
                pulses * 2.0 * math.pi / self.encoder_pulses_per_rev
            )
        elif index == self.VELOCITY_ACTUAL_INDEX:
            pulses_per_sec = int.from_bytes(value, "little", signed=True)
            reduction_ratio = self._get_mechanical_reduction_ratio(motor_id)
            if reduction_ratio is not None:
                motor["actual_velocity"] = (
                    pulses_per_sec
                    * 60.0
                    / (self.pulses_per_motor_rev * reduction_ratio)
                )
        elif index == self.TORQUE_ACTUAL_INDEX:
            torque_raw = int.from_bytes(value[:2], "little", signed=True)
            motor["actual_torque"] = torque_raw / 10.0

    def _parse_tpdo_feedback(self, can_id, data):
        """Decode the default TPDO1 position and TPDO2 velocity mappings."""
        node_id = can_id & 0x7F
        motor = self._get_motor(node_id)
        if motor is None:
            return
        if 0x180 <= can_id < 0x200 and len(data) >= 7:
            motor["status_word"] = int.from_bytes(data[0:2], "little")
            position = int.from_bytes(data[3:7], "little", signed=True)
            motor["actual_position"] = (
                position * 2.0 * math.pi / self.encoder_pulses_per_rev
            )
        elif 0x280 <= can_id < 0x300 and len(data) >= 4:
            velocity = int.from_bytes(data[0:4], "little", signed=True)
            reduction_ratio = self._get_mechanical_reduction_ratio(node_id)
            if reduction_ratio is not None:
                motor["actual_velocity"] = (
                    velocity
                    * 60.0
                    / (self.pulses_per_motor_rev * reduction_ratio)
                )
            if len(data) >= 6:
                motor["actual_torque"] = int.from_bytes(
                    data[4:6], "little", signed=True
                ) / 10.0

    def parse_motor_feedback(self, can_id, data):
        """Parse SDO responses and the AIMOTOR default TPDO mappings."""
        if self.SDO_TX_BASE <= can_id < self.SDO_TX_BASE + 0x80:
            self._parse_sdo_feedback(can_id - self.SDO_TX_BASE, bytes(data))
        elif 0x180 <= can_id < 0x300:
            self._parse_tpdo_feedback(can_id, bytes(data))

    def uint16_to_float(self, x, x_min, x_max, bits):
        """Convert an unsigned integer to a linearly scaled float."""
        span = (1 << bits) - 1
        return (x_max - x_min) * x / span + x_min

    def update_odometry(self):
        """Integrate wheel feedback and publish a planar odometry message."""
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time
        if dt <= 0 or len(self.motors) < 2:
            return

        left_vel = self.motors[0]["actual_velocity"]
        right_vel = self.motors[1]["actual_velocity"]
        rpm_to_mps = 2.0 * math.pi * self.wheel_radius / 60.0
        left_linear = left_vel * rpm_to_mps
        right_linear = right_vel * rpm_to_mps
        linear_velocity = (right_linear + left_linear) / 2.0
        angular_velocity = (right_linear - left_linear) / self.wheel_base
        self.x += linear_velocity * dt * math.cos(self.th)
        self.y += linear_velocity * dt * math.sin(self.th)
        self.th += angular_velocity * dt
        self.publish_odometry(linear_velocity, angular_velocity)

    def publish_odometry(self, linear_velocity, angular_velocity):
        """Publish the integrated differential-drive odometry."""
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
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
        """Return a geometry_msgs Quaternion for the supplied Euler angles."""
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
        """Receive EMCY, SDO, TPDO, and heartbeat frames in a daemon thread."""
        rospy.loginfo("Starting AIMOTOR ROS1 CANopen receive thread...")
        consecutive_errors = 0
        while self.running and not rospy.is_shutdown():
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
                elif 0x700 <= can_id < 0x780 and message.data:
                    motor = self._get_motor(can_id - 0x700)
                    if motor is not None:
                        motor["online"] = message.data[0] in (0x00, 0x05, 0x7F)
            except Exception as exc:
                consecutive_errors += 1
                rospy.logerr(
                    "CAN receive error (%s/5): %s", consecutive_errors, exc
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
        rospy.loginfo("Stopped CANopen receive thread")

    def start_receive_thread(self):
        """Start the daemon thread that drains the CAN receive queue."""
        self.receive_thread = threading.Thread(
            target=self.receive_can_frames,
            name="aimotor-can-rx",
            daemon=True,
        )
        self.receive_thread.start()

    def timer_callback(self, _event=None):
        """Send targets, poll feedback, integrate odometry, and publish state."""
        if (
            self.command_timeout_sec > 0
            and time.monotonic() - self.last_speed_command_time
            > self.command_timeout_sec
        ):
            with self._state_lock:
                for motor in self.motors:
                    motor["velocity"] = 0.0

        self.send_speed_commands()
        self._feedback_tick += 1
        if self._feedback_tick >= 5:
            self._feedback_tick = 0
            self.query_motor_feedback()
        self.update_odometry()

        velocity_msg = Float32MultiArray()
        velocity_msg.data = [
            float(motor["actual_velocity"]) for motor in self.motors
        ]
        self.velocity_publisher.publish(velocity_msg)

        feedback_msg = Float32MultiArray()
        feedback_data = []
        for motor in self.motors:
            feedback_data.extend(
                [
                    float(motor["id"]),
                    motor["actual_position"],
                    motor["actual_velocity"],
                    motor["actual_torque"],
                    motor["actual_temperature"],
                ]
            )
        feedback_msg.data = feedback_data
        self.motor_feedback_publisher.publish(feedback_msg)

    def stop_all_motors(self):
        """Send zero speed and disable-operation frames during shutdown."""
        if self._stopping:
            return
        self._stopping = True
        self.running = False
        if self.bus is None:
            return
        for motor in self.motors:
            motor["velocity"] = 0.0
            self.motor_set_speed(motor["id"], 0.0)
            time.sleep(0.01)
        for motor in self.motors:
            self.motor_disable(motor["id"])
            time.sleep(0.01)

    def destroy_node(self):
        """ROS2-compatible cleanup alias for callers shared with the old code."""
        self.stop_all_motors()
        if self.timer is not None:
            self.timer.shutdown()
        if self.subscription is not None:
            self.subscription.unregister()
        if self.receive_thread is not None:
            self.receive_thread.join(timeout=1.0)
        if self.bus is not None:
            try:
                self.bus.shutdown()
            except Exception as exc:
                rospy.logwarn("CAN bus shutdown failed: %s", exc)
            self.bus = None
        self.can_initialized = False


def main():
    """Run the standalone ROS1 AIMOTOR node."""
    rospy.init_node("can_motor_driver", anonymous=False)
    motor_driver = None
    try:
        motor_driver = CanMotorDriver()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:
        rospy.logerr("Unexpected error: %s", exc)
    finally:
        if motor_driver is not None:
            motor_driver.destroy_node()


if __name__ == "__main__":
    main()
