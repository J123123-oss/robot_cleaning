"""Kinco iSMK brushes (IDs 3/4) sharing the AIMotor wheels' CAN transport.

Protocol reference: iSMK manual 20250523, sections 5.2, 7.1--7.3,
7.11 and 9.3. Public speeds remain gearbox output shaft r/min.
"""

import importlib
import json
import math
import threading
import time

from std_msgs.msg import Float32MultiArray, String


AimotorDriver = importlib.import_module(
    "motor_control.motor_driver(AIMotor)"
).CanMotorDriver


class CanMotorDriver(AimotorDriver):
    """Use Kinco objects for brushes and the existing AIMotor API for wheels."""

    KINCO_FAULT_TABLE = {
        0x0000: "无故障",
        0x2320: "驱动器功率部分短路或电机短路",
        0x2321: "电流采样饱和",
        0x2350: "电机或驱动器过载",
        0x3210: "驱动器总线电压过高",
        0x3220: "驱动器总线电压过低",
        0x4210: "驱动器温度过高",
        0x4310: "电机温度过高",
        0x5112: "逻辑电压低",
        0x5210: "电流传感器故障",
        0x5441: "负限位报错",
        0x5442: "正限位报错",
        0x5443: "预使能报警",
        0x6010: "软件看门狗复位",
        0x6011: "异常中断",
        0x6012: "SPI 故障",
        0x6310: "EEPROM 数据错误",
        0x6320: "电机型号配置错误",
        0x6321: "电机动力线缺相",
        0x7110: "驱动器制动电阻异常",
        0x7306: "主编码器计数错误",
        0x7320: "通讯式编码器多圈错误",
        0x7330: "通讯式编码器校验错误",
        0x7331: "通讯式编码器没有连接或没有回应",
        0x7382: "主编码器ABZ 故障",
        0x7400: "MCU 故障",
        0x8100: "总线通讯错误",
        0x81FF: "总线通讯超时",
        0x8611: "实际跟随误差超过允许",
        0x8A80: "输入脉冲频率过高",
        0x8A81: "全闭环检查错误",
    }
    KINCO_ERROR_REGISTER_TABLE = {
        0: "一般错误",
        1: "电流",
        2: "电压",
        3: "温度",
        4: "通讯错误",
        5: "设备配置文件特定",
        6: "编码器",
        7: "保留",
    }
    KINCO_MOTOR_IDS = (3, 4)
    KINCO_VELOCITY_MODE = 3
    SDO_READ_1_RESPONSE = 0x4F
    KINCO_CONTROL_SHUTDOWN = 0x0006
    KINCO_CONTROL_SWITCH_ON = 0x0007
    KINCO_CONTROL_ENABLE_OPERATION = 0x000F
    KINCO_POSITION_INDEX = 0x6063
    KINCO_ACTUAL_CURRENT_INDEX = 0x6078
    KINCO_ERROR_STATUS_INDEX = 0x2601
    KINCO_ERROR_STATUS_2_INDEX = 0x2602
    KINCO_TEMPERATURE_INDEX = 0x6410
    KINCO_TEMPERATURE_SUBINDEX = 0x19
    KINCO_FEEDBACK_FIELDS = (
        "output_speed_rpm", "motor_speed_rpm", "velocity_raw",
        "motor_position_rad", "position_raw", "output_current_arms",
        "output_current_ap", "current_raw", "temperature_c",
        "status_word", "mode_display", "fault_code", "error_register",
        "error_status", "error_status_2",
    )

    def __init__(
        self, *args, kinco_encoder_resolution=65536,
        kinco_reduction_ratio=10.0, kinco_acceleration_rps2=10.0,
        kinco_deceleration_rps2=10.0, kinco_peak_current_a=47.9,
        kinco_guard_time_ms=100, kinco_guard_life_factor=5, **kwargs
    ):
        """Configure Kinco conversion before the inherited threads start."""
        self.kinco_encoder_resolution = self._validate_positive_int_parameter(
            kinco_encoder_resolution, "kinco_encoder_resolution"
        )
        self.kinco_reduction_ratio = self._validate_positive_float_parameter(
            kinco_reduction_ratio, "kinco_reduction_ratio"
        )
        self.kinco_acceleration_rps2 = self._validate_positive_float_parameter(
            kinco_acceleration_rps2, "kinco_acceleration_rps2"
        )
        self.kinco_deceleration_rps2 = self._validate_positive_float_parameter(
            kinco_deceleration_rps2, "kinco_deceleration_rps2"
        )
        self.kinco_peak_current_a = self._validate_positive_float_parameter(
            kinco_peak_current_a, "kinco_peak_current_a"
        )
        self._kinco_ramp_raw(self.kinco_acceleration_rps2)
        self._kinco_ramp_raw(self.kinco_deceleration_rps2)
        self._kinco_acceleration_ready = {}
        self.kinco_guard_time_ms = self._validate_positive_int_parameter(
            kinco_guard_time_ms, "kinco_guard_time_ms"
        )
        self.kinco_guard_life_factor = self._validate_positive_int_parameter(
            kinco_guard_life_factor, "kinco_guard_life_factor"
        )
        if not 50 <= self.kinco_guard_time_ms <= 65535:
            raise ValueError("kinco_guard_time_ms must be in 50..65535")
        if not 2 <= self.kinco_guard_life_factor <= 255:
            raise ValueError("kinco_guard_life_factor must be in 2..255")
        self._kinco_guard_nodes = {}
        super().__init__(*args, **kwargs)
        self.kinco_feedback_publisher = self.create_publisher(
            String, "/motor/kinco_feedback", 10
        )
        self.get_logger().info(
            "Kinco brush protocol enabled for configured IDs 3/4: "
            f"encoder={self.kinco_encoder_resolution}, "
            f"reduction={self.kinco_reduction_ratio}, "
            f"ipeak={self.kinco_peak_current_a}A"
        )

    @classmethod
    def describe_kinco_fault(cls, fault_code):
        """Decode manual 9.3.3.6 table 9-24; 7331 retains both listed meanings."""
        if fault_code is None:
            return None
        return cls.KINCO_FAULT_TABLE.get(
            fault_code, f"手册未定义故障码 0x{fault_code:04X}"
        )

    @classmethod
    def decode_kinco_error_register(cls, error_register):
        """Decode all set 1001 bits from table 9-25; unread registers remain null."""
        if error_register is None:
            return None
        return [
            description for bit, description in cls.KINCO_ERROR_REGISTER_TABLE.items()
            if error_register & (1 << bit)
        ]

    def get_kinco_feedback_snapshot(self):
        """Return cached Kinco samples with per-field monotonic ages; unread values are null."""
        now = time.monotonic()
        with self._feedback_lock:
            feedback = []
            for motor in self.motors:
                if motor["id"] not in self.KINCO_MOTOR_IDS:
                    continue
                samples = motor.get("kinco_feedback", {})
                ages = {
                    field: max(0.0, now - samples[field][1])
                    if field in samples else None
                    for field in self.KINCO_FEEDBACK_FIELDS
                }
                values = {
                    field: samples[field][0] if field in samples else None
                    for field in self.KINCO_FEEDBACK_FIELDS
                }
                feedback.append({
                    "id": motor["id"],
                    "target_output_speed_rpm": motor["velocity"],
                    **values,
                    "fault_description": self.describe_kinco_fault(values["fault_code"]),
                    "error_register_descriptions": self.decode_kinco_error_register(
                        values["error_register"]
                    ),
                    "sample_age_sec": ages,
                })
            return feedback

    def poll_feedback(self):
        """Publish the shared feedback and a named Kinco snapshot without extra CAN requests."""
        snapshot = super().poll_feedback()
        publisher = getattr(self, "kinco_feedback_publisher", None)
        if publisher is not None:
            message = String()
            message.data = json.dumps({
                "timestamp_unix_sec": time.time(),
                "motors": self.get_kinco_feedback_snapshot(),
            }, allow_nan=False, ensure_ascii=False)
            publisher.publish(message)
        return snapshot

    def motor_set_speed(self, motor_id: int, speed: float, *, send_allowed=None) -> bool:
        """Write Kinco DEC = motor rpm * 512 * encoder resolution / 1875."""
        if motor_id not in self.KINCO_MOTOR_IDS:
            return super().motor_set_speed(motor_id, speed)
        if not self._validate_motor_id(motor_id):
            return False
        try:
            speed = float(speed)
            raw_speed = (
                speed * self.kinco_reduction_ratio
                * 512 * self.kinco_encoder_resolution / 1875
            )
            if not math.isfinite(raw_speed):
                raise ValueError("speed must be finite")
            raw_speed = round(raw_speed)
            if not -(1 << 31) <= raw_speed < (1 << 31):
                raise ValueError("speed exceeds Kinco INT32 range")
        except (TypeError, ValueError, OverflowError) as exc:
            self.get_logger().error(f"Invalid Kinco velocity {speed!r}: {exc}")
            return False
        if not self._sdo_write(
            motor_id, self.TARGET_VELOCITY_INDEX, 0, self.SDO_WRITE_4,
            raw_speed.to_bytes(4, "little", signed=True), send_allowed=send_allowed,
        ):
            return False
        self._get_motor(motor_id)["velocity"] = speed
        return True

    def motor_set_direction(self, motor_id: int, direction=None) -> bool:
        """Configure and verify 607E polarity only after confirming shutdown."""
        if motor_id not in self.KINCO_MOTOR_IDS:
            return super().motor_set_direction(motor_id, direction)
        if not self._validate_motor_id(motor_id):
            return False
        if direction is None:
            direction = self.motor_directions.get(motor_id)
        try:
            direction = self._validate_motor_direction(
                direction, f"kinco_motor_direction_{motor_id}"
            )
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return False
        polarity = 1 if direction == -1 else 0
        with self._sdo_transaction_lock:
            if not self.motor_set_speed(motor_id, 0.0):
                return False
            if not self.motor_disable(motor_id):
                return False
            if not self._wait_kinco_shutdown(motor_id):
                return False
            if not self._sdo_write(
                motor_id, self.COMMAND_POLARITY_INDEX, 0, self.SDO_WRITE_1,
                bytes((polarity,)),
            ):
                return False
            response = self._read_sdo_response(
                motor_id, self.COMMAND_POLARITY_INDEX, 0,
                self.SDO_READ_1_RESPONSE,
            )
            actual = None if response is None else response[4]
            if actual != polarity:
                self.get_logger().error(
                    f"Kinco node {motor_id} direction readback failed: "
                    f"direction={direction}, requested 607E={polarity}, "
                    f"actual={actual}; initialization must remain disabled"
                )
                return False
        self.motor_directions[motor_id] = direction
        self.get_logger().info(
            f"Kinco node {motor_id} direction confirmed: "
            f"direction={direction}, 607E={polarity}"
        )
        return True

    def _wait_kinco_shutdown(self, motor_id, timeout_sec=0.5):
        """Wait for a live ready-to-switch-on status before changing polarity."""
        deadline = time.monotonic() + timeout_sec
        status = None
        while time.monotonic() < deadline:
            response = self._read_sdo_response(
                motor_id, self.STATUSWORD_INDEX, 0, self.SDO_READ_2_RESPONSE,
                timeout_sec=max(0.001, deadline - time.monotonic()),
            )
            if response is None:
                break
            status = int.from_bytes(response[4:6], "little")
            if status & 0x006F == 0x0021:
                return True
            if status & 0x0008:
                break
            time.sleep(0.01)
        self.get_logger().error(
            f"Kinco node {motor_id} shutdown not confirmed: "
            f"6041={None if status is None else hex(status)}; "
            "direction was not changed"
        )
        return False

    def motor_set_mode(self, motor_id: int, mode: int) -> bool:
        """Select Kinco profile velocity mode (3, wire value 03h)."""
        if motor_id not in self.KINCO_MOTOR_IDS:
            return super().motor_set_mode(motor_id, mode)
        if not self._validate_motor_id(motor_id):
            return False
        if mode != self.KINCO_VELOCITY_MODE:
            self.get_logger().error(
                f"Unsupported Kinco velocity mode for motor {motor_id}: {mode}"
            )
            return False
        return self._sdo_write(
            motor_id,
            self.MODES_OF_OPERATION_INDEX,
            0,
            self.SDO_WRITE_1,
            self.KINCO_VELOCITY_MODE.to_bytes(1, "little", signed=True),
        )

    def _kinco_ramp_raw(self, value):
        """Convert rps/s to Kinco's unsigned 32-bit acceleration units."""
        value = self._validate_positive_float_parameter(value, "kinco_ramp")
        raw_value = value * 65536 * self.kinco_encoder_resolution / 4000000
        if not math.isfinite(raw_value):
            raise ValueError("Kinco ramp must be finite")
        raw_value = round(raw_value)
        if not 1 <= raw_value <= 0xFFFFFFFF:
            raise ValueError("Kinco ramp must fit in a nonzero UINT32")
        return raw_value

    def _set_kinco_ramp(self, motor_id, index, value, verify=False):
        """Write a validated Kinco ramp and optionally check its readback."""
        try:
            raw_value = self._kinco_ramp_raw(value)
        except (ValueError, OverflowError) as exc:
            self.get_logger().error(str(exc))
            return False
        if not self._sdo_write(
            motor_id, index, 0, self.SDO_WRITE_4,
            raw_value.to_bytes(4, "little"),
        ):
            return False
        receive_thread = getattr(self, "receive_thread", None)
        if verify and receive_thread is not None and receive_thread.is_alive():
            response = self._read_sdo_response(
                motor_id, index, 0, self.SDO_READ_4_RESPONSE
            )
            if response is None:
                return False
            actual = int.from_bytes(response[4:8], "little")
            if actual != raw_value:
                self.get_logger().error(
                    f"Kinco node {motor_id} ramp readback mismatch: "
                    f"requested={raw_value}, actual={actual}"
                )
                return False
        return True

    def motor_set_acceleration(self, motor_id: int, acceleration=None) -> bool:
        """Use separate Kinco rps/s values for brushes."""
        if motor_id not in self.KINCO_MOTOR_IDS:
            return super().motor_set_acceleration(motor_id, acceleration)
        with self._sdo_transaction_lock:
            result = self._set_kinco_ramp(
                motor_id, self.PROFILE_ACCELERATION_INDEX,
                self.kinco_acceleration_rps2 if acceleration is None else acceleration,
                verify=True,
            )
        if not hasattr(self, "_kinco_acceleration_ready"):
            self._kinco_acceleration_ready = {}
        self._kinco_acceleration_ready[motor_id] = result
        return result

    def motor_set_deceleration(self, motor_id: int, deceleration=None) -> bool:
        """Read back Kinco's raw deceleration before allowing enable."""
        if motor_id not in self.KINCO_MOTOR_IDS:
            return super().motor_set_deceleration(motor_id, deceleration)
        transaction_lock = getattr(
            self, "_sdo_transaction_lock", threading.Lock()
        )
        with transaction_lock:
            return self._set_kinco_ramp(
                motor_id, self.PROFILE_DECELERATION_INDEX,
                self.kinco_deceleration_rps2
                if deceleration is None else deceleration,
                verify=True,
            )

    def motor_enable(self, motor_id: int) -> bool:
        """Enable Kinco velocity mode and leave 6040h in operation-enabled state."""
        if motor_id not in self.KINCO_MOTOR_IDS:
            return super().motor_enable(motor_id)
        if not getattr(self, "_kinco_acceleration_ready", {}).get(motor_id, True):
            self.get_logger().error(
                f"Kinco node {motor_id} remains disabled after acceleration failure"
            )
            return False
        if not self.motor_set_speed(motor_id, 0.0):
            return False
        if not self._write_u16(
            motor_id, self.CONTROLWORD_INDEX, self.KINCO_CONTROL_SHUTDOWN
        ):
            return False
        if not self.configure_kinco_guard(motor_id):
            self.get_logger().error(
                f"Kinco node {motor_id} remains disabled: communication guard not confirmed"
            )
            return False
        for controlword in (
            self.KINCO_CONTROL_SWITCH_ON,
            self.KINCO_CONTROL_ENABLE_OPERATION,
        ):
            if not self._write_u16(
                motor_id, self.CONTROLWORD_INDEX, controlword
            ):
                return False
        return self.wait_motor_status(motor_id, 0x006F, 0x0027)

    def configure_kinco_guard(self, motor_id):
        """回读确认节点保护参数，发送首条 RTR 后才允许使能。"""
        if (motor_id not in self.KINCO_MOTOR_IDS
                or not self._validate_motor_id(motor_id)):
            return False
        settings = (
            (0x1017, 0, 2, self.SDO_READ_2_RESPONSE),
            (0x100C, self.kinco_guard_time_ms, 2, self.SDO_READ_2_RESPONSE),
            (0x100D, self.kinco_guard_life_factor, 1, self.SDO_READ_1_RESPONSE),
            (0x6007, 1, 2, self.SDO_READ_2_RESPONSE),
        )
        with self._sdo_transaction_lock:
            for index, value, size, response_command in settings:
                command = self.SDO_WRITE_1 if size == 1 else self.SDO_WRITE_2
                if not self._sdo_write(
                    motor_id, index, 0, command, value.to_bytes(size, "little")
                ):
                    self.get_logger().error(
                        f"Kinco node {motor_id} guard write failed: 0x{index:04X}"
                    )
                    return False
                response = self._read_sdo_response(
                    motor_id, index, 0, response_command
                )
                actual = (
                    None if response is None
                    else int.from_bytes(response[4:4 + size], "little")
                )
                if actual != value:
                    self.get_logger().error(
                        f"Kinco node {motor_id} guard readback failed: "
                        f"0x{index:04X}, requested={value}, actual={actual}"
                    )
                    return False
        if not self.send_can_frame(0x700 + motor_id, b"", is_remote_frame=True):
            return False
        with self._feedback_lock:
            self._kinco_guard_nodes[motor_id] = time.monotonic()
        return True

    def service_periodic_can_frames(self):
        """复用 CAN 发送线程发送 RTR；主控退出或发送线程停顿即停止喂保护。"""
        if not self.running or self._stopping:
            return
        now = time.monotonic()
        with self._feedback_lock:
            nodes = tuple(self._kinco_guard_nodes.items())
        for motor_id, last_sent in nodes:
            if now - last_sent < self.kinco_guard_time_ms / 1000.0:
                continue
            if self._send_can_frame_direct(
                0x700 + motor_id, b"", is_remote_frame=True
            ):
                with self._feedback_lock:
                    self._kinco_guard_nodes[motor_id] = time.monotonic()
                self._can_tx_last_send_time = time.monotonic()

    def motor_disable(self, motor_id: int) -> bool:
        """Release a Kinco shaft with 6040h=0006h."""
        if motor_id not in self.KINCO_MOTOR_IDS:
            return super().motor_disable(motor_id)
        return self._write_u16(motor_id, self.CONTROLWORD_INDEX, 0x0006)

    def preserves_enable_on_shutdown(self, motor_id):
        """混合驱动仅允许 AIMotor 保持使能，步科退出仍失能。"""
        return motor_id not in self.KINCO_MOTOR_IDS

    def motor_clear_fault(self, motor_id: int) -> bool:
        """Reset Kinco faults using the manual's 6040h=0086h."""
        if motor_id not in self.KINCO_MOTOR_IDS:
            return super().motor_clear_fault(motor_id)
        return self._write_u16(motor_id, self.CONTROLWORD_INDEX, 0x0086)

    def initialize_motors(self, motor_ids=None, *, enable_kinco=True):
        """Configure selected nodes, optionally deferring brush enable."""
        selected_ids = tuple(self.motor_ids if motor_ids is None else motor_ids)
        enable_ids = (
            selected_ids if enable_kinco else
            tuple(motor_id for motor_id in selected_ids
                  if motor_id not in self.KINCO_MOTOR_IDS)
        )
        return super().initialize_motors(
            motor_ids=selected_ids, enable_motor_ids=enable_ids
        )

    def read_brush_velocity(self, motor_id, timeout_sec):
        """Read one live 606C sample as output-shaft rpm."""
        response = self._read_sdo_response(
            motor_id, self.VELOCITY_ACTUAL_INDEX, 0, self.SDO_READ_4_RESPONSE,
            timeout_sec=timeout_sec,
        )
        if response is None:
            return None
        raw = int.from_bytes(response[4:8], "little", signed=True)
        return raw * 1875 / (
            512 * self.kinco_encoder_resolution * self.kinco_reduction_ratio
        )

    def prepare_motor_initialization(self, motor_id):
        """Restore RTR protection before resetting a Kinco fault left by an outage."""
        if motor_id in self.KINCO_MOTOR_IDS:
            if not self.configure_kinco_guard(motor_id):
                return False
        return super().prepare_motor_initialization(motor_id)

    def motor_query_feedback(self, motor_id, include_velocity=True):
        """Read only documented Kinco objects; PDO maps are not assumed."""
        if motor_id not in self.KINCO_MOTOR_IDS:
            return super().motor_query_feedback(motor_id, include_velocity)
        objects = [
            (self.STATUSWORD_INDEX, 0),
            (self.MODES_OF_OPERATION_DISPLAY_INDEX, 0),
            (self.KINCO_POSITION_INDEX, 0),
        ]
        if include_velocity:
            objects.append((self.VELOCITY_ACTUAL_INDEX, 0))
        objects.extend((
            (self.FAULT_CODE_INDEX, 0),
            (self.KINCO_ERROR_STATUS_INDEX, 0),
            (self.KINCO_ERROR_STATUS_2_INDEX, 0),
            (self.KINCO_TEMPERATURE_INDEX, self.KINCO_TEMPERATURE_SUBINDEX),
            (self.KINCO_ACTUAL_CURRENT_INDEX, 0),
        ))
        results = [
            self._sdo_read(motor_id, index, subindex)
            for index, subindex in objects
        ]
        return all(results)

    def _parse_sdo_feedback(self, motor_id, data):
        """Decode Kinco uploads and wake pending inherited SDO transactions."""
        if motor_id not in self.KINCO_MOTOR_IDS:
            return super()._parse_sdo_feedback(motor_id, data)
        if len(data) != 8:
            return
        motor = self._get_motor(motor_id)
        if motor is None:
            return
        index = int.from_bytes(data[1:3], "little")
        subindex = data[3]
        pending_lock = getattr(self, "_pending_sdo_lock", None)
        if pending_lock is not None:
            with pending_lock:
                pending = self._pending_sdo_responses.get(
                    (motor_id, index, subindex)
                )
                if pending and data[0] in (
                    pending["expected_command"], self.SDO_ABORT
                ):
                    pending["data"] = bytes(data)
                    pending["event"].set()
        expected_subindex = (
            self.KINCO_TEMPERATURE_SUBINDEX
            if index == self.KINCO_TEMPERATURE_INDEX else 0
        )
        if subindex != expected_subindex:
            return
        expected_command = (
            0x4F if index == self.MODES_OF_OPERATION_DISPLAY_INDEX
            else self.SDO_READ_4_RESPONSE
            if index in (self.KINCO_POSITION_INDEX, self.VELOCITY_ACTUAL_INDEX)
            else self.SDO_READ_2_RESPONSE
        )
        if data[0] != expected_command:
            return
        fault_changed = False
        with self._feedback_lock:
            values = {}
            if index == self.VELOCITY_ACTUAL_INDEX:
                motor["actual_velocity"] = (
                    int.from_bytes(data[4:8], "little", signed=True) * 1875
                    / (512 * self.kinco_encoder_resolution)
                    / self.kinco_reduction_ratio
                )
                values = {
                    "output_speed_rpm": motor["actual_velocity"],
                    "motor_speed_rpm": (
                        motor["actual_velocity"] * self.kinco_reduction_ratio
                    ),
                    "velocity_raw": int.from_bytes(data[4:8], "little", signed=True),
                }
            elif index == self.KINCO_POSITION_INDEX:
                motor["actual_position"] = (
                    int.from_bytes(data[4:8], "little", signed=True)
                    * 2 * math.pi / self.kinco_encoder_resolution
                )
                values = {
                    "motor_position_rad": motor["actual_position"],
                    "position_raw": int.from_bytes(data[4:8], "little", signed=True),
                }
            elif index == self.STATUSWORD_INDEX:
                motor["status_word"] = int.from_bytes(data[4:6], "little")
                values = {"status_word": motor["status_word"]}
            elif index == self.MODES_OF_OPERATION_DISPLAY_INDEX:
                motor["mode_display"] = int.from_bytes(data[4:5], "little", signed=True)
                values = {"mode_display": motor["mode_display"]}
            elif index == self.FAULT_CODE_INDEX:
                fault_code = int.from_bytes(data[4:6], "little")
                fault_changed = motor["fault_code"] != fault_code
                motor["fault_code"] = fault_code
                values = {"fault_code": fault_code}
            elif index == self.KINCO_ERROR_STATUS_INDEX:
                motor["kinco_error_status"] = int.from_bytes(data[4:6], "little")
                values = {"error_status": motor["kinco_error_status"]}
            elif index == self.KINCO_ERROR_STATUS_2_INDEX:
                motor["kinco_error_status_2"] = int.from_bytes(data[4:6], "little")
                values = {"error_status_2": motor["kinco_error_status_2"]}
            elif index == self.KINCO_TEMPERATURE_INDEX:
                motor["actual_temperature"] = int.from_bytes(
                    data[4:6], "little", signed=True
                )
                values = {"temperature_c": motor["actual_temperature"]}
            elif index == self.KINCO_ACTUAL_CURRENT_INDEX:
                raw_current = int.from_bytes(data[4:6], "little", signed=True)
                motor["actual_current"] = (
                    raw_current * self.kinco_peak_current_a / (1.414 * 2048)
                )
                motor["actual_current_updated_monotonic"] = time.monotonic()
                values = {
                    "output_current_arms": motor["actual_current"],
                    "output_current_ap": raw_current * self.kinco_peak_current_a / 2048,
                    "current_raw": raw_current,
                }
            samples = motor.setdefault("kinco_feedback", {})
            updated = time.monotonic()
            samples.update({field: (value, updated) for field, value in values.items()})
        if fault_changed:
            log = self.get_logger().error if fault_code else self.get_logger().info
            log(
                f"Kinco node {motor_id} 603F=0x{fault_code:04X} "
                f"({self.describe_kinco_fault(fault_code)})"
            )
            self.publish_motor_fault_codes()
        if index == self.VELOCITY_ACTUAL_INDEX:
            message = Float32MultiArray()
            message.data = [float(motor_id), motor["actual_velocity"]]
            self.motor_velocity_feedback_publisher.publish(message)

    def _parse_tpdo_feedback(self, can_id, data):
        """Keep AIMotor PDO parsing on wheels; Kinco uses explicit SDO reads."""
        if (can_id & 0x7F) not in self.KINCO_MOTOR_IDS:
            super()._parse_tpdo_feedback(can_id, data)

    def parse_motor_fault(self, can_id, data):
        """Decode EMCY bytes per manual 9.3.3.6 tables 9-23/24/25, leaving byte 3 unused."""
        motor_id = can_id - self.EMCY_BASE
        if motor_id not in self.KINCO_MOTOR_IDS:
            return super().parse_motor_fault(can_id, data)
        motor = self._get_motor(motor_id)
        if motor is None or len(data) != 8:
            return
        fault_code = int.from_bytes(data[0:2], "little")
        error_register = data[2]
        error_status = int.from_bytes(data[4:6], "little")
        error_status_2 = int.from_bytes(data[6:8], "little")
        with self._feedback_lock:
            motor["fault_code"] = fault_code
            motor["kinco_error_register"] = error_register
            motor["kinco_error_status"] = error_status
            motor["kinco_error_status_2"] = error_status_2
            updated = time.monotonic()
            motor.setdefault("kinco_feedback", {}).update({
                "fault_code": (fault_code, updated),
                "error_register": (error_register, updated),
                "error_status": (error_status, updated),
                "error_status_2": (error_status_2, updated),
            })
        register_description = "、".join(
            self.decode_kinco_error_register(error_register)
        ) or "无错误位"
        log = self.get_logger().error if fault_code else self.get_logger().info
        log(
            f"Kinco node {motor_id} EMCY=0x{fault_code:04X} "
            f"({self.describe_kinco_fault(fault_code)}), "
            f"1001=0x{error_register:02X} ({register_description}), "
            f"2601=0x{error_status:04X}, 2602=0x{error_status_2:04X}"
        )
        self.publish_motor_fault_codes()
