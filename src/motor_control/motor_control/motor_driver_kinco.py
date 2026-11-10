"""Kinco iSMK brushes (IDs 3/4) sharing the AIMotor wheels' CAN transport.

Protocol reference: iSMK manual 20250523, sections 5.2, 7.1--7.3,
7.11 and 9.3. Public speeds remain gearbox output shaft r/min.
"""

import importlib
import math
import threading

from std_msgs.msg import Float32MultiArray


AimotorDriver = importlib.import_module(
    "motor_control.motor_driver(AIMotor)"
).CanMotorDriver


class CanMotorDriver(AimotorDriver):
    """Use Kinco objects for brushes and the existing AIMotor API for wheels."""

    KINCO_MOTOR_IDS = (3, 4)
    KINCO_POSITION_INDEX = 0x6063
    KINCO_ERROR_STATUS_INDEX = 0x2601
    KINCO_ERROR_STATUS_2_INDEX = 0x2602
    KINCO_TEMPERATURE_INDEX = 0x6410
    KINCO_TEMPERATURE_SUBINDEX = 0x19

    def __init__(
        self, *args, kinco_encoder_resolution=65536,
        kinco_reduction_ratio=1.0, kinco_acceleration_rps2=100.0,
        kinco_deceleration_rps2=100.0, **kwargs
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
        self._kinco_ramp_raw(self.kinco_acceleration_rps2)
        self._kinco_ramp_raw(self.kinco_deceleration_rps2)
        self._kinco_acceleration_ready = {}
        super().__init__(*args, **kwargs)
        self.get_logger().info(
            "Kinco brush protocol enabled for configured IDs 3/4: "
            f"encoder={self.kinco_encoder_resolution}, "
            f"reduction={self.kinco_reduction_ratio}"
        )

    def motor_set_speed(self, motor_id: int, speed: float) -> bool:
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
        self._get_motor(motor_id)["velocity"] = speed
        return self._write_i32(motor_id, self.TARGET_VELOCITY_INDEX, raw_speed)

    def motor_set_direction(self, motor_id: int, direction=None) -> bool:
        """Kinco 607E uses 0/1 rather than AIMotor's bit 6."""
        if motor_id not in self.KINCO_MOTOR_IDS:
            return super().motor_set_direction(motor_id, direction)
        if direction is None:
            direction = self.motor_directions.get(motor_id)
        try:
            direction = self._validate_motor_direction(
                direction, f"motor_direction_{motor_id}"
            )
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return False
        result = self._sdo_write(
            motor_id, self.COMMAND_POLARITY_INDEX, 0, self.SDO_WRITE_1,
            bytes((1 if direction == -1 else 0,)),
        )
        if result:
            self.motor_directions[motor_id] = direction
        return result

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
        result = self._set_kinco_ramp(
            motor_id, self.PROFILE_ACCELERATION_INDEX,
            self.kinco_acceleration_rps2 if acceleration is None else acceleration,
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
        """Clear stale targets before Kinco's 06h -> 0Fh enable sequence."""
        if motor_id not in self.KINCO_MOTOR_IDS:
            return super().motor_enable(motor_id)
        if not getattr(self, "_kinco_acceleration_ready", {}).get(motor_id, True):
            self.get_logger().error(
                f"Kinco node {motor_id} remains disabled after acceleration failure"
            )
            return False
        return (
            self.motor_set_speed(motor_id, 0.0)
            and self._write_u16(motor_id, self.CONTROLWORD_INDEX, 0x0006)
            and self._write_u16(motor_id, self.CONTROLWORD_INDEX, 0x000F)
        )

    def motor_disable(self, motor_id: int) -> bool:
        """Release a Kinco shaft with 6040h=0006h."""
        if motor_id not in self.KINCO_MOTOR_IDS:
            return super().motor_disable(motor_id)
        return self._write_u16(motor_id, self.CONTROLWORD_INDEX, 0x0006)

    def motor_clear_fault(self, motor_id: int) -> bool:
        """Reset Kinco faults using the manual's 6040h=0086h."""
        if motor_id not in self.KINCO_MOTOR_IDS:
            return super().motor_clear_fault(motor_id)
        return self._write_u16(motor_id, self.CONTROLWORD_INDEX, 0x0086)

    def initialize_motors(self):
        """Release brushes before configuring their direction."""
        for motor in self.motors:
            if motor["id"] in self.KINCO_MOTOR_IDS:
                if not self.motor_disable(motor["id"]):
                    raise RuntimeError(
                        f"Cannot release Kinco node {motor['id']} for configuration"
                    )
        super().initialize_motors()

    def motor_query_feedback(self, motor_id, include_velocity=True):
        """Read only documented Kinco objects; PDO maps are not assumed."""
        if motor_id not in self.KINCO_MOTOR_IDS:
            return super().motor_query_feedback(motor_id, include_velocity)
        objects = [
            (self.STATUSWORD_INDEX, 0),
            (self.KINCO_POSITION_INDEX, 0),
        ]
        if include_velocity:
            objects.append((self.VELOCITY_ACTUAL_INDEX, 0))
        objects.extend((
            (self.FAULT_CODE_INDEX, 0),
            (self.KINCO_ERROR_STATUS_INDEX, 0),
            (self.KINCO_ERROR_STATUS_2_INDEX, 0),
            (self.KINCO_TEMPERATURE_INDEX, self.KINCO_TEMPERATURE_SUBINDEX),
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
            self.SDO_READ_4_RESPONSE
            if index in (self.KINCO_POSITION_INDEX, self.VELOCITY_ACTUAL_INDEX)
            else self.SDO_READ_2_RESPONSE
        )
        if data[0] != expected_command:
            return
        fault_changed = False
        with self._feedback_lock:
            if index == self.VELOCITY_ACTUAL_INDEX:
                motor["actual_velocity"] = (
                    int.from_bytes(data[4:8], "little", signed=True) * 1875
                    / (512 * self.kinco_encoder_resolution)
                    / self.kinco_reduction_ratio
                )
            elif index == self.KINCO_POSITION_INDEX:
                motor["actual_position"] = (
                    int.from_bytes(data[4:8], "little", signed=True)
                    * 2 * math.pi / self.kinco_encoder_resolution
                )
            elif index == self.STATUSWORD_INDEX:
                motor["status_word"] = int.from_bytes(data[4:6], "little")
            elif index == self.FAULT_CODE_INDEX:
                fault_code = int.from_bytes(data[4:6], "little")
                fault_changed = motor["fault_code"] != fault_code
                motor["fault_code"] = fault_code
            elif index == self.KINCO_ERROR_STATUS_INDEX:
                motor["kinco_error_status"] = int.from_bytes(data[4:6], "little")
            elif index == self.KINCO_ERROR_STATUS_2_INDEX:
                motor["kinco_error_status_2"] = int.from_bytes(data[4:6], "little")
            elif index == self.KINCO_TEMPERATURE_INDEX:
                motor["actual_temperature"] = int.from_bytes(
                    data[4:6], "little", signed=True
                )
        if fault_changed:
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
        """Preserve Kinco EMCY and both error words without AIMotor decoding."""
        motor_id = can_id - self.EMCY_BASE
        if motor_id not in self.KINCO_MOTOR_IDS:
            return super().parse_motor_fault(can_id, data)
        motor = self._get_motor(motor_id)
        if motor is None or len(data) != 8:
            return
        with self._feedback_lock:
            motor["fault_code"] = int.from_bytes(data[0:2], "little")
            motor["kinco_error_status"] = int.from_bytes(data[4:6], "little")
            motor["kinco_error_status_2"] = int.from_bytes(data[6:8], "little")
        self.get_logger().info(
            f"Kinco node {motor_id} EMCY=0x{motor['fault_code']:04X}, "
            f"2601=0x{motor['kinco_error_status']:04X}, "
            f"2602=0x{motor['kinco_error_status_2']:04X}"
        )
        self.publish_motor_fault_codes()
