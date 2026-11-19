"""Serialize Kinco brush transitions without blocking motion callbacks on SDO reads."""

import threading
import time
from enum import IntEnum


BRUSH_STOP_TIMEOUT_SEC = 3.0


class BrushAction(IntEnum):
    DISABLE = 0
    HOLD_ZERO = 1
    RUN = 2


class KincoBrushControl:
    def __init__(self, driver, motor_ids, can_run):
        self._driver = driver
        self._motor_ids = tuple(motor_ids)
        self._can_run = can_run
        self._condition = threading.Condition()
        self._desired = (BrushAction.DISABLE, 0.0, None)
        self._applied = self._desired
        self._deadline = None
        self._disable_pending = False
        self._urgent = False
        self._busy = False
        self._disabling = False
        self._closing = False
        self._enabled = set()
        self.fault = None
        self.speed = 0.0
        self._worker = threading.Thread(target=self._run, name="kinco-brush", daemon=True)
        self._worker.start()

    def request(self, action, speed=0.0, *, source=None, valid_for=None, urgent=False):
        """Replace the desired command while preserving a pending disable edge."""
        with self._condition:
            if self._closing or (self.fault and action != BrushAction.DISABLE):
                return
            if action == BrushAction.DISABLE:
                if self._desired[0] != action or urgent:
                    self._disable_pending = True
                self._urgent |= urgent
            self._desired = (action, speed, source)
            self._deadline = None if valid_for is None else time.monotonic() + valid_for
            self._condition.notify_all()

    def wait_idle(self):
        with self._condition:
            self._condition.wait_for(
                lambda: not self._busy and not self._disable_pending
                and self._desired == self._applied
            )

    @property
    def wheel_motion_blocked(self):
        with self._condition:
            return (
                self._disable_pending or self._disabling or self.fault is not None
                or (self._desired[0] == BrushAction.RUN and self.speed == 0.0)
            )

    def reset_fault(self):
        with self._condition:
            self.fault = None

    def close(self, *, urgent=False):
        """Stop the brushes before closing the worker and its CAN access."""
        self.request(BrushAction.DISABLE, urgent=urgent)
        self.wait_idle()
        with self._condition:
            self._closing = True
            self._condition.notify_all()
        self._worker.join()

    def _run(self):
        while True:
            with self._condition:
                if self._deadline is not None and time.monotonic() >= self._deadline:
                    self._desired = (BrushAction.DISABLE, 0.0, None)
                    self._deadline = None
                    self._disable_pending = True
                if self._closing:
                    return
                if not self._disable_pending and self._desired == self._applied:
                    timeout = None if self._deadline is None else max(
                        0.0, self._deadline - time.monotonic()
                    )
                    self._condition.wait(timeout)
                    continue
                command = self._desired
                if self._disable_pending:
                    command = (BrushAction.DISABLE, 0.0, None)
                    self._disable_pending = False
                urgent, self._urgent = self._urgent, False
                self._busy = True
                self._disabling = command[0] == BrushAction.DISABLE
            try:
                action, _, source = command
                if action == BrushAction.RUN:
                    command = self._start(source)
                elif action == BrushAction.HOLD_ZERO:
                    if self._enabled:
                        self._write_speed(0.0)
                else:
                    self._disable(urgent)
            except Exception as error:
                details = []
                for name, operation, args in (
                    ("零速", self._driver.motor_set_speed, (0.0,)),
                    ("失能", self._driver.motor_disable, ()),
                ):
                    for motor_id in self._motor_ids:
                        try:
                            if not operation(motor_id, *args):
                                details.append(f"{name} ID {motor_id}")
                        except Exception as stop_error:
                            details.append(str(stop_error))
                self._enabled.clear()
                self.speed = 0.0
                with self._condition:
                    self.fault = f"{error}; {'; '.join(details)}" if details else str(error)
                    command = (BrushAction.DISABLE, 0.0, None)
                    self._desired = command
                    self._deadline = None
            finally:
                with self._condition:
                    if command is not None:
                        self._applied = command
                    self._busy = False
                    self._disabling = False
                    self._condition.notify_all()

    def _may_run(self, source):
        return (
            not self._disable_pending
            and self._desired[0] == BrushAction.RUN
            and self._desired[2] == source
            and (self._deadline is None or time.monotonic() < self._deadline)
            and self._can_run(source)
        )

    def _start(self, source):
        for motor_id in self._motor_ids:
            with self._condition:
                if not self._may_run(source):
                    return self._cancel_start(source)
            if motor_id not in self._enabled:
                status = self._driver.read_motor_status_word(motor_id)
                if status is None or status & 0x0008:
                    raise RuntimeError(f"滚刷 {motor_id} 状态异常，禁止使能")
                with self._condition:
                    if not self._may_run(source):
                        return self._cancel_start(source)
                if not self._driver.motor_enable(motor_id):
                    raise RuntimeError(f"滚刷 {motor_id} 使能失败")
                self._enabled.add(motor_id)
        with self._condition:
            if not self._may_run(source):
                return self._cancel_start(source)
            command = self._desired

        def send_allowed():
            with self._condition:
                return self._desired == command and self._may_run(source)

        for motor_id in self._motor_ids:
            if not self._driver.motor_set_speed(
                motor_id, command[1],
                send_allowed=send_allowed,
            ):
                with self._condition:
                    if self._desired != command:
                        return None
                    if not self._may_run(source):
                        return self._cancel_start(source)
                raise RuntimeError(f"步科滚刷 {motor_id} 速度下发失败")
        self.speed = command[1]
        return command

    def _cancel_start(self, source):
        if (self._desired[0] == BrushAction.RUN and self._desired[2] == source
                and not self._disable_pending):
            self._desired = (BrushAction.DISABLE, 0.0, None)
            self._disable_pending = True
            self._deadline = None
        return None

    def _write_speed(self, speed):
        results = [
            self._driver.motor_set_speed(motor_id, speed)
            for motor_id in self._motor_ids
        ]
        if not all(results):
            raise RuntimeError("步科滚刷速度下发失败")
        self.speed = speed

    def _disable(self, urgent):
        if not self._enabled and not urgent:
            return
        self._write_speed(0.0)
        deadline = time.monotonic() + BRUSH_STOP_TIMEOUT_SEC
        for motor_id in sorted(self._enabled):
            while not urgent:
                with self._condition:
                    if self._urgent:
                        break
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise RuntimeError(f"滚刷 {motor_id} 停稳确认超时")
                rpm = self._driver.read_brush_velocity(motor_id, min(remaining, 0.15))
                if rpm is not None and abs(rpm) <= 1.0:
                    break
                if rpm is None:
                    raise RuntimeError(f"滚刷 {motor_id} 转速回读失败")
                time.sleep(0.02)
        if not all([self._driver.motor_disable(motor_id) for motor_id in self._motor_ids]):
            raise RuntimeError("步科滚刷失能指令发送失败")
        if not urgent:
            for motor_id in self._motor_ids:
                if not self._driver.wait_motor_status(motor_id, 0x006F, 0x0021):
                    raise RuntimeError(f"滚刷 {motor_id} 失能未确认")
        self._enabled.clear()
