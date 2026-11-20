"""与 ROS 无关的定位位移检查和新样本到点确认。"""

import math


class PositionJumpGuard:
    """在米制平面坐标上检查位移；跳变后冻结界限并等待可信位置恢复。"""

    def __init__(self, tolerance=0.30, max_interval=1.0, recovery_samples=3):
        self.tolerance = tolerance
        self.max_interval = max_interval
        self.recovery_samples = recovery_samples
        self.blocked = False
        self.last_seen_stamp = None
        self.anchor = None
        self.anchor_stamp = None
        self.allowed_distance = tolerance
        self.last_distance = 0.0
        self.recovery_count = 0
        self.blocked_anchor = None

    def discard(self, stamp_ns):
        """记录无效质量样本的时序并打断恢复确认，不修改可信位置或阻塞状态。"""
        if stamp_ns > 0 and (
            self.last_seen_stamp is None or stamp_ns > self.last_seen_stamp
        ):
            self.last_seen_stamp = stamp_ns
        self.recovery_count = 0

    def update(
        self, position, stamp_ns, active, reference=None, reference_tolerance=0.30,
        *, linear_velocity_mps=0.0,
    ):
        """只接受递增时间戳；自动模式中的异常坐标不更新可信锚点。"""
        if stamp_ns <= 0:
            if not self.blocked:
                self.blocked_anchor = self.anchor
                self.allowed_distance = self.tolerance
            self.blocked = active
            self.recovery_count = 0
            return False
        if self.last_seen_stamp is not None and stamp_ns <= self.last_seen_stamp:
            return False
        self.last_seen_stamp = stamp_ns
        if not active:
            self.blocked = False
        if reference is not None and not all(math.isfinite(value) for value in reference):
            reference = None
        check_anchor = (
            self.blocked_anchor if self.blocked
            else reference if reference is not None
            else self.anchor
        )
        if not self.blocked:
            if reference is not None:
                self.allowed_distance = reference_tolerance
            elif self.anchor_stamp is not None and math.isfinite(linear_velocity_mps):
                elapsed = min((stamp_ns - self.anchor_stamp) / 1e9, self.max_interval)
                self.allowed_distance = abs(linear_velocity_mps) * elapsed + self.tolerance
            else:
                self.allowed_distance = self.tolerance
        if position is None or not all(math.isfinite(value) for value in position):
            if not self.blocked:
                self.blocked_anchor = check_anchor
            self.blocked = active
            self.recovery_count = 0
            return False
        if self.blocked and check_anchor is None:
            self.blocked_anchor = check_anchor = position
            self.allowed_distance = self.tolerance
        if check_anchor is not None and active:
            self.last_distance = math.dist(position, check_anchor)
            if self.last_distance > self.allowed_distance:
                self.blocked_anchor = check_anchor
                self.blocked = True
                self.recovery_count = 0
                return False
        if self.blocked:
            self.recovery_count += 1
            if self.recovery_count < self.recovery_samples:
                return False
        self.anchor = position
        self.anchor_stamp = stamp_ns
        self.blocked = False
        self.blocked_anchor = None
        self.recovery_count = 0
        return True


class ArrivalConfirmation:
    """按目标和新定位序号累计连续到点样本，重复帧不增加计数。"""

    def __init__(self, required_samples=3):
        self.required_samples = required_samples
        self.target = None
        self.last_sequence = None
        self.count = 0

    def reset(self):
        """清除连续性，保留已消费的样本序号，防止同一帧被重新计数。"""
        self.target = None
        self.count = 0

    def update(self, target, sequence, distance, tolerance=0.1, valid=True):
        """仅新的有效范围内样本累计；越界、无效或换目标立即清除旧计数。"""
        if target != self.target:
            self.target = target
            self.count = 0
        if not valid or not math.isfinite(distance) or distance < 0.0:
            self.last_sequence = sequence
            self.count = 0
            return False
        if self.last_sequence is not None and sequence <= self.last_sequence:
            return self.count >= self.required_samples
        self.last_sequence = sequence
        if distance < tolerance:
            self.count += 1
        else:
            self.count = 0
        return self.count >= self.required_samples
