#!/usr/bin/env python3
"""Run one randomized dock alignment check through ROS2.

Sequence: UNLOADING -> wait HOLD -> random LEFT/RIGHT for 2s -> HOLD ->
LOADING -> capture the magnetic masks immediately before the first 6-bit
end-marker. The report is JSON and is suitable for comparing runs.
"""
from __future__ import annotations

import argparse
import json
import random
import time
from dataclasses import dataclass, asdict
from datetime import datetime
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, UInt16


WEIGHTS = {1: -3.0, 2: -1.0, 4: -0.5, 8: -0.2,
           16: 0.2, 32: 0.5, 64: 1.0, 128: 3.0}


def magnetic_error(mask: int) -> Optional[float]:
    values = [weight for bit, weight in WEIGHTS.items() if mask & bit]
    return sum(values) / len(values) if values else None


@dataclass
class MaskSample:
    time: float
    mask: int
    error: Optional[float]
    bit_count: int


class DockAlignmentCheck(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("dock_alignment_check")
        self.args = args
        self.command_pub = self.create_publisher(String, "robot_cmd", 10)
        self.create_subscription(String, "robot_state", self._state_cb, 20)
        self.create_subscription(UInt16, "/magnetic/mask", self._mask_cb, 50)
        self.status: Optional[str] = None
        self.error_code: Optional[int] = None
        self.last_mask = 0
        self.samples: list[MaskSample] = []
        self.phase = "INIT"

    def _state_cb(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            self.status = payload.get("status")
            self.error_code = payload.get("error")
            self.get_logger().info(f"当前机器人状态: {self.status}")
        except json.JSONDecodeError:
            self.get_logger().warning(f"忽略非JSON状态: {msg.data[:160]}")

    def _mask_cb(self, msg: UInt16) -> None:
        self.last_mask = int(msg.data) & 0xFF
        if self.phase == "LOADING":
            sample = MaskSample(
                time=time.time(),
                mask=self.last_mask,
                error=magnetic_error(self.last_mask),
                bit_count=self.last_mask.bit_count(),
            )
            self.samples.append(sample)
            self.samples = self.samples[-self.args.window:]

    def command(self, value: str) -> None:
        msg = String()
        # motor_control 的 MQTT/ROS 指令入口按 JSON 解析 command 字段；
        # 发送裸的 UNLOADING/LOADING 会被当成无效字符串拒绝。
        msg.data = json.dumps({"command": value}, ensure_ascii=False)
        self.command_pub.publish(msg)
        self.get_logger().info(f"发送 {value}")

    def wait_for_command_connection(self, timeout: float) -> bool:
        """Wait until motor_control is subscribed before publishing commands."""
        end = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < end:
            if self.command_pub.get_subscription_count() > 0:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def spin_for(self, duration: float) -> None:
        end = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_status(self, expected: str, timeout: float) -> bool:
        end = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.status == expected:
                return True
        return False

    def wait_status_transition(
        self, active: str, terminal: str, timeout: float
    ) -> bool:
        """Wait for an active state first, then its terminal state."""
        end = time.monotonic() + timeout
        active_seen = False
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.status == active:
                active_seen = True
            elif active_seen and self.status == terminal:
                return True
        return False

    def wait_for_state_message(self, timeout: float) -> bool:
        """Ensure ROS discovery and the robot state subscription are active."""
        end = time.monotonic() + timeout
        next_request = 0.0
        while rclpy.ok() and time.monotonic() < end:
            now = time.monotonic()
            if now >= next_request:
                self.command("GET_STATUS")
                next_request = now + 1.0
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.status is not None:
                return True
        return False

    def run(self) -> dict:
        direction = random.choice(("OFFSET_LEFT", "OFFSET_RIGHT"))
        started = datetime.now().isoformat(timespec="seconds")
        # DISABLE/HOLD 状态下状态发布周期可能很长，先主动请求一次状态，
        # 否则仅等待 /robot_state 会在 10 秒内拿不到任何消息。
        if not self.wait_for_command_connection(self.args.command_timeout):
            raise RuntimeError(
                f"{self.args.command_timeout}s内/robot_cmd没有订阅者；"
                "请确认motor_control节点已启动且ROS_DOMAIN_ID一致"
            )
        self.command("GET_STATUS")
        if not self.wait_for_state_message(self.args.state_timeout):
            raise RuntimeError(
                f"{self.args.state_timeout}s内未收到/robot_state；"
                "请确认脚本与motor_control处于相同ROS_DOMAIN_ID和ROS网络"
            )
        if self.status not in ("HOLD", "ENABLE", "DISABLE"):
            raise RuntimeError(
                f"当前状态为{self.status}，无法开始检测；脚本未下发运动指令"
            )
        # 每轮开始都显式使能。LOADING 完成后节点会切回 DISABLE，
        # 连续多轮测试必须重新发送 ENABLE 才能再次执行 UNLOADING。
        self.command("ENABLE")
        if not self.wait_status("ENABLE", self.args.enable_timeout):
            raise RuntimeError(
                f"ENABLE未在{self.args.enable_timeout}s内生效，当前={self.status}"
            )
        self.phase = "UNLOADING"
        self.command("UNLOADING")
        if not self.wait_status_transition(
            "UNLOADING", "HOLD", self.args.unloading_timeout
        ):
            raise RuntimeError(f"UNLOADING未在{self.args.unloading_timeout}s内进入HOLD，当前={self.status}")

        self.phase = "OFFSET"
        self.command(direction)
        if not self.wait_status(direction, self.args.offset_timeout):
            raise RuntimeError(f"{direction}未在{self.args.offset_timeout}s内生效，当前={self.status}")
        # OFFSET 由motor_control内部完成SHIFT→RECENTER并自动进入HOLD。
        # 不能按固定 offset_seconds 提前发送HOLD，否则会在回正阶段开始前
        # 打断动作，导致测试车体几乎原地不动。
        if not self.wait_status("HOLD", self.args.offset_complete_timeout):
            self.command("HOLD")
            raise RuntimeError(
                f"{direction}偏移流程未在{self.args.offset_complete_timeout}s内完成，"
                f"当前={self.status}"
            )
        self.spin_for(self.args.settle_seconds)

        self.phase = "LOADING"
        self.samples.clear()
        self.command("LOADING")
        if not self.wait_status("LOADING", self.args.loading_start_timeout):
            raise RuntimeError(
                f"LOADING未在{self.args.loading_start_timeout}s内生效，当前={self.status}"
            )
        deadline = time.monotonic() + self.args.loading_timeout
        end_sample: Optional[MaskSample] = None
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.samples and self.samples[-1].bit_count >= self.args.end_bits:
                end_sample = self.samples[-1]
                break
            if self.status == "HOLD" and len(self.samples) > 3:
                break

        self.command("HOLD")
        self.wait_status("DISABLE", self.args.disable_timeout)
        before = self.samples[:-1] if end_sample else self.samples
        result = self._classify(before[-self.args.window:])
        return {
            "started": started,
            "offset_direction": direction,
            "offset_seconds": self.args.offset_seconds,
            "end_marker": asdict(end_sample) if end_sample else None,
            "samples_before_end_marker": [asdict(s) for s in before[-self.args.window:]],
            "assessment": result,
            "final_status": self.status,
        }

    @staticmethod
    def _classify(samples: list[MaskSample]) -> dict:
        if not samples:
            return {"result": "NO_DATA", "reason": "进仓结束前没有收到磁导航状态"}
        errors = [s.error for s in samples if s.error is not None]
        avg_error = sum(errors) / len(errors) if errors else None
        edge_hits = sum(1 for s in samples if s.mask & 0xC3)
        center_hits = sum(1 for s in samples if (s.mask & 0x3C) and not (s.mask & 0xC3))
        if avg_error is None:
            result = "NO_MAGNETIC_SIGNAL"
        elif abs(avg_error) <= 0.4 and edge_hits <= max(1, len(samples) // 3):
            result = "LIKELY_CENTERED"
        elif abs(avg_error) <= 0.8:
            result = "NEAR_CENTER_NEEDS_REVIEW"
        else:
            result = "OFF_CENTER"
        return {
            "result": result,
            "average_error": avg_error,
            "edge_hit_samples": edge_hits,
            "center_hit_samples": center_hits,
            "sample_count": len(samples),
        }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--offset-seconds", type=float, default=4.0)
    parser.add_argument("--offset-timeout", type=float, default=5.0)
    parser.add_argument("--offset-complete-timeout", type=float, default=12.0,
                        help="等待OFFSET内部SHIFT/RECENTER自动回到HOLD的超时时间")
    parser.add_argument("--settle-seconds", type=float, default=1.0)
    parser.add_argument("--unloading-timeout", type=float, default=45.0)
    parser.add_argument("--loading-timeout", type=float, default=60.0)
    parser.add_argument("--loading-start-timeout", type=float, default=10.0)
    parser.add_argument("--state-timeout", type=float, default=10.0,
                        help="等待机器人初始状态的超时时间")
    parser.add_argument("--command-timeout", type=float, default=10.0,
                        help="等待/robot_cmd订阅者建立的超时时间")
    parser.add_argument("--enable-timeout", type=float, default=10.0,
                        help="等待电机进入ENABLE的超时时间")
    parser.add_argument("--disable-timeout", type=float, default=15.0,
                        help="等待进仓完成后回到DISABLE的超时时间")
    parser.add_argument("--end-bits", type=int, default=6)
    parser.add_argument("--window", type=int, default=20)
    parser.add_argument("--runs", type=int, default=1,
                        help="连续测试次数，每轮都会重新随机选择左右偏移方向")
    parser.add_argument("--output", type=Path, default=Path("dock_alignment_report.json"))
    args = parser.parse_args()

    rclpy.init()
    node = DockAlignmentCheck(args)
    try:
        reports = []
        for run_index in range(max(1, args.runs)):
            node.get_logger().info(
                f"开始第 {run_index + 1}/{max(1, args.runs)} 轮检测"
            )
            report = node.run()
            report["run_index"] = run_index + 1
            reports.append(report)
            if run_index + 1 < max(1, args.runs):
                node.spin_for(args.settle_seconds)
        output = reports[0] if len(reports) == 1 else {
            "run_count": len(reports),
            "runs": reports,
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(output, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        print(json.dumps(output, ensure_ascii=False, indent=2))
        return 0
    except KeyboardInterrupt:
        if rclpy.ok():
            node.command("HOLD")
        return 130
    except Exception as exc:
        node.get_logger().error(str(exc))
        if rclpy.ok():
            node.command("HOLD")
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())




