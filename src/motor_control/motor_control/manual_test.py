#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
手动测试脚本：前进 + 滚刷 20s 开 / 20s 关 循环

用法：
  source install/setup.bash
  ros2 run motor_control manual_test

安全：
  - 运行前确保机器人处于安全空旷区域
  - 任意时刻 Ctrl+C 会立即停车并切回 NORMAL 模式
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import String

# ======================== 可调参数 ========================
SPEED_FORWARD = 8.0     # 前进速度 (motor_control 单位，MAX=10)
BRUSH_SPEED = 18.0      # 滚刷速度
DURATION_EACH = 20.0    # 每段时间 (秒)
PUBLISH_RATE = 10.0     # 发布频率 (Hz)
# ==========================================================


class ManualTestNode(Node):
    def __init__(self):
        super().__init__("manual_test")

        self.speed_pub = self.create_publisher(Vector3, "/rtk/motor_speed", 10)
        self.cmd_pub = self.create_publisher(String, "/keyboard/control", 10)

        self.phase_labels = ["滚刷开", "滚刷关"]
        self.cycle_duration = DURATION_EACH * 2

        self.cycle_count = 0
        self.last_logged_phase = -1

        self.get_logger().info(
            "=" * 60 + "\n"
            "  手动测试即将开始（循环模式）\n"
            f"  序列: 滚刷开({BRUSH_SPEED}) {DURATION_EACH}s → "
            f"滚刷关 {DURATION_EACH}s\n"
            f"  全程前进，速度={SPEED_FORWARD}\n"
            "  按 Ctrl+C 紧急停止"
            + "\n" + "=" * 60
        )

        self.countdown_remaining = 3
        self.timer = self.create_timer(1.0, self.countdown_tick)

    def countdown_tick(self):
        if self.countdown_remaining > 0:
            self.get_logger().info(f"  倒计时 {self.countdown_remaining} ...")
            self.countdown_remaining -= 1
        else:
            self.get_logger().info("  开始循环!")
            self.destroy_timer(self.timer)
            self.start_test()

    def start_test(self):
        self.cmd_pub.publish(String(data="r"))
        self.get_logger().info("已切换到 AUTO_CLEANING 模式")
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / PUBLISH_RATE, self.control_loop)

    def control_loop(self):
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9

        phase_pos = elapsed % self.cycle_duration
        phase_idx = int(phase_pos / DURATION_EACH)  # 0=滚刷开, 1=滚刷关
        phase_elapsed = phase_pos - phase_idx * DURATION_EACH
        remaining = DURATION_EACH - phase_elapsed

        brush_on = (phase_idx == 0)

        msg = Vector3()
        msg.x = -SPEED_FORWARD
        msg.y = SPEED_FORWARD
        msg.z = float(BRUSH_SPEED if brush_on else 0.0)
        self.speed_pub.publish(msg)

        if phase_idx != self.last_logged_phase:
            self.last_logged_phase = phase_idx
            if phase_idx == 0:
                self.cycle_count += 1
                self.get_logger().info(f"═══ 第 {self.cycle_count} 周期 ═══")
            self.get_logger().info(
                f"──▶ [{self.phase_labels[phase_idx]}]"
            )

        sec = int(phase_elapsed)
        prev_sec = int((phase_elapsed - 1.0 / PUBLISH_RATE))
        if sec != prev_sec:
            self.get_logger().info(
                f"  [{self.phase_labels[phase_idx]}] 剩余 {remaining:.0f}s  |  "
                f"左轮={msg.x:+.1f}  右轮={msg.y:+.1f}  刷={msg.z:.0f}"
            )

    def destroy_node(self):
        stop = Vector3()
        self.speed_pub.publish(stop)
        self.cmd_pub.publish(String(data="h"))
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ManualTestNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        node.get_logger().info("收到中断信号，安全停车...")
        stop = Vector3()
        node.speed_pub.publish(stop)
        node.cmd_pub.publish(String(data="h"))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
