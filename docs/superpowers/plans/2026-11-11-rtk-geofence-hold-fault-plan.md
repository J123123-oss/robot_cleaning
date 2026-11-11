# RTK 电子围栏 HOLD 与故障上报实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** 连续 3 帧电子围栏越界后让机器人进入 `HOLD`，并通过现有 `/robot_state` MQTT 链路上报故障位 `1024`。

**Architecture:** `rtk_nav.py` 在现有越界锁存点发布 `/keyboard/control=HOLD` 和 `/rtk/geofence_fault_code=1024`；`motor_control.py` 订阅故障码、锁存故障位并复用现有 HOLD 状态机；MQTT 桥接无需改变主题或协议。

**Tech Stack:** ROS2 Python、`std_msgs/String`、`std_msgs/Int16`、现有 JSON `/robot_state`。

**Spec:** `docs/superpowers/specs/2026-11-11-rtk-geofence-hold-fault-design.md`

## Global Constraints

- `ERROR_GEOFENCE_OUTSIDE = 1024`，不得使用 `129`。
- 连续 3 帧越界才触发；未达到 3 帧只停车当前导航输出，不锁存故障。
- 电子围栏故障保持锁存，不能因自动重新进入区域而恢复运动。
- MQTT 继续使用现有 `/robot_state`，不新增 MQTT 主题。

## Review Focus

- 重复越界回调不能重复发布 HOLD 或反复累加故障：测试锁存单次触发。
- 越界后 HOLD 必须真正经过现有控制状态机：测试 `/keyboard/control` 指令。
- 故障位必须与已有位掩码组合：测试 `1024 | 512`。
- 人工复位不能清除电子围栏故障：测试故障仍在状态 JSON 中。

### Task 1: RTK 越界事件发布

**Files:**
- Modify: `src/rtk_nav/rtk_nav/rtk_nav.py`
- Test: `src/rtk_nav/test/test_rtk_geofence_contract.py`

- [ ] 添加 `ERROR_GEOFENCE_OUTSIDE = 1024` 和 `Int16` 故障发布器。
- [ ] 在 `trip_geofence_stop()` 首次锁存时发布 `HOLD` 和故障码；重复调用只保持停车。
- [ ] 添加源码契约测试，固定话题、故障码和单次触发逻辑。

### Task 2: 电机控制接收与 MQTT 状态

**Files:**
- Modify: `src/motor_control/motor_control/motor_control.py`
- Test: `src/motor_control/test/test_geofence_fault_contract.py`

- [ ] 添加同值 `ERROR_GEOFENCE_OUTSIDE = 1024`。
- [ ] 订阅 `/rtk/geofence_fault_code`，收到非零故障后锁存位并调用现有 `switch_state("HOLD")`。
- [ ] 将故障位并入 `build_error_code()`，由现有 `publish_state()` 的 `error` 字段上报 MQTT。
- [ ] 测试故障位组合、HOLD 调用和故障锁存。

### Task 3: Verification and commit

- [ ] 运行 RTK 与 motor_control 定向测试、`py_compile`。
- [ ] 检查 `git diff --check`。
- [ ] 提交 Conventional Commit。
