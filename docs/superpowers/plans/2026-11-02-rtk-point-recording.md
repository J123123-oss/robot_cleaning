# WAIT_RTK Point Recording Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** 通过 MQTT `WAIT_RTK` 在 `rtk_points.md` 追加当前坐标、序号、有效性和时间。

**Architecture:** 在 `motor_control` 中复用现有 `robot_cmd` 回调；纯文件格式和序号逻辑放入小型无 ROS 辅助模块，节点只负责提取状态并调用它。

**Tech Stack:** Python、ROS2 rclpy、pytest、UTF-8 Markdown/YAML-compatible records。

**Spec:** `docs/superpowers/specs/2026-11-02-rtk-point-recording-design.md`

## Global Constraints

- `valid` 仅当定位与定向状态都为 4。
- 坐标使用车体中心 `current_lon/current_lat`。
- 文件固定默认在 `src/rtk_nav/rtk_nav/config/rtk_points.md`，路径可由 ROS 参数覆盖。

## Review Focus

- 原始字符串与 JSON 命令都必须触发一次记录：测试 `test_wait_rtk_command_forms`。
- 任一状态非 4 必须写 `valid: false`：测试 `test_record_point_marks_invalid_rtk_status`。
- 已有序号必须递增且空文件从 1 开始：测试 `test_next_sequence`。
- 非法坐标不能写入：测试 `test_invalid_coordinates_are_rejected`。

### Task 1: Pure point recorder

**Files:**
- Create: `src/motor_control/motor_control/rtk_point_recorder.py`
- Test: `src/motor_control/test/test_rtk_point_recorder.py`

- [ ] 先写上述失败测试，验证序号扫描、格式化和坐标校验。
- [ ] 实现 `record_rtk_point(path, lon, lat, position_status, orientation_status, timestamp=None) -> bool`，追加 YAML 兼容列表项并返回是否写入。
- [ ] 运行该测试文件并确认通过。

### Task 2: MotorControlNode command integration

**Files:**
- Modify: `src/motor_control/motor_control/motor_control.py`

- [ ] 增加 `rtk_points_file` 参数和记录器调用。
- [ ] 在 JSON 与原始字符串命令分支中识别 `WAIT_RTK`，使用现有坐标与 `self.rtk_position_status`、`self.rtk_orientation_status`，不改变运动状态。
- [ ] 运行点记录测试及静态语法检查。

### Task 3: Launch and document artifact

**Files:**
- Modify: `src/rtk_nav/launch/run.launch.py`
- Create: `src/rtk_nav/rtk_nav/config/rtk_points.md`

- [ ] 将默认路径传给 `motor_control`，创建带说明头的空记录文档。

### Task 4: Verification

- [ ] 运行 `pytest src/motor_control/test/test_rtk_point_recorder.py`。
- [ ] 运行 `python -m py_compile` 检查修改后的 Python 文件。
- [ ] 运行 `colcon test --packages-select motor_control`（若当前环境缺少 ROS 依赖，记录实际失败原因）。
