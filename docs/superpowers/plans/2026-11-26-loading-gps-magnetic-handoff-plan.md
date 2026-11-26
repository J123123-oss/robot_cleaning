# 进仓寻 GPS 磁条接管 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在进仓 GPS 寻点和微调阶段检测到连续新鲜磁条后，可靠切换到现有磁导航进仓流程。

**Architecture:** `magnetic_mask_callback()` 维护 GPS 阶段的连续磁条采样计数，`handle_loading_step()` 通过一个小型状态转移辅助方法消费该计数。接管只负责停车和初始化 `LOADING_DIRECT_BACKWARD` 所需状态，后续沿用现有磁导航逻辑。

**Tech Stack:** Python、ROS2 定时器回调、pytest、AST 离线单元测试。

**Spec:** `docs/superpowers/specs/2026-11-26-loading-gps-magnetic-handoff-design.md`

## Global Constraints

- 只在 `LOADING_NAV_TO_GPS`、`LOADING_TURN` 和 `LOADING_POS_RECOVER` 阶段累积磁条接管计数。
- 连续两帧非零原始磁条信号才允许接管，零值采样清除计数。
- 接管后复用 `LOADING_DIRECT_BACKWARD` 的既有磁导航和末端横向磁条完成逻辑。
- 不调整磁导航速度、末端完成阈值、充电流程和 GPS 超时策略。
- 保留工作区中与本任务无关的已有改动，不混入提交。

## Review Focus

- 单帧非零信号不得提前结束 GPS 阶段。
- 非零信号被零值打断后必须重新累计。
- 接管必须停车并初始化磁导航阶段，不能只修改字符串状态。
- 接管后末端磁条完成判定仍保持原有连续采样要求。
- 其他进仓阶段不能被该计数逻辑重复触发。

---

### Task 1: Pin the GPS-to-magnetic handoff behavior

**Files:**
- Create: `src/motor_control/test/test_loading_gps_magnetic_handoff.py`
- Modify: `src/motor_control/motor_control/motor_control.py`

**Interfaces:**
- Consumes: `magnetic_mask_callback()` raw mask updates and GPS loading phase state.
- Produces: `_handoff_to_magnetic_loading_if_ready(current_time) -> bool`.

- [x] **Step 1: Write the failing test**

  Add tests for two consecutive non-zero samples causing handoff, one sample not causing handoff, and a zero sample resetting the sequence. Assert `loading_phase`, zero-speed output, and magnetic stage initialization.

- [x] **Step 2: Run the focused test to verify it fails**

  Run `python -m pytest -q -p no:cacheprovider src/motor_control/test/test_loading_gps_magnetic_handoff.py`.

  Expected result is failure because the handoff helper and counter state do not exist yet.

- [x] **Step 3: Implement the minimal state transition**

  Add a two-sample counter initialized on LOADING entry. Update it only for the GPS-related phases in `magnetic_mask_callback()`. Implement `_handoff_to_magnetic_loading_if_ready()` and call it before GPS movement logic in `handle_loading_step()`. Initialize the same search and direct-loading timing fields used by the normal transition.

- [x] **Step 4: Run the focused test to verify it passes**

  Run the focused test again and expect all cases to pass.

- [x] **Step 5: Run related regression tests**

  Run `python -m pytest -q -p no:cacheprovider src/motor_control/test/test_loading_gps_magnetic_handoff.py src/motor_control/test/test_dock_charging_gate.py src/motor_control/test/test_magnetic_navigation.py`.

### Task 2: Document and commit

**Files:**
- Modify: `FIX_SUMMARY.md`

- [x] **Step 1: Record the actual validation result**

  Add the GPS-to-magnetic handoff behavior and the exact focused test result.

- [ ] **Step 2: Check the patch boundary**

  Run `git diff --check` and inspect staged paths to ensure unrelated existing changes remain unstaged.

- [ ] **Step 3: Commit the focused change**

  ```bash
  git add docs/superpowers/specs/2026-11-26-loading-gps-magnetic-handoff-design.md docs/superpowers/plans/2026-11-26-loading-gps-magnetic-handoff-plan.md src/motor_control/motor_control/motor_control.py src/motor_control/test/test_loading_gps_magnetic_handoff.py FIX_SUMMARY.md
  git commit -m "fix: hand off loading GPS alignment to magnetic navigation"
  ```
