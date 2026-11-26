# 低电量自动充电恢复实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** 让低于 90% 且实际充电电流不超过 1.0A 的停机机器人可靠重新发起充电请求。

**Architecture:** 保留服务成功即进入 `is_charging` 的既有语义，只在 `DISABLE` 状态收到低于或等于 1.0A 的电流时同步断充标志。恢复请求继续使用现有磁导航门控和请求节流。

**Tech Stack:** Python、ROS2、pytest。

**Spec:** `docs/superpowers/specs/2026-11-26-low-battery-charge-recovery-design.md`

## Global Constraints

- `DISABLE` 状态下 `charging_i <= 1.0A` 统一视为断充。
- 充电请求统一经过 `_request_charge_if_docked()`。
- `DISABLE`、新鲜原始磁导航掩码和至少 6 位触发仍是必要条件。
- 不新增低电流看门狗。

## Review Focus

- `DISABLE` 状态收到 `0.04A` 时，下一次恢复检查仍能请求充电。
- 服务成功仍保持 `is_charging=True`，不改变原有服务应答语义。
- 充电服务失败后，下一次重试遵守现有请求节流。
- 现有磁导航掩码过期或不足 6 位时，低电量仍不能启动充电。

### Task 1: Add low-current recovery regression tests

**Files:**
- Modify: `src/motor_control/test/test_dock_charging_gate.py`
- Test: `src/motor_control/test/test_dock_charging_gate.py`

**Interfaces:**
- Consumes: `charge_resume_callback`, `_start_charge_callback`, existing dock-gate fixture.
- Produces: regression coverage for the confirmed `0.04A` behavior.

- [ ] **Step 1: Write the failing tests**

  Add a test that sends `charging_i=0.04` while `current_status="DISABLE"` and asserts `is_charging=False`, `is_charge_paused=True`, and a later recovery request. Add a compatibility assertion that a successful service response still sets `is_charging=True`.

- [ ] **Step 2: Run the focused tests**

  Run `pytest -q src/motor_control/test/test_dock_charging_gate.py`.

  Expected: the new low-current test fails because `charging_volt_curr_cb()` currently does not restore the paused state after a `DISABLE`-state disconnect.

### Task 2: Fix charge-state confirmation and retry gating

**Files:**
- Modify: `src/motor_control/motor_control/motor_control.py:2548-2558`

**Interfaces:**
- Consumes: `charging_i`, `charge_resume_threshold`, `_request_charge_if_docked()`.
- Produces: `is_charging=False` and `is_charge_paused=True` after a `DISABLE`-state current sample at or below `1.0A`, allowing the existing recovery callback to request charging.

- [ ] **Step 1: Implement the minimum state fix**

  In `charging_volt_curr_cb()`, when `current_status` is `DISABLE` and `charging_i <= 1.0`, set `is_charging=False` and `is_charge_paused=True`. Leave `_start_charge_callback()` unchanged so a successful service response still sets `is_charging=True`.

- [ ] **Step 2: Run focused tests**

  Run `pytest -q src/motor_control/test/test_dock_charging_gate.py`.

  Expected: all dock-charging tests pass.

### Task 3: Update change summary and run package validation

**Files:**
- Modify: `FIX_SUMMARY.md`

- [ ] **Step 1: Record the change**

  Add a dated entry for the low-battery recovery fix, including the `0.04A` interpretation and offline validation status.

- [ ] **Step 2: Run the motor-control test suite**

  Run `pytest -q src/motor_control/test`.

  Expected: the package tests pass without new failures.

- [ ] **Step 3: Review the final diff**

  Run `git diff --check` and inspect `git diff --stat` plus the changed files.
