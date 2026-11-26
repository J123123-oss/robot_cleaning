# START RTK Fixed Gate Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans (recommended) or superpowers:subagent-driven-development to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Block START when the latest RTK sample is not dual Fixed and valid, while reusing RTK fault code 4.

**Architecture:** Cache WTRTK position status and `position_data_valid` alongside the existing orientation status in `MotorControlNode`, keeping the WTRTK snapshot separate from `/car_center_gps`. Add the START gate in `switch_state()` before the existing battery gate, while allowing the explicit pending magnetic `UNLOADING` path to retain its bypass. On rejection, set `ERROR_RTK_NOT_FIXED`, publish the current state, and return.

**Tech Stack:** Python, ROS2 messages, pytest, AST-based offline tests.

**Spec:** `docs/superpowers/specs/2026-11-26-start-rtk-fixed-gate-design.md`

## Global Constraints

- START requires WTRTK `position_status == 4`, `fix_status == 4`, and `position_data_valid == True`, except for an explicit pending magnetic `UNLOADING` request.
- Reuse `ERROR_RTK_NOT_FIXED = 4`.
- Preserve the existing battery threshold and START flow after the RTK gate passes.
- Do not change independent `UNLOADING` behavior.
- Update `FIX_SUMMARY.md` with offline and field verification status.

## Review Focus

- Position Fixed with orientation non-Fixed must be rejected and report code 4.
- Orientation Fixed with position non-Fixed must be rejected and report code 4.
- Both statuses Fixed with invalid position data must be rejected and report code 4.
- Both statuses Fixed with valid position data must reach the existing START path.
- A rejected START must preserve the current state and avoid starting the unloading process.
- A pending magnetic `UNLOADING` request must continue to enter the existing START initialization path.

### Task 1: Add failing START gate regression tests

**Files:**
- Create: `src/motor_control/test/test_start_rtk_fixed_gate.py`
- Test: `src/motor_control/test/test_start_rtk_fixed_gate.py`

**Interfaces:**
- Consumes: `MotorControlNode.wtrtk_callback`, `MotorControlNode.switch_state`, `ERROR_RTK_NOT_FIXED`.
- Produces: Regression tests that fail against the current battery-only START gate.

- [x] **Step 1: Write tests for the three rejection cases and the allowed case**

  Extract the production methods with AST as the existing `motor_control` tests do. Feed a fake WTRTK message through `wtrtk_callback`, invoke `switch_state("u")`, assert rejection preserves `current_status`, sets bit 4, and calls `publish_state`. Assert a valid dual-Fixed sample proceeds past the RTK gate.

- [x] **Step 2: Run the focused tests and verify the expected failure**

  Run `pytest -q src/motor_control/test/test_start_rtk_fixed_gate.py`.
  Expected result: the non-Fixed cases fail because START currently proceeds without the RTK gate.

### Task 2: Implement RTK state caching and START gate

**Files:**
- Modify: `src/motor_control/motor_control/motor_control.py:778-780`
- Modify: `src/motor_control/motor_control/motor_control.py:1059-1064`
- Modify: `src/motor_control/motor_control/motor_control.py:2869-2880`

**Interfaces:**
- Consumes: WTRTK fields `position_status`, `fix_status`, and `position_data_valid`.
- Produces: `MotorControlNode.rtk_position_data_valid` and START rejection behavior.

- [x] **Step 1: Store the latest position validity**

  Initialize `self.wtrtk_position_status` and `self.rtk_position_data_valid` to safe non-Fixed values, and update both from every WTRTK message in `wtrtk_callback`; leave `/car_center_gps` updates separate.

- [x] **Step 2: Add the START RTK gate**

  Before the existing low-battery check, unless `_magnetic_unloading_request_pending` is true, require WTRTK position and orientation statuses to equal `4` and `rtk_position_data_valid` to be true. On failure, set `self.rtk_error_code |= ERROR_RTK_NOT_FIXED`, log the rejection, call `publish_state()`, and return without changing `current_status`.

- [x] **Step 3: Run the focused tests and verify they pass**

  Run `pytest -q src/motor_control/test/test_start_rtk_fixed_gate.py`.
  Expected result: all new cases pass.

### Task 3: Record and broaden verification

**Files:**
- Modify: `FIX_SUMMARY.md`

- [x] **Step 1: Run related motor-control tests**

  Run `pytest -q src/motor_control/test`.
  Expected result: no regressions in the motor-control test suite.

- [x] **Step 2: Update the change summary**

  Add a dated entry describing the START RTK Fixed gate, reused error code 4, offline test result, and unperformed field verification.

- [x] **Step 3: Inspect the final diff**

  Run `git diff --check` and `git status --short`.
  Confirm only the intended files are modified and existing untracked route files remain untouched.
