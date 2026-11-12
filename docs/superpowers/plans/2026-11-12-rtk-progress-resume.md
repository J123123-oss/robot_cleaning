# RTK Progress Resume Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Persist RTK cleaning progress and restore it only after an explicit MQTT `RESUME` command.

**Architecture:** Add a pure JSON progress store under `rtk_nav`, write snapshots from the navigation node at route start and waypoint transitions, and expose a `/rtk/resume_progress` request topic. `motor_control` parses `RESUME` and publishes that request without changing motor state.

**Tech Stack:** Python 3, ROS2 `std_msgs/String`, JSON, pytest.

**Spec:** `docs/superpowers/specs/2026-11-12-rtk-progress-resume-design.md`

## Global Constraints

- Default startup must remain on the configured route at waypoint index 0.
- `RESUME` must not start motors; operator still sends `ENABLE` and `AUTO_CLEANING`.
- Existing geofence entry gate remains authoritative.
- Corrupt or incompatible snapshots must fail closed to default startup.

## Review Focus

- A corrupt snapshot must not crash node startup.
- A snapshot for a missing route or missing fence must not be applied.
- A waypoint index outside the loaded route must not be applied.
- A RESUME command during AUTO or bin motion must not alter navigation.
- Completing a task must remove stale progress.

### Task 1: Progress store and contract tests

**Files:**
- Create: `src/rtk_nav/rtk_nav/progress_store.py`
- Test: `src/rtk_nav/test/test_progress_resume_contract.py`

- [ ] Write failing tests for atomic save/load, malformed snapshots, and required fields.
- [ ] Run the focused tests and verify they fail because the store is absent.
- [ ] Implement `save_progress(path, payload)` and `load_progress(path)` with validation and atomic replacement.
- [ ] Run focused tests and verify they pass.

### Task 2: RTK navigation persistence and restore

**Files:**
- Modify: `src/rtk_nav/rtk_nav/rtk_nav.py`
- Test: `src/rtk_nav/test/test_progress_resume_contract.py`

- [ ] Add failing AST/behavior tests for snapshot fields, waypoint transition writes, `/rtk/resume_progress`, and safe restore.
- [ ] Run focused tests and verify the integration assertions fail.
- [ ] Add snapshot path initialization, persistence hooks, resume subscriber, route/index validation, and completion cleanup.
- [ ] Run focused tests and verify they pass.

### Task 3: MQTT RESUME bridge

**Files:**
- Modify: `src/motor_control/motor_control/motor_control.py`
- Test: `src/motor_control/test/test_progress_resume_contract.py`

- [ ] Write a failing test that `RESUME` publishes the ROS request and does not call `switch_state`.
- [ ] Run the focused test and verify it fails.
- [ ] Add the publisher and command branch, preserving the existing state gate behavior.
- [ ] Run focused tests and the full package tests.

### Task 4: Verification and commit

- [ ] Run `pytest src/rtk_nav/test src/motor_control/test`.
- [ ] Run `python -m py_compile` for both changed nodes.
- [ ] Review the diff and commit only the progress-resume files.
