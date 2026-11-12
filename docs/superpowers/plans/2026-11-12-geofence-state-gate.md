# 电子围栏状态入口门控 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** 阻止缺少围栏副文件或已锁存围栏故障的机器人进入 `AUTO_CLEANING`，并统一发布 `status=HOLD`、`nav_status=PAUSE`。

**Architecture:** 在 `motor_control.switch_state()` 入口做路线围栏副文件和故障锁存预检，失败时直接保持或切换到底盘 `HOLD`。`rtk_nav` 保留运行时二次拦截，缺少副文件时在首点导航前停车；越界锁存后在 HOLD 期间通过 `/rtk/geofence_recovery_ready` 报告回到允许区域。导航状态回调只把 RTK 的 `PAUSE` 写入 `nav_status`，底盘 `HOLD` 由 `status` 表达。

**Tech Stack:** Python 3、ROS2 `std_msgs`、pytest、现有 RTK/电机状态话题。

**Spec:** `docs/superpowers/specs/2026-11-12-geofence-state-gate-design.md`

## Global Constraints

- 当前路线围栏文件必须为 `os.path.splitext(rtk_path_file)[0] + ".fence.json"`。
- 故障位固定为 `ERROR_GEOFENCE_OUTSIDE = 1024`。
- 缺少围栏时禁止首点导航。
- 人工介入暂停对外发布 `nav_status=PAUSE`，不能发布 `HOLD`。
- 不修改现场路径、YAML 和未相关文件。

## Review Focus

- 缺少 `.fence.json` 时，`AUTO_CLEANING` 入口不能短暂切换成功。
- 已锁存 1024 故障时，重复 AUTO 指令不能清除故障或切回 AUTO。
- 人工把定位移回围栏内侧后，下一次 AUTO 可以清除 1024 并恢复。
- 电子围栏 PAUSE 到达电机节点后，状态组合必须是 `status=HOLD`、`nav_status=PAUSE`。
- 直接绕过 MQTT、由 RTK 发布 AUTO 状态时，`rtk_nav` 仍需在首点前停车。
- 围栏文件存在且无故障时，原有 AUTO 入口行为保持不变。

### Task 1: Add failing state contract tests

**Files:**
- Modify: `src/rtk_nav/test/test_rtk_geofence_contract.py`
- Modify: `src/rtk_nav/test/test_nav_pause_protocol_contract.py`
- Modify: `src/motor_control/test/test_geofence_fault_contract.py`

- [ ] **Step 1: Write failing assertions**
  - Assert `rtk_nav.enforce_geofence()` checks `geofence_payload is None` before the initial-approach allowance.
  - Assert manual `PAUSE` handling assigns `self.nav_status = "PAUSE"` and does not assign `"HOLD"`.
  - Assert `switch_state()` contains an AUTO entry fence precheck before `self.current_status = new_state`.
  - Assert the precheck uses the same-stem `.fence.json` path and rejects `geofence_fault_latched`.

- [ ] **Step 2: Run the focused tests and confirm they fail for the missing behavior**

```bash
python -m pytest -q -p no:cacheprovider \
  src/rtk_nav/test/test_rtk_geofence_contract.py \
  src/rtk_nav/test/test_nav_pause_protocol_contract.py \
  src/motor_control/test/test_geofence_fault_contract.py
```

### Task 2: Enforce the AUTO entry gate in motor_control

**Files:**
- Modify: `src/motor_control/motor_control/motor_control.py`

**Interfaces:**
- Add a private helper that checks `self.geofence_fault_latched` and the current route's same-stem `.fence.json`.
- The helper returns `(allowed: bool, reason: str)` or an equivalent local contract.

- [ ] **Step 1: Implement the precheck before `current_status` changes**
  - For missing sidecar or latched geofence fault, set `nav_status = "PAUSE"`, latch the 1024 fault, stop wheel/brush commands, switch to `HOLD` if needed, publish state, and return.
  - Do not call the precheck for other states.

- [ ] **Step 2: Update route switching to keep `self.rtk_path_file` current**
  - When `handle_route_change()` resolves a route file, assign it before publishing the route change so the next AUTO precheck uses the correct sidecar.

- [ ] **Step 3: Add `_check_auto_cleaning_entry()` recovery gate**
  - Reject missing sidecars and latched faults without `/rtk/geofence_recovery_ready`.
  - Clear the local 1024 latch only after the RTK readiness signal.

- [ ] **Step 4: Run the focused tests and verify they pass**

### Task 3: Keep RTK runtime behavior aligned

**Files:**
- Modify: `src/rtk_nav/rtk_nav/rtk_nav.py`
- Modify: `src/motor_control/motor_control/motor_control.py`

- [ ] **Step 1: Restore missing-sidecar priority in `enforce_geofence()`**
  - Check `geofence_payload is None` before any first-waypoint allowance.
  - Call `trip_geofence_stop("sidecar_missing")` and return `True`.

- [ ] **Step 2: Map manual RTK PAUSE to `nav_status=PAUSE`**
  - Keep `switch_state("h")` for the bottom-level status.
  - Never write `"HOLD"` into `nav_status` from `rtk_nav_status_callback()`.

- [ ] **Step 3: Prevent repeated AUTO recovery logs**
  - Since the motor entry gate rejects AUTO while the geofence fault is latched, no `AUTO_CLEANING` state message should be published after a geofence stop.
  - Preserve RTK's secondary `state_callback()` guard for direct or legacy state publishers.

- [ ] **Step 4: Publish recovery readiness while HOLD**
  - Add `/rtk/geofence_recovery_ready`, reset it on a new route or new fence stop,
    and set it only when the latched robot is inside the loaded fence.

### Task 4: Verify and commit

**Files:**
- No additional production files.

- [ ] **Step 1: Run focused tests, syntax checks, and diff checks**

```bash
python -m pytest -q -p no:cacheprovider \
  src/rtk_nav/test/test_rtk_geofence_contract.py \
  src/rtk_nav/test/test_nav_pause_protocol_contract.py \
  src/motor_control/test/test_geofence_fault_contract.py
python -m py_compile \
  src/rtk_nav/rtk_nav/rtk_nav.py \
  src/motor_control/motor_control/motor_control.py
git diff --check
```

- [ ] **Step 2: Review status and ensure现场 `test.txt`、`hangzhou.yaml`、`test_1112.txt` are not staged**

- [ ] **Step 3: Commit**

```bash
git add src/rtk_nav/rtk_nav/rtk_nav.py \
  src/motor_control/motor_control/motor_control.py \
  src/rtk_nav/test/test_rtk_geofence_contract.py \
  src/rtk_nav/test/test_nav_pause_protocol_contract.py \
  src/motor_control/test/test_geofence_fault_contract.py \
  docs/superpowers/specs/2026-11-12-geofence-state-gate-design.md \
  docs/superpowers/plans/2026-11-12-geofence-state-gate.md
git commit -m "fix(state): gate auto cleaning on geofence readiness"
```
