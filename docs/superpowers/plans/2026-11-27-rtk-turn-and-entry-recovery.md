# RTK Turn And Entry Recovery Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Correct turn timeout, turn-time boundary interruption, turn slip/displacement, and reverse first-waypoint entry failures observed on 2026-11-27, while retaining fail-safe stops.

**Architecture:** Keep the work in `rtk_nav` and its existing recovery/state-machine boundaries. Give every turn an explicit target, direction, progress evidence, and interrupt result; do not let sensor-blocked turns masquerade as completed or ordinary heading failures. For initial reverse entry, use a single motion-direction convention from target selection through Stanley steering, heading checks, recovery, and wheel output.

**Tech Stack:** ROS 2 Jazzy, Python, pytest, existing `RTKNavControlNode` state machine and navigation safety helpers.

**Spec:** Incident evidence in remote `motor_start_log/run20261127.2.log`, `run20261127.3.log`, and the user's pasted 15:50–15:58 field note; project constraints in `AGENTS.md`.

## Global Constraints

- Do not weaken boundary IO protections or continue motion through a blocked turn.
- Before a retreat, direction change, or turn, confirm stopped using fresh post-stop left/right wheel feedback.
- Preserve trusted RTK position and existing position-jump recovery rules.
- A manual-intervention pause is cleared only after leaving AUTO and explicit re-entry; do not auto-resume it on sensor release alone.
- Add the dated change, actual behavior, and offline/field verification separately to `FIX_SUMMARY.md` when implementation is completed.
- Do not deploy to the robot or publish remotely without explicit authorization.

## Review Focus

- A boundary sensor triggers during rotation and later releases: stale calibration output must not restart motion before the event is resolved; cover raw-trigger and confirmed-trigger paths.
- A turn makes no measurable progress but never reaches the nominal timeout: timeout must stop and publish a clear cause rather than leave a silent pause or endless attempt.
- A turn reports angular completion while chassis position drifts: distinguish expected RTK noise from displacement beyond a field-calibrated limit; verify no false slip alarm on a stationary turn.
- Reverse first-entry is selected: path bearing, chassis heading, heading-error checks, recovery target, and wheel signs must all describe the same physical motion.
- Stale, duplicated, invalid, or quality-degraded RTK/IMU/wheel samples occur during a turn: they must not count as progress, turn completion, or arrival.

---

### Task 1: Reconstruct and pin incident behaviors

**Files:**
- Inspect: `src/rtk_nav/test/test_boundary_calibration_contract.py`
- Inspect: `src/rtk_nav/test/test_waypoint_tracking_recovery.py`
- Inspect: `src/rtk_nav/test/test_navigation_position_safety.py`
- Inspect: `src/rtk_nav/test/test_geofence_recovery_heading.py`
- Modify: add focused cases to the existing relevant test modules above

**Interfaces:**
- Consumes: current navigation turn, boundary sensor, position-safety, and initial-entry behaviors.
- Produces: regression cases for the four incident classes before implementation changes.

- [ ] **Step 1: Add sensor-interrupted turn regression cases**
  Assert a raw matching trigger forces zero speed immediately, a confirmed trigger does not report calibration success, sensor release alone does not clear a manual lock, and explicit AUTO re-entry still requires fresh stopped feedback before moving.
- [ ] **Step 2: Add turn-stall and turn-displacement regression cases**
  Assert no angular progress reaches a bounded failure/pause with a distinguishable cause; assert a turn with excessive trusted position displacement is not accepted as successful, while bounded stationary RTK noise is.
- [ ] **Step 3: Add reverse-entry regression case**
  Use a target roughly opposite the current chassis heading and assert reverse selection yields near-zero motion-heading error, reverse-translated path geometry, and wheel commands that move toward the target.
- [ ] **Step 4: Run focused tests and verify each fails for the incident behavior**
  Run the relevant pytest nodes; record the exact failing assertions before production edits.

### Task 2: Make turn progress and timeout explicit

**Files:**
- Modify: `src/rtk_nav/rtk_nav/rtk_nav.py` (`calibrate_heading_at_waypoint`, calibration failure/pause handling)
- Test: focused calibration cases in `src/rtk_nav/test/test_boundary_calibration_contract.py` or the nearest existing calibration test module

**Interfaces:**
- Consumes: fresh IMU yaw, target heading, calibration origin, and current retry state.
- Produces: a bounded calibration result with explicit outcomes for success, no-progress timeout, sensor interruption, and invalid/stale heading input; callers must not conflate these outcomes.

- [ ] **Step 1: Specify progress from fresh heading samples**
  Test that only fresh, valid heading updates can advance completion or reset no-progress tracking; repeated/stale values cannot extend a stalled turn indefinitely.
- [ ] **Step 2: Separate total timeout from no-progress detection**
  Test both a slow but converging turn and a stalled turn; retain a bounded overall timeout and fail safely when progress is absent, without silently returning success above tolerance.
- [ ] **Step 3: Propagate the concrete calibration outcome to callers**
  Ensure initial-entry, waypoint-calibration, and retry paths preserve whether failure came from blockage, invalid data, no progress, or total timeout.
- [ ] **Step 4: Publish an operator-visible pause reason and fault indication**
  Ensure state/MQTT reporting distinguishes calibration timeout from generic `PAUSE`; retain sensor and motor diagnostics and do not mislabel a healthy motor as a motor fault.
- [ ] **Step 5: Run focused calibration tests**
  Verify success, bounded slow convergence, stale-input stall, no-progress timeout, retry exhaustion, and pause-state reporting.

### Task 3: Keep boundary interruption latched through turn recovery

**Files:**
- Modify: `src/rtk_nav/rtk_nav/rtk_nav.py` (calibration boundary service and resume transitions)
- Test: `src/rtk_nav/test/test_boundary_calibration_contract.py`, `src/rtk_nav/test/test_raw_boundary_stop_contract.py`

**Interfaces:**
- Consumes: raw and confirmed boundary IO, current calibration target, drive feedback, and manual-intervention state.
- Produces: explicit continue, restart-calibration, or pause-for-operator behavior; a blocked turn cannot be interpreted as a completed turn.

- [ ] **Step 1: Pin event ordering around a sensor trigger**
  Test trigger arrival before, during, and immediately after a calibration generator step; discard any nonzero speed computed during the triggering step.
- [ ] **Step 2: Require release plus safe-action checks before restarting**
  Test that sensor release is necessary but not sufficient: direction-specific IO must be fresh and clear, and the chassis must be confirmed stopped from fresh post-stop feedback before any new turn or retreat.
- [ ] **Step 3: Preserve manual lock and expose its reason**
  Test that an unresolved/unsafe turn enters the existing manual-intervention pause path, publishes a clear reason, and cannot resume solely because the sensor deasserted.
- [ ] **Step 4: Run boundary interruption tests**
  Verify immediate stop, minimum stop hold, bounded safe retreat if allowed, no-safe-direction pause, IO timeout pause, and explicit operator re-entry behavior.

### Task 4: Detect turn displacement without mistaking noise for slip

**Files:**
- Modify: `src/rtk_nav/rtk_nav/rtk_nav.py` and, only if needed, `src/rtk_nav/navigation_position_safety.py`
- Test: `src/rtk_nav/test/test_navigation_position_safety.py` and focused calibration tests
- Document: `docs/navigation_position_safety.md` if the turn-specific position rule belongs to the shared safety contract

**Interfaces:**
- Consumes: trusted, timestamp-increasing RTK positions captured at turn start and throughout the turn; turn state and target heading.
- Produces: turn displacement/progress evidence that can reject false successful calibration without changing global position-jump acceptance policy.

- [ ] **Step 1: Add tests for turn-start anchor and trusted samples**
  Assert invalid, repeated, out-of-order, or non-Fixed position samples do not update the turn displacement estimate.
- [ ] **Step 2: Define displacement outcome separately from position-jump rejection**
  Test bounded RTK noise as acceptable, sustained displacement beyond a configurable/derived limit as turn-risk evidence, and position jumps under the existing global guard unchanged.
- [ ] **Step 3: Integrate turn displacement with completion and recovery**
  A heading tolerance hit cannot complete a turn if trusted displacement indicates skid/translation; stop, preserve the anchor, and route to a distinct recovery/pause result.
- [ ] **Step 4: Calibrate limits from recorded data and document them**
  Derive threshold/window from stationary-turn and controlled-slip samples; do not select a constant from the single incident trace alone.
- [ ] **Step 5: Run position-safety and calibration tests**
  Verify noise tolerance, sustained drift detection, invalid sample handling, and unchanged ordinary RTK navigation behavior.

### Task 5: Unify reverse first-waypoint direction semantics

**Files:**
- Modify: `src/rtk_nav/rtk_nav/rtk_nav.py` (`move_to_first_waypoint` and initial recovery target selection)
- Test: `src/rtk_nav/test/test_waypoint_tracking_recovery.py`, `src/rtk_nav/test/test_geofence_recovery_heading.py`

**Interfaces:**
- Consumes: current pose, first waypoint, selected `FORWARD`/`BACKWARD` motion, trusted position, and Stanley controller.
- Produces: one consistent physical motion direction used for body-heading target, path tangent, cross-track geometry, heading anomaly checks, recovery calibration, and output wheel signs.

- [ ] **Step 1: Add table-driven forward/reverse geometry tests**
  Cover targets ahead, behind, and near 90 degrees; assert selected direction, body heading, motion bearing, and normalized heading error agree.
- [ ] **Step 2: Make initial recovery retain or deliberately change drive direction**
  If recovery switches direction, recompute all dependent geometry and speed signs atomically; otherwise keep the selected reverse chassis heading for calibration.
- [ ] **Step 3: Prevent anomaly recovery from targeting the wrong bearing**
  Assert the second heading recovery uses the selected chassis heading for reverse motion and does not reapply an uncoordinated `+180` or output sign flip.
- [ ] **Step 4: Verify distance convergence in a deterministic trajectory test**
  Simulate the selected wheel outputs and assert target distance decreases, heading error stays away from the false ±180-degree condition, and no spurious force-bearing divergence occurs.
- [ ] **Step 5: Run initial-entry and geofence regression tests**
  Verify forward and reverse entry, overshoot recovery, fence checks, and normal forward Stanley behavior.

### Task 6: Update incident documentation and run acceptance checks

**Files:**
- Modify: `docs/navigation_position_safety.md` if required by Task 4
- Modify: `FIX_SUMMARY.md`
- Test: `src/rtk_nav/test/` focused suite, then the full `rtk_nav` package tests

**Interfaces:**
- Consumes: implemented behavior and measured threshold evidence from Tasks 2–5.
- Produces: operator-facing recovery semantics, a dated change record, and offline/field acceptance evidence.

- [ ] **Step 1: Document failure classes and operator-visible outcomes**
  Explain turn stall, IO-interrupted turn, turn displacement, and reverse-entry failures without implying sensor release automatically resumes motion.
- [ ] **Step 2: Add a dated `FIX_SUMMARY.md` entry**
  State affected modules, behavior changes, safety invariants, test evidence, and field validation separately.
- [ ] **Step 3: Run focused regression tests, then all `rtk_nav` tests**
  Run focused test modules first, then `colcon test --packages-select rtk_nav`; report any unrelated pre-existing failures separately.
- [ ] **Step 4: Prepare a staged field acceptance checklist**
  Begin with wheels raised/controlled fixture and stationary RTK checks; then test forward and reverse entry, blocked-turn stop/release, no-progress stop, and measured displacement. Require operator at emergency stop and verify event/fault reporting before any outdoor run.
- [ ] **Step 5: Do not deploy or run autonomous field motion without authorization**
  Provide build/test results and the exact field checklist; deployment remains a separate explicitly approved action.
