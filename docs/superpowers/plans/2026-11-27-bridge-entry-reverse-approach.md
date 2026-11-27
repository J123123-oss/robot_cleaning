# Bridge Entry Reverse Approach Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Allow only the first waypoint of a `bridge`-named area to use a minimum-turn reverse approach, then return to normal forward traversal after entry alignment.

**Architecture:** Reuse the existing planner-to-path area-name chain. The navigation node classifies the current target segment from `waypoint_areas`, chooses `FORWARD` or `BACKWARD` once at a bridge-area boundary, and applies the existing initial reverse Stanley convention to the bridge entrance segment. Arrival confirmation, deceleration, geofence checks, boundary IO checks, and final waypoint heading calibration remain shared with ordinary waypoint movement.

**Tech Stack:** ROS 2 Jazzy, Python, pytest, existing `RTKNavControlNode` and AST-isolated navigation tests.

**Spec:** `docs/superpowers/specs/2026-11-27-bridge-entry-reverse-approach-design.md`

**Plan correction:** `_mid` points are only inserted for long transitions; the requested bridge approaches are short. Direction is selected at the cleaning endpoint before its final heading calibration, then retained for the bridge entry target. Pause and AUTO heading-gate recovery use the same direction-aware heading. Direction is cleared after final alignment at the bridge entrance, before driving forward inside the bridge.

## Global Constraints

- Match bridge areas case-insensitively by the `bridge` name prefix.
- Reverse only the approach from the cleaning endpoint through transition waypoints to the first waypoint of a bridge area.
- Choose forward at a turn angle of 90 degrees or less and reverse only above 90 degrees.
- Preserve the existing 0.10 m arrival tolerance and three-new-trusted-sample confirmation for ordinary waypoints.
- Preserve the existing low-distance speed scaling and all geofence, boundary IO, localization, and stop-confirmation gates.
- Do not modify unrelated worktree changes.
- Update `FIX_SUMMARY.md` with behavior and verification results after implementation.

## Review Focus

- A non-bridge area with a large heading error must retain forward behavior.
- A bridge target exactly 90 degrees from the current heading must retain forward behavior.
- A bridge target beyond 90 degrees must use a body heading offset by 180 degrees and swap the Stanley wheel outputs.
- The second waypoint inside the same bridge area must not inherit reverse mode.
- Arrival confirmation and low-distance scaling must run identically for reverse bridge entry.

---

### Task 1: Add bridge direction and output regression tests

**Files:**
- Modify: `src/rtk_nav/test/test_segment_direction_recovery.py`
- Modify: `src/rtk_nav/test/test_waypoint_tracking_recovery.py`

**Interfaces:**
- Consumes: current `waypoint_areas`, target waypoint index, current IMU heading, and Stanley output.
- Produces: failing tests for bridge-area detection, minimum-turn selection, reverse output conversion, and direction reset.

- [ ] **Step 1: Write the failing tests**

  Add tests for `bridge-2-3` as the first target in a bridge area, a non-bridge target with the same geometry, the exact 90-degree boundary, a reverse target, and the next target after bridge entry.

- [ ] **Step 2: Run the focused tests to verify the expected failures**

  Run `pytest -q src/rtk_nav/test/test_segment_direction_recovery.py src/rtk_nav/test/test_waypoint_tracking_recovery.py`.

  Expected result: the new bridge behavior assertions fail because the current normal waypoint path has no bridge-specific direction state.

### Task 2: Implement bridge-area classification and minimum-turn selection

**Files:**
- Modify: `src/rtk_nav/rtk_nav/rtk_nav.py` near `NavState`, navigation context initialization, and waypoint tracking helpers

**Interfaces:**
- Consumes: `waypoint_areas`, `current_waypoint_idx`, current IMU heading, and the forward path bearing.
- Produces: `_is_bridge_area_name(area_name) -> bool`, `_is_bridge_entry_waypoint(index) -> bool`, and `_select_bridge_drive_direction(path_bearing) -> str`.

- [ ] **Step 1: Implement the area and boundary helpers**

  Add helpers that safely handle missing or short `waypoint_areas`, normalize area names with `strip().lower()`, match the `bridge` prefix, and identify only the first waypoint whose area is bridge while the preceding waypoint is not bridge.

- [ ] **Step 2: Implement minimum-turn selection**

  Use `select_geofence_recovery_heading(self.imu_yaw, path_bearing)` so the existing normalized angle convention determines the shortest body-heading target. Store `FORWARD` or `BACKWARD` in the navigation context only for the active bridge entrance segment.

- [ ] **Step 3: Run the focused classification tests**

  Run the new helper tests from Task 1 and confirm they pass while the motion-output tests remain focused on the next task.

### Task 3: Apply reverse Stanley semantics only to bridge entry

**Files:**
- Modify: `src/rtk_nav/rtk_nav/rtk_nav.py` in `multi_waypoint_nav_generator`
- Modify: `src/rtk_nav/test/test_segment_direction_recovery.py`

**Interfaces:**
- Consumes: bridge direction state from Task 2, normal path geometry, current distance, and the existing `stanley_steering_control`.
- Produces: forward motion for normal and forward bridge segments, reverse motion for a selected bridge entry, and a final heading target that remains the waypoint's original heading.

- [ ] **Step 1: Add a failing output test**

  Assert that a reverse bridge entry calls Stanley with the virtual heading `imu_yaw + 180 degrees`, positive speed magnitude, the forward path bearing, and swaps the returned left and right speeds. Assert that the next bridge waypoint emits normal forward output.

- [ ] **Step 2: Run the test and verify it fails**

  Run the specific new tests with `pytest -q`.

- [ ] **Step 3: Implement the minimal motion change**

  Initialize bridge direction after the current path segment is available. For `BACKWARD`, use the existing initial-entry convention of virtual heading plus 180 degrees, positive Stanley velocity, then swap wheel outputs. Keep path geometry, arrival checks, low-distance scaling, boundary filtering, and geofence checks unchanged.

- [ ] **Step 4: Reset bridge direction on target transition**

  Clear the bridge entry direction when the target is completed or when the next target is not the first waypoint of a bridge area. Preserve the existing final heading calibration so the entrance is aligned before the next waypoint moves forward.

- [ ] **Step 5: Run focused motion tests**

  Run `pytest -q src/rtk_nav/test/test_segment_direction_recovery.py src/rtk_nav/test/test_waypoint_tracking_recovery.py`.

### Task 4: Verify arrival and speed behavior under reverse entry

**Files:**
- Modify: `src/rtk_nav/test/test_waypoint_tracking_recovery.py`
- Modify: `src/rtk_nav/test/test_navigation_completion.py` only if a shared completion contract requires coverage

**Interfaces:**
- Consumes: reverse bridge navigation context and existing arrival confirmation helpers.
- Produces: regression coverage proving reverse mode does not bypass the 0.10 m threshold, three-sample confirmation, or low-distance speed scaling.

- [ ] **Step 1: Add failing behavioral tests**

  Assert that an unconfirmed sample inside 0.10 m yields zero output, confirmed arrival starts final calibration, and a point at 0.75 m uses the same scaled base speed as forward movement.

- [ ] **Step 2: Run the focused tests and verify the intended failures**

  Run the specific tests and confirm the failure is caused by missing bridge reverse behavior rather than fixture errors.

- [ ] **Step 3: Implement only required shared-state wiring**

  Keep arrival and speed calculations shared. Adjust bridge context setup or reset only where the tests demonstrate state leakage.

- [ ] **Step 4: Run the focused behavior tests**

  Run the focused test modules and confirm all bridge and ordinary waypoint cases pass.

### Task 5: Update documentation and validate the package

**Files:**
- Modify: `FIX_SUMMARY.md`
- Modify: `docs/navigation_position_safety.md` only if the shared arrival rule wording needs clarification

**Interfaces:**
- Consumes: implemented bridge entry behavior and test results.
- Produces: dated change record with offline validation and explicit field-validation status.

- [ ] **Step 1: Run focused regression tests**

  Run the bridge, segment-direction, waypoint-tracking, and navigation-position tests.

- [ ] **Step 2: Run syntax and diff checks**

  Run `python -m compileall src/rtk_nav/rtk_nav/rtk_nav.py` and `git diff --check`.

- [ ] **Step 3: Run the full available `rtk_nav` test command**

  Run `colcon test --packages-select rtk_nav` when the ROS 2 environment is available. Record unrelated collection or baseline failures separately.

- [ ] **Step 4: Add the dated `FIX_SUMMARY.md` entry**

  Record the bridge-name trigger, minimum-turn rule, reverse entry and forward exit behavior, focused test results, package-test results, and that no outdoor autonomous validation was performed.
