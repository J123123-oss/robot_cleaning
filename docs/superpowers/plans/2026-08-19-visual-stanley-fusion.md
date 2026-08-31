# Visual Stanley Fusion Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task with verification checkpoints.

**Goal:** Add bounded camera-based heading and lateral correction to the existing RTK Stanley controller without replacing RTK control or bypassing safety states.

**Architecture:** `rtk_nav.py` subscribes to `/grid_line/angle_deviation` and `/grid_line/detection_confidence`, stores the latest valid visual sample, and computes a separately limited steering correction. `stanley_steering_control()` adds that correction only in AUTO moving states when RTK is ready, the visual sample is fresh and confident, and boundary/calibration/pause gates are inactive. Existing RTK heading and GPS lateral terms remain authoritative.

**Tech Stack:** ROS2 rclpy, `geometry_msgs/Vector3`, `std_msgs/Float32`, Python AST/direct contract tests.

---

### Task 1: Add the failing visual Stanley contract

**Files:**
- Modify: `src/rtk_nav/test/test_visual_correction_switch_contract.py`

- [ ] **Step 1: Add assertions for the visual subscribers, parameters, freshness gate, and Stanley fusion.**

  Assert that `rtk_nav.py` contains subscriptions to both visual topics, declares the five fusion parameters, has a visual callback, has a visual steering helper, checks `self.rtk_solution_ready`, `self.current_control_mode`, `NavState.WAYPOINT_MOVE`/`NavState.INITIAL_MOVE`, `self.boundary_correct_locked`, and adds the visual term before the existing `±45` clamp.

- [ ] **Step 2: Run the direct contract test and confirm it fails because the integration is absent.**

  Run:

  ```powershell
  python -c "import runpy; ns=runpy.run_path('src/rtk_nav/test/test_visual_correction_switch_contract.py'); ns['test_rtk_stanley_consumes_fresh_visual_correction']()"
  ```

  Expected: `AssertionError` for the missing visual subscription or fusion implementation.

### Task 2: Implement visual input and bounded Stanley fusion

**Files:**
- Modify: `src/rtk_nav/rtk_nav/rtk_nav.py`

- [ ] **Step 1: Add parameters and state.**

  Declare `visual_heading_gain` default `1.0`, `visual_lateral_gain` default `10.0` degrees per meter, `visual_max_steering_deg` default `10.0`, `visual_confidence_threshold` default `0.5`, and `visual_timeout_sec` default `0.5`. Store the latest `Vector3` sample, confidence, and monotonic receive timestamp.

- [ ] **Step 2: Subscribe to the existing visual output topics.**

  Subscribe to `/grid_line/angle_deviation` with a callback that accepts only finite `x`, `y`, and `z >= 0.5`; otherwise clear the valid flag. Subscribe to `/grid_line/detection_confidence` and store finite confidence values.

- [ ] **Step 3: Add a helper that enforces all runtime gates.**

  Return zero when visual correction is disabled, the sample is invalid/stale/under-confident, RTK is not ready, control mode is not `AUTO_CLEANING`, navigation state is not `INITIAL_MOVE` or `WAYPOINT_MOVE`, boundary correction is locked, calibration/retreat is active, or the force-bearing mode is active. Otherwise compute:

  ```python
  correction = (
      self.visual_lateral_gain * visual_lateral_m
      - self.visual_heading_gain * visual_heading_deg
  )
  ```

  Clamp it to `[-visual_max_steering_deg, visual_max_steering_deg]`.

- [ ] **Step 4: Add the visual correction to Stanley before the existing total steering clamp.**

  Keep the current RTK terms unchanged and calculate `total_steering = steering_correction - heading_error + visual_correction`.

### Task 3: Expose and document the fusion configuration

**Files:**
- Modify: `src/rtk_nav/launch/run.launch.py`
- Modify: `docs/superpowers/specs/2026-08-19-rtk-visual-path-context-design.md`

- [ ] **Step 1: Pass the five visual fusion parameters from launch.**

  Add them to the `rtk_navigator` parameter dictionary while retaining `enable_visual_correction` as the master switch.

- [ ] **Step 2: Update the design boundary.**

  Replace the statement that visual output is not consumed by `rtk_nav` with the implemented bounded fusion contract, including the safety gates and the fact that field sign calibration remains required.

### Task 4: Verify the implementation

**Files:**
- Test: `src/rtk_nav/test/test_visual_correction_switch_contract.py`

- [ ] **Step 1: Run all direct visual contract tests.**

  Run:

  ```powershell
  python -c "import runpy; ns=runpy.run_path('src/rtk_nav/test/test_visual_correction_switch_contract.py'); names=sorted(name for name in ns if name.startswith('test_')); [ns[name]() for name in names]; print(f'direct contract tests: PASS ({len(names)}/{len(names)})')"
  ```

  Expected: all tests pass.

- [ ] **Step 2: Parse and compile changed Python files in a temporary directory.**

  Run:

  ```powershell
  python -c "import ast, py_compile, tempfile; from pathlib import Path; files=['src/rtk_nav/rtk_nav/rtk_nav.py','src/rtk_nav/launch/run.launch.py','src/rtk_nav/test/test_visual_correction_switch_contract.py']; [ast.parse(Path(f).read_text(encoding='utf-8')) for f in files]; d=Path(tempfile.mkdtemp(prefix='visual_stanley_compile_')); [py_compile.compile(f,cfile=str(d/(Path(f).stem+'.pyc')),doraise=True) for f in files]; print('AST and py_compile: PASS')"
  ```

- [ ] **Step 3: Check only the touched business files for whitespace errors.**

  Run `git diff --check -- src/rtk_nav/rtk_nav/rtk_nav.py src/rtk_nav/launch/run.launch.py src/rtk_nav/test/test_visual_correction_switch_contract.py docs/superpowers/specs/2026-08-19-rtk-visual-path-context-design.md`.

