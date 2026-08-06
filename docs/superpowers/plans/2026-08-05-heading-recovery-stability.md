# RTK Heading Recovery Stability Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Prevent AUTO navigation from resuming Stanley tracking until a recovered dual-Fixed heading has settled and been aligned to the active path.

**Architecture:** Keep one 30-second circular-heading history while the AUTO gate is pending. Gate release requires both the existing five-second range and a 30-second range to be at most one degree. A release into `WAYPOINT_MOVE` changes to `WAYPOINT_CALIB` and starts the existing calibration generator before the navigation generator can issue a Stanley speed command.

**Tech Stack:** Python 3, ROS 2 Humble (`rclpy`), `pytest`, AST source-contract tests.

---

### Task 1: Lock the Recovery Gate Contract

**Files:**
- Create: `src/rtk_nav/test/test_heading_recovery_gate_contract.py`
- Test: `src/rtk_nav/test/test_heading_recovery_gate_contract.py`

- [ ] **Step 1: Write the failing test**

```python
def test_heading_gate_requires_short_and_settle_windows():
    source, tree = _source_tree()
    callback = ast.unparse(_function(tree, "heading_callback"))
    assert "HEADING_STABILITY_SETTLE_WINDOW" in source
    assert "HEADING_STABILITY_SETTLE_RANGE" in source
    assert "short_range <= HEADING_STABILITY_RANGE" in callback
    assert "settle_range <= HEADING_STABILITY_SETTLE_RANGE" in callback
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest src/rtk_nav/test/test_heading_recovery_gate_contract.py -q`

Expected: FAIL because the settle-window constants and checks do not exist.

- [ ] **Step 3: Add source-contract tests for reset, timeout, and alignment**

```python
def test_gate_loss_resets_sampling_epoch_and_timeout_start():
    callback = ast.unparse(_function(_source_tree()[1], "heading_callback"))
    assert "self._auto_heading_gate_start_time = None" in callback

def test_gate_release_aligns_waypoint_move_before_generator_creation():
    timer = ast.unparse(_function(_source_tree()[1], "rtk_timer_callback"))
    assert "_start_auto_heading_gate_path_alignment" in timer
    assert timer.index("_start_auto_heading_gate_path_alignment") < timer.index("multi_waypoint_nav_generator")
```

- [ ] **Step 4: Run tests to verify they fail**

Run: `python -m pytest src/rtk_nav/test/test_heading_recovery_gate_contract.py -q`

Expected: FAIL on the new contract assertions.

### Task 2: Implement Dual-Window Sampling and Recovery Alignment

**Files:**
- Modify: `src/rtk_nav/rtk_nav/rtk_nav.py:111-115`
- Modify: `src/rtk_nav/rtk_nav/rtk_nav.py:195-202`
- Modify: `src/rtk_nav/rtk_nav/rtk_nav.py:1888-1938`
- Modify: `src/rtk_nav/rtk_nav/rtk_nav.py:4185-4212`
- Modify: `src/rtk_nav/rtk_nav/rtk_nav.py:4448-4510`

- [ ] **Step 1: Define the long settle window and timeout**

```python
HEADING_STABILITY_SETTLE_WINDOW = 30.0
HEADING_STABILITY_SETTLE_RANGE = 1.0
AUTO_HEADING_GATE_TIMEOUT = 180.0
```

- [ ] **Step 2: Retain only gate-pending samples and require both ranges**

```python
tracking_active = self._auto_heading_gate_pending
while history and now - history[0][0] > HEADING_STABILITY_SETTLE_WINDOW:
    history.popleft()
is_stable = (
    short_range <= HEADING_STABILITY_RANGE
    and settle_range <= HEADING_STABILITY_SETTLE_RANGE
)
```

- [ ] **Step 3: Reset the dual-Fixed epoch on every quality loss**

```python
self._heading_stability_history.clear()
self._auto_heading_gate_fixed_since = None
self._auto_heading_gate_start_time = None
```

- [ ] **Step 4: Start stopped path alignment before generator creation**

```python
if self._auto_heading_gate_path_alignment_pending:
    if not self._start_auto_heading_gate_path_alignment():
        self.publish_stop_speed()
        return
```

The helper resolves the saved target/path direction, calls
`start_heading_recalibration()`, switches to `NavState.WAYPOINT_CALIB`, and
does not clear the alignment flag until that transition succeeds.

- [ ] **Step 5: Run contract tests to verify they pass**

Run: `python -m pytest src/rtk_nav/test/test_heading_recovery_gate_contract.py -q`

Expected: PASS.

### Task 3: Record and Verify the Fix

**Files:**
- Modify: `STANLEY_FIX_SUMMARY.md`

- [ ] **Step 1: Add a dated entry**

Document the slow-drift evidence, 5-second plus 30-second contract, 180-second
timeout, and the pre-Stanley path-alignment transition. State explicitly that
`rtk_status` remains unchanged.

- [ ] **Step 2: Run focused verification**

Run: `C:\Users\000\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe -B -m py_compile src\rtk_nav\rtk_nav\rtk_nav.py`

Run: `C:\Users\000\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe -m pytest src\rtk_nav\test\test_heading_recovery_gate_contract.py src\rtk_nav\test\test_boundary_calibration_contract.py -q`

Expected: syntax compilation and both focused test files pass.

- [ ] **Step 3: Check only intended files and commit**

Run: `git diff --check -- src/rtk_nav/rtk_nav/rtk_nav.py src/rtk_nav/test/test_heading_recovery_gate_contract.py STANLEY_FIX_SUMMARY.md docs/superpowers/plans/2026-08-05-heading-recovery-stability.md`

Run: `git diff --cached --name-only`

Expected: no whitespace errors; staging includes only the four listed files.
