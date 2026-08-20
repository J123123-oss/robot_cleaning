# Visual Correction Master Switch Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a default-off `enable_visual_correction` launch switch on the `camera` branch and make `rtk_nav` gate future visual path-context output without changing existing RTK navigation behavior.

**Architecture:** `run.launch.py` declares the global launch argument, passes it to `rtk_nav`, and conditionally starts the packaged `line_detector_node`. `rtk_nav.py` publishes the current path context as valid only when the switch and RTK navigation gates pass. The detector consumes that context and publishes invalid output on timeout, state loss, or failed geometry.

**Tech Stack:** ROS2 Humble launch, `rclpy`, Python `pytest`, existing `rtk_nav` node.

---

### Task 1: Add the launch-level switch

**Files:**
- Modify: `src/rtk_nav/launch/run.launch.py`
- Test: `src/rtk_nav/test/test_visual_correction_switch_contract.py`

- [ ] **Step 1: Write the failing contract test**

Create a source-level test that loads the launch file as text and asserts that it declares
`enable_visual_correction` with default `false` and passes `LaunchConfiguration("enable_visual_correction")`
to the `rtk_navigator` parameters.

```python
from pathlib import Path


LAUNCH = Path(__file__).parents[1] / "launch" / "run.launch.py"


def test_launch_declares_visual_correction_default_off_and_passes_it_to_rtk():
    source = LAUNCH.read_text(encoding="utf-8")
    assert '"enable_visual_correction"' in source
    assert 'default_value=TextSubstitution(text="false")' in source
    assert 'LaunchConfiguration("enable_visual_correction")' in source
    assert '"enable_visual_correction": LaunchConfiguration("enable_visual_correction")' in source
```

- [ ] **Step 2: Run the focused test and verify it fails**

Run:

```bash
pytest -q src/rtk_nav/test/test_visual_correction_switch_contract.py
```

Expected: FAIL because the launch argument and RTK parameter are not yet present.

- [ ] **Step 3: Implement the launch argument and parameter pass-through**

Import/use the existing launch APIs and add:

```python
declare_visual_correction_arg = DeclareLaunchArgument(
    "enable_visual_correction",
    default_value=TextSubstitution(text="false"),
    description="Enable visual correction path context; default is disabled",
)
```

Register `declare_visual_correction_arg` in the launch description and add this entry to
the `rtk_navigator` parameter list:

```python
{"enable_visual_correction": LaunchConfiguration("enable_visual_correction")}
```

Do not add the root-level `line_detector_node.py` as a launch `Node` because it has no
installed ROS2 console entry point.

- [ ] **Step 4: Run the focused test and syntax check**

Run:

```bash
pytest -q src/rtk_nav/test/test_visual_correction_switch_contract.py
python -m py_compile src/rtk_nav/launch/run.launch.py
```

Expected: the contract test passes and `py_compile` exits with code 0.

- [ ] **Step 5: Commit the launch change**

```bash
git add src/rtk_nav/launch/run.launch.py src/rtk_nav/test/test_visual_correction_switch_contract.py
git commit -m "feat: add visual correction launch switch"
```

### Task 2: Add the RTK node parameter and gate

**Files:**
- Modify: `src/rtk_nav/rtk_nav/rtk_nav.py`
- Test: `src/rtk_nav/test/test_visual_correction_switch_contract.py`

- [ ] **Step 1: Extend the failing contract test**

Add source assertions that `rtk_nav.py` declares `enable_visual_correction`, stores its
boolean value, and checks it at the visual path-context publication boundary.

```python
RTK_NAV = Path(__file__).parents[1] / "rtk_nav" / "rtk_nav.py"


def test_rtk_declares_and_uses_visual_correction_switch():
    source = RTK_NAV.read_text(encoding="utf-8")
    assert 'declare_parameter("enable_visual_correction", False)' in source
    assert "self.enable_visual_correction" in source
    assert "if not self.enable_visual_correction" in source
```

- [ ] **Step 2: Run the focused test and verify it fails**

Run:

```bash
pytest -q src/rtk_nav/test/test_visual_correction_switch_contract.py
```

Expected: FAIL because `rtk_nav.py` does not yet declare or consume the switch.

- [ ] **Step 3: Implement the RTK parameter and visual-context gate**

Declare the parameter during `RTKNavControlNode.__init__`:

```python
self.declare_parameter("enable_visual_correction", False)
self.enable_visual_correction = bool(
    self.get_parameter("enable_visual_correction").value
)
```

Publish `/rtk/visual_path_context` from the 10 Hz RTK timer. Set `x` to the active
Stanley path direction, `y` to `imu_yaw`, and `z=1` only when the switch, AUTO mode,
RTK fixed solution, active move state, finite path direction, and finite heading all
hold. Otherwise publish `z=0`. This context must not alter existing `/rtk/motor_speed`,
RTK status, pause, heading-gate, or boundary behavior.

- [ ] **Step 4: Run focused tests and existing RTK tests**

Run:

```bash
pytest -q src/rtk_nav/test/test_visual_correction_switch_contract.py
pytest -q src/rtk_nav/test
python -m py_compile src/rtk_nav/rtk_nav/rtk_nav.py
git diff --check
```

Expected: all focused and existing tests pass, compilation succeeds, and `git diff --check`
prints no errors.

- [ ] **Step 5: Commit the RTK gate**

```bash
git add src/rtk_nav/rtk_nav/rtk_nav.py src/rtk_nav/test/test_visual_correction_switch_contract.py
git commit -m "feat: gate visual correction context in rtk nav"
```

### Task 3: Verify branch scope and runtime contract

**Files:**
- No source changes expected.

- [ ] **Step 1: Verify branch and staged scope**

Run:

```bash
git branch --show-current
git status --short
git log -2 --oneline
```

Expected: current branch is `camera`; unrelated pre-existing worktree files are not
staged or committed.

- [ ] **Step 2: Verify launch usage contract**

Run:

```bash
ros2 launch rtk_nav run.launch.py enable_visual_correction:=false
```

Expected: the launch starts with visual correction disabled and existing RTK navigation
continues without a visual correction dependency. A ROS2 runtime environment and hardware
or replay data are required; if unavailable, record that this runtime check was not run.

- [ ] **Step 3: Record the enabled-mode limitation**

Confirm in the handoff that `enable_visual_correction:=true` only enables the RTK-side
future visual-context gate in this change. The root-level `line_detector_node.py` is not
yet launched until it is installed as a package executable.
