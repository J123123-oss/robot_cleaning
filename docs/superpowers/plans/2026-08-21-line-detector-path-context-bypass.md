# Line Detector Path Context Bypass Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an opt-in test parameter that lets the white-line detector run without RTK path context while preserving the default visual-correction safety gate.

**Architecture:** `GridLineDetector` retains `enable_visual_correction` as its first runtime gate. A new `bypass_path_context_gate` parameter defaults to `false`; when true it bypasses only the path-context validity and timeout branch in `image_callback`. Launch and RTK navigation ownership remain unchanged.

**Tech Stack:** ROS 2 Humble Python (`rclpy`), `pytest`, AST-based source contract tests.

---

### Task 1: Define the safety contract

**Files:**
- Modify: `src/rtk_nav/test/test_visual_correction_switch_contract.py`
- Test: `src/rtk_nav/test/test_visual_correction_switch_contract.py`

- [x] **Step 1: Write the failing test**

Add this test after `test_line_detector_defaults_disabled_and_publishes_invalid_output_when_disabled`:

```python
def test_line_detector_path_context_bypass_defaults_off_and_preserves_visual_switch():
    source = LINE_DETECTOR_SOURCE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    initializer = ast.unparse(_function(tree, "__init__"))
    image_callback = ast.unparse(_function(tree, "image_callback"))

    assert "self.declare_parameter('bypass_path_context_gate', False)" in initializer
    assert "self.bypass_path_context_gate" in initializer
    assert "if not self.enable_visual_correction:" in image_callback
    assert "not self.bypass_path_context_gate" in image_callback
    assert "not self.path_context_valid" in image_callback
    assert "self.path_context_timeout_sec" in image_callback
```

- [x] **Step 2: Run the targeted test to verify it fails**

Run:

```bash
pytest -q src/rtk_nav/test/test_visual_correction_switch_contract.py::test_line_detector_path_context_bypass_defaults_off_and_preserves_visual_switch
```

Expected: the test fails because `bypass_path_context_gate` is not yet declared or used.

- [x] **Step 3: Write the minimal implementation**

In `GridLineDetector.__init__`, directly after the existing `enable_visual_correction` declaration and assignment, declare and cache:

```python
self.declare_parameter('bypass_path_context_gate', False)
self.bypass_path_context_gate = bool(
    self.get_parameter('bypass_path_context_gate').value
)
```

In `GridLineDetector.image_callback`, leave the first visual switch branch unchanged. Wrap the existing path-context validity and timeout condition with the bypass guard:

```python
if (
    not self.bypass_path_context_gate
    and (
        not self.path_context_valid
        or time.monotonic() - self.last_path_context_time
        > self.path_context_timeout_sec
    )
):
    self.path_context_valid = False
    self.reset_reacquisition()
    self.publish_invalid()
    return
```

- [x] **Step 4: Run the targeted test to verify it passes**

Run:

```bash
pytest -q src/rtk_nav/test/test_visual_correction_switch_contract.py::test_line_detector_path_context_bypass_defaults_off_and_preserves_visual_switch
```

Expected: `1 passed`.

- [x] **Step 5: Run the visual correction contract suite**

Run:

```bash
pytest -q src/rtk_nav/test/test_visual_correction_switch_contract.py src/rtk_nav/test/test_line_detector_calibration_contract.py src/rtk_nav/test/test_coarse_white_line_contract.py
```

Expected: all tests pass.

- [ ] **Step 6: Commit**

```bash
git add src/rtk_nav/rtk_nav/line_detector_node.py src/rtk_nav/test/test_visual_correction_switch_contract.py
git commit -m "feat: add line detector path context test bypass"
```

Commit only the two files above; do not include pre-existing workspace changes.
