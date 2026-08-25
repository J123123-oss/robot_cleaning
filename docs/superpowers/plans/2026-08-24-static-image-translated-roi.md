# Static Image Translated ROI Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Publish a fixed-size static camera frame by slicing a translated ROI from the original image and clamping the ROI to valid source bounds.

**Architecture:** Keep the complete image unchanged after `image_path` loading. Add a pure `extract_translated_roi` helper that calculates a clamped integer source origin and returns a copied NumPy slice; the timer callback uses this helper only for static-image mode. Keep live camera capture, ROS topic names, and image encoding unchanged.

**Tech Stack:** Python 3, OpenCV, NumPy, ROS2 `rclpy`, AST-based pytest contract tests.

---

### Task 1: Add failing ROI translation tests

**Files:**
- Modify: `src/rtk_nav/test/test_visual_correction_switch_contract.py`
- Test: `src/rtk_nav/test/test_visual_correction_switch_contract.py`

- [ ] **Step 1: Add an AST helper for the new pure function.**

Parse `camera_publisher_node.py`, select the top-level `extract_translated_roi` function, and execute it with a namespace containing `numpy as np`. Keep the existing ROS-free AST test style.

- [ ] **Step 2: Add the behavior test before production code exists.**

Use a uniquely valued `np.arange` source image and assert that a translated ROI has the requested shape and matches the corresponding source slice. Include positive and negative translations that exceed both source bounds; assert the result equals the top-left or bottom-right valid ROI rather than containing zeros or duplicated border pixels.

- [ ] **Step 3: Add the source contract test.**

Assert that the camera publisher declares `crop_x`, `crop_y`, `translate_x`, and `translate_y`, calls `extract_translated_roi` for static frames, and contains none of `cv2.warpAffine`, `cv2.warpPerspective`, `cv2.remap`, `np.roll`, `BORDER_REPLICATE`, or `BORDER_REFLECT`.

- [ ] **Step 4: Run the focused tests and verify the expected RED state.**

Run:

```powershell
python -m pytest src/rtk_nav/test/test_visual_correction_switch_contract.py -q
```

Expected: the new helper lookup or behavior tests fail because `extract_translated_roi` and the new parameters do not yet exist; unrelated existing tests must remain collectible.

### Task 2: Implement clamped source slicing

**Files:**
- Modify: `src/rtk_nav/rtk_nav/camera_publisher_node.py`

- [ ] **Step 1: Add the pure extraction helper.**

Implement a helper that validates the output size, rounds the requested origin, clamps it to valid source bounds, and returns a copied NumPy slice. The implementation must use source slicing only, preserving original pixels and fixed output dimensions.

- [ ] **Step 2: Declare and read the four static-image parameters.**

Declare `crop_x`, `crop_y`, `translate_x`, and `translate_y` with default `0`, then read them after the existing camera parameters. Keep `width` and `height` as the published static-frame dimensions.

- [ ] **Step 3: Use the helper in `timer_callback`.**

Replace the static branch's direct `self.static_frame` assignment with a call to `extract_translated_roi`, passing the configured crop and translation parameters. Leave the live camera `cap.read()` branch unchanged.

### Task 3: Verify syntax, regression scope, and diff

**Files:**
- Verify: `src/rtk_nav/rtk_nav/camera_publisher_node.py`
- Verify: `src/rtk_nav/test/test_visual_correction_switch_contract.py`

- [ ] **Step 1: Run the focused visual test set.**

```powershell
python -m pytest src/rtk_nav/test/test_visual_correction_switch_contract.py src/rtk_nav/test/test_visual_line_geometry.py -q
```

- [ ] **Step 2: Parse and compile the changed Python files.**

```powershell
python -c "import ast, py_compile; from pathlib import Path; files=['src/rtk_nav/rtk_nav/camera_publisher_node.py','src/rtk_nav/test/test_visual_correction_switch_contract.py']; [ast.parse(Path(f).read_text(encoding='utf-8')) for f in files]; [py_compile.compile(f,doraise=True) for f in files]; print('AST and py_compile: PASS')"
```

- [ ] **Step 3: Run whitespace and targeted diff checks.**

```powershell
git diff --check -- src/rtk_nav/rtk_nav/camera_publisher_node.py src/rtk_nav/test/test_visual_correction_switch_contract.py
```
