# Path-Aware Fixed Boundary Line Geometry Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (- [ ]) syntax for tracking.

**Goal:** Replace the detector's all-line average-center lateral estimate with a path-aware boundary-pair estimate driven by "/rtk/visual_path_context".

**Architecture:** "line_detector_node.py" receives the active path and vehicle headings, transforms the path axis into image coordinates, classifies Hough segments into path-parallel and path-perpendicular groups, and computes lateral error from the midpoint of the nearest valid boundary pair. Invalid or stale context and incomplete geometry publish z=0 and zero confidence. The existing "rtk_nav.py" context publisher and Stanley consumer remain unchanged.

**Tech Stack:** ROS2 Python, geometry_msgs/Vector3, OpenCV HoughLinesP, NumPy, AST/unittest contract tests.

---

## File Map

- Modify: src/rtk_nav/rtk_nav/line_detector_node.py
  - Add path-context state, geometry helpers, boundary-pair selection, reacquisition, and invalid-output handling.
  - Replace average Hough-center lateral estimation with path-normal boundary projection.
- Modify: src/rtk_nav/launch/run.launch.py
  - Pass enable_visual_correction to the detector and expose the geometry parameters used by the node.
- Modify: src/rtk_nav/test/test_visual_line_geometry.py
  - Extend pure geometry coverage for path-axis projection and boundary-pair translation behavior.
- Create: src/rtk_nav/test/test_path_aware_boundary_contract.py
  - Test source-level contracts and path-context invalidation without importing ROS2.

src/rtk_nav/rtk_nav/rtk_nav.py already publishes /rtk/visual_path_context with the approved x/y/z contract and is not changed by this plan.

### Task 1: Add Failing Geometry Tests

**Files:**
- Modify: src/rtk_nav/test/test_visual_line_geometry.py
- Create: src/rtk_nav/test/test_path_aware_boundary_contract.py

- [ ] **Step 1: Add translation-invariance tests before changing production code**

Add tests that extract top-level helpers from line_detector_node.py with AST and assert the desired geometry:

~~~
def test_boundary_pair_is_invariant_to_translation_along_path_axis():
    helpers = _helpers()
    lines = [
        (440, 0, 440, 480, 480.0, 90.0, 440.0, 240.0),
        (200, 0, 200, 480, 480.0, 90.0, 200.0, 240.0),
    ]
    shifted = [
        (440, 100, 440, 580, 480.0, 90.0, 440.0, 340.0),
        (200, 100, 200, 580, 480.0, 90.0, 200.0, 340.0),
    ]
    first = helpers["select_boundary_pair"](lines, 90.0, 640, 480, 500.0)
    second = helpers["select_boundary_pair"](
        shifted, 90.0, 640, 480, 500.0
    )
    assert first is not None and second is not None
    first_error = (first[2] + first[3]) / 2.0 - first[4]
    second_error = (second[2] + second[3]) / 2.0 - second[4]
    assert math.isclose(first_error, second_error, abs_tol=1e-9)


def test_boundary_pair_changes_with_path_normal_translation():
    helpers = _helpers()
    lines = [
        (470, 0, 470, 480, 480.0, 90.0, 470.0, 240.0),
        (230, 0, 230, 480, 480.0, 90.0, 230.0, 240.0),
    ]
    pair = helpers["select_boundary_pair"](lines, 90.0, 640, 480, 500.0)
    assert pair is not None
    error = (pair[2] + pair[3]) / 2.0 - pair[4]
    assert math.isclose(error, 30.0, abs_tol=1e-9)
~~~

Add a contract test that requires the detector to declare and use /rtk/visual_path_context, line_angle_tolerance_deg, path_context_timeout_sec, boundary_pair_max_gap_px, and reacquire_frames, and that the image callback has an invalid-output path.

- [ ] **Step 2: Run the new tests and verify they fail for the missing helpers/contracts**

Run:

~~~
python -m unittest discover -s src/rtk_nav/test -p test_path_aware_boundary_contract.py
~~~

Expected: FAIL because the current detector has no path-context subscription and no select_boundary_pair helper. Do not modify production code before observing this failure.

- [ ] **Step 3: Commit the test-only red state**

Stage only the two test files and commit:

~~~
git add src/rtk_nav/test/test_visual_line_geometry.py src/rtk_nav/test/test_path_aware_boundary_contract.py
git commit -m "test: define path-aware boundary geometry"
~~~

### Task 2: Implement Path Context and Geometry Helpers

**Files:**
- Modify: src/rtk_nav/rtk_nav/line_detector_node.py

- [ ] **Step 1: Add pure helpers and node state**

Implement these top-level helpers with the approved angle conventions:

~~~
def wrap180(angle_deg):
    return (float(angle_deg) + 180.0) % 360.0 - 180.0


def undirected_angle(angle_deg):
    return (float(angle_deg) + 90.0) % 180.0 - 90.0


def undirected_angle_distance(angle_a, angle_b):
    return abs(undirected_angle(float(angle_a) - float(angle_b)))
~~~

Implement select_boundary_pair(lines, axis_angle_deg, width, height, max_gap_px) by projecting each line center onto the path-normal vector:

~~~
axis_rad = math.radians(axis_angle_deg)
normal_x = -math.sin(axis_rad)
normal_y = math.cos(axis_rad)
center_projection = (
    (width / 2.0) * normal_x + (height / 2.0) * normal_y
)
projection = line[6] * normal_x + line[7] * normal_y
~~~

Select the closest candidate on each side of center_projection; reject missing sides, non-positive gaps, and gaps greater than max_gap_px.

Add the path-context subscription and state:

~~~
self.path_context_sub = self.create_subscription(
    Vector3, '/rtk/visual_path_context', self.path_context_callback, 10
)
self.declare_parameter('enable_visual_correction', False)
self.declare_parameter('line_angle_tolerance_deg', 15.0)
self.declare_parameter('path_context_timeout_sec', 0.5)
self.declare_parameter('boundary_pair_max_gap_px', 1200.0)
self.declare_parameter('reacquire_frames', 3)
self.path_direction_deg = 0.0
self.vehicle_heading_deg = 0.0
self.path_context_valid = False
self.last_path_context_time = 0.0
self.valid_streak = 0
self.last_path_direction_deg = None
~~~

- [ ] **Step 2: Add context callback, invalid publishing, and reacquisition helpers**

path_context_callback must accept only finite msg.x/msg.y with msg.z >= 0.5, update the headings, and reset valid_streak when the path direction changes by more than 20 degrees or context becomes invalid. Use time.monotonic() for timeout checks.

publish_invalid must publish:

~~~
result = Vector3()
result.z = 0.0
confidence = Float32()
confidence.data = 0.0
~~~

If retaining the diagnostic topic, publish String(data='invalid') there as well. All logger calls must pass one rendered string, for example self.get_logger().error(f'图像处理错误: {exc}').

- [ ] **Step 3: Run the geometry and contract tests**

Run:

~~~
python -m unittest discover -s src/rtk_nav/test -p test_path_aware_boundary_contract.py
python -c "import ast; ast.parse(open('src/rtk_nav/rtk_nav/line_detector_node.py', encoding='utf-8').read()); print('AST parse OK')"
~~~

Expected: the new pure helper and source contract tests pass. ROS2 imports are not required for these tests because helper extraction is AST-based.

### Task 3: Replace Detection with Path-Aware Boundary Geometry

**Files:**
- Modify: src/rtk_nav/rtk_nav/line_detector_node.py

- [ ] **Step 1: Add context gating at the start of image_callback**

Before image conversion, reject disabled, invalid, or stale context:

~~~
if not self.enable_visual_correction:
    self.publish_invalid()
    return
if (
    not self.path_context_valid
    or time.monotonic() - self.last_path_context_time
    > self.path_context_timeout_sec
):
    self.reset_reacquisition()
    self.publish_invalid()
    return
~~~

On exceptions, reset reacquisition and publish invalid output rather than leaving the last valid result active.

- [ ] **Step 2: Replace angle grouping and average-center measurement**

Keep the existing grayscale, blur, Canny, Hough, and debug-image publishing pipeline, but represent each segment as:

~~~
(x1, y1, x2, y2, length, line_angle, center_x, center_y)
~~~

Do not normalize all lines into one dominant group. Compute:

~~~
relative_path_heading = wrap180(
    self.path_direction_deg - self.vehicle_heading_deg
)
directed_path_axis_image = wrap180(
    90.0 - relative_path_heading + self.camera_angle_offset
)
path_axis_image = undirected_angle(directed_path_axis_image)
cross_axis_image = undirected_angle(directed_path_axis_image + 90.0)
~~~

Classify each line when either error is within self.line_angle_tolerance_deg:

~~~
parallel_error = undirected_angle_distance(line_angle, path_axis_image)
perpendicular_error = undirected_angle_distance(
    line_angle, cross_axis_image
)
~~~

Use select_boundary_pair on parallel_group. Compute weighted parallel_angle and cross_angle; require both groups, a boundary pair, and finite values before declaring valid geometry.

- [ ] **Step 3: Calculate errors and validity from the selected geometry**

Use the boundary projection pair rather than avg_line_center_x:

~~~
_, _, left_projection, right_projection, center_projection = pair
lateral_pixel_error = (
    (left_projection + right_projection) / 2.0 - center_projection
)
lateral_m = self.pixels_to_lateral_meters(lateral_pixel_error)
~~~

Use the perpendicular group for signed heading error, normalize it to [-90, 90], and do not subtract camera_angle_offset a second time in image_callback; the offset is already included in the expected path axis.

Increment valid_streak only for valid geometry. Set detected = valid_streak >= reacquire_frames; otherwise return zero errors and zero confidence. Draw parallel lines in green and perpendicular lines in blue so the selected geometry is visible in /grid_line/detected_image.

- [ ] **Step 4: Run tests and static checks**

Run:

~~~
python -m unittest discover -s src/rtk_nav/test -p test_path_aware_boundary_contract.py
python -m unittest discover -s src/rtk_nav/test -p test_line_detector_calibration_contract.py
python -c "import ast; ast.parse(open('src/rtk_nav/rtk_nav/line_detector_node.py', encoding='utf-8').read()); print('AST parse OK')"
git diff --check -- src/rtk_nav/rtk_nav/line_detector_node.py src/rtk_nav/test/test_visual_line_geometry.py src/rtk_nav/test/test_path_aware_boundary_contract.py
~~~

Expected: all unittest commands exit 0, AST prints AST parse OK, and git diff --check prints no errors.

### Task 4: Wire Launch Parameters and Verify the Contract

**Files:**
- Modify: src/rtk_nav/launch/run.launch.py
- Modify: src/rtk_nav/test/test_path_aware_boundary_contract.py

- [ ] **Step 1: Pass detector parameters from launch**

Declare launch arguments for camera_angle_offset, camera_height, camera_pitch_deg, focal_length_px, min_line_count, line_angle_tolerance_deg, path_context_timeout_sec, boundary_pair_max_gap_px, and reacquire_frames. Pass them to the detector using ParameterValue(LaunchConfiguration(name), value_type=float) for floating-point parameters and value_type=int for integer parameters. Keep enable_visual_correction as a boolean launch condition and parameter.

- [ ] **Step 2: Add launch source contracts**

Assert that the launch file contains each argument, passes it to line_detector_node, and uses the correct value_type. Assert that the node remains conditionally enabled by enable_visual_correction and that the detector subscribes to /rtk/visual_path_context.

- [ ] **Step 3: Run source contracts and diff checks**

Run:

~~~
python -m unittest discover -s src/rtk_nav/test -p test_path_aware_boundary_contract.py
python -c "import ast; ast.parse(open('src/rtk_nav/launch/run.launch.py', encoding='utf-8').read()); print('launch AST parse OK')"
git diff --check
~~~

Expected: source contracts pass, launch AST prints launch AST parse OK, and the diff check has no output.

### Task 5: Ubuntu ROS2 Verification and Commit

**Files:**
- Modify: src/rtk_nav/rtk_nav/line_detector_node.py
- Modify: src/rtk_nav/launch/run.launch.py
- Modify: src/rtk_nav/test/test_visual_line_geometry.py
- Create: src/rtk_nav/test/test_path_aware_boundary_contract.py

- [ ] **Step 1: Build the package on Ubuntu**

Run:

~~~
colcon build --symlink-install --packages-select rtk_nav
~~~

Expected: the rtk_nav package builds successfully.

- [ ] **Step 2: Run the package tests**

Run:

~~~
colcon test --packages-select rtk_nav
colcon test-result --verbose
~~~

Expected: no failed tests and the geometry/contracts are included in the result.

- [ ] **Step 3: Run the static-image smoke test**

Start the detector with enable_visual_correction:=true, publish a valid context, and feed the same image in three forms: original, translated along the current path axis, and translated along the path-normal axis. Confirm:

~~~
path-axis translation: lateral y remains within detector noise
path-normal translation: lateral y changes with stable sign
invalid context: z=0 and confidence=0
~~~

Use distinct image output filenames and restart the static camera publisher after replacing an image because it loads the static frame once at startup.

- [ ] **Step 4: Review the final diff and commit only scoped files**

Run:

~~~
git diff --check
git diff --stat
git status --short
~~~

Stage only the four files listed in this task and commit:

~~~
git add src/rtk_nav/rtk_nav/line_detector_node.py src/rtk_nav/launch/run.launch.py src/rtk_nav/test/test_visual_line_geometry.py src/rtk_nav/test/test_path_aware_boundary_contract.py
git commit -m "feat: use path-aware boundary geometry for visual offset"
~~~

Do not stage generated caches, logs, unrelated navigation changes, or the previously existing calibration test unless it is explicitly part of the final diff review.

