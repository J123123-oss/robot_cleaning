# Grid-Line RTSP Streaming Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Stream `/grid_line/detected_image` as a low-latency H.264 RTSP feed to a configured media server so the backend can pull its HTTP-FLV playback URL.

**Architecture:** Add a ROS 2 node that subscribes to the annotated image and delegates frame pacing, FFmpeg command construction, latest-frame replacement, and child-process recovery to a small standard-library core. The node converts BGR frames to RGB before writing them to an FFmpeg subprocess; FFmpeg publishes RTSP over TCP. Launch integration is opt-in and does not change the detector or motor-control path.

**Tech Stack:** Python 3, ROS 2 jazzy `rclpy`, `sensor_msgs/msg/Image`, `cv_bridge`, FFmpeg, standard-library threading/subprocess/urllib, pytest/unittest contract tests.

**Spec:** `docs/superpowers/specs/2026-09-09-grid-line-rtsp-streaming-design.md`

## Global Constraints

- Stream source is `/grid_line/detected_image` with type `sensor_msgs/msg/Image`.
- The robot publishes to a configurable RTSP URL; the backend consumes the media server's HTTP-FLV playback URL.
- The streamer is enabled by default and targets `rtsp://127.0.0.1:8554/live/grid_line`.
- The frame buffer depth is one and old frames are replaced rather than queued.
- FFmpeg is launched without `shell=True`, and runtime URLs/credentials are not committed or logged in full.
- Existing visual correction and motor-control behavior remains unchanged.
- Unit tests must not require ROS hardware, an RTSP server, or a running FFmpeg encoder.

---

## File Map

- Create `src/rtk_nav/rtk_nav/grid_line_streamer_core.py`: ROS-independent URL validation, FFmpeg command construction, latest-frame buffer, and bounded FFmpeg process lifecycle.
- Create `src/rtk_nav/rtk_nav/grid_line_streamer.py`: ROS 2 node adapter, parameter handling, image conversion, subscription, worker thread, and shutdown.
- Create `src/rtk_nav/test/test_grid_line_streamer_contract.py`: pure-core unit tests plus source/launch/setup/documentation contract checks.
- Modify `src/rtk_nav/setup.py`: register `grid_line_streamer`.
- Modify `src/rtk_nav/launch/run.launch.py`: add opt-in streaming arguments and node.
- Modify `src/rtk_nav/launch/camera_indoor_test.launch.py`: expose the same stream options for indoor testing.
- Create `src/rtk_nav/README_GRID_LINE_RTSP_STREAMING.md`: Ubuntu dependency, launch examples, media-server contract, and troubleshooting.

`src/rtk_nav/package.xml` does not change because FFmpeg is an external Ubuntu runtime dependency, not a Python or ROS package dependency.

### Task 1: Add failing core contract tests

**Files:**
- Create: `src/rtk_nav/test/test_grid_line_streamer_contract.py`
- Reference: `src/rtk_nav/rtk_nav/grid_line_streamer_core.py`

**Interfaces produced for later tasks:**

```python
def validate_rtsp_url(value: str) -> str:
    """Return a normalized RTSP URL or raise ValueError."""

def build_ffmpeg_command(
    ffmpeg_path: str,
    width: int,
    height: int,
    fps: float,
    bitrate: str,
    preset: str,
    rtsp_url: str,
) -> list[str]:
    """Build an argv list for raw RGB input and RTSP output."""

class LatestFrameBuffer:
    def put(self, frame) -> None: ...
    def take(self): ...
    def wait_for_first(self, stop_event, timeout: float): ...

class FfmpegStreamWorker:
    def __init__(self, command_factory, popen_factory, reconnect_sec, sleep_fn): ...
    def write_frame(self, process, frame) -> None: ...
    def close_process(self, process) -> None: ...
```

- [ ] **Step 1: Write tests for RTSP URL validation and command construction.**

  Load the core module from its file path and assert that `rtsp://server:8554/live/grid_line` is accepted, empty/non-RTSP URLs raise `ValueError`, and the command contains these exact stages:

  ```python
  command = build_ffmpeg_command(
      "ffmpeg", 640, 360, 10.0, "800k", "veryfast",
      "rtsp://server:8554/live/grid_line",
  )
  assert command[:4] == ["ffmpeg", "-hide_banner", "-loglevel", "warning"]
  assert command[command.index("-f") + 1] == "rawvideo"
  assert command[command.index("-pix_fmt") + 1] == "rgb24"
  assert "-s" in command and command[command.index("-s") + 1] == "640x360"
  assert "-r" in command and command[command.index("-r") + 1] == "10"
  assert "-c:v" in command and command[command.index("-c:v") + 1] == "libx264"
  assert "-tune" in command and command[command.index("-tune") + 1] == "zerolatency"
  assert command[-5:] == [
      "-f", "rtsp", "-rtsp_transport", "tcp",
      "rtsp://server:8554/live/grid_line",
  ]
  assert "shell=True" not in inspect.getsource(build_ffmpeg_command)
  ```

- [ ] **Step 2: Write tests for latest-frame replacement.**

  Put `frame-1`, then `frame-2`, and assert `take()` returns only `frame-2`. Verify an empty buffer returns `None` without blocking when `take()` is called.

- [ ] **Step 3: Write tests for process writes and cleanup.**

  Use a fake process with `stdin.write`, `stdin.close`, `terminate`, `wait`, and `poll`. Assert `write_frame()` writes `frame.tobytes()` once and flushes. Assert `close_process()` closes stdin, terminates a live process, and waits with a bounded timeout.

- [ ] **Step 4: Run the new test file and verify it fails for missing implementation.**

  Run from `src/rtk_nav`:

  ```bash
  python -m pytest test/test_grid_line_streamer_contract.py -q
  ```

  Expected result: collection/import failures because the core module and interfaces do not exist yet.

### Task 2: Implement the ROS-independent streaming core

**Files:**
- Create: `src/rtk_nav/rtk_nav/grid_line_streamer_core.py`
- Test: `src/rtk_nav/test/test_grid_line_streamer_contract.py`

**Interfaces consumed:** the signatures defined in Task 1.

**Interfaces produced:** the completed helpers and classes used by the ROS node in Task 4.

- [ ] **Step 1: Implement `validate_rtsp_url`.**

  Parse with `urllib.parse.urlparse`, require scheme `rtsp` and a non-empty `netloc`, preserve the path/query, and return the original string stripped of surrounding whitespace. Raise `ValueError("rtsp_url must be a valid rtsp:// URL")` for empty, malformed, or non-RTSP values.

- [ ] **Step 2: Implement `build_ffmpeg_command`.**

  Validate positive width/height/fps and a non-empty bitrate/preset before returning this argument sequence:

  ```text
  ffmpeg -hide_banner -loglevel warning
    -f rawvideo -pix_fmt rgb24 -s WIDTHxHEIGHT -r FPS -i pipe:0
    -an -c:v libx264 -preset PRESET -tune zerolatency
    -pix_fmt yuv420p -b:v BITRATE -f rtsp -rtsp_transport tcp RTSP_URL
  ```

  Format integral FPS as `10`, non-integral FPS as a stable decimal, and return only a list. Do not invoke a process or use a shell in this helper.

- [ ] **Step 3: Implement `LatestFrameBuffer`.**

  Use a `threading.Condition` and one private frame slot. `put(frame)` replaces the slot and notifies waiters. `take()` returns the current slot and clears it. `wait_for_first(stop_event, timeout)` waits in short intervals until a frame arrives, the stop event is set, or the deadline expires; return `None` on stop/timeout. Never retain more than one pending frame.

- [ ] **Step 4: Implement `FfmpegStreamWorker` process helpers.**

  Keep subprocess ownership in this class. `write_frame()` calls `process.stdin.write(frame.tobytes())` and `flush()`, allowing `BrokenPipeError`/`OSError` to reach the caller. `close_process()` closes stdin if available, terminates a live child, waits up to five seconds, and kills only if the child remains alive. All operations must tolerate an already-exited child.

- [ ] **Step 5: Run the focused core tests.**

  Run:

  ```bash
  python -m pytest test/test_grid_line_streamer_contract.py -q
  ```

  Expected result: all tests covering URL validation, command construction, frame replacement, writes, and bounded cleanup pass. ROS imports and launch tests remain skipped until their later tasks are implemented.

### Task 3: Add failure/reconnect behavior to the core

**Files:**
- Modify: `src/rtk_nav/rtk_nav/grid_line_streamer_core.py`
- Modify: `src/rtk_nav/test/test_grid_line_streamer_contract.py`

**Interfaces produced:**

```python
def run(self, frame_buffer, stop_event, frame_period, command_for_shape):
    """Pace the latest frame, restart FFmpeg after failure, and stop cleanly."""
```

- [ ] **Step 1: Add a fake-process reconnection test.**

  Create a fake process whose first `stdin.write()` raises `BrokenPipeError`, then provide a second fake process through `popen_factory`. Run the worker loop with a stop event that is set after the second process receives one frame. Assert the first process is closed, the second command is launched, and no queued history is replayed. Add a second fake factory that raises `FileNotFoundError` and assert the worker keeps the ROS process alive while waiting for the next retry.

- [ ] **Step 2: Implement bounded worker-loop recovery.**

  Start only after `wait_for_first()` returns a frame. Derive `(height, width)` from `frame.shape[:2]`, launch the process through `popen_factory(command, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)`, and write at most the newest frame once per `frame_period`. Keep the last current frame for pacing, but replace it immediately when `take()` returns a newer frame. On `BrokenPipeError`, `OSError`, or a non-`None` `poll()` result, close the child, wait `reconnect_sec` through `stop_event.wait()`, and retry from the newest available frame. If dimensions change, close and relaunch with the new command.

- [ ] **Step 3: Add process-exit and shutdown tests.**

  Assert a non-zero `poll()` causes cleanup and retry. Assert setting the stop event exits the loop, closes the current process, and does not sleep for the full reconnect delay.

- [ ] **Step 4: Run the core test suite.**

  Run:

  ```bash
  python -m pytest test/test_grid_line_streamer_contract.py -q
  ```

  Expected result: all pure-core tests pass without launching real FFmpeg.

### Task 4: Add the ROS 2 streamer node

**Files:**
- Create: `src/rtk_nav/rtk_nav/grid_line_streamer.py`
- Modify: `src/rtk_nav/test/test_grid_line_streamer_contract.py`

**Interfaces consumed:** `LatestFrameBuffer`, `FfmpegStreamWorker`, `build_ffmpeg_command`, and `validate_rtsp_url` from `grid_line_streamer_core.py`.

**Interfaces produced:** console entry point `grid_line_streamer` and ROS parameters `rtsp_url`, `fps`, `bitrate`, `preset`, `reconnect_sec`, `ffmpeg_path`, and `topic`.

- [ ] **Step 1: Add source-level node contract tests.**

  Read the node source and assert it contains `sensor_msgs.msg.Image`, `/grid_line/detected_image`, `QoSProfile` with `KEEP_LAST`/depth `1`/`BEST_EFFORT`, `CvBridge`, a worker thread, `build_ffmpeg_command`, `FfmpegStreamWorker`, and a `destroy_node()` cleanup path. Read the core source separately for `subprocess.Popen`. Assert the subscription callback only places the message into the latest-frame buffer and does not call FFmpeg or network APIs.

- [ ] **Step 2: Implement parameter parsing and validation.**

  Declare the six streaming parameters plus `topic` defaulting to `/grid_line/detected_image`. Validate `rtsp_url` using the core helper, require finite positive `fps` and `reconnect_sec`, and require non-empty bitrate/preset/FFmpeg path. Raise a clear `ValueError` during startup for invalid configuration.

- [ ] **Step 3: Implement the latest-frame ROS callback.**

  Subscribe with a `QoSProfile` matching the detector's debug publisher. The callback calls `self.frame_buffer.put(message)` and performs no image conversion, blocking I/O, or child-process operations.

- [ ] **Step 4: Implement the worker adapter.**

  Convert a ROS message to `bgr8` with `CvBridge` inside the worker. Reject unsupported/malformed frames with throttled warnings. Build a command from the first frame's dimensions, pass it to `FfmpegStreamWorker`, and use `numpy`/OpenCV frame arrays only in the worker thread. Start the worker as a daemon and keep ROS spinning independently.

- [ ] **Step 5: Implement shutdown.**

  Set the stop event, wake the frame buffer, let the worker close FFmpeg, join it for at most five seconds, and then call the base `destroy_node()`. `main()` must destroy the node and call `rclpy.shutdown()` in `finally`.

- [ ] **Step 6: Compile the node and run source contracts.**

  Run:

  ```bash
  python -m py_compile rtk_nav/grid_line_streamer_core.py rtk_nav/grid_line_streamer.py
  python -m pytest test/test_grid_line_streamer_contract.py -q
  ```

  Expected result: the node compiles and all core/node contract tests pass. Running the node itself is not required on Windows.

### Task 5: Register the console script and launch configuration

**Files:**
- Modify: `src/rtk_nav/setup.py`
- Modify: `src/rtk_nav/launch/run.launch.py`
- Modify: `src/rtk_nav/launch/camera_indoor_test.launch.py`
- Modify: `src/rtk_nav/test/test_grid_line_streamer_contract.py`

- [ ] **Step 1: Add failing setup/launch contract assertions.**

  Assert `setup.py` contains `grid_line_streamer = rtk_nav.grid_line_streamer:main`. Assert both launch files declare `enable_grid_line_stream`, `grid_line_stream_rtsp_url`, `grid_line_stream_fps`, `grid_line_stream_bitrate`, `grid_line_stream_preset`, `grid_line_stream_reconnect_sec`, and `ffmpeg_path`, with `enable_grid_line_stream` defaulting to `true`, the RTSP URL defaulting to `rtsp://127.0.0.1:8554/live/grid_line`, and a conditional `executable='grid_line_streamer'` node.

- [ ] **Step 2: Register the console script.**

  Add the entry point beside the existing `line_detector_node` and `openmv_serial_publisher_node` entries. Do not add FFmpeg to `package.xml`.

- [ ] **Step 3: Add normal-launch arguments and node.**

  Declare the seven arguments with these defaults: `true`, `rtsp://127.0.0.1:8554/live/grid_line`, `10.0`, `800k`, `veryfast`, `2.0`, and `ffmpeg`. Add a node conditional on `enable_grid_line_stream`, pass all parameters, and document in the argument descriptions that visual correction and `publish_debug_images:=true` are required for `/grid_line/detected_image` to be generated.

- [ ] **Step 4: Add indoor-launch arguments and node.**

  Mirror the same declarations and node parameter mapping in `camera_indoor_test.launch.py`, whose line detector is always present. Keep the indoor launch's existing defaults and node order otherwise unchanged.

- [ ] **Step 5: Run launch/setup contracts and compile.**

  Run:

  ```bash
  python -m pytest test/test_grid_line_streamer_contract.py -q
  python -m py_compile setup.py launch/run.launch.py launch/camera_indoor_test.launch.py
  ```

  Expected result: the registered entry point and both opt-in launch configurations pass their source contracts.

### Task 6: Document operation and backend URL mapping

**Files:**
- Create: `src/rtk_nav/README_GRID_LINE_RTSP_STREAMING.md`
- Modify: `src/rtk_nav/test/test_grid_line_streamer_contract.py`

- [ ] **Step 1: Add documentation contract assertions.**

  Assert the README contains the FFmpeg install command, `/grid_line/detected_image`, an RTSP publish example, an HTTP-FLV playback example, `enable_grid_line_stream`, `publish_debug_images:=true`, reconnect behavior, and the statement that the HTTP-FLV URL is not the robot's RTSP push target.

- [ ] **Step 2: Write the Ubuntu setup and launch procedure.**

  Document:

  ```bash
  sudo apt update
  sudo apt install ffmpeg
  source install/setup.bash
  ros2 launch rtk_nav run.launch.py \
    enable_visual_correction:=true \
    publish_debug_images:=true \
    enable_grid_line_stream:=true \
    grid_line_stream_rtsp_url:=rtsp://media-server:8554/live/grid_line
  ```

  Explain that the media server must convert the RTSP publish path to the backend playback URL, for example `http://media-server/live/grid_line.flv`. Do not put real hostnames, passwords, or tokens in the repository.

- [ ] **Step 3: Document troubleshooting.**

  Include checks for `which ffmpeg`, ROS topic presence, media-server RTSP ingest, and backend HTTP-FLV playback. State that the streamer drops old frames, retries after connection failures, and cannot work when `publish_debug_images:=false` or the visual detector is disabled.

- [ ] **Step 4: Run the documentation contract.**

  Run:

  ```bash
  python -m pytest test/test_grid_line_streamer_contract.py -q
  ```

  Expected result: all documentation assertions pass.

### Task 7: Run the complete verification suite

**Files:**
- No additional files.

- [ ] **Step 1: Run focused streamer tests and existing visual contracts.**

  Run:

  ```bash
  python -m pytest \
    test/test_grid_line_streamer_contract.py \
    test/test_video_to_v4l2_contract.py \
    test/test_openmv_serial_publisher_contract.py \
    test/test_visual_correction_switch_contract.py \
    test/test_line_detector_relative_offset.py \
    -q
  ```

  Expected result: all selected tests pass; failures in unrelated pre-existing worktree tests must be reported separately rather than reverted.

- [ ] **Step 2: Compile all modified Python files.**

  Run:

  ```bash
  python -m py_compile \
    rtk_nav/grid_line_streamer_core.py \
    rtk_nav/grid_line_streamer.py \
    setup.py \
    launch/run.launch.py \
    launch/camera_indoor_test.launch.py
  ```

- [ ] **Step 3: Check whitespace and inspect the scoped diff.**

  Run:

  ```bash
  git diff --check
  git diff -- \
    src/rtk_nav/rtk_nav/grid_line_streamer_core.py \
    src/rtk_nav/rtk_nav/grid_line_streamer.py \
    src/rtk_nav/setup.py \
    src/rtk_nav/launch/run.launch.py \
    src/rtk_nav/launch/camera_indoor_test.launch.py \
    src/rtk_nav/README_GRID_LINE_RTSP_STREAMING.md \
    src/rtk_nav/test/test_grid_line_streamer_contract.py
  git status --short
  ```

  Confirm unrelated existing changes remain untouched.

- [ ] **Step 4: Perform deployment-level verification on Ubuntu.**

  With the actual media server configured, verify:

  ```bash
  ros2 topic hz /grid_line/detected_image
  ffplay http://media-server/live/grid_line.flv
  ```

  Confirm that the backend can pull the same HTTP-FLV URL and that reconnecting the media server does not stop the ROS launch. This step cannot be completed in the current Windows workspace without the deployment and media server.
