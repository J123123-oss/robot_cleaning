# Video Playback Debug Controls Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add adjustable playback speed and interactive pause/resume controls that keep `/dev/video0` outputting the last frame for camera debugging.

**Architecture:** Extend the existing `video_to_v4l2.py` wrapper instead of changing `camera_publisher_node`. ffmpeg will use `-readrate SPEED`; the wrapper will poll single-character terminal input and send `SIGSTOP`/`SIGCONT`. v4l2loopback will be created with `sustain_framerate=1`, which repeats the latest frame while ffmpeg is paused.

**Tech Stack:** Python 3, `argparse`, `termios`, `tty`, `select`, POSIX signals, ffmpeg, v4l2loopback, standard-library `unittest`.

---

### Task 1: Add failing contract tests

**Files:**
- Modify: `src/rtk_nav/test/test_video_to_v4l2_contract.py`

- [x] **Step 1: Test speed parsing and ffmpeg read rate**

  Add tests that assert the default `speed` is `1.0`, custom `--speed 2.5` is parsed as a float, zero/negative speed is rejected, and `build_ffmpeg_command` contains `-readrate 2.5` without the old `-re` flag.

- [x] **Step 2: Test loopback sustain configuration**

  Extend the fake command-runner test so the captured `modprobe` argument list contains `sustain_framerate=1`.

- [x] **Step 3: Test pause signal selection and documentation**

  Add a fake process test for a helper that sends `SIGSTOP` when entering pause and `SIGCONT` when resuming. Add README assertions for `--speed`, `Space`/`p`, `q`, and the module reload instruction.

- [x] **Step 4: Run the focused tests and verify they fail for the missing behavior**

  Run from `src/rtk_nav`:

  ```bash
  python -m unittest discover -s test -p 'test_video_to_v4l2_contract.py' -v
  ```

  Expected result: failures for missing speed parsing, `-readrate`, sustain configuration, pause helper, and documentation text.

### Task 2: Implement speed and loopback frame persistence

**Files:**
- Modify: `src/rtk_nav/rtk_nav/video_to_v4l2.py`

- [x] **Step 1: Add speed validation and command construction**

  Add `positive_float`, parse `--speed` with default `1.0`, and build the ffmpeg prefix as `-readrate SPEED`. Keep the existing crop/scale/fps/pixel-format output arguments unchanged.

- [x] **Step 2: Enable v4l2loopback frame sustain**

  Add `sustain_framerate=1` to the `modprobe v4l2loopback` command. Preserve the existing behavior when the requested device already exists, because the utility must not unload a device that may be in use.

- [x] **Step 3: Add pause signal helper**

  Implement `set_process_paused(process, paused)` using `SIGSTOP` and `SIGCONT`, with a clear runtime error on non-POSIX platforms where the signals are unavailable.

### Task 3: Implement terminal controls and process cleanup

**Files:**
- Modify: `src/rtk_nav/rtk_nav/video_to_v4l2.py`

- [x] **Step 1: Add single-key terminal polling**

  Add a context manager that uses `termios`/`tty` only when stdin is a TTY and gracefully disables keyboard controls otherwise. Poll input with `select` at a short interval, mapping `Space` and `p` to pause/resume and `q` to shutdown.

- [x] **Step 2: Replace blocking wait with a control loop**

  Start ffmpeg, print the controls, poll `process.poll()`, and toggle the paused state. On shutdown, resume a stopped process before sending `SIGINT`; wait up to five seconds, then terminate. Always restore terminal settings in `finally`.

- [x] **Step 3: Run the focused tests and verify they pass**

  Run:

  ```bash
  python -m unittest discover -s test -p 'test_video_to_v4l2_contract.py' -v
  ```

  Expected result: all video replay contract tests pass.

### Task 4: Update Ubuntu debugging documentation

**Files:**
- Modify: `src/rtk_nav/README_VIDEO_TO_V4L2.md`

- [x] **Step 1: Document speed and controls**

  Show `--speed 0.5` and explain `Space`/`p` pause/resume and `q` exit. State that pause keeps the current frame available through `/dev/video0`.

- [x] **Step 2: Document existing-device reload**

  Explain that a device created before `sustain_framerate=1` must be recreated after stopping readers, using `sudo modprobe -r v4l2loopback`, then rerunning the replay tool. Warn not to unload a physical camera driver.

### Task 5: Verify the complete change

**Files:**
- No additional files

- [x] **Step 1: Run focused and existing visual contract tests**

  Run:

  ```bash
  python -m unittest discover -s test -p 'test_video_to_v4l2_contract.py' -v
  python -c 'import importlib.util, inspect; p="test/test_visual_correction_switch_contract.py"; s=importlib.util.spec_from_file_location("visual_contract", p); m=importlib.util.module_from_spec(s); s.loader.exec_module(m); tests=[(n, f) for n, f in vars(m).items() if n.startswith("test_") and inspect.isfunction(f)]; [(name, fn()) for name, fn in tests]; print(f"PASS: {len(tests)} visual contract tests")'
  ```

- [x] **Step 2: Compile modified Python files and validate the package entry point**

  Run from `src/rtk_nav`:

  ```bash
  python -m py_compile rtk_nav/video_to_v4l2.py setup.py test/test_video_to_v4l2_contract.py
  python setup.py --name
  ```

- [x] **Step 3: Inspect the feature diff and preserve unrelated work**

  Run:

  ```bash
  git diff --check
  git diff -- src/rtk_nav/rtk_nav/video_to_v4l2.py src/rtk_nav/test/test_video_to_v4l2_contract.py src/rtk_nav/README_VIDEO_TO_V4L2.md
  git status --short
  ```

  Confirm only the feature files changed as a result of this task; do not stage or revert unrelated worktree changes.
