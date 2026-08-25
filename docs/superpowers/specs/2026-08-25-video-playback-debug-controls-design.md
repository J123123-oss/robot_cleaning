# Video Playback Debug Controls Design

## Goal

Extend `video_to_v4l2` so recorded-video testing can be slowed down or sped up and paused without losing the current camera image. While paused, `/dev/video0` must continue serving the last frame as a static image to `camera_publisher_node`.

## Scope

The change is limited to the existing standalone video-to-V4L2 command, its contract tests, and its Ubuntu usage documentation. The ROS camera publisher remains unchanged and continues to read `/dev/video0`.

## Design

### Playback speed

Add a `--speed` option accepting a positive floating-point value with a default of `1.0`:

- `--speed 2.0` reads and outputs the recording at approximately twice normal speed.
- `--speed 0.5` reads and outputs the recording at approximately half normal speed.
- Output dimensions and output FPS remain controlled by `--width`, `--height`, and `--fps`.

The ffmpeg command will use `-readrate SPEED` instead of the fixed `-re` flag. This keeps the existing video filter and V4L2 output format unchanged while applying speed to the input clock.

### Pause and resume

The command will run an interactive terminal controller:

- `Space` or `p` toggles pause/resume.
- `q` requests a clean stop.
- `Ctrl-C` remains a clean stop fallback.

The terminal is temporarily put into single-character mode on Linux and restored on every exit path. When pausing, the wrapper sends `SIGSTOP` to ffmpeg; when resuming, it sends `SIGCONT`.

### Static current-frame output

When the loopback module is created by this tool, the `modprobe` command will set `sustain_framerate=1`. This makes v4l2loopback repeat the most recently written frame at the configured device rate while ffmpeg is stopped, so readers continue receiving the same image instead of blocking or advancing.

If `/dev/video0` already exists, the tool will not unload or replace it automatically. The README will explain that an old loopback module must be unloaded and recreated once for the new module parameter to take effect, after stopping any consumers.

### Process lifecycle

The main process will poll ffmpeg while checking terminal input. On `q`, `Ctrl-C`, or terminal exit it resumes a paused child before sending `SIGINT`, waits briefly, and terminates only if necessary. Terminal settings are restored in a `finally` block.

## Error handling

- `--speed <= 0` is rejected by argument parsing.
- Non-interactive stdin skips keyboard polling but still supports `Ctrl-C`.
- On platforms without POSIX terminal signals, the command reports that interactive pause is only supported on Linux instead of corrupting terminal state.
- Missing ffmpeg, missing input video, missing `v4l2loopback`, and failure to create `/dev/video0` retain the existing clear error behavior.

## Testing

Contract tests will cover:

- parsing the default and custom playback speeds;
- rejecting zero and negative speeds;
- including `-readrate` in the ffmpeg command;
- including `sustain_framerate=1` in the loopback module command;
- pause/resume signal selection with a fake process;
- documentation of keyboard controls and the module reload requirement.

The current Windows workspace cannot perform the final Linux signal, kernel-module, ffmpeg, or `/dev/video0` integration test. Those steps remain for the Ubuntu test host.
