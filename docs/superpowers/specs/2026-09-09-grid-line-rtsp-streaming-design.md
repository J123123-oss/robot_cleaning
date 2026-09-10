# Grid-Line Detection RTSP Streaming Design

## Goal

Expose the annotated grid-line detection result to the backend video player.
The robot publishes the ROS image topic `/grid_line/detected_image` as an
H.264 RTSP stream to an existing media server. The media server exposes the
corresponding HTTP-FLV playback URL, which is entered into the backend
monitoring address field.

## Confirmed Requirements

- The backend player pulls the final playback URL; the robot does not push to
  the HTTP-FLV URL shown by the backend UI.
- The streamed source is `/grid_line/detected_image`.
- The robot-side transport from the streamer to the media server is RTSP.
- The backend/media-server deployment is outside this repository and must
  provide an RTSP publish endpoint plus an HTTP-FLV playback endpoint.
- Existing visual correction and motor-control behavior must remain unchanged.
- Streaming must not accumulate stale frames or block the detection timer.

## Scope

### In scope

- Add an independent ROS 2 streamer node in the `rtk_nav` package.
- Subscribe to `sensor_msgs/msg/Image` on `/grid_line/detected_image` with a
  latest-frame QoS/queue policy.
- Convert the BGR image to raw frames for an FFmpeg subprocess.
- Encode H.264 with low-latency settings and publish to a configurable RTSP
  URL using TCP transport.
- Reconnect the FFmpeg process after a broken pipe or non-zero exit without
  stopping the ROS process.
- Add launch arguments and a disabled-by-default launch action.
- Document the external FFmpeg dependency and the relationship between the
  RTSP publish URL and the HTTP-FLV playback URL.
- Add unit/contract tests for command construction, latest-frame behavior,
  process failure handling, and launch configuration.

### Out of scope

- Implementing or deploying the backend media server.
- Generating the HTTP-FLV URL in the robot software.
- Streaming the raw camera topic or the gray/binary/edge debug topics.
- Adding authentication storage or credentials to source code.
- Changing the visual detection algorithm or control outputs.

## Existing Data Flow

```text
OpenMV JPEG
  -> /camera/color/image/compressed
  -> GridLineDetector
  -> /grid_line/detected_image (sensor_msgs/msg/Image, bgr8)
```

The detector currently generates debug images only when debug image publication
is enabled and a subscriber exists. The streamer is therefore a real subscriber
and the launch configuration must keep detected-image publication enabled when
streaming is enabled. This dependency must be explicit in the launch contract
and documentation; it must not be hidden in the streamer.

## Proposed Architecture

### `grid_line_streamer` node

Add `rtk_nav/grid_line_streamer.py` and register a
`grid_line_streamer` console entry point.

The node has one ROS callback and one worker loop:

1. The callback stores the newest `Image` message and replaces any older
   pending message. It performs no network I/O and does not wait for FFmpeg.
2. The worker converts the newest message to `bgr8`, starts FFmpeg after the
   first valid frame establishes the input dimensions, and writes raw BGR
   frames to FFmpeg stdin.
3. If the worker cannot write because FFmpeg or the RTSP connection failed, it
   closes the process, waits for the configured reconnect delay, and retries
   using the newest available frame.
4. On node destruction, the worker stops accepting frames, closes stdin,
   terminates FFmpeg, and joins within a bounded timeout.

The queue depth is one. Dropping an old frame is intentional: a monitoring
stream should show the current detection state instead of replaying stale
frames after a network stall.

### FFmpeg pipeline

The command is built as an argument list and launched without a shell:

```text
ffmpeg -hide_banner -loglevel warning
  -f rawvideo -pix_fmt bgr24 -s WIDTHxHEIGHT -r FPS -i pipe:0
  -an -c:v libx264 -preset veryfast -tune zerolatency
  -pix_fmt yuv420p -b:v BITRATE -f rtsp -rtsp_transport tcp RTSP_URL
```

The input dimensions come from the ROS image. The frame rate, bitrate, FFmpeg
binary, and RTSP URL are ROS parameters. The URL may contain runtime
credentials, so it must not be committed or printed in full logs.

### Launch integration

Add a launch argument and conditional node to both the normal and indoor test
launches, following the existing package conventions:

| Parameter | Default | Purpose |
| --- | --- | --- |
| `enable_grid_line_stream` | `true` | Start the streamer node |
| `grid_line_stream_rtsp_url` | `rtsp://127.0.0.1:8554/live/grid_line` | RTSP publish endpoint |
| `grid_line_stream_fps` | `10.0` | Encoded stream frame rate |
| `grid_line_stream_bitrate` | `800k` | H.264 target bitrate |
| `grid_line_stream_preset` | `veryfast` | FFmpeg encoder preset |
| `grid_line_stream_reconnect_sec` | `2.0` | Retry delay after failure |
| `ffmpeg_path` | `ffmpeg` | FFmpeg executable |

The streamer is enabled by default and targets the local ZLMediaKit RTSP
endpoint. When enabled, launch validation must reject an empty RTSP URL with a clear error. The existing
`publish_debug_images` setting remains the source of truth for whether the
detected image is generated; the documentation and tests must make the
required combination explicit.

## Error Handling

- Missing FFmpeg: log a throttled actionable error and keep ROS alive; do not
  crash motor or navigation nodes.
- Empty or invalid RTSP URL: fail the streamer node configuration before
  starting the worker.
- No image received: wait without spawning a process.
- Broken pipe, FFmpeg exit, or connection failure: tear down the process and
  retry with a bounded delay.
- Unsupported image encoding or malformed dimensions: log the frame error and
  continue with later frames.
- Shutdown: terminate the child process and restore all worker resources even
  if FFmpeg is already gone.

## Testing

Tests must not require ROS hardware, an RTSP server, or a running FFmpeg
encoder. They will verify:

1. The FFmpeg command uses raw BGR input, low-latency H.264, RTSP TCP output,
   the configured dimensions/rate/bitrate, and no `shell=True`.
2. An image callback replaces an older pending frame instead of building a
   backlog.
3. A fake FFmpeg process receives frames and is restarted after a broken pipe
   or non-zero exit.
4. Shutdown closes the process within the bounded cleanup path.
5. Launch files expose the streamer arguments, keep the node disabled by
   default, and pass the configured RTSP URL to the node.
6. Existing line detector and visual correction contract tests continue to
   pass.

Runtime verification on Ubuntu will additionally check that the media server
receives the RTSP stream and that the backend can pull its HTTP-FLV URL. Those
checks require deployment-specific server addresses and are not part of the
repository unit test suite.

## Operational Contract

The backend configuration must use the HTTP-FLV playback address generated by
the media server, for example:

```text
http://media-server/live/grid_line.flv
```

The robot configuration must use the matching RTSP publish address, for
example:

```text
rtsp://media-server:8554/live/grid_line
```

The exact paths and ports are media-server-specific and remain runtime
configuration. No credentials or deployment URLs will be added to the
repository.
