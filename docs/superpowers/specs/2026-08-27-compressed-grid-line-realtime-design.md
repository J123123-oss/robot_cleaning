# Compressed Grid-Line Realtime Detection Design

## Goal

Keep the video replay, camera publisher, and dynamic grid-line detector on a
single `640x480 @ 30 FPS` contract while replacing the ROS raw image transport
with JPEG-compressed image messages and preventing stale-frame backlog.

## Scope and constraints

- The source video is portrait `720x1280`.
- The video is not rotated.
- The output must remain `640x480`; aspect ratio is preserved by scaling to
  cover and center-cropping, so there is no non-uniform stretch or padding.
- `/dev/video0` remains a raw V4L2/YUYV interface. Compression applies to the
  ROS camera topic after the camera publisher reads the V4L2 frame.
- The existing visual correction topics and result semantics remain unchanged.
- Existing unrelated worktree changes must not be included.

## Architecture

`video_to_v4l2` produces `640x480` YUYV frames at 30 FPS using an
aspect-preserving scale and center crop. `camera_publisher_node` reads those
frames and publishes `sensor_msgs/msg/CompressedImage` on
`/camera/color/image_compressed` using JPEG quality 80. The publisher uses a
latest-frame appsink configuration where GStreamer is available.

`line_detector_node` subscribes to the compressed topic with a sensor-data QoS
profile and depth 1. Its image callback only decodes and stores the newest
frame; a timer processes the newest frame at the configured detection rate.
The detector never waits for or processes old frames. The default target is
30 FPS, but the node reports the measured processing time and effective rate;
if processing takes longer than 33 ms, it continues with the newest frame and
reports the actual rate rather than accumulating latency.

High-bandwidth debug image topics are disabled by default and can be enabled
with a separate low rate. The angle, lateral error, confidence, and run-axis
topics keep their existing names and message types.

## Data flow

```text
720x1280 video
  -> ffmpeg: scale-to-cover + center crop, no rotation
  -> /dev/video0: 640x480 YUYV @ 30 FPS
  -> camera publisher: BGR frame -> JPEG quality 80
  -> /camera/color/image_compressed: CompressedImage @ target 30 FPS
  -> line detector: depth-1 latest frame -> JPEG decode -> detection
  -> existing grid-line result topics
```

## Configuration

The following parameters are explicit and consistent across the components:

| Component | Parameter | Default |
| --- | --- | ---: |
| video replay | width | 640 |
| video replay | height | 480 |
| video replay | fps | 30 |
| camera publisher | width | 640 |
| camera publisher | height | 480 |
| camera publisher | fps | 30 |
| camera publisher | jpeg_quality | 80 |
| line detector | detection_fps | 30.0 |
| line detector | publish_debug_images | false |
| line detector | debug_image_fps | 1.0 |

When reading `/dev/video0`, `image_path` must be empty. A non-empty static
image path remains available for static-image tests but is not the runtime
video path.

## Performance and correctness checks

The implementation must include tests that verify:

1. The replay filter preserves the fixed `640x480` output without rotation.
2. The camera publisher constructs a compressed image with the configured
   JPEG quality and publishes the compressed topic.
3. The line detector decodes compressed messages, stores only the latest
   frame, and does not call the expensive detector from the subscription
   callback.
4. Debug images are disabled or rate-limited independently from result
   publication.
5. The relevant Python files compile and the existing visual contract tests
   continue to pass.

On Ubuntu, runtime verification uses:

```bash
ros2 topic hz /camera/color/image_compressed
ros2 topic hz /grid_line/angle_deviation
ros2 topic echo /camera/color/image_compressed --once
```

The target is at least 27 FPS for compressed camera messages and at least 25
FPS for detector results during a 30-second run. If the detector's measured
processing time exceeds 40 ms, the result is a measured throughput limitation
that requires further algorithm reduction; the latest-frame policy must still
keep latency bounded.
