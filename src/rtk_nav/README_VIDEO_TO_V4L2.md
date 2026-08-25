# Recorded Video To `/dev/video0`

This tool replays a recorded video into a Linux V4L2 loopback device. The existing
`camera_publisher_node` can then read the replay as if it were a physical camera.

## Install Ubuntu dependencies

```bash
sudo apt update
sudo apt install ffmpeg v4l2loopback-dkms v4l2loopback-utils
```

The replay command loads `v4l2loopback` automatically when `/dev/video0` is
missing. It does not install system packages automatically.

## Build

From the workspace root:

```bash
colcon build --symlink-install --packages-select rtk_nav
source install/setup.bash
```

## Replay a video

Start the replay process and keep it running:

```bash
ros2 run rtk_nav video_to_v4l2 \
  --video /path/to/recorded.mp4 \
  --device /dev/video0 \
  --width 640 \
  --height 480 \
  --fps 30 \
  --speed 0.5 \
  --loop
```

The output keeps the requested size. It preserves the source aspect ratio and
center-crops the excess area, so the output has no black border and no
non-uniform stretch. `--speed 1.0` is normal speed, `--speed 2.0` is twice
speed, and `--speed 0.5` is half speed.

The replay process accepts single-key controls in the terminal where it is
running:

- Press `Space` or `p` to pause/resume.
- Press `q` to quit.
- Press `Ctrl-C` to stop.

When paused, ffmpeg stops decoding and v4l2loopback repeats the last submitted
frame with `sustain_framerate=1`. The ROS camera publisher therefore continues
receiving a static image instead of a blank frame or a changing video frame.

Then start the existing camera publisher in another terminal:

```bash
source install/setup.bash
ros2 run rtk_nav camera_publisher_node --ros-args \
  -p device_id:=0 \
  -p width:=640 \
  -p height:=480 \
  -p fps:=30 \
  -p use_gstreamer:=false
```

Check that the ROS image topic receives frames:

```bash
ros2 topic hz /camera/color/image_raw
ros2 topic echo /camera/color/image_raw --once
```

With `exclusive_caps=1`, `/dev/video0` may initially report an output-only
capability. Start the replay process first; after `ffmpeg` opens the device, the
camera publisher can read it as a capture device.

If `/dev/video0` was created before this tool enabled `sustain_framerate=1`,
stop the replay and all camera consumers, then recreate the virtual device:

```bash
sudo modprobe -r v4l2loopback
```

Run that command only when `/dev/video0` is the virtual loopback device. Do not
unload a physical camera driver. The next replay command will load
`v4l2loopback` again with the frame-sustain option.

## Options

- `--video`: recorded video path, required.
- `--device`: V4L2 output device, default `/dev/video0`.
- `--width`, `--height`: fixed output dimensions, defaults `640` and `480`.
- `--fps`: output frame rate, default `30`.
- `--speed`: playback speed multiplier, default `1.0`.
- `--loop`: restart the video when it reaches the end.
- `--no-create-device`: require the V4L2 device to already exist.

Stop the replay with `Ctrl-C`. If the device is busy, stop any previous replay
process before starting another one.

If an older version is forcibly terminated and the shell no longer echoes
input, type the following command and press Enter:

```bash
stty sane
```
