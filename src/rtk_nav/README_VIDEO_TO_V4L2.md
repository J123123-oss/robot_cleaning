# Video to V4L2 and compressed vision pipeline

The recorded video path is portrait-preserving:

```text
720x1280 video --scale without crop/rotation--> /dev/video0 (YUYV 360x640 @ 30 FPS)
    --> camera_publisher --JPEG quality 80--> /camera/color/image_compressed
    --> grid_line_detector (latest frame only) --> /grid_line/angle_deviation
```

The V4L2 device remains raw YUYV. JPEG compression is only used for the ROS
image topic. The expected 720x1280 input is scaled directly to 360x640, so the
complete portrait image is retained.

## Ubuntu test

```bash
cd ~/robot_cleaning
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select rtk_nav
source install/setup.bash

python3 -m rtk_nav.video_to_v4l2 --video p.mp4 --loop
ros2 launch rtk_nav run.launch.py enable_visual_correction:=true
ros2 topic hz /camera/color/image_compressed
ros2 topic hz /grid_line/angle_deviation
ros2 topic echo /camera/color/image_compressed --once
```

The replay command supports `--speed`, `--loop`, Space or `p` to pause/resume,
`q` to quit, and Ctrl-C to stop. Pause sends no new video frames, so the
v4l2loopback `sustain_framerate=1` option is used when the module is created to
keep the last frame visible to readers.

Debug image topics are disabled by default. Enable them at most once per
second when needed:

```bash
ros2 launch rtk_nav run.launch.py enable_visual_correction:=true publish_debug_images:=true debug_image_fps:=1.0
```
