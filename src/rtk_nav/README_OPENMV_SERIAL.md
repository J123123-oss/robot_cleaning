# OpenMV H7 Plus USB 图像传输

本方案让 OpenMV H7 Plus 使用 MicroPython 采集 JPEG，通过 USB 虚拟串口
发送到 Ubuntu，再由 ROS 2 节点发布压缩图像。

```text
OpenMV H7 Plus -> /dev/ttyACM0 -> openmv_serial_publisher_node
                                      -> /camera/color/image/compressed
                                      -> line_detector_node
```

## 1. Ubuntu 准备

```bash
sudo apt update
sudo apt install python3-serial
sudo usermod -aG dialout $USER
```

注销并重新登录后，连接 OpenMV USB 数据线并检查设备：

```bash
ls -l /dev/ttyACM* /dev/ttyUSB*
```

OpenMV 直连通常是 `/dev/ttyACM0`。如果使用 USB 转串口模块，可能是
`/dev/ttyUSB0`。

## 2. OpenMV IDE

在 OpenMV IDE 中打开仓库内的：

```text
src/rtk_nav/rtk_nav/openmv_camera_stream.py
```

先点击运行按钮确认图像采集正常，然后将脚本保存到开发板为：

```text
main.py
```

脚本默认使用 `320x240`、JPEG 质量 70、10 FPS。图像帧格式为：

```text
OMV1 + version + type + sequence + payload_length + JPEG + CRC32
```

启动 Ubuntu ROS 节点前，关闭 OpenMV IDE 对串口的占用。

### 补光灯握手单独调试

在主机节点未运行时，将 `src/rtk_nav/rtk_nav/openmv_fill_light_debug.py` 在
OpenMV IDE 中打开，并下载到开发板为 `main.py`。该脚本不初始化摄像头，也不发送
图像数据，只通过 USB 输出补光灯握手日志。

串口助手中发送以下命令时必须带换行（`LF` 或 `CRLF` 均可）：

```text
LIGHT_OFF
LIGHT_ON
LIGHT_HEARTBEAT
```

预期看到：

```text
[FillLight] RX LIGHT_OFF -> OFF duty_u16=0
[FillLight] RX LIGHT_ON -> ON duty_u16=32767
[FillLight] RX LIGHT_HEARTBEAT -> ON duty_u16=32767
```

发送 `LIGHT_ON` 或 `LIGHT_HEARTBEAT` 后超过约 `2` 秒没有继续发送心跳，预期看到：

```text
[FillLight] HEARTBEAT_TIMEOUT -> OFF duty_u16=0
```

验证完成后，再把 `openmv_camera_stream.py` 下载为 `main.py`，恢复图像传输。

补光灯使用 Light Shield 的 `P6`，默认由主机节点控制：节点连接后发送开启命令并
每 `0.5` 秒发送一次心跳；节点正常退出会发送关闭命令。即使主机异常退出但 USB
仍保持连接，OpenMV 超过约 `2` 秒未收到心跳也会自动关闭补光灯。仅在 OpenMV IDE
中单独运行脚本时，补光灯保持关闭。

## 3. 构建并启动

```bash
cd ~/robot_cleaning
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select rtk_nav
source install/setup.bash
```

单独启动串口图像节点：

```bash
ros2 run rtk_nav openmv_serial_publisher_node --ros-args \
  -p serial_port:=/dev/ttyACM0 \
  -p baudrate:=115200
```

如果暂时不通过已安装的 ROS2 命令启动，也可以直接运行源码文件；此时必须
保留 `--ros-args`：

```bash
source /opt/ros/humble/setup.bash
/usr/bin/python \
  ~/robot_cleaning/src/rtk_nav/rtk_nav/openmv_serial_publisher_node.py \
  --ros-args \
  -p serial_port:=/dev/ttyACM0 \
  -p baudrate:=115200
```

使用总启动文件时：

```bash
ros2 launch rtk_nav run.launch.py \
  camera_serial_port:=/dev/ttyACM0 \
  camera_serial_baud:=115200 \
  camera_serial_no_data_timeout:=5.0 \
  enable_visual_correction:=true
```

总启动文件固定使用 OpenMV 串口节点，不再启动 `camera_publisher_node`。
识线节点与图像查看器共用标准压缩传输话题。

## 4. 验证 ROS 图像

```bash
ros2 topic hz /camera/color/image/compressed
ros2 topic echo /camera/color/image/compressed --once
ros2 run rqt_image_view rqt_image_view
```

在 `rqt_image_view` 中选择基础话题 `/camera/color/image`，传输方式选择
`compressed`。不要把 `/camera/color/image/compressed` 当作基础话题再次拼接
传输后缀。

预期频率接近 OpenMV 脚本中的 `TARGET_FPS`。

## 注意事项

- USB CDC 的波特率参数主要用于串口 API 兼容；直接 USB CDC 时通常不限制实际 USB 传输速率。
- Ubuntu 端连续 `5` 秒收不到任何字节会主动关闭串口并重连，可通过
  `camera_serial_no_data_timeout` 调整。
- OpenMV IDE 和 ROS 节点不能同时占用同一个串口。
- 如果运行的是外接 UART 转串口，确认电平为 3.3 V，并使用足够高的波特率。
- OpenMV 默认输出横向 `320x240`。如果用于现有视觉纠偏，需要重新标定相机安装角度、焦距和图像轴方向。
- 串口断开后 Ubuntu 节点会自动重连；损坏或不完整的 JPEG 帧会被丢弃。
