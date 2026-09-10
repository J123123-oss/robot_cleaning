# 栅格线检测图像 RTSP 推流

主机端 `grid_line_streamer` 节点订阅检测标注图像：

```text
/grid_line/detected_image
        -> FFmpeg H.264
        -> RTSP media server
        -> HTTP-FLV playback URL
```

摄像头固件不需要修改。主机需要安装 FFmpeg，并且 RTSP 地址必须指向一个
支持 RTSP 发布、同时能提供 HTTP-FLV 播放地址的媒体服务器，例如 SRS、
ZLMediaKit 或现有后台媒体服务。

## 安装条件

本流程适用于直接运行在 Ubuntu ARM64 主机上的 ZLMediaKit，不依赖 Docker。
主机至少应满足：

- `uname -m` 输出 `aarch64`，`dpkg --print-architecture` 输出 `arm64`；
- Ubuntu 22.04/24.04，能够访问 GitHub 或配置好的 Git 镜像；
- 建议至少有 2 GB 可用内存和 1 GB 可用磁盘空间；内存较小时编译使用 `-j1`；
- 主机有稳定的局域网或 VPN 地址，后台能够访问 TCP `8080`；
- 主机没有其他程序占用 RTSP `8554` 和 HTTP `8080` 端口；
- 能够使用 `sudo` 安装依赖和启动服务。

检查架构和端口占用：

```bash
uname -m
dpkg --print-architecture
sudo ss -lntup | grep -E ':(8554|8080)\b'
```

如果当前设备的定制内核缺少 Docker 所需的 `overlay` 或 Netfilter 模块，
不要继续使用 Docker 部署媒体服务器，直接使用下面的原生 ARM64 编译流程。

## 安装原生 ARM64 ZLMediaKit

安装编译依赖。FFmpeg 是主机端 `grid_line_streamer` 编码所需的运行依赖，
ZLMediaKit 本身从源码编译：

如果其余编译依赖已经安装，只需补装 FFmpeg，可执行 `sudo apt install ffmpeg`。

```bash
sudo apt update
sudo apt install -y \
  git cmake build-essential pkg-config libssl-dev libsdl2-dev ffmpeg
```

下载源码并初始化子模块：

```bash
cd ~
git clone --depth=1 --recurse-submodules \
  https://github.com/ZLMediaKit/ZLMediaKit.git
cd ~/ZLMediaKit
git submodule update --init --recursive
```

在 ARM64 主机本地编译。低内存设备使用 `-j1`，避免编译时耗尽内存：

```bash
cd ~/ZLMediaKit
mkdir -p release/linux
cd release/linux
cmake ../.. -DCMAKE_BUILD_TYPE=Release
cmake --build . -j1
```

编译完成后，程序和配置通常位于：

```text
~/ZLMediaKit/release/linux/Release/MediaServer
~/ZLMediaKit/release/linux/Release/config.ini
```

如果上述路径不存在，查找实际生成位置：

```bash
find ~/ZLMediaKit -type f -name MediaServer -print
find ~/ZLMediaKit -type f -name config.ini -print
```

## 配置 ZLMediaKit

进入包含 `MediaServer` 和 `config.ini` 的目录：

```bash
cd ~/ZLMediaKit/release/linux/Release
cp config.ini config.ini.backup
nano config.ini
```

确认或修改以下配置。若配置文件中已经是这些值，不要重复添加同名字段：

```ini
[http]
port=8080

[rtsp]
port=8554
```

端口用途如下：

| 端口 | 协议 | 用途 |
| --- | --- | --- |
| `8554/tcp` | RTSP | `grid_line_streamer` 发布视频 |
| `8080/tcp` | HTTP | 后台播放 HTTP-FLV，也用于 ZLMediaKit API |
| `10000/udp` | RTP | 仅在使用 RTSP UDP 传输时需要 |

当前 `grid_line_streamer` 已使用 `-rtsp_transport tcp`，所以本地链路不依赖
`10000/udp`。不要将 API 的 `secret` 提交到代码或日志中；查询媒体列表时再
按配置文件中的实际密钥临时传入。

## 启动原生媒体服务

先在前台启动，便于直接查看日志：

```bash
cd ~/ZLMediaKit/release/linux/Release
./MediaServer
```

看到服务启动后，另开终端检查端口：

```bash
sudo ss -lntup | grep -E ':(8554|8080)\b'
```

确认无误后，可以在后台运行：

```bash
cd ~/ZLMediaKit/release/linux/Release
./MediaServer -d
```

停止后台实例前先找到进程，避免误杀其他媒体服务：

```bash
pgrep -af MediaServer
```

若需要开机启动，建议使用 systemd 的前台进程模式。创建服务文件前确认
实际用户名和 ZLMediaKit 路径：

```ini
[Unit]
Description=ZLMediaKit media server
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=forlinx
Group=forlinx
WorkingDirectory=/home/forlinx/ZLMediaKit/release/linux/Release
ExecStart=/home/forlinx/ZLMediaKit/release/linux/Release/MediaServer
Restart=on-failure
RestartSec=3

[Install]
WantedBy=multi-user.target
```

将内容保存为 `/etc/systemd/system/zlmediakit.service` 后执行：

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now zlmediakit
sudo systemctl status zlmediakit --no-pager
```

如果使用 systemd，不要同时手动启动另一个 `MediaServer` 实例，否则会发生
端口冲突。

确认生成的是 ARM64 原生程序：

```bash
file ~/ZLMediaKit/release/linux/Release/MediaServer
```

预期输出中应包含：

```text
ELF 64-bit ... ARM aarch64 ...
```

## 随总启动文件运行

先确保 `publish_debug_images` 没有关闭。两个 launch 文件默认开启推流，默认
RTSP 发布地址为本机 ZLMediaKit：

```bash
cd ~/robot_cleaning
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch rtk_nav run.launch.py \
  publish_debug_images:=true \
  enable_grid_line_stream:=true \
  grid_line_stream_rtsp_url:=rtsp://127.0.0.1:8554/live/grid_line
```

如果连接其他媒体服务器，再覆盖默认地址：

```bash
ros2 launch rtk_nav run.launch.py \
  grid_line_stream_rtsp_url:=rtsp://media-server:8554/live/grid_line
```

如果 ZLMediaKit 和 ROS2 在同一台主机，使用本机回环地址：

```text
rtsp://127.0.0.1:8554/live/grid_line
```

后台播放器不能使用 `127.0.0.1`，应使用机器人主机的局域网或 VPN 地址。
例如主机 IP 为 `192.168.0.6` 时：

```text
http://192.168.0.6:8080/live/grid_line.live.flv
```

如果媒体服务器与 ROS2 不在同一台主机，推流地址应改为媒体服务器可达的主机名
或 IP，例如：

```text
rtsp://media-server:8554/live/grid_line
```

媒体服务器生成的 HTTP-FLV 播放地址对应为：

```text
http://media-server:8080/live/grid_line.live.flv
```

不同媒体服务器的 HTTP-FLV 后缀可能不同，RTSP 发布地址和 HTTP-FLV 播放地址的路径需要由媒体服务器配置保持对应。
不要把带账号密码的地址写入代码或提交到 Git。

## 地址配置位置

两类地址用途不同：

- `rtsp://127.0.0.1:8554/live/grid_line` 是主机推给 ZLMediaKit 的发布地址，默认配置在两个 launch 文件的 `grid_line_stream_rtsp_url` 参数中，也可以由 `motor_start.sh` 的 `grid_line_stream_rtsp_url:=...` 覆盖。
- `http://192.168.0.6:8080/live/grid_line.live.flv` 是后台播放器的拉流地址，配置在后台播放器的 `src`、`url` 或等价播放源字段中，不配置在摄像头或 ROS2 图像话题中。

ZLMediaKit 的端口配置在 `~/ZLMediaKit/release/linux/Release/config.ini`；本机当前使用 RTSP `8554` 和 HTTP `8080`。`live` 是应用名，`grid_line` 是流名，来自 RTSP 发布地址的路径。

如果后台不在同一局域网，`192.168.0.6` 这类私有地址不能直接从公网访问，
需要先建立 VPN，或在网关做端口转发并配置访问控制。推荐只允许后台服务器
访问 `8080/tcp`：

```bash
sudo ufw status
sudo ufw allow from BACKEND_IP to any port 8080 proto tcp
```

这里的 `BACKEND_IP` 仅表示后台服务器的实际 IP，不要原样执行。若 UFW 未启用，
还需要检查上级路由器或云防火墙是否放行 `8080/tcp`。本机推流使用
`127.0.0.1:8554` 时，不需要对外开放 `8554/tcp`；只有外部客户端直接拉 RTSP
时才需要开放该端口。

## 独立运行

```bash
source install/setup.bash
ros2 run rtk_nav grid_line_streamer --ros-args \
  -p rtsp_url:=rtsp://127.0.0.1:8554/live/grid_line \
  -p fps:=10.0 \
  -p bitrate:=800k
```

节点只缓存最新图像帧。FFmpeg 或 RTSP 连接断开时，节点保持 ROS 运行并按
`reconnect_sec` 自动重连。推流默认启动；如果只需要运行检测而不推流，可传入
`enable_grid_line_stream:=false`。

## 开机自启

在主机 `~/robot_cleaning/can_ch340_init.sh` 中启动 ZLMediaKit，在
`~/robot_cleaning/motor_start.sh` 中启动 ROS2 总启动文件。总启动文件必须保持
`publish_debug_images:=true`，否则 `/grid_line/detected_image` 不会产生。

修改启动脚本后，先检查语法，再重启服务：

```bash
bash -n ~/robot_cleaning/can_ch340_init.sh
bash -n ~/robot_cleaning/motor_start.sh
sudo systemctl restart can_ch340_init.service motor_start.service
```

确认服务和进程：

```bash
systemctl is-active can_ch340_init.service motor_start.service
pgrep -af MediaServer
ros2 node list | grep grid_line_streamer
```

## 验证完整链路

先检查 ROS2 图像源：

```bash
command -v ffmpeg
ros2 topic hz /grid_line/detected_image
curl -I http://192.168.0.6:8080/live/grid_line.live.flv
ffprobe -v error -show_entries stream=codec_name,width,height,r_frame_rate \
  http://192.168.0.6:8080/live/grid_line.live.flv
```

如果没有图像，确认识别节点使用了：

```bash
publish_debug_images:=true
```

该参数是现有识别节点生成 `/grid_line/detected_image` 的开关。

### 常见问题

- `FFmpeg 推流进程已退出`：检查 `command -v ffmpeg`、RTSP 地址和
  `~/ZLMediaKit/release/linux/Release/mediaserver-start.log`。
- HTTP-FLV 返回 `404`：确认 ZLMediaKit 已注册 `app=live`、`stream=grid_line`，并使用
  `/live/grid_line.live.flv`，不要使用 `/live/grid_line.flv`。
- 没有 `/grid_line/detected_image`：确认视觉检测节点已启动，并将
  `publish_debug_images` 设为 `true`。
- ZLMediaKit 出现 `Not rtp packet`：该日志通常来自 UDP/RTP 或 GB28181 入口；本推流器使用
  RTSP TCP，不会把 ROS2/DDS 数据发送到该媒体端口。

确认 RTSP 流已经注册并能拉取视频。以下命令需要在推流 launch 保持运行时执行：

```bash
ffmpeg -hide_banner -loglevel info \
  -rtsp_transport tcp \
  -i rtsp://127.0.0.1:8554/live/grid_line \
  -t 10 -an -f null -
```

成功时应看到 `Video: h264` 和持续增长的 `frame=`。再验证后台使用的 HTTP-FLV：

```bash
ffmpeg -hide_banner -loglevel info \
  -i http://127.0.0.1:8080/live/grid_line.live.flv \
  -t 10 -an -f null -
```

后台播放器使用机器人主机的局域网或 VPN 地址，例如：

```text
http://192.168.0.6:8080/live/grid_line.live.flv
```

确认监听状态：

```bash
sudo ss -lntup | grep -E ':(8554|8080)\b'
```

如果查询媒体列表，ZLMediaKit 的 API 可能要求 `secret`。先在本机配置中查看
密钥名称，不要把实际密钥粘贴到日志、代码或 Git：

```bash
grep -n -A6 '^\[api\]' \
  ~/ZLMediaKit/release/linux/Release/config.ini
```

然后将下面的 `REPLACE_WITH_API_SECRET` 替换为本机密钥后查询：

```bash
curl -G -sS \
  'http://127.0.0.1:8080/index/api/getMediaList' \
  --data-urlencode 'secret=REPLACE_WITH_API_SECRET' \
  --data-urlencode 'schema=rtsp' \
  --data-urlencode 'vhost=__defaultVhost__' \
  --data-urlencode 'app=live' \
  --data-urlencode 'stream=grid_line'
```

返回 `code: 0` 且 `data` 中存在 `grid_line` 时，表示 RTSP 流已被 ZLMediaKit
注册。返回 `404 Stream Not Found` 时，先检查 `grid_line_streamer` 是否运行、
`/grid_line/detected_image` 是否有数据以及 RTSP 推送地址是否为
`rtsp://127.0.0.1:8554/live/grid_line`。
