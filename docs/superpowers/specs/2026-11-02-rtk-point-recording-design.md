# WAIT_RTK 采点记录设计

## 意图

通过 MQTT 向 `robot/HANGZHOU_1/cmd` 发布 `WAIT_RTK` 后，机器人记录一次当前车体中心 RTK 坐标，包含序号、`{lon, lat}` 点对象、定位有效性和本地时间，写入 `src/rtk_nav/rtk_nav/config/rtk_points.md`，便于现场移动机器人逐点采集，后续复制到 YAML 区域配置。

## 目标与边界

- 目标：复用现有 MQTT→ROS `/robot_cmd` 链路；每条 `WAIT_RTK` 指令追加一条记录；记录坐标使用 RTK 节点已完成天线偏移修正的 `current_lon/current_lat`。
- 目标：兼容 MQTT 原始字符串 `WAIT_RTK`，也兼容现有桥接可能转发的 `{"command": "WAIT_RTK"}` JSON。
- 目标：文件不存在时自动创建；序号从现有记录的最大序号继续递增。
- 边界：不改变导航状态机、电机动作或 MQTT 主题配置；不自动修改现有 YAML；坐标无效时不写入。

## 记录格式

每条记录使用 YAML 兼容的 Markdown 列表项，坐标保留 Python 浮点字符串形式，时间使用本地 `YYYY-MM-DD HH:MM:SS`：

```yaml
- seq: 1
  point: {lon: 110.64741623970805, lat: 35.605286201553355}
  valid: true
  position_status: 4
  orientation_status: 4
  time: "2026-11-02 14:30:00"
```

`valid` 仅在 `self.rtk_position_status == 4` 且 `self.rtk_orientation_status == 4` 时为 `true`；任一状态不是 4 都记录为 `false`，但不丢弃坐标，方便现场排查。这样既能直接阅读，也能在后续脚本中按 `seq`、`point`、`valid`、`time` 解析并转换到 YAML 的 `calib_point_*` 或其他区域点位。

## 实现方案

在已有 `MotorControlNode` 的 `robot_cmd` 回调中处理 `WAIT_RTK`。该节点已经维护车体中心坐标 `current_lon/current_lat`、定位状态 `self.rtk_position_status` 和定向状态 `self.rtk_orientation_status`，无需再订阅或复制 RTK 状态。收到原始字符串或 JSON 形式的 `WAIT_RTK` 后调用独立的格式化与追加函数。记录文件路径作为 ROS 参数 `rtk_points_file`，默认指向与 YAML 同目录的 `rtk_points.md`，launch 文件传入同一默认路径。追加操作使用 UTF-8 文本追加并在写入前扫描现有序号；单次回调只追加一条记录。

## 验证

新增单元测试覆盖：原始字符串和 JSON 指令识别、序号递增、坐标/时间格式、定位/定向状态生成 `valid`、无效坐标不写入。另运行 `rtk_nav` 包现有 pytest 与 flake8 检查。

## 备选方案

- 独立采点节点：职责隔离更强，但需要新增节点、launch 和安装配置，且会重复订阅已有的坐标和状态，超出本次小改动范围。
- MQTT 桥接层直接写文件：改动更少，但拿不到 RTK 节点完成天线偏移修正后的车体中心坐标，点位会有系统偏差。

本次采用在现有 `rtk_nav` 节点内扩展的方案，因为它同时满足链路复用、坐标正确和改动可控。
