# Auto RTK 路径规划器

`full_path_planner_auto` 用一次外围 RTK 轨迹和少量方向参考轨迹生成连续蛇形清扫路线。它不需要为每个矩形区域标定三点，也不需要在两个矩形之间手动配置桥接区域。

## 输入文件

输入是一个 JSON 文件，坐标顺序为 `[经度, 纬度]`：

```json
{
  "boundary": [
    [110.6474, 35.6060],
    [110.6480, 35.6060],
    [110.6480, 35.6052],
    [110.6474, 35.6052]
  ],
  "guides": [
    [
      [110.6475, 35.6053],
      [110.6475, 35.6059]
    ]
  ],
  "no_go": [],
  "start": [110.6475, 35.6053]
}
```

- `boundary`：机器人沿可清扫总区域外围采集的一圈轨迹。首尾可以重复，也可以不重复。
- `guides`：一条或多条方向参考轨迹。第一版使用其首尾方向估计统一清扫方向。
- `no_go`：可选的内部禁行多边形列表。连接线穿过这些区域时，规划失败并报错。
- `start`：可选的起点提示；缺省时按扫描线顺序从第一条清扫线开始。

外围边界应尽量沿可清扫区域的实际边缘采集，而不是用一个包住所有障碍物的粗略外包框。若内部通道、设备或断开区域不能通行，应采集为 `no_go` 多边形。

## 运行

在 ROS2 工作空间构建后：

```bash
source install/setup.bash
ros2 run rtk_nav full_path_planner_auto \
  --input /path/to/auto_map.json \
  --output /path/to/auto_route.json \
  --sweep-spacing 1.0 \
  --edge-clearance 0.3 \
  --max-connector 8.0
```

也可以直接运行 Python 模块：

```bash
python -m rtk_nav.auto_path_planner \
  --input auto_map.json \
  --output auto_route.json
```

命令会生成：

- `auto_route.json`：自动规划器主输出，按顺序包含 `coverage` 和 `connector` 段；
- `auto_route.geojson`：相同路线的 GeoJSON，可在 QGIS 中检查。

如果找不到安全的连接线，或连接距离超过 `--max-connector`，命令返回非零状态并不生成一条未经验证的直线。
