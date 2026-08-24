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

## 多区域和复杂轮廓格式

当一个大型清扫任务包含 E9、E10、E11 和连接桥时，使用 `rtk_auto_map_v2`：

```json
{
  "format": "rtk_auto_map_v2",
  "guides": [
    [[110.6473, 35.6045], [110.6478, 35.6045]]
  ],
  "regions": [
    {
      "id": "E9",
      "polygons": [
        {
          "boundary": [[110.6472, 35.6050], [110.6478, 35.6050], [110.6478, 35.6053], [110.6472, 35.6053]],
          "holes": []
        }
      ]
    },
    {
      "id": "E10",
      "polygons": [
        {
          "boundary": [[110.6472, 35.6048], [110.6478, 35.6048], [110.6478, 35.6050], [110.6472, 35.6050]],
          "holes": [
            [[110.64745, 35.60486], [110.64755, 35.60486], [110.64755, 35.60496], [110.64745, 35.60496]]
          ]
        }
      ]
    },
    {
      "id": "E11",
      "polygons": [
        {
          "boundary": [[110.6472, 35.6045], [110.6478, 35.6045], [110.6478, 35.6048], [110.6472, 35.6048]],
          "holes": []
        }
      ]
    }
  ],
  "connectors": [
    {
      "id": "bridge_9-10B",
      "from": "E9",
      "to": "E10",
      "path": [[110.64775, 35.6050], [110.64776, 35.60498]]
    },
    {
      "id": "bridge_10A-11B",
      "from": "E10",
      "to": "E11",
      "path": [[110.64775, 35.6048], [110.64776, 35.60475]]
    }
  ],
  "order": ["E9", "bridge_9-10B", "E10", "bridge_10A-11B", "E11"]
}
```

四点区域也可以直接写成 `regions[].boundary`，不需要转换成旧 YAML 的三点标定格式。
解析器同时接受 `[lon, lat]` 点数组、`{"lon": ..., "lat": ...}` 点对象，以及
`top_left/top_right/bottom_right/bottom_left` 命名角点。缺少 `guides` 时，规划器使用
最长边推导清扫方向；缺少 `order` 时，仅在连接桥构成单向链的情况下自动推导顺序。

- `regions[].polygons` 可以放一个凹形外轮廓，也可以放多个相接的错位矩形；多个 polygon 会合并为一个逻辑清扫区。
- `holes` 是局部缺口、设备区或不允许清扫的区域。扫描线会自动分裂，内部连接会绕开缺口；无法绕行时规划失败。
- `edge_distance_lon` 和 `edge_distance_lat` 可配置四边的非对称边缘距离，单位是米。顺序保持旧 YAML 逻辑：`edge_distance_lon=[右,左]`、`edge_distance_lat=[下,上]`；正值向内缩进，负值向外扩展。区域级字段会继承到所有 polygon，polygon 自己配置时优先。
- 边缘距离用于四角矩形 polygon；它会先调整清扫几何，再叠加命令行的统一 `--edge-clearance`。桥架端点仍按原始外围边界校验。
- `connection_tolerance_m` 可选，仅用于同一区域多个 polygon 之间的小间隙连接。它不会扩大 GeoJSON 边界，孔洞仍然禁止通行；缺省为 `0`。
- `connectors` 使用 RTK 实际采集的桥接轨迹，默认只通行、不计为清扫覆盖。`from`、`to` 和 `order` 决定连接方向。
- `back_` 开头的返回轨迹可以保留在输入文件中，但不要放进本次正向清扫的 `order`。
- 例如 E9/E10/E11 的正向顺序为 `E9 -> bridge_9-10B -> E10 -> bridge_10A-11B -> E11`。

例如 E13 的两个 polygon 可以分别配置旧 YAML 中的四边距离：

```json
{
  "id": "E13",
  "connection_tolerance_m": 3.0,
  "polygons": [
    {
      "edge_distance_lon": [0.15, 0.5],
      "edge_distance_lat": [0.3, 0.3],
      "boundary": [[110.6472707611, 35.6041331221], [110.6474216066, 35.6041331221], [110.6474211547, 35.6042619680], [110.6472707611, 35.6042620942]],
      "holes": []
    },
    {
      "edge_distance_lon": [0.3, 0.1],
      "edge_distance_lat": [0.3, 0.3],
      "boundary": [[110.6474291479, 35.6041538960], [110.6477649148, 35.6041538960], [110.6477646446, 35.6042622178], [110.6474291479, 35.6042619846]],
      "holes": []
    }
  ]
}
```

`connection_tolerance_m` 是米制的临时跨隙阈值，应根据现场可通行宽度设置，不应把相距较远的独立区域合并。E12/E13/E14 示例见仓库根目录的 `auto_map_e12_e14.json`。

当一个区域包含多个相接或错位 polygon 时，规划器会先生成所有扫描段，再在安全连接图上优化扫描顺序：优先压低最长连接距离，其次压低连接总距离。这样带有加长块或窄通道的区域不会机械地把支路留到最后，再横向返回下一个桥架。最多 16 条扫描段的多 polygon 区域使用有界状态搜索；更大的区域保留线性蛇形顺序，以控制规划时间。`--max-connector` 同时约束优化阶段和最终输出阶段。

区域内部不再自动把所有矩形两两连接；规划器只在同一 `region` 的允许几何内寻找安全连接。跨区域必须有明确 connector，这样不会把两个相邻但实际不可通行的区域误连成一条直线。

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
- `auto_route.geojson`：标准 `FeatureCollection`，包含 `boundary` Polygon、`bridge`
  LineString、`coverage` 清扫线和 `connector` 连接线，可直接在 QGIS 或 geojson.io 中检查。

## 可视化检查

推荐直接打开命令生成的 `auto_route.geojson`：

1. 在 QGIS 中将 `.geojson` 拖入地图，按 `kind` 分类显示 `coverage` 和 `connector`。
2. 再按 `region_id` 区分 E9/E10/E11，按 `connector_id` 检查 `bridge_9-10B`、`bridge_10A-11B` 是否落在正确位置。
3. 也可以在 geojson.io 导入文件进行快速检查。覆盖段是 `coverage`，区域内部绕行段是带 `region_id` 的 `connector`，跨区域桥是带 `connector_id` 的 `connector`。

输出的路线 JSON 和 GeoJSON 均保留同一段顺序；如果图上出现跨越缺口的线，先检查输入 polygon 的 `holes` 和 `edge-clearance`，不要直接放宽桥接上限。

如果找不到安全的连接线，或连接距离超过 `--max-connector`，命令返回非零状态并不生成一条未经验证的直线。
