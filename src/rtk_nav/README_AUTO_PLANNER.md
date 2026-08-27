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
- `start`：可选的实际起点提示。区域有前置桥架时，规划器按桥架进入端点最近的区域角点选择第一条清扫线；没有实际起点时才按默认扫描顺序开始。

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

由旧 YAML 三点标定转换生成的 `auto_map_*.json`，每个 polygon 还会包含
`source_area`、`boundary_order`、`manual_calibration_points` 和
`boundary_point_annotations`。由于 JSON 不支持 `//` 注释，点位说明保存在这些字段中：
`boundary` 的五个点按 `D -> A -> B -> C -> D` 排列，其中第 1 个 D 是由
`A + C - B` 自动计算的西南角，第 2 个 A、第 3 个 B 和第 4 个 C 是 YAML 中需要
手动采集的三个标定点，第 5 个 D 是自动复制第 1 个点的闭合点。注释中的
`source` 标记 `manual` 或 `automatic`，`coordinate` 是对应的实际坐标。
E16 为了保持矩形，使用正交化逻辑，第 4 个点标记为 `C_prime`，它不是原始
`calib_point_c`，而是根据 A/B/C 自动计算的正交角点；原始 C 仍保存在
`manual_calibration_points.C` 中。

根级 `defaults`（也兼容旧式 `default`）可提供旧 YAML 中的公共默认值：
`interval`、`start_corner`、`swap_wh_select`、`edge_distance_lon` 和
`edge_distance_lat`。边缘距离按“根级 → region 级 → polygon 级”继承，越具体的配置
优先；命令行显式传入 `--sweep-spacing` 时优先于 `defaults.interval`。

- `regions[].polygons` 可以放一个凹形外轮廓，也可以放多个相接的错位矩形；多个 polygon 会合并为一个逻辑清扫区。
- `holes` 是局部缺口、设备区或不允许清扫的区域。扫描线会自动分裂，内部连接会绕开缺口；无法绕行时规划失败。
- `edge_distance_lon` 和 `edge_distance_lat` 可配置四边的非对称边缘距离，单位是米。顺序保持旧 YAML 逻辑：`edge_distance_lon=[右,左]`、`edge_distance_lat=[下,上]`；正值向内缩进，负值向外扩展。区域级字段会继承到所有 polygon，polygon 自己配置时优先。
- connector 也可以单独配置 `edge_distance_lon` 和 `edge_distance_lat`。connector 自身配置按轴优先于源/目标 polygon 的边缘距离；没有 connector 配置时才使用区域边界投影。对 bridge 路径，`edge_distance_lat=[起点侧,终点侧]` 沿 RTK 路径的 A→B 方向计算，负值可使末端向目标区域延伸；`edge_distance_lon` 的非对称差值用于桥线横向中心偏移。旧 YAML 转换时 bridge 会保存解析后的显式值或默认值。
- 多 polygon region 如果完全没有显式边缘距离，规划器不会把公共默认值分别内缩到每个子 polygon，避免相邻 polygon 被人为切开；需要边距时请在 region 或 polygon 上显式配置。
- 边缘距离用于四角 polygon；规划器按四条实际边分别沿内法线偏移，并通过相邻偏移边求交生成新角点，因此斜边不会被轴对齐矩形覆盖。它会先调整清扫几何，再叠加命令行的统一 `--edge-clearance`。未配置 connector 偏移时，显式桥架首尾点按原始外围边界校验后投影到源/目标 polygon 的有效内缩边界；配置 connector 偏移时，bridge 段保留自身偏移后的首尾点，必要的边界锚点先以短补接段参与安全校验。首尾点的位移会按桥架累计长度平滑插值到中间采样点；最终 route 输出按 `order` 将跨区域 bridge 两侧连续的区域收尾段合并为一条 bridge 轨迹。
- `connection_tolerance_m` 可选，仅用于同一区域多个 polygon 之间的小间隙连接。它不会扩大 GeoJSON 边界，孔洞仍然禁止通行；缺省为 `0`。
- `connectors` 使用 RTK 实际采集的桥接轨迹，默认只通行、不计为清扫覆盖。`from`、`to` 和 `order` 决定连接方向。
- `back_` 开头的返回轨迹可以保留在输入文件中，但不要放进本次正向清扫的 `order`。
- 有前置和后置桥架的区域会分别按两端桥架最近的区域角点选择起点和终点。若完整蛇形方向无法同时对齐两端，规划器会沿最后一条清扫线反向退回到另一端，再用区域内经过安全校验的正交路径连接后置桥架，避免生成跨区域的长斜线；回退段同样受 `--max-connector` 和禁行区域约束。
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

当一个区域包含多个相接或错位 polygon 时，规划器会先生成所有扫描段，再在安全连接图上优化扫描顺序。候选代价为：

`总路径长度 + --turn-penalty-m × 转弯次数 + --max-connector-penalty × 最大连接距离`

并以总长度、转弯次数、最大连接距离作为稳定的次级排序。这样带有加长块或窄通道的区域不会机械地把支路留到最后，再横向返回下一个桥架。最多 12 个 coverage group 使用精确状态搜索；更大的区域使用有界 beam search 控制规划时间，同时保留出口候选。`--max-connector` 同时约束优化阶段和最终输出阶段。

区域内部不再自动把所有矩形两两连接；规划器只在同一 `region` 的允许几何内寻找安全连接。跨区域必须有明确 connector，这样不会把两个相邻但实际不可通行的区域误连成一条直线。

桥架轨迹与清扫线之间的短区域内补接会优先使用一条经过安全校验的斜向直线；如果斜线穿过边界、孔洞或超过 `--max-connector`，自动回退到原有正交连接。清扫线之间的内部连接仍保持正交绕行。跨区域 bridge 的源区收尾、bridge 本体和目标区起始补接会按 `order` 合并到同一个 `connector_id` 段中；coverage 之间的清扫连接仍保持各自的 route 顺序。

## 运行

在 ROS2 工作空间构建后：

```bash
source install/setup.bash
ros2 run rtk_nav full_path_planner_auto \
  --input /path/to/auto_map.json \
  --output /path/to/auto_route.json \
  --sweep-spacing 1.0 \
  --edge-clearance 0.3 \
  --max-connector 8.0 \
  --turn-penalty-m 1.0 \
  --max-connector-penalty 1.0
```

也可以直接运行 Python 模块：

```bash
python -m rtk_nav.auto_path_planner \
  --input auto_map.json \
  --output auto_route.json \
  --txt-output auto_route.txt \
  --txt-spacing 15.0
```

命令会生成：

- `auto_route.json`：自动规划器主输出，按顺序包含 `coverage` 和 `connector` 段；
- `auto_route.geojson`：标准 `FeatureCollection`，包含 `boundary` Polygon、`bridge`
  LineString、`coverage` 清扫线和 `connector` 连接线，可直接在 QGIS 或 geojson.io 中检查。
- `auto_route.txt`：可选的旧导航器兼容航点文件，格式为
  `序号,经度,纬度,航向角(度)`。每个 coverage/connector 的角点都会保留，长线段按
  `--txt-spacing` 插值；默认值为 15 m，与 `full_path_planner_dense.py` 的
  `DEFAULT_DENSE_SPACING` 一致。TXT 中的 `#` 区域/连接标记会被导航器忽略但可用于人工检查。

地图输入和路线输出必须使用不同路径。推荐使用 `auto_map_*.json` 保存
`rtk_auto_map_v2`，使用 `auto_route_*.json` 和 `auto_route_*.geojson` 保存规划结果；
不要把 GeoJSON 输出重新作为地图输入。

## 可视化检查

推荐直接打开命令生成的 `auto_route.geojson`：

1. 在 QGIS 中将 `.geojson` 拖入地图，按 `kind` 分类显示 `coverage` 和 `connector`。
2. 再按 `region_id` 区分 E9/E10/E11，按 `connector_id` 检查 `bridge_9-10B`、`bridge_10A-11B` 是否落在正确位置。
3. 也可以在 geojson.io 导入文件进行快速检查。覆盖段是 `coverage`，coverage 之间的区域内部绕行段是带 `region_id` 的 `connector`，跨区域桥及其两侧收尾补接是带 `connector_id` 的单条 `connector`。

输出的路线 JSON 和 GeoJSON 均保留同一段顺序；如果图上出现跨越缺口的线，先检查输入 polygon 的 `holes` 和 `edge-clearance`，不要直接放宽桥接上限。

如果找不到安全的连接线，或连接距离超过 `--max-connector`，命令返回非零状态并不生成一条未经验证的直线。
