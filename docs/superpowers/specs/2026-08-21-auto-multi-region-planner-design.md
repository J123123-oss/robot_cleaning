# Auto 多区域复杂轮廓路径规划设计

## 目标

扩展 `auto_path_planner.py`，使一次 RTK 建图可以表达并规划 E9/E10/E11 这类多个清扫区域及其连接桥。输入可以描述凹形边界、错位矩形、多个 polygon 组成的逻辑区域和局部缺口，输出按指定顺序包含清扫段与桥接通行段。

## 兼容性与范围

- 保留现有 `boundary/guides/no_go/start` 单区域格式，自动转换成一个匿名区域。
- 新增 `rtk_auto_map_v2` 格式；不修改 `full_path_planner_dense.py`、`rtk_nav.py` 和手工 YAML 流程。
- 自动模式仍输出独立 JSON 和 GeoJSON，不转换成旧 `.txt` 航点文件。
- 不引入 Shapely、UTM 等第三方几何依赖，继续使用 Python 标准库。

## 输入模型

```json
{
  "format": "rtk_auto_map_v2",
  "guides": [[[110.0, 35.0], [110.0, 35.001]]],
  "regions": [
    {
      "id": "E9",
      "polygons": [
        {"boundary": [[109.999, 35.0], [110.0, 35.0], [110.0, 35.001], [109.999, 35.001]], "holes": []}
      ]
    },
    {
      "id": "E10",
      "polygons": [
        {"boundary": [[110.0, 35.0], [110.001, 35.0], [110.001, 35.001], [110.0, 35.001]], "holes": []}
      ],
      "start": [110.0, 35.0]
    }
  ],
  "connectors": [
    {"id": "bridge_9-10B", "from": "E9", "to": "E10", "path": [[110.0, 35.0], [110.0, 35.001]]}
  ],
  "order": ["E9", "bridge_9-10B", "E10"]
}
```

- `regions[].id` 是区域唯一标识。
- `regions[].polygons` 可以包含多个不相连或错位 polygon；每个 polygon 有一个外轮廓 `boundary` 和可选 `holes`。
- `holes` 表示局部缺口、设备禁行区或不参与清扫的内部区域。
- `connectors[].path` 是人工采集的安全通道轨迹，默认只作为通行段，不生成清扫线。源点必须在 `from` 区域、终点必须在 `to` 区域，所有线段不得穿过任意 region hole；桥接段可以位于清扫区域外。
- `connectors[].from/to` 必须与相邻区域匹配；`order` 是唯一执行顺序，区域和桥接 ID 交替出现。
- `guides` 提供统一清扫方向；没有单独 guide 的区域复用全局方向。

## 算法

1. 将经纬度投影到以首个区域点为原点的局部米制坐标。
2. 使用 guide 首尾向量的双角度平均估计无方向清扫轴。
3. 对每个区域的每个 polygon 逐条扫描。外轮廓产生可用区间，holes 从区间中扣除；多个 polygon 的区间合并后按扫描行蛇形排序。
4. 相邻清扫段之间用直线或两段轴对齐折线连接，并按采样点检查是否仍在同一区域的允许几何内。找不到安全连接或超过上限则抛出 `PlanningError`。
5. 按 `order` 插入区域内部清扫路线和显式 connector。connector 路径逐线段检查端点和 hole 穿越，输出为 `kind=connector`、`connector_id` 的通行段。

区域内部连接不允许穿过其他区域、hole 或区域外部；区域之间不猜测桥接路线，必须由 connector 明确提供。

## 输出与可视化

路线 JSON 继续使用 `rtk_auto_route_v1`，但每个段可增加：

```json
{
  "kind": "coverage",
  "region_id": "E10",
  "points": [[110.0, 35.0], [110.001, 35.0]],
  "length_m": 10.0
}
```

GeoJSON 的每个 Feature 保留 `kind`、`region_id` 或 `connector_id`，并增加 `sequence`，可直接在 QGIS、geojson.io 或现有 `_plot_multi_area_path` 风格的绘图脚本中按颜色查看区域清扫和桥接通行关系。

## 错误处理

缺失 ID、重复 ID、非法 polygon、非法 order、connector 端点不匹配、区域无可用扫描线、区域内部无安全连接或 connector 超长，都必须返回可读的 `PlanningError`；CLI 返回 2，不能写出未经验证的路线。

## 测试

新增纯 Python 契约测试覆盖：旧格式兼容、多区域解析、多 polygon、凹形轮廓、holes 缺口、connector 顺序和输出元数据，以及无安全路径时的失败行为。使用 `unittest`、`py_compile` 和 `git diff --check` 验证，不依赖 ROS2 或第三方几何库。
