# Auto Planner Region Guides Design

## Goal

让 `003-E12-E14.yaml` 成为生成 full map 的唯一配置来源，同时保证生成的
`rtk_auto_map_v2` 保留根级 `guides`，并允许每个清扫区域用
`horizontal` 或 `vertical` 覆盖默认清扫方向。

## Configuration Contract

- YAML 根级 `guides` 可选，格式保持现有地图格式的经纬度轨迹列表。
- YAML 未提供根级 `guides` 时，转换器从生成后的区域边界中选择实际最长边，写入
  一个两点根级 guide。该 guide 是没有区域覆盖时的默认方向。
- YAML 清扫区域可添加 `guide: horizontal` 或 `guide: vertical`。
- `E13_long_block` 与 `E13` 都映射到逻辑区域 `E13`；同一逻辑区域的多个 YAML
  多边形必须使用相同的 guide，否则转换失败并提示冲突。
- 生成的 v2 map 在 `regions[].guide` 中保存区域方向；根级 `guides` 仍保存实际轨迹
  点，不改为方向字符串。
- 不配置区域 guide 时，区域沿根级 guide 的方向规划。根级没有显式 guide 时的
  默认行为仍是按最长实际边推导。

## Architecture

`Region` 增加可选的 `guide` 字段。v2 解析器验证方向值，并在 GeoJSON 边界属性中
暴露它。规划入口先从根级 guides 得到全局默认轴角，再将每个区域的 guide 映射为
水平 `0` 弧度或垂直 `pi / 2` 弧度，区域覆盖扫描、区域内连接和进出区域的起止点
使用该区域轴角。

桥架和跨区域连接保持独立：桥架端点偏移、桥架长度和桥架自身方向继续按原有
connector path 计算，不被任一区域的 coverage guide 旋转。已有的 E14 斜边和四边
实际偏移逻辑保持不变。

## Validation and Compatibility

- `guide` 必须是字符串，且只能是 `horizontal` 或 `vertical`。
- v2 地图不含 `regions[].guide` 时行为不变，继续使用根级 guide 或最长边回退。
- 旧版单区域地图继续使用 `legacy` 区域，不要求新增字段。
- 转换器显式复制根级 guides；缺失时自动补齐，因此从 YAML 生成的 full map 可稳定
  复现默认方向。

## Tests

覆盖以下行为：根级 guides 的保留和缺失时最长边补齐；区域 guide 的解析、水平/垂直
轴角映射和非法值报错；E13 两个 YAML 多边形共享逻辑区域 guide；转换后的 full map
经过 `load_map` 和 `plan_route` 后区域 coverage 使用各自方向，且桥架顺序和现有
E12-E14 输出契约不变。
