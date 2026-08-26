# Auto Planner Ordered Connector Merge Design

**Goal:** 将显式跨区域 connector 两侧的区域收尾连接、桥架和目标区起始连接拼成一条连续轨迹，同时保持覆盖路线和安全规划语义不变。

## Scope

`map.order` 是跨区域连接的权威顺序。对每个同时具有 `from` 和 `to` 的 connector，输出后处理从该 connector 的 route segment 向前吸收 `from` 区域末尾连续的、无 `connector_id` 的 connector，再向后吸收 `to` 区域开头连续的、无 `connector_id` 的 connector。合并结果保留显式 connector 的 `connector_id`、`from_region` 和 `to_region`，并删除被吸收段的区域内部标签。

不合并 coverage 段，不改变 coverage 的生成顺序，不重新计算或放宽任何安全连接校验。没有 `from/to` 的顺序桥段保持独立 ID 和原顺序，避免无法表达多个输入桥 ID 的复合 Segment。

## Data Flow

`plan_route()` 先按现有逻辑生成所有 Segment，完成后按 `order` 逆序处理显式跨区域 connector。每个合并段通过去除相邻重复端点构造新的 LineString，长度为被合并段长度之和；随后重新计算总长度、最大 connector 长度和转弯次数。GeoJSON、route JSON 和 TXT 复用同一组 Segment，因此三种输出会看到相同的连续桥架。

## Validation

- 单元测试验证合并段首尾连续、只保留显式 bridge 的身份字段、长度等于原子段长度之和。
- 单元测试验证同一区域 coverage 之间的内部 connector 没有被错误吸收到 bridge 中。
- E12/E13/E14 full map 重新规划，检查两个 inter-region bridge 各输出一个 route segment，coverage 数不变，total length 不变，connector 数减少。
- 运行 planner focused tests、语法检查和 `git diff --check`。
