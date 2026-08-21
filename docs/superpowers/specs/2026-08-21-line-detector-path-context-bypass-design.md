# 白线检测路径上下文测试旁路设计

## 目标

让现场人员能够在未连接 RTK 路径上下文时，单独验证相机输入和粗白线检测算法；
同时保持实际导航的默认安全门控不变。

## 现有安全策略

- `enable_visual_correction` 默认值为 `false`。
- `run.launch.py` 仅在该开关为 `true` 时启动相机发布节点和
  `grid_line_detector`。
- RTK 视觉路径上下文只有在视觉总开关、RTK 固定解、自动清扫模式和有效行驶状态
  同时满足时才发布 `z=1`；其他情况下发布 `z=0`。
- 检测节点遇到视觉总开关关闭、路径上下文无效或上下文超时时，发布
  `angle_deviation.z=0` 和零置信度。

上述行为继续作为生产默认值。

## 新参数

`line_detector_node.py` 新增 ROS 参数：

```text
bypass_path_context_gate: false
```

它只用于测试，默认必须为 `false`。参数为 `true` 时，检测节点跳过路径上下文
的有效性和超时判断；参数不得影响 `enable_visual_correction`。

因此纯相机/粗白线验证必须同时显式开启两个参数：

```bash
ros2 run rtk_nav line_detector_node --ros-args \
  -p enable_visual_correction:=true \
  -p bypass_path_context_gate:=true
```

`enable_visual_correction` 仍是运行许可开关，避免误将一个测试参数变成默认启用
视觉链路的旁路。

## 运行行为

| visual correction | bypass path context | 路径上下文 | 输出 |
| --- | --- | --- | --- |
| false | 任意值 | 任意值 | 始终无效，`z=0` |
| true | false | 无效或超时 | 无效，`z=0` |
| true | false | 有效且新鲜 | 使用 RTK 路径轴进行检测 |
| true | true | 任意值 | 运行相机和粗白线检测，不等待 RTK 上下文 |

测试旁路模式继续使用节点初始化时的默认运行轴，除非随后收到有效路径上下文；
它的目的仅是隔离图像采集和白线算法，不能作为实际导航纠偏的验证。

## 实现边界

- 仅修改 `line_detector_node.py` 和相应契约测试。
- 不将旁路参数加入 `run.launch.py`，防止常规 launch 命令意外暴露测试旁路。
- 不改动 `rtk_nav.py`、Stanley 融合逻辑或视觉总开关。
- 不改变无效结果的消息格式：`angle_deviation.z=0` 与置信度 `0.0`。

## 测试

静态契约测试应验证：

1. 参数 `bypass_path_context_gate` 声明为 `false`；
2. `image_callback` 仍优先检查 `enable_visual_correction`；
3. 默认模式保留路径上下文有效性与超时门控；
4. 显式旁路时可越过该门控；
5. 现有 `run.launch.py` 默认关闭且条件启动检测节点的约束保持成立。

ROS 现场验证仍需分别执行：

```bash
source install/setup.bash
ros2 launch rtk_nav run.launch.py enable_visual_correction:=true
ros2 node list | grep grid_line
ros2 node info /grid_line_detector
```

以及上文的独立旁路启动命令。单元测试无法替代真实相机、RTK 和底盘的低速安全验证。
