# Launch 可调 RTK Stanley 与视觉纠偏增益设计

## 目标

通过 `run.launch.py` 的 launch 参数分别调整 RTK Stanley 横向控制增益和视觉附加纠偏增益，支持关闭视觉后的纯 RTK 对照测试，也支持开启视觉后独立调参。

## 现状与约束

- `rtk_nav.py` 的 `get_adaptive_stanley_k()` 当前在距离目标小于 1.3 m 时返回 `0.42`，否则返回 `0.45`。
- 视觉纠偏已由 `enable_visual_correction` 总开关控制，视觉附加项仍需要保持独立的限幅和新鲜度/置信度门控。
- 现有总转向限幅为 `[-45, 45]` 度，本次不改变底盘速度分配和安全状态机。
- 当前工作区包含其他未提交改动，本次只修改本需求涉及的节点、launch 和契约测试。

## 方案

保留 RTK 的近目标自适应结构，新增两个 ROS 参数：

| 参数 | 默认值 | 作用 |
| --- | ---: | --- |
| `stanley_k_path` | `0.45` | 常规路径段的 Stanley 横向增益 |
| `stanley_k_near_target` | `0.42` | 距目标小于 1.3 m 时的 Stanley 横向增益 |

`run.launch.py` 同时声明并转发视觉参数：

| 参数 | 默认值 | 单位/作用 |
| --- | ---: | --- |
| `visual_heading_gain` | `0.2` | 每度视觉航向误差产生的附加转向角 |
| `visual_lateral_gain` | `10.0` | 每米视觉横向误差产生的附加转向角 |
| `visual_max_steering_deg` | `3.0` | 视觉附加转向角限幅 |
| `visual_confidence_threshold` | `0.5` | 视觉样本最低置信度 |
| `visual_timeout_sec` | `0.5` | 视觉样本有效期 |

RTK 和视觉参数都通过 `ParameterValue(LaunchConfiguration(...), value_type=float)` 转发，启动时可以单独覆盖。例如：

```text
ros2 launch rtk_nav run.launch.py enable_visual_correction:=false stanley_k_path:=0.35
ros2 launch rtk_nav run.launch.py enable_visual_correction:=true visual_heading_gain:=0.1 visual_lateral_gain:=6.0 visual_max_steering_deg:=2.0
```

## 数据流与行为

`run.launch.py` 声明参数并传给 `rtk_nav`。节点初始化时读取两个 Stanley 参数，`get_adaptive_stanley_k()` 只根据当前距离选择 `stanley_k_near_target` 或 `stanley_k_path`。视觉纠偏仍通过已有 `get_visual_steering_correction()` 独立计算，最后叠加到 RTK Stanley 结果；视觉关闭时附加项为零。

## 测试策略

契约测试覆盖：

1. launch 声明了两个 RTK Stanley 参数和五个视觉参数，并以浮点类型转发。
2. RTK 节点声明并读取两个 Stanley 参数。
3. 近目标和常规距离分别选择对应的 Stanley 增益。
4. 原有视觉总开关、视觉门控和视觉纠偏计算测试继续通过。

不在无 ROS2 硬件环境中执行摄像头、RTK 或底盘闭环测试；补充 AST、`py_compile`、契约测试和 `git diff --check`。
