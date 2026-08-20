# RTK 动态换向视觉纠偏设计

## 目标

在不使用光流法的前提下，让 `line_detector_node.py` 支持 RTK 清扫路径的
动态换向，并输出与当前路径段一致的视觉航向偏差和横向偏移量。

核心原则是：

```text
rtk_nav 提供当前正在跟踪的路径段方向
line_detector 根据该方向选择平行线组和垂直线组
视觉只测量相对当前路径的航向误差和横向误差
```

视觉节点不自行读取航点文件，也不使用实时车头方向决定应该选择哪组线。
实时车头方向只用于把 RTK 世界坐标中的路径方向转换到相机坐标系。

## 当前代码事实与边界

- `rtk_nav.py` 已在 Stanley 路段初始化时计算
  `self.stanley_path_direction`。
- `self.stanley_path_direction` 是当前活动路径段的绝对方位角，角度约定为
  0° 北、顺时针增加，与现有 `imu_yaw` 基准一致。
- 当前 `line_detector_node.py` 只保留数量最多的一个角度组，并且把接近水平
  和接近竖直的线折叠到同一角度范围，不能区分两组线。
- 当前横向偏移始终使用图像 X 方向，不能适配横向运行。
- `line_detector_node.py` 发布的视觉误差由 `rtk_nav.py` 消费，但只作为受限的
  Stanley 附加修正；RTK 航向误差和 GPS 横向误差仍是主控制量。
- `optical_flow_displacement_node.py` 不修改、不参与该方案。

## 架构

### 视觉纠偏总开关

由 `run.launch.py` 声明全局启动参数：

```text
enable_visual_correction:=false
```

默认值必须为 `false`。该参数传递给 `rtk_nav` 的同名 ROS 参数，由导航节点
统一作为视觉纠偏链路的总门控：

- `false`：不发布有效视觉路径上下文，RTK 原有导航、Stanley 控制和安全状态机
  保持原行为；
- `true`：允许发布有效视觉路径上下文，供已注册的视觉节点使用；
- 无论开关状态如何，RTK 质量、导航状态、航向门控和边界安全条件仍然独立
  生效，视觉开关不能绕过这些条件。

`line_detector_node.py` 应注册为 `rtk_nav` 包的 `line_detector_node` 可执行入口，
并由 `run.launch.py` 使用同一个参数通过 `IfCondition` 启动。这样关闭开关时节点
不会启动，开启开关时节点和 RTK 上下文使用同一配置，避免出现 launch 看似开启但
节点实际未运行的假配置。

### 新增 RTK 话题

由 `rtk_nav.py` 发布：

```text
/rtk/visual_path_context   geometry_msgs/Vector3
```

字段定义：

```text
x = path_direction_deg     当前活动路径段绝对方位角，单位：度
y = vehicle_heading_deg    当前 RTK/INS 车体航向，单位：度
z = valid                   1.0=可用于视觉选择，0.0=无效
```

发布频率建议为 10 Hz，与导航定时器一致。路径方向或导航状态变化时立即刷新；
没有活动路径段、没有有效车体航向、RTK 质量不满足现有导航要求，或导航处于
`IDLE`/`COMPLETED` 等不需要视觉纠偏的状态时，发布 `z=0`。

### 路径段方向来源

路径方向必须来自 `rtk_nav` 当前真正用于 Stanley 控制的活动路径段，不能读取
航点文件中的 waypoint heading 代替路径段方位角。

路段切换时执行以下顺序：

1. 确定新的路径段起点和终点；
2. 计算起点到终点的方位角；
3. 更新 `self.stanley_path_direction`；
4. 发布新的 `/rtk/visual_path_context`；
5. 视觉节点在收到新上下文后清空上一运行轴的线组缓存，避免换向瞬间沿用旧
   方向的检测结果。

在现有 `current_waypoint_idx` 语义下，活动路段使用 Stanley 已确定的
`stanley_path_start -> target_waypoint`。如果后续将索引语义调整为“当前航点”，
则必须保持同一约定：`当前航点 -> 下一个航点` 的方位角就是视觉路径方向。
两者不能混用。

## 视觉坐标与运行轴

### 路径方向转换

视觉节点收到 RTK 上下文后计算：

```text
relative_path_heading = wrap180(path_direction_deg - vehicle_heading_deg)
```

再叠加相机安装航向偏置：

```text
path_heading_camera = wrap180(
    relative_path_heading + camera_angle_offset
)
```

`camera_angle_offset` 的正负方向必须通过现场标定确认，且与现有
`camera_angle_offset` 的发布修正约定统一，不能在检测函数和回调中重复扣除。

路径方向用于选择线组时按 180° 周期比较，因为一条栅格线没有正反方向：

```text
axis_angle = wrap180(path_heading_camera)
axis_angle_undirected = axis_angle modulo 180°
```

### 两组线的定义

对 Hough 线段保留原始图像角度，不再使用当前实现中把水平和竖直线都归一化为
0° 的逻辑。每条线段归入：

```text
parallel_group      与当前路径轴平行的线组
perpendicular_group 与当前路径轴垂直的线组
```

分组依据是线段方向与 `axis_angle` 的最小无向夹角：

```text
parallel_error = undirected_angle_distance(line_angle, axis_angle)
perpendicular_error = undirected_angle_distance(
    line_angle, axis_angle + 90°
)
```

仅当最小误差不超过配置阈值时才进入对应线组。推荐初始阈值为 15°，实际值
通过现场数据调整。不能用“线段数量最多”替代路径轴匹配；数量最多只用于同一
目标方向内的置信度排序。

## 纠偏量计算

### 航向偏差

航向偏差使用 `perpendicular_group`，因为跨越组件的拼缝应当与机器人运行方向
正交。

对该组线段按长度加权计算平均方向，并计算其相对于期望垂直方向的有符号误差：

```text
expected_cross_angle = path_heading_camera + 90°
heading_error = wrap180(line_mean_angle - expected_cross_angle)
```

最终将误差限制在 `[-90°, 90°]`，避免线段端点方向反转造成 180° 跳变。

若当前相机安装和投影模型使“图像线角”与实际平面角存在明显透视偏差，先在
固定相机安装下使用标定直线拟合修正；不能直接把未标定的图像角度声称为真实
车体航向误差。

### 横向偏移

横向偏移使用 `parallel_group`，因为它表示沿当前运行方向延伸的通道边界或板缝。

不能继续使用所有检测线段中心点的平均值。应在当前运行轴的横向坐标上：

1. 将平行线段投影到与路径轴正交的图像坐标；
2. 在图像中心左右分别寻找最近的有效边界线；
3. 计算左右边界的中点作为通道中心；
4. 计算通道中心相对图像中心的横向像素偏移；
5. 使用针孔模型和相机俯角/高度换算为米。

```text
channel_center = (left_boundary + right_boundary) / 2
lateral_pixel_error = channel_center - image_center
lateral_m = calibrated_pixel_to_meter(lateral_pixel_error)
```

如果只检测到一侧边界，不能假设图像中心就是通道中心。此时可使用已标定的
通道宽度进行单边估计，但置信度必须降低；如果没有可靠的单边模型，则发布
无效结果。

对于横向运行，横向坐标不再固定为图像 X 方向，而是使用路径轴的正交方向。
因此同一套投影计算同时适用于纵向运行和横向运行。

## Stanley 融合

`rtk_nav.py` 订阅 `/grid_line/angle_deviation` 和
`/grid_line/detection_confidence`。视觉样本只有在 `detected=1`、置信度达到
阈值且未超过超时时间时才有效。有效样本产生独立的附加转向角：

```text
visual_correction = visual_lateral_gain * lateral_error_m
                    - visual_heading_gain * heading_error_deg
```

附加转向角限制在 `[-visual_max_steering_deg, visual_max_steering_deg]`，再与
现有 Stanley 结果相加，并继续使用原有总转向 `[-45°, 45°]` 限幅。视觉修正不
替换 RTK 横向误差或 RTK 航向误差。

视觉修正的运行门控包括：视觉总开关、RTK Fixed 就绪、`AUTO_CLEANING`、
`INITIAL_MOVE`/`WAYPOINT_MOVE`、边界矫正未锁定、未处于原地校准/几何撤退/强制
方位角模式。任一条件不满足时附加转向角为零。

## 输出接口

保持现有话题：

```text
/grid_line/angle_deviation   geometry_msgs/Vector3
```

字段定义保持：

```text
x = heading_error_deg
y = lateral_error_m
z = detected
```

建议增加诊断字段到独立话题，避免改变已有消费者协议：

```text
/grid_line/run_axis           std_msgs/String
    "vertical" / "horizontal" / "invalid"，仅用于诊断当前路径轴

/grid_line/detection_confidence std_msgs/Float32
```

`detected=1` 的必要条件：

- RTK 路径上下文有效且未过期；
- 平行线组达到最少数量；
- 垂直线组达到最少数量；
- 横向边界或单边估计满足配置的几何质量条件；
- 航向误差和横向误差均为有限数值。

否则：

```text
x = 0.0
y = 0.0
z = 0.0
confidence = 0.0
```

视觉节点不能将“没有检测到线”报告成有效的零偏差。

## 换向与状态处理

换向期间，`path_direction_deg` 发生接近 90° 或 180° 的变化时：

1. 视觉节点检测到路径上下文版本/方向变化；
2. 清空旧方向的平行线、垂直线、边界配对和角度平滑缓存；
3. 进入短暂重捕获阶段；
4. 新方向的两组线连续满足质量条件后才重新发布 `detected=1`。

路径方向相差 180° 时，线组选择不应变化，因为运行轴相同；但有符号横向误差
的正负必须依据新的路径方向重新定义，确保 RTK 控制器的左右纠偏符号一致。

原地旋转或 `WAYPOINT_CALIB` 期间不输出可用于行驶纠偏的视觉误差，除非后续
明确增加“校准中视觉辅助”的独立协议。视觉检测有效不等于允许底盘运动。

## 参数

保留现有参数并补充：

```text
camera_angle_offset       相机航向安装偏置
camera_height              相机距板面高度
camera_pitch_deg           相机俯角
focal_length_px            焦距
min_line_count              每组最少线段数
line_angle_tolerance_deg   线组匹配角度阈值，默认 15°
path_context_timeout_sec   RTK上下文超时，默认 0.5s
boundary_pair_max_gap_px   左右边界最大允许间隔
reacquire_frames           换向后连续有效帧数，默认 3
```

## 错误处理与安全边界

- RTK 上下文超时或无效时停止发布有效视觉纠偏。
- 角度组不明确、左右边界无法配对、相机参数非法时发布无效结果并记录节流
  警告。
- 视觉节点不直接发布 `/rtk/motor_speed`；`rtk_nav` 只在上述门控通过时把视觉
  附加项加入 Stanley，不绕过 RTK 固定解、航向门控、边界传感器和暂停状态。
- 本设计不改变 `motor_control` 的安全判断和现有 RTK 状态机。

## 测试与验收

### 静态/单元测试

- 路径方向 0°、90°、180°、270° 时，平行/垂直线组选择正确；
- 角度跨越 0/360° 时 `wrap180` 不产生跳变；
- 路径方向变化 90° 后旧线组缓存被清空；
- RTK 上下文超时后 `detected=0`；
- 缺少任一线组时不发布有效零偏差；
- 左右边界配对计算的横向偏移正负符合车体坐标约定；
- 现有相机安装偏置只应用一次。

### 录包/离线图像验收

至少准备四类数据：

1. 纵向直行，机器人位于通道中心；
2. 纵向左偏、右偏；
3. 横向直行，机器人位于通道中心；
4. 纵向到横向换向以及横向到纵向换向。

每类数据应记录：

```text
RTK路径段方向
RTK车体航向
RTK/测量得到的横向真实偏差
RTK/人工测量得到的航向真实偏差
视觉输出 heading_error、lateral_error、confidence、detected
```

验收重点不是“检测线段数量最多”，而是：

- 换向前后选择的线组与当前路径轴一致；
- 视觉角度误差和横向误差的正负方向稳定；
- 误差量与实测偏差保持单调关系；
- 视觉无效时不会输出可被误认为零纠偏的有效消息。

### 运行验证边界

设计和单元测试不能证明实际 ROS、相机、RTK 或底盘闭环效果。现场验证应先以
视觉总开关关闭或视觉增益为零的方式确认话题和时间戳，再以低速、可急停条件
逐步启用小增益，核对视觉误差正负方向和左右轮修正方向后再提高增益。
