# 路径感知固定边界线检测设计

## 目标

修正 `line_detector_node.py` 当前使用所有 Hough 线段平均中心计算横向偏移的问题。
横向偏移必须由当前清扫路径的横向边界线对计算，避免图像沿路径方向上下移动时把线段筛选变化误判为横向偏移。

## 已确认约束

- 路径方向和车体航向来自 `/rtk/visual_path_context`，消息类型为 `geometry_msgs/Vector3`。
- `msg.x` 为当前路径绝对航向角，`msg.y` 为车体航向角，`msg.z >= 0.5` 表示上下文有效。
- 机器人可能沿不同方向清扫，不能把图像 X 轴永久当作横向轴。
- 输出协议保持不变：`/grid_line/angle_deviation` 的 `x` 为航向偏差、`y` 为横向偏差（米）、`z` 为检测有效标志。
- 当前 `camera_height`、`camera_pitch_deg` 和经验标定的 `focal_length_px` 保留；本次只改变几何取样方式，不重新解释相机物理内参。
- 无效上下文、线组不足或边界无法配对时必须发布无效结果（`z=0`、置信度为 0），不能发布有效的零偏差。

## 方案

视觉节点订阅 `/rtk/visual_path_context`，计算路径轴相对相机图像的方向：

```text
relative_path_heading = wrap180(path_direction_deg - vehicle_heading_deg)
directed_path_axis_image = wrap180(
    90.0 - relative_path_heading + camera_angle_offset
)
```

每条 Hough 线段保留无向角度，并按与路径轴的最小无向夹角分组：

```text
parallel_group      与路径方向平行，用于左右边界
perpendicular_group 与路径方向垂直，用于航向偏差
```

横向偏移计算步骤：

1. 对 `parallel_group` 中的线段中心点沿路径法向投影。
2. 以图像中心的法向投影为分界，分别寻找最近的左、右边界线。
3. 限制左右边界间距在 `boundary_pair_max_gap_px` 内。
4. 用两条边界投影的中点减去图像中心投影，得到带符号横向像素误差。
5. 使用现有高度、俯角和焦距参数换算为米制横向误差。

这样，沿路径轴方向的图像平移不会改变边界法向投影；只有沿路径法向的移动才会改变横向误差。

航向偏差继续使用 `perpendicular_group` 的长度加权平均方向，并相对期望的路径法向计算有符号角度误差。

## 状态与安全

- 路径上下文超时后清空有效检测状态并发布无效结果。
- 路径方向变化时清空旧方向的有效帧计数，连续满足几何条件 `reacquire_frames` 帧后才恢复有效输出。
- `run_axis` 继续作为独立诊断话题发布 `vertical`、`horizontal` 或 `invalid`。
- 不改变 `rtk_nav.py` 的视觉纠偏门控、Stanley 控制和底盘安全状态机。

## 参数

保留现有参数，并使用以下新增/已有参数控制几何筛选：

| 参数 | 默认值 | 作用 |
| --- | ---: | --- |
| `line_angle_tolerance_deg` | `15.0` | 平行/垂直线组角度容差 |
| `path_context_timeout_sec` | `0.5` | 路径上下文最大有效间隔 |
| `boundary_pair_max_gap_px` | `1200.0` | 左右边界最大间距 |
| `reacquire_frames` | `3` | 换向后连续有效帧数 |

## 测试与验收

添加不依赖 ROS 运行时的几何契约测试，覆盖：

- 路径轴为 0°、90°、180°、270° 时线组分类稳定；
- 左右对称边界的横向误差为 0；
- 沿路径轴平移边界线，横向误差不变；
- 沿法向平移边界线，横向误差按预期改变且正负方向稳定；
- 路径上下文无效或过期时不输出有效偏移；
- 路径方向改变 90° 后清空旧检测状态。

在当前 Windows 环境执行 AST、契约测试和 `git diff --check`；ROS2、真实相机和底盘闭环测试留给 Ubuntu 现场环境。
