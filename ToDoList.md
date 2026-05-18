**Stanley 调试 TODO**

- [ ] 确认电机符号约定  
  前进是否为 `left=-v, right=+v`，左转/右转是否为左右同号。

- [ ] 修正 Stanley 差速映射  
  重点检查 [rtk_nav.py](/home/ubuntu/robot_cleaning/src/rtk_nav/rtk_nav/rtk_nav.py:1149)：
  ```python
  left_speed = -velocity - speed_diff
  right_speed = velocity + speed_diff
  ```
  优先尝试改为：
  ```python
  left_speed = -velocity + speed_diff
  right_speed = velocity + speed_diff
  ```

- [ ] 验证转向方向  
  人为让车偏离路径左/右两侧，观察 `speed_diff` 是否让车往路径方向回正。若方向反了，只反一个量：`speed_diff` 或 `lateral_error`。

- [ ] 航点切换时重置 Stanley 路径缓存  
  在航点切换、重置导航、切换路线时清理：
  ```python
  self.stanley_path_start = None
  self.stanley_path_direction = None
  ```

- [ ] 增加 Stanley 调试日志  
  打印：
  ```text
  waypoint_idx, distance, path_start, path_end,
  path_direction, imu_yaw, heading_error,
  lateral_error, k, steering_correction,
  total_steering, speed_diff, left_speed, right_speed
  ```

- [ ] 单段直线测试  
  用 5-10m 直线路径测试，确认车辆能稳定贴线，而不是只前进不纠偏。

- [ ] 偏离路径测试  
  车体放在线左侧/右侧约 0.3m，分别验证回正方向。

- [ ] 再调参数  
  初步建议：
  ```python
  MAX_LATERAL_ERROR = 0.3   # 现为 0.15，可先放宽
  STRAIGHT_MAX_CORRECTION = 3.0
  STANLEY_K_BASE = 0.4
  LOW_DISTANCE = 1.5
  ```

- [ ] 检查线段误差计算  
  当前 `calculate_lateral_error()` 计算的是无限延长线误差，`t` 算了但没用。若到航点附近容易被上一段“拉住”，改成基于线段投影点的横向误差。

- [ ] 实车记录日志并复盘  
  保存一次完整运行日志，重点看：航点切换后 `path_start/path_direction` 是否正确，`speed_diff` 是否连续且方向合理。