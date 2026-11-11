# RTK 电子围栏越界 HOLD 与故障上报设计

## 意图

当 RTK 导航节点连续 3 帧确认机器人位于电子围栏允许区域外时，立即请求底盘进入 `HOLD`，并将电子围栏故障码加入现有 `/robot_state` JSON，由 MQTT 桥接继续上报云端。

## 方案

采用导航节点触发、底盘节点执行的方案：

- `rtk_nav.py` 保留现有连续 3 帧确认和锁存逻辑。
- 确认越界后发布 `/keyboard/control` 的 `HOLD` 指令，使 `motor_control.py` 复用现有 HOLD 状态机。
- 同时发布独立电子围栏故障码话题，避免把安全事件编码进控制指令。
- `motor_control.py` 订阅故障码，锁存 `ERROR_GEOFENCE_OUTSIDE = 129`，并将故障码/原因加入现有 `robot_state` JSON。
- 清除故障需要人工复位；仅恢复定位或重新进入围栏不自动解除 HOLD，避免机器人未经确认自行恢复运动。

## 接口约定

- 故障码话题：`/rtk/geofence_fault_code`，消息类型 `std_msgs/Int16`。
- 故障码：`1024`，含义 `ERROR_GEOFENCE_OUTSIDE`，作为现有位掩码 `512` 后的下一位。
- 状态上报字段：沿用现有故障字段结构，增加电子围栏故障码和原因 `geofence_outside`。
- 触发时只发布一次故障事件，后续保持锁存；状态周期上报继续携带该故障。

## 非目标

- 不改变电子围栏几何生成、区域膨胀半径或运行时允许区域判定。
- 不新增自动越界恢复或自动解除 HOLD。
- 不改变 MQTT 主题，由现有 `/robot_state` 桥接负责上报。

## 验证

- 单元测试覆盖：连续 3 帧触发、未满 3 帧不触发、故障码锁存、HOLD 指令发布和状态 JSON 字段。
- 运行 `rtk_nav` 相关测试与 `motor_control` 相关测试。
