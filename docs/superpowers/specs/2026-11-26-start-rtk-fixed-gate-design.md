# START 启动前 RTK 固定解门控设计

## 意图

START 出仓任务当前只检查电量，仓内 RTK 非固定解时仍可能启动任务。启动前增加与自动导航一致的 RTK 双 Fixed 且定位数据有效门控，避免在无法满足导航定位条件时执行出仓。

## 目标

- `START` 仅在最新 WTRTK 样本满足 `position_status == 4`、`fix_status == 4` 且 `position_data_valid` 为真时继续执行。
- 非固定解或定位数据无效时拒绝 START，保持原状态。
- 拒绝时复用已有 `ERROR_RTK_NOT_FIXED = 4`，通过现有状态发布链路上报。
- 固定解通过后继续执行现有电量门控和 START 流程。

## 非目标

- 不修改 RTK 导航运行中的暂停、恢复和故障清除逻辑。
- 不新增故障码、话题、参数或配置项。
- 不改变独立 `UNLOADING` 流程的既有门控。

## 方案

在 `MotorControlNode` 中保存最新 WTRTK 的定位数据有效标志，并在 `switch_state()` 的 START 门控处使用定位状态、定向状态和有效标志进行判断。失败时置位已有 RTK 非固定解故障码，发布状态后立即返回。

## 验证

- 回归测试覆盖双 Fixed 且数据有效时允许 START。
- 回归测试覆盖定位非 Fixed、定向非 Fixed、定位数据无效时拒绝 START并上报故障码。
- 运行新增测试及 `motor_control` 相关测试。
- 在 `FIX_SUMMARY.md` 记录离线验证结果，现场验证状态明确标注为未执行。
