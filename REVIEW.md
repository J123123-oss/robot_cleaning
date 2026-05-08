# 项目 Review

## 目标

这份 review 用于两件事：

1. 快速定位当前项目中最容易阻塞联调和上线的问题
2. 快速确认“这个机器人到底要做到什么程度”以及哪些需求还没有被明确

适用对象：

- 代码接手人
- 现场联调人员
- 产品/项目负责人
- 需要排查“为什么跑不起来/为什么行为不符合预期”的开发人员

---

## 一、优先处理的问题

### P0: MQTT 桥接支持远程执行终端命令，存在高风险

位置：

- [src/mqtt_ros2/mqtt_ros2/mqtt_ros2_bridge.py](D:/Li_2025_06_30/资料合集/robot_cleaning-0402_branch/src/mqtt_ros2/mqtt_ros2/mqtt_ros2_bridge.py:147)
- [src/mqtt_ros2/mqtt_ros2/mqtt_ros2_bridge.py](D:/Li_2025_06_30/资料合集/robot_cleaning-0402_branch/src/mqtt_ros2/mqtt_ros2/mqtt_ros2_bridge.py:176)
- [src/mqtt_ros2/mqtt_ros2/mqtt_ros2_bridge.py](D:/Li_2025_06_30/资料合集/robot_cleaning-0402_branch/src/mqtt_ros2/mqtt_ros2/mqtt_ros2_bridge.py:217)

现象：

- 收到 MQTT `topic_command` 后会直接执行本机 shell 命令
- 使用了 `subprocess.Popen(..., shell=True, ...)`
- 执行结果会回传 MQTT

影响：

- 只要 MQTT 凭据泄露、主题权限配置过宽，远端就可以直接控制机器人主机
- 这是上线级别风险，不只是“代码风格问题”

建议：

- 如果不是刚需，直接移除远程 shell 执行能力
- 如果必须保留，至少增加命令白名单、鉴权、审计和开关配置
- 生产环境应默认关闭

### P1: 启动配置强依赖固定设备路径和固定文件路径，可移植性很差

位置：

- [src/rtk_nav/launch/run.launch.py](D:/Li_2025_06_30/资料合集/robot_cleaning-0402_branch/src/rtk_nav/launch/run.launch.py:27)
- [src/rtk_nav/launch/run.launch.py](D:/Li_2025_06_30/资料合集/robot_cleaning-0402_branch/src/rtk_nav/launch/run.launch.py:36)
- [src/rtk_nav/launch/run.launch.py](D:/Li_2025_06_30/资料合集/robot_cleaning-0402_branch/src/rtk_nav/launch/run.launch.py:46)
- [src/rtk_nav/launch/run.launch.py](D:/Li_2025_06_30/资料合集/robot_cleaning-0402_branch/src/rtk_nav/launch/run.launch.py:60)
- [src/rtk_nav/launch/run.launch.py](D:/Li_2025_06_30/资料合集/robot_cleaning-0402_branch/src/rtk_nav/launch/run.launch.py:78)
- [src/rtk_nav/launch/run.launch.py](D:/Li_2025_06_30/资料合集/robot_cleaning-0402_branch/src/rtk_nav/launch/run.launch.py:89)

现象：

- 串口设备名写死为 `/dev/ttyS4`、`/dev/laser`、`/dev/charging`、`/dev/WTRTK`
- RTK 路径文件写死为 `/home/ztl/...`
- 录包回放文件也写死为 `/home/ztl/...`

影响：

- 换一台设备、换用户名、换部署目录后，launch 很可能直接失效
- 现场排障时不容易分清是“功能问题”还是“环境路径问题”

建议：

- 把串口、路径文件、MQTT 参数统一改成 launch 参数或 yaml 参数
- 给一份“开发环境默认值”和“一线部署默认值”

### P1: README 编码异常，文档可读性不足，交接成本高

位置：

- [README.md](D:/Li_2025_06_30/资料合集/robot_cleaning-0402_branch/README.md:1)

现象：

- README 出现明显乱码
- 虽然能勉强看出流程，但无法作为稳定交付文档使用

影响：

- 新人很难快速启动
- 需求、流程、操作步骤容易和代码实际行为脱节

建议：

- 统一转为 UTF-8
- 把“启动步骤、模式切换、充电指令、路径文件说明、节点关系”重新整理

### P1: 配置和业务逻辑耦合较深，需求变更成本高

主要表现：

- 设备路径、MQTT 地址、账号密码、路径文件都混在代码或 launch 里
- 清扫流程、进仓逻辑、充电恢复阈值等大量业务参数直接写在节点内部

影响：

- 需求稍有变化就需要改代码，而不是改配置
- 同一套程序适配不同机器人/园区会比较痛苦

建议：

- 抽出一层“部署配置”
- 再抽出一层“行为策略配置”

### P2: 项目存在较多“现场定制痕迹”，标准化边界不清

主要表现：

- 多处使用固定机器人 ID、固定 MQTT Broker、固定账号密码
- 路径规划输出目录和运行目录是特定 Linux 用户路径
- launch 中同时保留大量注释掉的历史路径

影响：

- 当前仓库更像“某一台/某一批机器人”的工程快照
- 后续如果要复用到别的机器人，容易出现隐性兼容问题

建议：

- 明确哪些是“项目通用能力”
- 明确哪些是“当前客户/场地定制”

---

## 二、从代码看，当前需求大概率已经包含的能力

下面这些能力，从代码实现看基本是“已做过”的，而不是纯概念：

- 机器人底盘运动控制
- 键盘控制、遥控器控制、RTK 自动模式切换
- RTK 多航点导航
- 路径文件加载与切换
- 激光辅助进仓对位
- 出仓、进仓状态机
- 电池、电流、电压、温度采集
- 485 充电模块启停与故障查询
- MQTT 指令桥接与状态上报
- 区域路径规划与多区域路径拼接

这说明项目目标不是单点 demo，而是接近整机联调状态。

---

## 三、必须尽快确认的需求

下面这些问题不确认清楚，后续任何 bug 排查都会反复绕圈。

### 1. 控制权优先级

需要确认：

- 遥控、键盘、MQTT、RTK，谁优先级最高
- 抢占后是否允许自动恢复
- 进入 `HOLD`、`DISABLE`、`AUTO_CLEANING` 的准入条件分别是什么

### 2. RTK 导航成功判定

需要确认：

- 到点阈值到底是多少
- 航向误差允许范围是多少
- RTK 非固定解时是暂停、减速，还是立即退出
- 丢星/漂移时是否允许继续靠 IMU 惯导短时运行

### 3. 边界触发后的动作策略

需要确认：

- 六路 IO 传感器各自代表什么物理位置
- 触边后是立即后退、绕行，还是记作障碍点
- 目前逻辑是“保护避让”还是“完整绕障”

### 4. 进仓/回仓完成条件

需要确认：

- 进仓完成是按激光差值、距离阈值，还是充电接触状态判定
- 回仓失败时是否允许重试
- 重试次数和失败上报机制是什么

### 5. 充电策略

需要确认：

- 自动开始充电的前提条件
- 满充判定到底用电压电流、故障码还是 BMS 反馈
- 满充后恢复充电阈值是否固定为当前代码里的逻辑

### 6. 路径规划输入规范

需要确认：

- 现场到底使用“单区域矩形参数”还是“三点标定多区域 yaml”
- 路径文件的生产责任在谁
- 路径文件切换是人工操作还是平台下发

### 7. MQTT 的职责边界

需要确认：

- MQTT 只负责业务命令，还是也负责运维命令
- 是否真的需要远程终端执行
- 云端和本地控制发生冲突时谁优先

---

## 四、推荐的快速排查顺序

如果现场出现“机器人不动 / 导航异常 / 回仓失败 / 无法充电”，建议按这个顺序排：

1. 先看设备层  
   串口名、CAN、485、激光、RTK 设备是否都存在

2. 再看 launch 配置  
   路径文件、串口参数、MQTT 参数是否匹配当前机器

3. 再看 ROS 话题流  
   是否能看到 `/fix`、`/wtrtk_data`、`/io_data`、`/battery_data`、`/laser_distance`

4. 再看模式状态  
   当前是在 `NORMAL`、`REMOTE`、`AUTO_CLEANING`、`HOLD` 还是 `DISABLE`

5. 最后再看算法逻辑  
   包括 RTK 到点、边界纠偏、进仓对位和充电恢复

---

## 五、建议补齐的文档

为了让后续定位更快，建议下一步补这 4 份文档：

- `README.md`：重写启动说明和整体流程
- `TOPICS.md`：列出所有 ROS topic / service / parameter
- `DEPLOY.md`：列出串口映射、驱动、systemd、部署步骤
- `REQUIREMENTS.md`：把控制权、回仓、充电、异常处理规则写清楚

---

## 六、结论

这个项目的主体功能已经比较完整，但当前最大的阻碍不是“没有功能”，而是：

- 运维与安全边界不清
- 配置硬编码较多
- 文档和需求边界不够清晰

如果目标是“能快速接手并稳定联调”，优先级建议是：

1. 先修文档和配置入口
2. 再确认控制权、回仓、充电三类核心需求
3. 最后再做算法细节优化
