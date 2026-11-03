# WTRTK-4GAC NTRIP AT 配置设计

## 目标

将 `tools/wtrtk_ntrip_config.py` 从 WTRTK-4GA 的寄存器写入工具改为 WTRTK-4GAC 的 AT 配置工具，并支持读取当前 NTRIP 配置到 JSON。

## 范围

- 使用 4GAC 文档定义的 ASCII AT 命令：`AT`、`AT+CFG`、`AT+ADDR`、`AT+CORS`、`AT+MP`、`AT+SAVE`、`AT+EXIT`。
- `--backup PATH` 通过 `AT+ADDR?`、`AT+CORS?`、`AT+MP?` 查询当前配置并写 JSON。
- 配置流程使用 `AT+CFG` 进入配置模式，写入服务器地址/端口、账号密码和挂载点，执行 `AT+SAVE` 保存，最后退出配置模式。
- 保留 `--dry-run`，只打印将发送的 AT 命令，不打开串口；`--apply` 才实际写入。
- 保留密码交互输入，避免密码出现在命令行历史中。

## 非目标

- 不实现 4GAC 的 CAN 协议。
- 不修改 ROS2 串口节点或 NMEA/WTRTK 数据解析。
- 不把 MQTT 配置纳入本工具。

## 协议约束

- 每条 AT 命令以 `\\r\\n` 结尾。
- 查询响应按 `+ADDR=...`、`+CORS=...`、`+MP=...` 解析；允许同时存在异步 NMEA/WTRTK 文本，必须按行寻找目标响应。
- 字段长度由设备协议约束：服务器地址最多 31 字节，CORS 账号密码最多 31 字节，挂载点最多 23 字节；超限在发送前报错。
- 对串口响应设置超时；错误响应、超时和格式错误都返回非零退出码，不能静默生成备份。

## 接口

- `Config(domain: str, port: int, username: str, password: str, mountpoint: str)` 表示用户配置。
- `read_config(ser) -> dict[str, str | int]` 读取 `domain`、`port`、`username`、`password`、`mountpoint`。
- `backup_config(ser, path)` 调用 `read_config` 并以权限受限的 JSON 文件保存。
- CLI 保留 `--domain`、`--ntrip-port`、`--username`、`--password`、`--mountpoint`、`--apply`、`--dry-run`、`--backup`；仅备份模式不要求配置参数。

## 验证

- 测试 AT 命令编码、响应行解析、异步文本过滤、备份 JSON 内容和参数校验。
- 使用标准库 `unittest` 与 `py_compile`，不要求设备在线；现场运行前仍需停止其他占用 `/dev/WTRTK` 的进程。

## 方案

采用 AT 协议方案。文档同时列出了 Modbus，但 AT 命令是 4GAC 明确提供的 NTRIP 配置入口，能直接表达地址、端口、账号密码和挂载点，且读回接口完整，适合本工具的配置/备份职责。
