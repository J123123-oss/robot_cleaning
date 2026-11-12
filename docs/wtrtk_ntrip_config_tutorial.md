# WTRTK NTRIP 配置教程

工具默认使用串口设备 `/dev/WTRTK`。两个型号的默认波特率不同：

| 型号 | 工具 | 默认波特率 |
| --- | --- | ---: |
| WTRTK-4GA | `tools/wtrtk_ntrip_config_4ga.py` | `460800` |
| WTRTK-4GAC | `tools/wtrtk_ntrip_config_4gac.py` | `230400` |

## 1. 配置文件

`--restore` 使用 JSON 文件。4GA 使用设备原生字段名，4GAC 使用工具统一字段名。

4GA 格式示例：

```json
{
  "serverDomain": "cors.phibis.cn",
  "serverIp": "cors.phibis.cn",
  "serverPort": "8002",
  "account": "WT5100001603",
  "password": "4559",
  "mountpoint": "Auto"
}
```

密码以明文保存在文件中，请限制文件权限，不要提交到公共仓库。

## 2. 先做 dry-run

不加 `--apply` 时不会打开串口，只打印将要发送的 AT 命令：

```bash
python3 tools/wtrtk_ntrip_config_4ga.py \
  --restore wtrtk_ntrip_write_example.json
```

## 3. 写入 4GA

确认设备没有被其他程序占用后执行：

```bash
python3 tools/wtrtk_ntrip_config_4ga.py \
  --port-device /dev/WTRTK \
  --baud 460800 \
  --restore wtrtk_ntrip_write_example.json \
  --apply
```

4GA 固件对 `AT+CORS=` 和 `AT+NETRESET` 可能不返回回显。工具会记录警告并继续执行 `AT+MP`、`AT+SAVE` 和退出配置模式；这不代表已经通过网络完成 NTRIP 鉴权，仍需观察设备定位状态。

## 4. 写入 4GAC

4GAC 使用独立脚本和 `230400` 波特率：

4GAC 格式示例：

```json
{
  "domain": "103.143.19.54",
  "port": 8002,
  "username": "WT5100001603",
  "password": "4559",
  "mountpoint": "Auto"
}
```

```bash
python3 tools/wtrtk_ntrip_config_4gac.py \
  --port-device /dev/WTRTK \
  --baud 230400 \
  --domain 103.143.19.54 \
  --ntrip-port 8002 \
  --username WT5100001603 \
  --mountpoint Auto \
  --apply
```

省略 `--password` 时，工具会在终端交互输入密码，避免密码出现在 shell 历史中。

## 5. 查询和备份

4GAC 支持查询地址、账号和挂载点并保存为 JSON：

```bash
python3 tools/wtrtk_ntrip_config_4gac.py \
  --port-device /dev/WTRTK \
  --baud 230400 \
  --backup wtrtk_ntrip_backup.json
```

如果设备对 `AT+CORS?` 不返回数据，备份会失败；此时保留原始写入 JSON，后续使用 `--restore` 恢复。

## 6. 验证

```bash
python3 -m unittest discover -s tests -p 'test_wtrtk_ntrip_config_4ga.py' -v
python3 -m py_compile tools/wtrtk_ntrip_config_4ga.py tools/wtrtk_ntrip_config_4gac.py
```
