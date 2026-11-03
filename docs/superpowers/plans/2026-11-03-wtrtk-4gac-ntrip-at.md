# WTRTK-4GAC NTRIP AT 配置 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** 将 NTRIP 配置工具改为 WTRTK-4GAC 的 AT 协议，并通过 AT 查询把当前配置备份为 JSON。

**Architecture:** 保留单文件独立 CLI，抽出 AT 命令发送、按行响应读取和响应解析函数。配置模式负责写入和保存；备份模式负责查询并序列化。串口读取忽略异步 NMEA/WTRTK 行，只接受目标 `+...=` 响应。

**Tech Stack:** Python 3、pyserial、argparse、unittest、JSON。

**Spec:** `docs/superpowers/specs/2026-11-03-wtrtk-4gac-ntrip-at-design.md`

## Global Constraints

- 每条 AT 命令以 `\\r\\n` 结尾。
- 服务器地址最多 31 字节，CORS 账号密码最多 31 字节，挂载点最多 23 字节。
- `--backup` 模式不要求配置参数；写入必须显式使用 `--apply`。
- 现场运行前停止其他占用 `/dev/WTRTK` 的进程。

## Review Focus

- 异步 `$WTRTK`/NMEA 输出混入响应时，仍能找到目标 AT 响应行；测试：`test_read_at_response_skips_unsolicited_lines`。
- `+ADDR` 的域名和端口解析；测试：`test_parse_addr_response`。
- `+CORS` 中密码含逗号时保持账号与剩余密码；测试：`test_parse_cors_response_preserves_commas_in_password`。
- 字段超过设备限制或包含非 ASCII 字符时，在发送前失败；测试：`test_validate_config_limits`。
- 缺失查询响应或设备返回 `ERROR` 时，备份失败且不生成 JSON；测试：`test_read_config_rejects_error_response`。

### Task 1: AT 协议纯函数与测试

**Files:**
- Modify: `tools/wtrtk_ntrip_config.py`
- Test: `tests/test_wtrtk_ntrip_config.py`

**Interfaces:**
- Produces `Config(domain, port, username, password, mountpoint)`.
- Produces `validate_config(cfg)`, `parse_at_response(command, response)`, and response-line helpers used by serial code.

- [ ] **Step 1: Write failing tests** for CRLF command encoding, response parsing, field limits, comma-preserving password parsing, and error responses.
- [ ] **Step 2: Run** `python -m unittest discover -s tests -p 'test_wtrtk_ntrip_config.py' -v`; confirm the new tests fail against the Modbus implementation.
- [ ] **Step 3: Implement** AT constants, `Config`, validation, command encoding, and parsers without opening a serial port.
- [ ] **Step 4: Run** the focused unittest file; confirm all pure-function tests pass.

### Task 2: AT serial workflow and backup

**Files:**
- Modify: `tools/wtrtk_ntrip_config.py`
- Test: `tests/test_wtrtk_ntrip_config.py`

**Interfaces:**
- `send_at(ser, command, dry_run) -> list[str]` sends `<command>\\r\\n` and reads line-oriented responses.
- `read_config(ser) -> dict[str, str | int]` queries `AT+ADDR?`, `AT+CORS?`, and `AT+MP?`.
- `backup_config(ser, path)` writes the returned dictionary as mode-restricted JSON.

- [ ] **Step 1: Write failing tests** for scripted serial responses, asynchronous line filtering, read-config mapping, and no backup file on errors.
- [ ] **Step 2: Run** the focused unittest file; confirm the serial workflow tests fail.
- [ ] **Step 3: Implement** `send_at`, configuration-mode entry/exit, `read_config`, `backup_config`, and `configure` using `AT+CFG`, `AT+ADDR=`, `AT+CORS=`, `AT+MP=`, `AT+SAVE`, and `AT+EXIT`.
- [ ] **Step 4: Run** the focused unittest file; confirm all workflow tests pass.

### Task 3: CLI and final verification

**Files:**
- Modify: `tools/wtrtk_ntrip_config.py`
- Test: `tests/test_wtrtk_ntrip_config.py`

- [ ] **Step 1: Write failing CLI tests** for `--backup` without config arguments, configure-mode required arguments, dry-run output, and `--apply` dispatch.
- [ ] **Step 2: Run** focused tests and confirm failures.
- [ ] **Step 3: Update** help text, argument validation, exception handling, and `main()` to use the AT workflows.
- [ ] **Step 4: Run** `python -m unittest discover -s tests -p 'test_wtrtk_ntrip_config.py' -v`.
- [ ] **Step 5: Run** `python -m py_compile tools/wtrtk_ntrip_config.py tests/test_wtrtk_ntrip_config.py`.
- [ ] **Step 6: Inspect** the final diff and report that live `/dev/WTRTK` execution remains an on-device verification step.
