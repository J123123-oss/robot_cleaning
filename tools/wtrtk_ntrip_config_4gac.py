#!/usr/bin/env python3
"""Configure and inspect the WTRTK-4GAC NTRIP client over AT commands."""

from __future__ import annotations

import argparse
import getpass
import json
import os
import sys
import time
from dataclasses import dataclass

try:
    import serial
    from serial import SerialException
except ImportError as exc:  # pragma: no cover
    raise SystemExit("请先安装 pyserial: python3 -m pip install pyserial") from exc


AT_EOL = b"\r\n"
DEFAULT_TIMEOUT = 1.0
MAX_DOMAIN_BYTES = 31
MAX_CORS_BYTES = 31
MAX_MOUNTPOINT_BYTES = 23


@dataclass(frozen=True)
class Config:
    domain: str
    port: int
    username: str
    password: str
    mountpoint: str


def at_command(command: str) -> bytes:
    """Encode one 4GAC AT command with the required CRLF terminator."""
    if "\r" in command or "\n" in command:
        raise ValueError("AT 命令不能包含换行符")
    try:
        return command.encode("ascii") + AT_EOL
    except UnicodeEncodeError as exc:
        raise ValueError("AT 命令只能包含 ASCII 字符") from exc


def _validate_ascii_field(name: str, value: str, maximum: int, *, required: bool = True) -> None:
    if required and not value:
        raise ValueError(f"{name} 不能为空")
    if "\r" in value or "\n" in value:
        raise ValueError(f"{name} 不能包含换行符")
    try:
        length = len(value.encode("ascii"))
    except UnicodeEncodeError as exc:
        raise ValueError(f"{name} 只能包含 ASCII 字符") from exc
    if length > maximum:
        raise ValueError(f"{name} 超过设备限制 {maximum} 字节")


def validate_config(config: Config) -> None:
    _validate_ascii_field("NTRIP 服务器地址", config.domain, MAX_DOMAIN_BYTES)
    _validate_ascii_field("NTRIP 用户名", config.username, MAX_CORS_BYTES)
    _validate_ascii_field("NTRIP 密码", config.password, MAX_CORS_BYTES, required=False)
    _validate_ascii_field("NTRIP 挂载点", config.mountpoint, MAX_MOUNTPOINT_BYTES)
    if "," in config.username:
        raise ValueError("NTRIP 用户名不能包含逗号")
    if len(f"{config.username},{config.password}".encode("ascii")) > MAX_CORS_BYTES:
        raise ValueError(f"NTRIP 账号密码组合超过设备限制 {MAX_CORS_BYTES} 字节")
    if not 1 <= config.port <= 65535:
        raise ValueError("NTRIP 端口必须在 1 到 65535 之间")


def _parse_payload(line: str, prefix: str) -> str:
    if not line.startswith(prefix):
        raise ValueError(f"响应不是 {prefix} 格式: {line!r}")
    return line[len(prefix) :]


def parse_addr_response(line: str) -> tuple[str, int]:
    payload = _parse_payload(line, "+ADDR=")
    domain, separator, port_text = payload.rpartition(",")
    if not separator or not domain:
        raise ValueError(f"+ADDR 响应缺少服务器地址或端口: {line!r}")
    try:
        port = int(port_text)
    except ValueError as exc:
        raise ValueError(f"+ADDR 端口不是整数: {line!r}") from exc
    if not 1 <= port <= 65535:
        raise ValueError(f"+ADDR 端口超出范围: {line!r}")
    return domain, port


def parse_cors_response(line: str) -> tuple[str, str]:
    payload = _parse_payload(line, "+CORS=")
    username, separator, password = payload.partition(",")
    if not separator:
        raise ValueError(f"+CORS 响应缺少账号或密码分隔符: {line!r}")
    return username, password


def parse_mountpoint_response(line: str) -> str:
    mountpoint = _parse_payload(line, "+MP=")
    if not mountpoint:
        raise ValueError("+MP 响应中的挂载点为空")
    return mountpoint


def _response_line(raw: bytes) -> str:
    return raw.decode("ascii", errors="replace").strip("\x00\r\n ")


def read_at_response(
    ser: serial.Serial,
    expected_prefixes: tuple[str, ...],
    timeout: float | None = None,
) -> str:
    """Scan the mixed binary/text stream until a target AT response is found."""
    serial_timeout = float(getattr(ser, "timeout", DEFAULT_TIMEOUT) or DEFAULT_TIMEOUT)
    response_timeout = max(0.1, timeout if timeout is not None else serial_timeout)
    deadline = time.monotonic() + response_timeout
    received = bytearray()
    markers = tuple(prefix.encode("ascii") for prefix in expected_prefixes)
    while time.monotonic() < deadline:
        chunk = ser.read(64)
        if not chunk:
            continue
        received.extend(chunk)
        for marker in markers:
            start = received.find(marker)
            if start < 0:
                continue
            end = received.find(b"\n", start)
            if end < 0:
                # Some firmware emits the short +CFG echo without a complete
                # line terminator before resuming the binary stream.
                if marker == b"+CFG":
                    print("RX", "+CFG")
                    return "+CFG"
                continue
            line = _response_line(bytes(received[start : end + 1]))
            del received[: end + 1]
            if line == "ERROR" or line.startswith("+ERROR"):
                raise RuntimeError(f"设备返回错误: {line}")
            print("RX", line)
            return line
        if len(received) > 256:
            del received[:-64]
    expected = ", ".join(expected_prefixes)
    raise RuntimeError(f"等待 AT 响应超时（期望: {expected}）")


def send_at(
    ser: serial.Serial | None,
    command: str,
    expected_prefixes: tuple[str, ...],
    dry_run: bool = False,
) -> str | None:
    print("TX", command)
    if dry_run:
        return None
    assert ser is not None
    ser.reset_input_buffer()
    ser.write(at_command(command))
    ser.flush()
    return read_at_response(ser, expected_prefixes)


def enter_config(ser: serial.Serial | None, dry_run: bool = False) -> None:
    try:
        send_at(ser, "AT+CFG", ("+CFG",), dry_run)
    except RuntimeError as exc:
        if dry_run:
            raise
        print(f"警告：未捕获 +CFG 回显，继续查询当前配置：{exc}")


def exit_config(ser: serial.Serial | None, dry_run: bool = False) -> None:
    print("TX", "AT+EXIT")
    if dry_run:
        return
    assert ser is not None
    ser.reset_input_buffer()
    ser.write(at_command("AT+EXIT"))
    ser.flush()
    # 4GAC exits directly into the normal WTRAW/NMEA stream and does not
    # return a dedicated +EXIT line.


def read_config(ser: serial.Serial) -> dict[str, str | int]:
    enter_config(ser)
    try:
        addr_line = send_at(ser, "AT+ADDR?", ("+ADDR=",))
        cors_line = send_at(ser, "AT+CORS?", ("+CORS=",))
        mountpoint_line = send_at(ser, "AT+MP?", ("+MP=",))
        assert addr_line is not None
        assert cors_line is not None
        assert mountpoint_line is not None
        domain, port = parse_addr_response(addr_line)
        username, password = parse_cors_response(cors_line)
        mountpoint = parse_mountpoint_response(mountpoint_line)
        return {
            "domain": domain,
            "port": port,
            "username": username,
            "password": password,
            "mountpoint": mountpoint,
        }
    finally:
        exit_config(ser)


def backup_config(ser: serial.Serial, path: str) -> None:
    values = read_config(ser)
    with open(path, "w", encoding="utf-8") as output:
        json.dump(values, output, ensure_ascii=False, indent=2)
        output.write("\n")
    try:
        os.chmod(path, 0o600)
    except OSError:
        pass
    print(f"当前配置已保存到 {path}")


def configure(ser: serial.Serial | None, config: Config, dry_run: bool) -> None:
    validate_config(config)
    enter_config(ser, dry_run)
    try:
        send_at(ser, f"AT+ADDR={config.domain},{config.port}", ("+ADDR=",), dry_run)
        send_at(ser, f"AT+CORS={config.username},{config.password}", ("+CORS=",), dry_run)
        send_at(ser, f"AT+MP={config.mountpoint}", ("+MP=",), dry_run)
        send_at(ser, "AT+SAVE", ("SAVE", "+SAVE", "OK"), dry_run)
        send_at(ser, "AT+NETRESET", ("+NETRESET", "NETRESET"), dry_run)
    finally:
        exit_config(ser, dry_run)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="配置 WTRTK-4GAC 的 NTRIP 客户端")
    parser.add_argument("--port-device", default="/dev/WTRTK")
    parser.add_argument("--baud", type=int, default=230400)
    parser.add_argument("--domain", help="NTRIP 服务器域名或 IP")
    parser.add_argument("--ntrip-port", type=int)
    parser.add_argument("--username")
    parser.add_argument("--password")
    parser.add_argument("--mountpoint")
    parser.add_argument("--apply", action="store_true", help="实际写入；缺省为 dry-run")
    parser.add_argument("--dry-run", action="store_true", help="仅打印 AT 命令（默认行为）")
    parser.add_argument("--backup", help="通过 AT 查询当前 NTRIP 配置并保存为 JSON")
    parser.add_argument("--restore", help="从 JSON 读取 NTRIP 配置并写入设备")
    args = parser.parse_args()
    if args.backup is None and args.restore is None:
        missing = [
            option
            for option, value in (
                ("--domain", args.domain),
                ("--ntrip-port", args.ntrip_port),
                ("--username", args.username),
                ("--mountpoint", args.mountpoint),
            )
            if value is None
        ]
        if missing:
            parser.error(f"the following arguments are required: {', '.join(missing)}")
    return args


def main() -> int:
    args = parse_args()
    if args.backup:
        try:
            with serial.Serial(args.port_device, args.baud, timeout=1, write_timeout=1) as ser:
                backup_config(ser, args.backup)
        except (SerialException, OSError, RuntimeError, ValueError) as exc:
            print(f"备份失败: {exc}", file=sys.stderr)
            return 2
        return 0

    if args.restore:
        try:
            with open(args.restore, encoding="utf-8") as input_file:
                saved = json.load(input_file)
            config = Config(
                saved["domain"], int(saved["port"]), saved["username"],
                saved["password"], saved["mountpoint"]
            )
            validate_config(config)
        except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            print(f"恢复文件无效: {exc}", file=sys.stderr)
            return 2
    else:
        password = args.password if args.password is not None else getpass.getpass("NTRIP 密码: ")
        config = Config(args.domain, args.ntrip_port, args.username, password, args.mountpoint)
    if not args.apply:
        print("dry-run：不会打开或写入串口；使用 --apply 才会实际配置。")
        try:
            configure(None, config, True)
        except ValueError as exc:
            print(f"配置参数无效: {exc}", file=sys.stderr)
            return 2
        return 0
    try:
        with serial.Serial(args.port_device, args.baud, timeout=1, write_timeout=1) as ser:
            configure(ser, config, False)
    except (SerialException, OSError, RuntimeError, ValueError) as exc:
        print(f"串口操作失败: {exc}", file=sys.stderr)
        return 2
    print("配置已保存。请检查 WTRTK 的 NTRIP 状态和 NETRXCNT。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
