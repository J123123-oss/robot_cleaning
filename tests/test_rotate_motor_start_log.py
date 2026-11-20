"""Regression tests for the motor service log files."""

import os
from pathlib import Path
import subprocess
import sys


SCRIPT = Path(__file__).resolve().parents[1] / "rotate_motor_start_log.py"


def write_logs(log_dir, stream, max_bytes=1024):
    env = {
        **os.environ,
        "MOTOR_START_LOG_DIR": str(log_dir),
        "MOTOR_START_LOG_MAX_BYTES": str(max_bytes),
    }
    subprocess.run([sys.executable, str(SCRIPT)], input=stream, env=env, check=True)
    return list(log_dir.glob("run*.log"))


def test_logs_preserve_chinese_and_escape_invalid_bytes(tmp_path):
    message = "[motor_control-2] [WARN] 防跌落快触发：['back_left']"
    stream = b"\x1b[33m" + message.encode("utf-8") + b"\x1b[0m\nserial: \xff\n"

    (log_file,) = write_logs(tmp_path, stream)

    assert log_file.read_bytes().decode("utf-8") == f"{message}\nserial: \\xff\n"


def test_rotation_counts_bytes_written_after_normalization(tmp_path):
    logs = write_logs(tmp_path, b"\xff\nend\n", max_bytes=8)
    first, second = sorted(logs, key=lambda path: len(path.suffixes))

    assert second.name == f"{first.stem}.1.log"
    assert first.read_bytes() == b"\\xff\n"
    assert second.read_bytes() == b"end\n"
