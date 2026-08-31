#!/usr/bin/env python3
"""Split motor_start stdout/stderr by actual date and bounded file size."""

import os
import sys
from datetime import datetime
from pathlib import Path


DEFAULT_STATE_DIR = Path(
    os.environ.get("XDG_STATE_HOME", Path.home() / ".local" / "state")
)
LOG_DIR = Path(
    os.environ.get(
        "MOTOR_START_LOG_DIR",
        DEFAULT_STATE_DIR / "robot_cleaning",
    )
)
MAX_BYTES = int(os.environ.get("MOTOR_START_LOG_MAX_BYTES", str(10 * 1024 * 1024)))


def log_path(day: str, previous: Path | None = None, incoming_bytes: int = 0) -> Path:
    """Return an existing non-full path, or the next numbered log part."""
    part = 0
    if previous is not None:
        suffix = previous.stem.rsplit(".", maxsplit=1)
        if len(suffix) == 2 and suffix[1].isdigit():
            part = int(suffix[1]) + 1
        else:
            part = 1

    candidate = LOG_DIR / (f"run{day}.log" if part == 0 else f"run{day}.{part}.log")
    while candidate.exists() and candidate.stat().st_size + incoming_bytes > MAX_BYTES:
        part += 1
        candidate = LOG_DIR / f"run{day}.{part}.log"
    return candidate


def main() -> None:
    if MAX_BYTES <= 0:
        raise ValueError("MOTOR_START_LOG_MAX_BYTES must be greater than zero")

    LOG_DIR.mkdir(parents=True, exist_ok=True)
    active_day = ""
    active_path: Path | None = None
    active_file = None
    active_size = 0

    try:
        for line in sys.stdin.buffer:
            now_day = datetime.now().strftime("%Y%m%d")

            if now_day != active_day or active_size + len(line) > MAX_BYTES:
                previous_path = active_path if now_day == active_day else None
                if active_file is not None:
                    active_file.close()
                active_day = now_day
                active_path = log_path(active_day, previous_path, len(line))
                active_file = active_path.open("ab")
                active_size = active_path.stat().st_size

            active_file.write(line)
            active_file.flush()
            active_size += len(line)
    finally:
        if active_file is not None:
            active_file.close()


if __name__ == "__main__":
    main()
