#!/usr/bin/env python3
"""Replay a recorded video into a Linux V4L2 loopback device."""

from __future__ import annotations

import argparse
import math
import os
import select
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Callable, Optional, Sequence

try:
    import termios
    import tty
except ImportError:  # pragma: no cover - only used on non-POSIX hosts
    termios = None
    tty = None


DEFAULT_DEVICE = "/dev/video0"
INSTALL_HINT = (
    "Install dependencies with: sudo apt update && "
    "sudo apt install ffmpeg v4l2loopback-dkms v4l2loopback-utils"
)


def positive_int(value: str) -> int:
    """Parse a strictly positive integer for image and timing settings."""
    try:
        parsed = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"invalid positive integer: {value}") from exc
    if parsed <= 0:
        raise argparse.ArgumentTypeError("value must be greater than zero")
    return parsed


def even_positive_int(value: str) -> int:
    """Parse a positive even integer required by the YUYV pixel format."""
    parsed = positive_int(value)
    if parsed % 2:
        raise argparse.ArgumentTypeError("value must be even for yuyv422 output")
    return parsed


def positive_float(value: str) -> float:
    """Parse a finite positive floating-point value."""
    try:
        parsed = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"invalid positive number: {value}") from exc
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise argparse.ArgumentTypeError("value must be finite and greater than zero")
    return parsed


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    """Parse command-line options for the V4L2 replay process."""
    parser = argparse.ArgumentParser(
        description="Replay a recorded video into a v4l2loopback device."
    )
    parser.add_argument("--video", required=True, type=Path, help="input video file")
    parser.add_argument(
        "--device",
        default=DEFAULT_DEVICE,
        help=f"V4L2 output device (default: {DEFAULT_DEVICE})",
    )
    parser.add_argument("--width", default=640, type=even_positive_int)
    parser.add_argument("--height", default=480, type=positive_int)
    parser.add_argument("--fps", default=30, type=positive_int)
    parser.add_argument(
        "--speed",
        default=1.0,
        type=positive_float,
        help="playback speed multiplier (default: 1.0)",
    )
    parser.add_argument(
        "--loop",
        action="store_true",
        help="restart the video after it reaches the end",
    )
    parser.add_argument(
        "--no-create-device",
        dest="create_device",
        action="store_false",
        help="do not load v4l2loopback when the output device is missing",
    )
    parser.set_defaults(create_device=True)
    return parser.parse_args(argv)


def build_video_filter(width: int, height: int, fps: int) -> str:
    """Build a fixed-size filter without blank borders or non-uniform scaling."""
    return (
        f"scale={width}:{height}:force_original_aspect_ratio=increase,"
        f"crop={width}:{height}:(iw-ow)/2:(ih-oh)/2,"
        f"fps={fps},format=yuyv422"
    )


def build_ffmpeg_command(
    args: argparse.Namespace,
    *,
    ffmpeg_binary: str = "ffmpeg",
) -> list[str]:
    """Build an argv list for ffmpeg without invoking a shell."""
    command = [
        ffmpeg_binary,
        "-hide_banner",
        "-loglevel",
        "warning",
        "-readrate",
        str(args.speed),
    ]
    if args.loop:
        command.extend(["-stream_loop", "-1"])
    command.extend(
        [
            "-i",
            str(args.video),
            "-map",
            "0:v:0",
            "-an",
            "-vf",
            build_video_filter(args.width, args.height, args.fps),
            "-f",
            "v4l2",
            "-pix_fmt",
            "yuyv422",
            "-s",
            f"{args.width}x{args.height}",
            "-r",
            str(args.fps),
            str(args.device),
        ]
    )
    return command


def _running_as_root() -> bool:
    geteuid = getattr(os, "geteuid", None)
    return geteuid is None or geteuid() == 0


def ensure_v4l2_device(
    device: str,
    *,
    command_runner: Callable[..., object] = subprocess.run,
    executable_finder: Callable[[str], Optional[str]] = shutil.which,
    path_exists: Callable[[str], bool] = os.path.exists,
    is_root: Optional[Callable[[], bool]] = None,
    sleep_fn: Callable[[float], None] = time.sleep,
    wait_timeout: float = 3.0,
) -> bool:
    """Ensure the default loopback device exists, loading v4l2loopback if needed."""
    if path_exists(device):
        return False

    if device != DEFAULT_DEVICE:
        raise RuntimeError(
            f"{device} does not exist. Automatic creation only supports {DEFAULT_DEVICE}; "
            "create the requested loopback device manually."
        )

    modprobe = executable_finder("modprobe")
    if modprobe is None:
        raise RuntimeError(f"modprobe is unavailable. {INSTALL_HINT}")

    root_check = is_root or _running_as_root
    command = [
        modprobe,
        "v4l2loopback",
        "devices=1",
        "video_nr=0",
        "card_label=RecordedVideo",
        "exclusive_caps=1",
        "sustain_framerate=1",
    ]
    if not root_check():
        sudo = executable_finder("sudo")
        if sudo is None:
            raise RuntimeError(f"sudo is unavailable. {INSTALL_HINT}")
        command.insert(0, sudo)

    try:
        command_runner(command, check=True)
    except (FileNotFoundError, subprocess.CalledProcessError) as exc:
        raise RuntimeError(
            f"Unable to load v4l2loopback and create {DEFAULT_DEVICE}. {INSTALL_HINT}"
        ) from exc

    deadline = time.monotonic() + wait_timeout
    while time.monotonic() < deadline:
        if path_exists(device):
            return True
        sleep_fn(0.1)

    if not path_exists(device):
        raise RuntimeError(
            f"v4l2loopback loaded but {device} was not created. "
            "Check the kernel module and permissions."
        )


def set_process_paused(process: subprocess.Popen, paused: bool) -> None:
    """Stop or continue ffmpeg using POSIX process signals."""
    signal_name = "SIGSTOP" if paused else "SIGCONT"
    process_signal = getattr(signal, signal_name, None)
    if process_signal is None:
        raise RuntimeError("interactive pause/resume requires a POSIX system")
    process.send_signal(process_signal)


class _TerminalController:
    """Read single terminal characters while always restoring terminal state."""

    def __init__(self, stream):
        self.stream = stream
        self.enabled = False
        self._fd = None
        self._settings = None

    def __enter__(self):
        if termios is None or tty is None or not self.stream.isatty():
            return self

        self._fd = self.stream.fileno()
        try:
            self._settings = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd)
            self.enabled = True
        except (OSError, ValueError):
            self._fd = None
            self._settings = None
        return self

    def __exit__(self, _exc_type, _exc_value, _traceback):
        if self._settings is not None and self._fd is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._settings)
        self.enabled = False

    def read_key(self, timeout: float = 0.1) -> Optional[str]:
        if not self.enabled:
            time.sleep(timeout)
            return None
        readable, _, _ = select.select([self.stream], [], [], timeout)
        if readable:
            return self.stream.read(1)
        return None


def _wait_for_process_exit(process, timeout: float = 5.0) -> int:
    """Interrupt ffmpeg and terminate it if it does not exit promptly."""
    process.send_signal(signal.SIGINT)
    try:
        return process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        process.terminate()
        return process.wait()


def _shutdown_process(process, paused: bool) -> int:
    """Resume a paused child before asking it to exit."""
    if process.poll() is not None:
        return process.returncode
    if paused:
        set_process_paused(process, False)
    return _wait_for_process_exit(process)


def _missing_device_error(device: str) -> RuntimeError:
    return RuntimeError(
        f"V4L2 device {device} does not exist. Load v4l2loopback first or remove "
        "--no-create-device."
    )


def main(argv: Optional[Sequence[str]] = None) -> int:
    """Validate inputs, start ffmpeg, and return its exit status."""
    args = parse_args(argv)

    if not args.video.is_file():
        print(f"Input video does not exist: {args.video}", file=sys.stderr)
        return 2

    ffmpeg_binary = shutil.which("ffmpeg")
    if ffmpeg_binary is None:
        print(INSTALL_HINT, file=sys.stderr)
        return 2

    device_created = False
    try:
        if args.create_device:
            device_created = ensure_v4l2_device(args.device)
        elif not os.path.exists(args.device):
            raise _missing_device_error(args.device)
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        return 2

    command = build_ffmpeg_command(args, ffmpeg_binary=ffmpeg_binary)
    print(
        f"Writing {args.video} to {args.device} at "
        f"{args.width}x{args.height} @ {args.fps} FPS, speed {args.speed}x.",
        flush=True,
    )
    if not device_created:
        print(
            "Using an existing V4L2 device. If pause does not repeat the last "
            "frame, recreate v4l2loopback with sustain_framerate=1.",
            flush=True,
        )
    print("Controls: Space/p pause or resume, q quit, Ctrl-C stop.", flush=True)

    process = None
    paused = False
    try:
        process = subprocess.Popen(command)
        with _TerminalController(sys.stdin) as terminal:
            while True:
                return_code = process.poll()
                if return_code is not None:
                    return return_code

                key = terminal.read_key()
                if key in (" ", "p", "P"):
                    set_process_paused(process, not paused)
                    paused = not paused
                    state = "paused; holding current frame" if paused else "resumed"
                    print(f"Playback {state}.", flush=True)
                elif key in ("q", "Q"):
                    break
    except KeyboardInterrupt:
        if process is None:
            return 130
    except OSError as exc:
        print(f"Unable to start ffmpeg: {exc}", file=sys.stderr)
        return 2
    finally:
        if process is not None and process.poll() is None:
            try:
                _shutdown_process(process, paused)
            except RuntimeError as exc:
                print(f"Unable to resume paused ffmpeg: {exc}", file=sys.stderr)

    return process.returncode if process is not None else 130

if __name__ == "__main__":
    raise SystemExit(main())
