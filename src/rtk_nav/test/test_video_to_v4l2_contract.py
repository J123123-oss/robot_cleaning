# Copyright 2026
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

import importlib.util
import subprocess
import types
import unittest
from pathlib import Path

MODULE_PATH = Path(__file__).parents[1] / "rtk_nav" / "video_to_v4l2.py"
SETUP_PATH = Path(__file__).parents[1] / "setup.py"
README_PATH = Path(__file__).parents[1] / "README_VIDEO_TO_V4L2.md"


def _load_module():
    spec = importlib.util.spec_from_file_location("video_to_v4l2", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class VideoToV4L2ContractTest(unittest.TestCase):
    def test_parse_args_uses_v4l2_defaults(self):
        module = _load_module()

        args = module.parse_args(["--video", "recorded.mp4"])

        self.assertEqual(args.video, Path("recorded.mp4"))
        self.assertEqual(args.device, "/dev/video0")
        self.assertEqual(args.width, 640)
        self.assertEqual(args.height, 480)
        self.assertEqual(args.fps, 30)
        self.assertFalse(args.loop)
        self.assertTrue(args.create_device)
        self.assertEqual(args.speed, 1.0)

        fast_args = module.parse_args(
            ["--video", "recorded.mp4", "--speed", "2.5"]
        )
        self.assertEqual(fast_args.speed, 2.5)

    def test_parse_args_rejects_non_positive_output_settings(self):
        module = _load_module()

        with self.assertRaises(SystemExit):
            module.parse_args(["--video", "recorded.mp4", "--width", "0"])
        with self.assertRaises(SystemExit):
            module.parse_args(["--video", "recorded.mp4", "--height", "-1"])
        with self.assertRaises(SystemExit):
            module.parse_args(["--video", "recorded.mp4", "--fps", "0"])
        with self.assertRaises(SystemExit):
            module.parse_args(["--video", "recorded.mp4", "--width", "641"])
        with self.assertRaises(SystemExit):
            module.parse_args(["--video", "recorded.mp4", "--speed", "0"])
        with self.assertRaises(SystemExit):
            module.parse_args(["--video", "recorded.mp4", "--speed", "-0.5"])

    def test_build_ffmpeg_command_scales_and_crops_without_blank_or_stretched_edges(self):
        module = _load_module()
        args = module.parse_args(
            [
                "--video",
                "recorded file.mp4",
                "--device",
                "/dev/video7",
                "--width",
                "1280",
                "--height",
                "720",
                "--fps",
                "25",
                "--speed",
                "2.5",
                "--loop",
            ]
        )

        command = module.build_ffmpeg_command(args, ffmpeg_binary="/usr/bin/ffmpeg")

        self.assertEqual(
            command[:4],
            [
                "/usr/bin/ffmpeg",
                "-hide_banner",
                "-loglevel",
                "warning",
            ],
        )
        self.assertEqual(command[4:6], ["-readrate", "2.5"])
        self.assertNotIn("-re", command)
        self.assertEqual(
            ["-readrate", "2.5", "-stream_loop", "-1", "-i", "recorded file.mp4"],
            command[4:10],
        )
        self.assertIn(
            "scale=1280:720:force_original_aspect_ratio=increase,"
            "crop=1280:720:(iw-ow)/2:(ih-oh)/2,fps=25,format=yuyv422",
            command,
        )
        self.assertEqual(
            command[-9:],
            [
                "-f",
                "v4l2",
                "-pix_fmt",
                "yuyv422",
                "-s",
                "1280x720",
                "-r",
                "25",
                "/dev/video7",
            ],
        )

    def test_ensure_v4l2_device_reports_installation_when_modprobe_is_missing(self):
        module = _load_module()

        with self.assertRaisesRegex(RuntimeError, "v4l2loopback-dkms"):
            module.ensure_v4l2_device(
                "/dev/video0",
                path_exists=lambda _: False,
                executable_finder=lambda name: None if name == "modprobe" else name,
                is_root=lambda: True,
            )

    def test_ensure_v4l2_device_reports_module_load_failure(self):
        module = _load_module()

        def fail_to_load(*_args, **_kwargs):
            raise subprocess.CalledProcessError(1, "modprobe")

        with self.assertRaisesRegex(RuntimeError, "v4l2loopback-dkms"):
            module.ensure_v4l2_device(
                "/dev/video0",
                command_runner=fail_to_load,
                path_exists=lambda _: False,
                executable_finder=lambda name: f"/usr/bin/{name}",
                is_root=lambda: True,
            )

    def test_ensure_v4l2_device_enables_sustained_last_frame(self):
        module = _load_module()
        commands = []
        device_exists = [False]

        def command_runner(command, check):
            self.assertTrue(check)
            commands.append(command)
            device_exists[0] = True

        module.ensure_v4l2_device(
            "/dev/video0",
            command_runner=command_runner,
            path_exists=lambda _: device_exists[0],
            executable_finder=lambda name: f"/usr/bin/{name}",
            is_root=lambda: True,
            sleep_fn=lambda _: None,
        )

        self.assertEqual(len(commands), 1)
        self.assertIn("sustain_framerate=1", commands[0])

    def test_set_process_paused_sends_stop_and_continue(self):
        module = _load_module()

        class FakeProcess:
            def __init__(self):
                self.signals = []

            def send_signal(self, value):
                self.signals.append(value)

        process = FakeProcess()
        original_signal = module.signal
        module.signal = types.SimpleNamespace(SIGSTOP="STOP", SIGCONT="CONT")
        try:
            module.set_process_paused(process, True)
            module.set_process_paused(process, False)
        finally:
            module.signal = original_signal

        self.assertEqual(process.signals, ["STOP", "CONT"])

    def test_terminal_controller_restores_terminal_state_idempotently(self):
        module = _load_module()
        termios_calls = []
        signal_calls = []

        class FakeStream:
            def isatty(self):
                return True

            def fileno(self):
                return 7

        class FakeTermios:
            TCSADRAIN = "drain"
            TCSANOW = "now"

            @staticmethod
            def tcgetattr(fd):
                self.assertEqual(fd, 7)
                return ["original-settings"]

            @staticmethod
            def tcsetattr(fd, when, settings):
                termios_calls.append((fd, when, settings))

        class FakeTTY:
            @staticmethod
            def setcbreak(fd):
                self.assertEqual(fd, 7)

        class FakeSignals:
            SIGINT = "int"
            SIGTERM = "term"
            SIGHUP = "hup"

            @staticmethod
            def getsignal(value):
                return f"previous-{value}"

            @staticmethod
            def signal(value, handler):
                signal_calls.append((value, handler))

        class FakeAtexit:
            @staticmethod
            def register(handler):
                self.assertTrue(callable(handler))

            @staticmethod
            def unregister(handler):
                self.assertTrue(callable(handler))

        originals = (module.termios, module.tty, module.signal, module.atexit)
        module.termios = FakeTermios
        module.tty = FakeTTY
        module.signal = FakeSignals
        module.atexit = FakeAtexit
        try:
            with module._TerminalController(FakeStream()) as terminal:
                self.assertTrue(terminal.enabled)
                with self.assertRaises(KeyboardInterrupt):
                    terminal._handle_signal(FakeSignals.SIGINT, None)
            terminal.restore()
        finally:
            module.termios, module.tty, module.signal, module.atexit = originals

        self.assertEqual(
            termios_calls,
            [(7, "drain", ["original-settings"])],
        )
        self.assertGreaterEqual(len(signal_calls), 6)

    def test_terminal_controller_repairs_preexisting_broken_terminal(self):
        module = _load_module()
        termios_calls = []

        class FakeStream:
            def isatty(self):
                return True

            def fileno(self):
                return 7

        class FakeTermios:
            TCSADRAIN = "drain"
            TCSANOW = "now"
            ICANON = 0x0002
            ECHO = 0x0008
            ISIG = 0x0001
            IEXTEN = 0x8000
            OPOST = 0x0001

            @staticmethod
            def tcgetattr(fd):
                self.assertEqual(fd, 7)
                return [0, 0, 0, 0, 0, 0, []]

            @staticmethod
            def tcsetattr(fd, when, settings):
                termios_calls.append((fd, when, settings))

        class FakeTTY:
            @staticmethod
            def setcbreak(fd):
                self.assertEqual(fd, 7)

        class FakeSignals:
            SIGINT = "int"
            SIGTERM = "term"
            SIGHUP = "hup"

            @staticmethod
            def getsignal(value):
                return f"previous-{value}"

            @staticmethod
            def signal(_value, _handler):
                return None

        class FakeAtexit:
            @staticmethod
            def register(_handler):
                return None

            @staticmethod
            def unregister(_handler):
                return None

        originals = (module.termios, module.tty, module.signal, module.atexit)
        module.termios = FakeTermios
        module.tty = FakeTTY
        module.signal = FakeSignals
        module.atexit = FakeAtexit
        try:
            with module._TerminalController(FakeStream()):
                pass
        finally:
            module.termios, module.tty, module.signal, module.atexit = originals

        self.assertEqual(
            termios_calls,
            [
                (
                    7,
                    "drain",
                    [0, 1, 0, FakeTermios.ICANON | FakeTermios.ECHO | FakeTermios.ISIG | FakeTermios.IEXTEN, 0, 0, []],
                )
            ],
        )

    def test_setup_registers_video_to_v4l2_console_script(self):
        setup_source = SETUP_PATH.read_text(encoding="utf-8")

        self.assertIn(
            "video_to_v4l2 = rtk_nav.video_to_v4l2:main",
            setup_source,
        )

    def test_readme_documents_device_setup_and_camera_consumer(self):
        readme_source = README_PATH.read_text(encoding="utf-8")

        self.assertIn("sudo apt install ffmpeg v4l2loopback-dkms", readme_source)
        self.assertIn("ros2 run rtk_nav video_to_v4l2", readme_source)
        self.assertIn("ros2 run rtk_nav camera_publisher_node", readme_source)
        self.assertIn("/camera/color/image_raw", readme_source)
        self.assertIn("--speed", readme_source)
        self.assertIn("Space", readme_source)
        self.assertIn("sustain_framerate=1", readme_source)
        self.assertIn("modprobe -r v4l2loopback", readme_source)
