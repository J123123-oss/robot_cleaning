# Copyright 2026
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

import importlib.util
import subprocess
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
        self.assertEqual(
            ["-re", "-stream_loop", "-1", "-i", "recorded file.mp4"],
            command[4:9],
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
