# Copyright 2026
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

import ast
import importlib.util
from pathlib import Path


PACKAGE_ROOT = Path(__file__).parents[1]
VIDEO_SOURCE_PATH = PACKAGE_ROOT / "rtk_nav" / "video_to_v4l2.py"
CAMERA_SOURCE_PATH = PACKAGE_ROOT / "rtk_nav" / "camera_publisher_node.py"
DETECTOR_SOURCE_PATH = PACKAGE_ROOT / "rtk_nav" / "line_detector_node.py"
LAUNCH_SOURCE_PATH = PACKAGE_ROOT / "launch" / "run.launch.py"


def _load_video_module():
    spec = importlib.util.spec_from_file_location(
        "video_to_v4l2_contract_module", VIDEO_SOURCE_PATH
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _function(tree, name):
    return next(
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef) and node.name == name
    )


def _class_method(tree, class_name, name):
    detector_class = next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == class_name
    )
    return next(
        node
        for node in detector_class.body
        if isinstance(node, ast.FunctionDef) and node.name == name
    )


def _calls_named(function, name):
    return [
        node
        for node in ast.walk(function)
        if isinstance(node, ast.Call)
        and (
            (isinstance(node.func, ast.Name) and node.func.id == name)
            or (isinstance(node.func, ast.Attribute) and node.func.attr == name)
        )
    ]


def test_video_defaults_preserve_portrait_source_without_crop_or_rotation():
    module = _load_video_module()
    args = module.parse_args(["--video", "recorded.mp4"])

    assert (args.width, args.height, args.fps) == (360, 640, 30)
    video_filter = module.build_video_filter(360, 640, 30)
    assert "scale=360:640" in video_filter
    assert "fps=30" in video_filter
    for forbidden in ("crop=", "transpose", "rotate=", "pad="):
        assert forbidden not in video_filter


def test_video_command_outputs_raw_yuyv_to_video0():
    module = _load_video_module()
    args = module.parse_args(["--video", "recorded.mp4"])
    command = module.build_ffmpeg_command(args, ffmpeg_binary="ffmpeg")

    assert command[-1] == "/dev/video0"
    assert "-f" in command and command[command.index("-f") + 1] == "v4l2"
    assert "-pix_fmt" in command
    assert command[command.index("-pix_fmt") + 1] == "yuyv422"
    assert command[command.index("-s") + 1] == "360x640"
    assert command[command.index("-r") + 1] == "30"


def test_camera_publishes_jpeg_compressed_frames_with_latest_capture_buffer():
    source = CAMERA_SOURCE_PATH.read_text(encoding="utf-8")

    for required in (
        "CompressedImage",
        "/camera/color/image_compressed",
        "jpeg_quality",
        "IMWRITE_JPEG_QUALITY",
        "max-buffers=1",
        "drop=true",
        "sync=false",
    ):
        assert required in source
    assert "self.declare_parameter('width', 360)" in source
    assert "self.declare_parameter('height', 640)" in source
    assert "self.declare_parameter('fps', 30)" in source
    assert "self.declare_parameter('image_path', '')" in source


def test_detector_decodes_only_the_latest_compressed_frame_in_timer():
    source = DETECTOR_SOURCE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    initializer = ast.unparse(_function(tree, "__init__"))
    image_callback = _function(tree, "image_callback")
    timer_callback = _function(tree, "timer_callback")

    assert "CompressedImage" in source
    assert "/camera/color/image_compressed" in source
    assert "cv2.imdecode" in source
    assert "detection_fps" in initializer
    assert "publish_debug_images" in initializer
    assert "debug_image_fps" in initializer
    assert "depth=1" in initializer
    assert not _calls_named(image_callback, "detect_and_draw_grid_lines")
    assert _calls_named(timer_callback, "detect_and_draw_grid_lines")
    assert "self.latest_compressed_data = payload" in ast.unparse(image_callback)


def test_launch_passes_portrait_compression_and_detection_rate_defaults():
    source = LAUNCH_SOURCE_PATH.read_text(encoding="utf-8")
    for default in ("360", "640", "30", "80", "30.0", "false", "1.0"):
        assert f'TextSubstitution(text="{default}")' in source
    assert "camera_image_path" in source
    assert "image_path" in source
    assert "jpeg_quality" in source
    assert "detection_fps" in source
