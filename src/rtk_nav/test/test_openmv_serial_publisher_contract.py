"""Contract tests for the OpenMV ROS publisher integration."""

from pathlib import Path


PACKAGE_ROOT = Path(__file__).parents[1]
NODE_SOURCE = (PACKAGE_ROOT / "rtk_nav" / "openmv_serial_publisher_node.py").read_text(
    encoding="utf-8"
)
OPENMV_SOURCE = (PACKAGE_ROOT / "rtk_nav" / "openmv_camera_stream.py").read_text(
    encoding="utf-8"
)
SETUP_SOURCE = (PACKAGE_ROOT / "setup.py").read_text(encoding="utf-8")
PACKAGE_SOURCE = (PACKAGE_ROOT / "package.xml").read_text(encoding="utf-8")
LAUNCH_SOURCE = (PACKAGE_ROOT / "launch" / "run.launch.py").read_text(
    encoding="utf-8"
)


def test_openmv_protocol_is_shared_by_camera_and_host():
    for source in (NODE_SOURCE, OPENMV_SOURCE):
        assert "OMV1" in source
        assert "HEADER_FORMAT" in source
        assert "crc32" in source


def test_ros_node_publishes_existing_compressed_camera_topic():
    assert "CompressedImage" in NODE_SOURCE
    assert "/camera/color/image_compressed" in NODE_SOURCE
    assert "serial_port" in NODE_SOURCE
    assert "FrameStreamDecoder" in NODE_SOURCE
    assert "is_jpeg_payload" in NODE_SOURCE
    assert "reconnect" in NODE_SOURCE.lower()


def test_ros_node_supports_direct_file_execution():
    assert "if __package__:" in NODE_SOURCE
    assert "from openmv_serial_protocol import" in NODE_SOURCE


def test_openmv_node_is_registered_and_serial_dependency_declared():
    assert (
        "openmv_serial_publisher_node = "
        "rtk_nav.openmv_serial_publisher_node:main"
    ) in SETUP_SOURCE
    assert "<exec_depend>python3-serial</exec_depend>" in PACKAGE_SOURCE


def test_launch_selects_one_camera_source():
    assert '"camera_source"' in LAUNCH_SOURCE
    assert "camera_source')," in LAUNCH_SOURCE
    assert "'v4l2'" in LAUNCH_SOURCE
    assert "'openmv_serial'" in LAUNCH_SOURCE
    assert "openmv_serial_publisher_node" in LAUNCH_SOURCE


def test_openmv_script_captures_and_sends_jpeg_frames():
    for required in (
        "sensor.snapshot()",
        "frame.compress",
        "USB_VCP",
        "usb.send",
        "build_frame_packet",
    ):
        assert required in OPENMV_SOURCE


def test_openmv_sender_handles_short_writes():
    assert "def send_all(" in OPENMV_SOURCE
    assert "usb.send(data[offset:]" in OPENMV_SOURCE
    assert "offset += sent" in OPENMV_SOURCE
    assert "if send_all(usb, packet):" in OPENMV_SOURCE


def test_serial_node_reconnects_after_stalled_input_and_returns_failure():
    assert '"no_data_timeout_sec"' in NODE_SOURCE
    assert "time.monotonic()" in NODE_SOURCE
    assert "连续无数据" in NODE_SOURCE
    assert "exit_code = 1" in NODE_SOURCE
    assert "return exit_code" in NODE_SOURCE
