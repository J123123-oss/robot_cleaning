"""Contract tests for the OpenMV fill-light ownership protocol."""

from pathlib import Path


OPENMV_SOURCE = (
    Path(__file__).parents[1] / "rtk_nav" / "openmv_camera_stream.py"
).read_text(encoding="utf-8")
DEBUG_SOURCE = (
    Path(__file__).parents[1] / "rtk_nav" / "openmv_fill_light_debug.py"
).read_text(encoding="utf-8")


def test_fill_light_uses_openmv_h7_plus_light_shield_pin():
    assert "from machine import Pin, PWM" in OPENMV_SOURCE
    assert "H7 Plus" in OPENMV_SOURCE
    assert "PROTOCOL_VERSION = 1" in OPENMV_SOURCE
    assert 'PWM(Pin("P6"), freq=50_000, duty_u16=0)' in OPENMV_SOURCE


def test_fill_light_starts_off_until_the_host_claims_it():
    assert "LIGHT_BRIGHTNESS = 50" in OPENMV_SOURCE
    assert "light_brightness = LIGHT_BRIGHTNESS" in OPENMV_SOURCE
    assert "LIGHT_HEARTBEAT_TIMEOUT_MS = 2000" in OPENMV_SOURCE
    assert "LIGHT_STATUS_INTERVAL_MS = 1000" in OPENMV_SOURCE
    assert "LIGHT_STATUS_PACKET_TYPE = 2" in OPENMV_SOURCE
    assert "light_enabled = False" in OPENMV_SOURCE
    assert "set_fill_light(False)" in OPENMV_SOURCE


def test_fill_light_accepts_host_commands_and_expires_without_heartbeat():
    for required in (
        "LIGHT_ON",
        "LIGHT_OFF",
        "LIGHT_HEARTBEAT",
        "LIGHT_BRIGHTNESS_COMMAND_PREFIX",
        "usb.any()",
        "usb.recv(",
        "time.ticks_diff(",
        "def build_light_status_packet(",
        "def send_periodic_light_status(",
        "def set_light_brightness(",
        "send_all(usb, build_light_status_packet(",
    ):
        assert required in OPENMV_SOURCE
    assert "machine.reset" not in OPENMV_SOURCE
    assert "machine.soft_reset" not in OPENMV_SOURCE


def test_fill_light_is_released_when_usb_disconnects():
    assert OPENMV_SOURCE.index("set_fill_light(False)") < OPENMV_SOURCE.index(
        "time.sleep_ms(100)"
    )


def test_standalone_debug_script_disables_image_streaming():
    assert "USB_VCP" in DEBUG_SOURCE
    assert 'PWM(\n        Pin("P6")' in DEBUG_SOURCE
    assert "import sensor" not in DEBUG_SOURCE
    assert "OMV1" not in DEBUG_SOURCE
    assert "usb.send((\"[FillLight] \"" in DEBUG_SOURCE


def test_standalone_debug_script_handles_light_commands_and_timeout():
    for required in (
        "LIGHT_ON_COMMAND",
        "LIGHT_OFF_COMMAND",
        "LIGHT_HEARTBEAT_COMMAND",
        "LIGHT_BRIGHTNESS_COMMAND_PREFIX",
        "def handle_command(",
        "def set_light_brightness(",
        "RX ",
        "def expire_heartbeat(",
        "HEARTBEAT_TIMEOUT -> OFF",
    ):
        assert required in DEBUG_SOURCE
    assert "machine.reset" not in DEBUG_SOURCE
    assert "machine.soft_reset" not in DEBUG_SOURCE


def test_openmv_command_buffers_do_not_use_unsupported_bytearray_deletion():
    assert "del command_buffer[" not in DEBUG_SOURCE
    assert "del light_command_buffer[" not in OPENMV_SOURCE
    assert "global command_buffer" in DEBUG_SOURCE
    assert "global last_light_heartbeat_ms, light_command_buffer" in OPENMV_SOURCE
