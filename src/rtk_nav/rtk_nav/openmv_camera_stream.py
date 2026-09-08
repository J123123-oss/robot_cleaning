"""OpenMV H7 Plus MicroPython JPEG stream sender.

Copy this file to the camera as ``main.py``. The Ubuntu receiver expects the
OMV1 framed protocol implemented below.
"""

import sensor
import struct
import time
import ubinascii
from pyb import USB_VCP
from machine import Pin, PWM

MAGIC = b"OMV1"
PROTOCOL_VERSION = 1
FRAME_PACKET_TYPE = 1
LIGHT_STATUS_PACKET_TYPE = 2
HEADER_FORMAT = "<4sBBHII"
JPEG_QUALITY = 70
TARGET_FPS = 30
USB_SEND_TIMEOUT_MS = 1000
LIGHT_BRIGHTNESS = 50
LIGHT_HEARTBEAT_TIMEOUT_MS = 2000
LIGHT_STATUS_INTERVAL_MS = 1000
LIGHT_COMMAND_READ_BYTES = 64
LIGHT_ON_COMMAND = b"LIGHT_ON"
LIGHT_OFF_COMMAND = b"LIGHT_OFF"
LIGHT_HEARTBEAT_COMMAND = b"LIGHT_HEARTBEAT"

pwm = PWM(Pin("P6"), freq=50_000, duty_u16=0)
light_enabled = False
light_command_buffer = bytearray()
last_light_heartbeat_ms = None
last_light_status_ms = None


def build_packet(payload, sequence, packet_type):
    """Build one protocol packet with a length field and CRC32 checksum."""
    header = struct.pack(
        HEADER_FORMAT,
        MAGIC,
        PROTOCOL_VERSION,
        packet_type,
        0,
        sequence & 0xFFFFFFFF,
        len(payload),
    )
    checksum = ubinascii.crc32(header + payload) & 0xFFFFFFFF
    return header + payload + struct.pack("<I", checksum)


def build_frame_packet(payload, sequence):
    """Build one JPEG frame packet."""
    return build_packet(payload, sequence, FRAME_PACKET_TYPE)


def build_light_status_packet(enabled):
    """Build a small status packet confirming the applied light state."""
    payload = LIGHT_ON_COMMAND if enabled else LIGHT_OFF_COMMAND
    return build_packet(payload, 0, LIGHT_STATUS_PACKET_TYPE)


def jpeg_bytes(frame):
    """Compress an OpenMV frame and return its JPEG byte buffer."""
    compressed = frame.compress(quality=JPEG_QUALITY)
    if compressed is None:
        compressed = frame
    if hasattr(compressed, "bytearray"):
        return bytes(compressed.bytearray())
    return bytes(compressed)


def send_all(usb, data, timeout_ms=USB_SEND_TIMEOUT_MS):
    """Send the complete packet, handling short USB writes."""
    offset = 0
    deadline = time.ticks_add(time.ticks_ms(), int(timeout_ms))

    while offset < len(data):
        if not usb.isconnected():
            return False

        remaining_ms = time.ticks_diff(deadline, time.ticks_ms())
        if remaining_ms <= 0:
            return False

        sent = usb.send(data[offset:], timeout=remaining_ms)
        if sent is None:
            return False
        sent = int(sent)
        if sent <= 0:
            time.sleep_ms(1)
            continue
        offset += sent

    return True


def set_fill_light(enabled):
    """Set the Light Shield brightness without changing it every frame."""
    global light_enabled

    enabled = bool(enabled)
    if enabled == light_enabled:
        return

    duty_u16 = (LIGHT_BRIGHTNESS * 65535) // 100 if enabled else 0
    pwm.duty_u16(duty_u16)
    light_enabled = enabled


def process_light_commands(usb):
    """Apply newline-delimited light commands received from the host node."""
    global last_light_heartbeat_ms, light_command_buffer

    if not usb.any():
        return

    data = usb.recv(LIGHT_COMMAND_READ_BYTES, timeout=0)
    if not data:
        return

    status_enabled = None
    light_command_buffer.extend(data)
    if len(light_command_buffer) > 128:
        light_command_buffer = light_command_buffer[-128:]

    while True:
        separator = light_command_buffer.find(b"\n")
        if separator < 0:
            break

        command = bytes(light_command_buffer[:separator]).strip()
        light_command_buffer = light_command_buffer[separator + 1:]

        if command == LIGHT_ON_COMMAND or command == LIGHT_HEARTBEAT_COMMAND:
            last_light_heartbeat_ms = time.ticks_ms()
            set_fill_light(True)
            status_enabled = True
        elif command == LIGHT_OFF_COMMAND:
            last_light_heartbeat_ms = None
            set_fill_light(False)
            status_enabled = False

    if status_enabled is not None:
        send_all(usb, build_light_status_packet(status_enabled))


def send_periodic_light_status(usb):
    """Report the current light state so host-side command delivery is observable."""
    global last_light_status_ms

    now_ms = time.ticks_ms()
    if (
        last_light_status_ms is not None
        and time.ticks_diff(now_ms, last_light_status_ms) < LIGHT_STATUS_INTERVAL_MS
    ):
        return

    if send_all(usb, build_light_status_packet(light_enabled)):
        last_light_status_ms = now_ms


def expire_fill_light_lease():
    """Turn the light off when the host node stops sending heartbeats."""
    global last_light_heartbeat_ms

    if last_light_heartbeat_ms is None:
        return
    if (
        time.ticks_diff(time.ticks_ms(), last_light_heartbeat_ms)
        <= LIGHT_HEARTBEAT_TIMEOUT_MS
    ):
        return

    last_light_heartbeat_ms = None
    set_fill_light(False)


sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)

usb = USB_VCP()
sequence = 0
frame_delay_ms = max(1, int(1000 / TARGET_FPS))

while True:
    if not usb.isconnected():
        last_light_heartbeat_ms = None
        set_fill_light(False)
        time.sleep_ms(100)
        continue

    process_light_commands(usb)
    expire_fill_light_lease()
    send_periodic_light_status(usb)
    frame = sensor.snapshot()
    payload = jpeg_bytes(frame)
    packet = build_frame_packet(payload, sequence)
    if send_all(usb, packet):
        sequence = (sequence + 1) & 0xFFFFFFFF
    time.sleep_ms(frame_delay_ms)
