"""OpenMV H7 Plus MicroPython JPEG stream sender.

Copy this file to the camera as ``main.py``. The Ubuntu receiver expects the
OMV1 framed protocol implemented below.
"""

import sensor
import struct
import time
import ubinascii
from pyb import USB_VCP


MAGIC = b"OMV1"
PROTOCOL_VERSION = 1
FRAME_PACKET_TYPE = 1
HEADER_FORMAT = "<4sBBHII"
JPEG_QUALITY = 70
TARGET_FPS = 30
USB_SEND_TIMEOUT_MS = 1000


def build_frame_packet(payload, sequence):
    """Build one JPEG packet with a length field and CRC32 checksum."""
    header = struct.pack(
        HEADER_FORMAT,
        MAGIC,
        PROTOCOL_VERSION,
        FRAME_PACKET_TYPE,
        0,
        sequence & 0xFFFFFFFF,
        len(payload),
    )
    checksum = ubinascii.crc32(header + payload) & 0xFFFFFFFF
    return header + payload + struct.pack("<I", checksum)


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


sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)

usb = USB_VCP()
sequence = 0
frame_delay_ms = max(1, int(1000 / TARGET_FPS))

while True:
    if not usb.isconnected():
        time.sleep_ms(100)
        continue

    frame = sensor.snapshot()
    payload = jpeg_bytes(frame)
    packet = build_frame_packet(payload, sequence)
    if send_all(usb, packet):
        sequence = (sequence + 1) & 0xFFFFFFFF
    time.sleep_ms(frame_delay_ms)
