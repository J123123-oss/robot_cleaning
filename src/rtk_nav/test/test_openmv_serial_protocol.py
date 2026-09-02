"""Tests for the OpenMV serial JPEG framing protocol."""

from pathlib import Path
import sys

import pytest


PACKAGE_DIR = Path(__file__).parents[1] / "rtk_nav"
sys.path.insert(0, str(PACKAGE_DIR))

from openmv_serial_protocol import (  # noqa: E402
    FRAME_PACKET_TYPE,
    FrameStreamDecoder,
    build_frame_packet,
)


def test_frame_packet_round_trips_across_arbitrary_chunks():
    payload = b"\xff\xd8jpeg-payload\xff\xd9"
    packet = build_frame_packet(payload, 42)
    decoder = FrameStreamDecoder(max_payload_size=1024)

    packets = []
    for offset in range(0, len(packet), 3):
        packets.extend(decoder.feed(packet[offset:offset + 3]))

    assert len(packets) == 1
    assert packets[0].packet_type == FRAME_PACKET_TYPE
    assert packets[0].sequence == 42
    assert packets[0].payload == payload


def test_decoder_discards_noise_and_recovers_after_bad_crc():
    valid = build_frame_packet(b"\xff\xd8ok\xff\xd9", 7)
    corrupted = bytearray(build_frame_packet(b"\xff\xd8bad\xff\xd9", 6))
    corrupted[-1] ^= 0xFF
    decoder = FrameStreamDecoder(max_payload_size=1024)

    packets = decoder.feed(b"noise" + bytes(corrupted) + valid)

    assert [(packet.sequence, packet.payload) for packet in packets] == [
        (7, b"\xff\xd8ok\xff\xd9")
    ]


def test_decoder_rejects_oversized_payloads():
    packet = build_frame_packet(b"\xff\xd8large\xff\xd9", 1)
    decoder = FrameStreamDecoder(max_payload_size=4)

    assert decoder.feed(packet) == []


@pytest.mark.parametrize("payload", [b"", b"abc", b"\xff\xd8missing-end"])
def test_protocol_builder_accepts_binary_payloads(payload):
    packet = build_frame_packet(payload, 1)

    assert packet.startswith(b"OMV1")
