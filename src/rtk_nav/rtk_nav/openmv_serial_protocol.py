"""Framing helpers for JPEG frames sent by OpenMV over USB serial."""

from dataclasses import dataclass
import struct
import zlib


MAGIC = b"OMV1"
PROTOCOL_VERSION = 1
FRAME_PACKET_TYPE = 1
HEADER_FORMAT = "<4sBBHII"
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
CRC_SIZE = 4


@dataclass(frozen=True)
class FramePacket:
    """A validated packet decoded from the OpenMV byte stream."""

    packet_type: int
    sequence: int
    payload: bytes


def build_frame_packet(payload, sequence):
    """Build one framed JPEG packet."""
    payload = bytes(payload)
    header = struct.pack(
        HEADER_FORMAT,
        MAGIC,
        PROTOCOL_VERSION,
        FRAME_PACKET_TYPE,
        0,
        int(sequence) & 0xFFFFFFFF,
        len(payload),
    )
    checksum = zlib.crc32(header + payload) & 0xFFFFFFFF
    return header + payload + struct.pack("<I", checksum)


class FrameStreamDecoder:
    """Decode framed packets while tolerating partial reads and noise."""

    def __init__(self, max_payload_size=2 * 1024 * 1024):
        if int(max_payload_size) <= 0:
            raise ValueError("max_payload_size must be positive")
        self.max_payload_size = int(max_payload_size)
        self._buffer = bytearray()

    def reset(self):
        """Discard buffered bytes after a serial disconnect."""
        self._buffer.clear()

    def feed(self, data):
        """Append bytes and return every complete, checksum-valid packet."""
        if data:
            self._buffer.extend(data)

        packets = []
        while True:
            start = self._buffer.find(MAGIC)
            if start < 0:
                keep = max(0, len(MAGIC) - 1)
                if len(self._buffer) > keep:
                    del self._buffer[:-keep]
                break
            if start:
                del self._buffer[:start]
            if len(self._buffer) < HEADER_SIZE:
                break

            (
                magic,
                version,
                packet_type,
                _reserved,
                sequence,
                payload_size,
            ) = struct.unpack_from(HEADER_FORMAT, self._buffer)
            if (
                magic != MAGIC
                or version != PROTOCOL_VERSION
                or payload_size > self.max_payload_size
            ):
                del self._buffer[0]
                continue

            packet_size = HEADER_SIZE + payload_size + CRC_SIZE
            if len(self._buffer) < packet_size:
                break

            packet_end = HEADER_SIZE + payload_size
            packet_data = bytes(self._buffer[:packet_end])
            expected_crc = struct.unpack_from("<I", self._buffer, packet_end)[0]
            actual_crc = zlib.crc32(packet_data) & 0xFFFFFFFF
            if actual_crc != expected_crc:
                del self._buffer[0]
                continue

            packets.append(
                FramePacket(
                    packet_type=packet_type,
                    sequence=sequence,
                    payload=bytes(self._buffer[HEADER_SIZE:packet_end]),
                )
            )
            del self._buffer[:packet_size]

        return packets
