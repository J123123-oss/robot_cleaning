"""Read and set Kinco iSMK target current limits over CANopen SDO."""

import argparse
import math
import time

import can


CURRENT_LIMIT_INDEX = 0x6073
SAVE_CONTROL_INDEX = 0x2FF0
SAVE_CONTROL_SUBINDEX = 0x01
SDO_RX_BASE = 0x600
SDO_TX_BASE = 0x580
SDO_READ = 0x40
SDO_WRITE_2 = 0x2B
SDO_READ_2_RESPONSE = 0x4B
SDO_WRITE_RESPONSE = 0x60
SDO_ABORT = 0x80


class KincoProtocolError(RuntimeError):
    """Raised when a Kinco SDO transaction fails or times out."""


def _validate_motor_id(motor_id):
    if not isinstance(motor_id, int) or not 1 <= motor_id <= 0x7F:
        raise ValueError("motor_id must be an integer from 1 to 127")


def _validate_raw_value(raw_value):
    if not isinstance(raw_value, int) or not 0 <= raw_value <= 0xFFFF:
        raise ValueError("raw current-limit DEC must be an unsigned 16-bit value")


def build_sdo_read_frame(motor_id):
    """Build the SDO upload request for Kinco 0x6073:00."""
    _validate_motor_id(motor_id)
    return bytes((SDO_READ, 0x73, 0x60, 0, 0, 0, 0, 0))


def build_sdo_write_frame(motor_id, raw_value):
    """Build the SDO two-byte download request for Kinco 0x6073:00."""
    _validate_motor_id(motor_id)
    _validate_raw_value(raw_value)
    return bytes((
        SDO_WRITE_2, 0x73, 0x60, 0,
        raw_value & 0xFF, (raw_value >> 8) & 0xFF, 0, 0,
    ))


def build_sdo_save_control_frame(motor_id):
    """Build the Kinco command that saves control parameters."""
    _validate_motor_id(motor_id)
    return bytes((
        0x2F, SAVE_CONTROL_INDEX & 0xFF, SAVE_CONTROL_INDEX >> 8,
        SAVE_CONTROL_SUBINDEX, 0x01, 0, 0, 0,
    ))


def arms_to_dec(arms, ipeak):
    """Convert RMS current to the Kinco current-object DEC value."""
    arms = float(arms)
    ipeak = float(ipeak)
    if not math.isfinite(arms) or arms < 0:
        raise ValueError("arms must be finite and non-negative")
    if not math.isfinite(ipeak) or ipeak <= 0:
        raise ValueError("ipeak must be finite and positive")
    raw_value = round(arms * 2048 / (ipeak / 1.414))
    _validate_raw_value(raw_value)
    return raw_value


def dec_to_arms(raw_value, ipeak):
    """Convert a Kinco current-object DEC value to RMS current."""
    _validate_raw_value(raw_value)
    ipeak = float(ipeak)
    if not math.isfinite(ipeak) or ipeak <= 0:
        raise ValueError("ipeak must be finite and positive")
    return raw_value * (ipeak / 1.414) / 2048


def _send_frame(bus, motor_id, payload):
    message = can.Message(
        arbitration_id=SDO_RX_BASE + motor_id,
        data=payload,
        is_extended_id=False,
    )
    bus.send(message)


def _receive_response(
    bus, motor_id, expected_index, expected_subindex, expected_command, timeout
):
    deadline = time.monotonic() + timeout
    response_id = SDO_TX_BASE + motor_id
    while time.monotonic() < deadline:
        message = bus.recv(timeout=max(0.0, deadline - time.monotonic()))
        if message is None or message.arbitration_id != response_id:
            continue
        data = bytes(message.data)
        if len(data) < 8:
            continue
        response_index = int.from_bytes(data[1:3], "little")
        if response_index != expected_index or data[3] != expected_subindex:
            continue
        if data[0] == SDO_ABORT:
            code = int.from_bytes(data[4:8], "little")
            raise KincoProtocolError(
                f"Kinco SDO abort for 0x{expected_index:04X}:"
                f"00, code=0x{code:08X}"
            )
        if data[0] != expected_command:
            raise KincoProtocolError(
                f"unexpected Kinco SDO response command 0x{data[0]:02X}"
            )
        return data
    raise KincoProtocolError(
        f"timeout waiting for Kinco node {motor_id} response "
        f"to 0x{expected_index:04X}:00"
    )


def read_current_limit(bus, motor_id, timeout=0.5):
    """Read 0x6073:00 and return its unsigned raw DEC value."""
    _send_frame(bus, motor_id, build_sdo_read_frame(motor_id))
    response = _receive_response(
        bus, motor_id, CURRENT_LIMIT_INDEX, 0, SDO_READ_2_RESPONSE, timeout
    )
    return int.from_bytes(response[4:6], "little")


def set_current_limit(bus, motor_id, raw_value, timeout=0.5):
    """Write 0x6073:00, then save control parameters to nonvolatile storage."""
    _send_frame(bus, motor_id, build_sdo_write_frame(motor_id, raw_value))
    _receive_response(
        bus, motor_id, CURRENT_LIMIT_INDEX, 0, SDO_WRITE_RESPONSE, timeout
    )
    _send_frame(bus, motor_id, build_sdo_save_control_frame(motor_id))
    _receive_response(
        bus, motor_id, SAVE_CONTROL_INDEX, SAVE_CONTROL_SUBINDEX,
        SDO_WRITE_RESPONSE, timeout
    )
    return True


def open_bus(channel, bitrate):
    """Open the requested SocketCAN channel."""
    return can.Bus(interface="socketcan", channel=channel, bitrate=bitrate)


def _build_parser():
    parser = argparse.ArgumentParser(
        description="Read or set Kinco iSMK target current limit 0x6073:00"
    )
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--bitrate", type=int, default=500000)
    parser.add_argument("--timeout", type=float, default=0.5)
    subparsers = parser.add_subparsers(dest="command", required=True)

    read_parser = subparsers.add_parser("read")
    read_parser.add_argument("--motor-id", type=int, required=True)
    read_parser.add_argument("--ipeak", type=float)

    set_parser = subparsers.add_parser("set")
    set_parser.add_argument("--motor-id", type=int, required=True)
    value_group = set_parser.add_mutually_exclusive_group(required=True)
    value_group.add_argument("--raw-dec", type=int)
    value_group.add_argument("--arms", type=float)
    set_parser.add_argument("--ipeak", type=float)
    return parser


def main(argv=None):
    """Run the standalone read/set command."""
    parser = _build_parser()
    args = parser.parse_args(argv)
    if args.timeout <= 0 or not math.isfinite(args.timeout):
        parser.error("--timeout must be finite and positive")
    if args.command == "set" and args.arms is not None and args.ipeak is None:
        parser.error("--ipeak is required with --arms")
    if args.command == "set" and args.raw_dec is not None and args.ipeak is not None:
        parser.error("--ipeak can only be used with --arms")

    bus = open_bus(args.channel, args.bitrate)
    try:
        if args.command == "read":
            raw_value = read_current_limit(
                bus, args.motor_id, timeout=args.timeout
            )
            output = f"motor_id={args.motor_id} raw_dec={raw_value}"
            if args.ipeak is not None:
                output += f" arms={dec_to_arms(raw_value, args.ipeak):.3f}"
            print(output)
        else:
            raw_value = (
                args.raw_dec
                if args.raw_dec is not None
                else arms_to_dec(args.arms, args.ipeak)
            )
            set_current_limit(
                bus, args.motor_id, raw_value, timeout=args.timeout
            )
            print(f"motor_id={args.motor_id} raw_dec={raw_value} set")
    finally:
        bus.shutdown()
    return 0


if __name__ == "__main__":
    main()
