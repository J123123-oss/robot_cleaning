#!/usr/bin/env python3
"""Send one Modbus RTU frame to the charging controller."""

import serial


PORT = "/dev/battery_charging"
BAUDRATE = 19200
FRAME = bytes.fromhex("02 06 00 10 01 90 89 C0")


def main():
    with serial.Serial(PORT, BAUDRATE, timeout=1) as device:
        device.write(FRAME)
        device.flush()
        print(f"sent {len(FRAME)} bytes to {PORT}: {FRAME.hex(' ').upper()}")
        response = device.read_all()
        if response:
            print(f"received {len(response)} bytes: {response.hex(' ').upper()}")
        else:
            print("received: <none>")


if __name__ == "__main__":
    main()
