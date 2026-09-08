"""Standalone OpenMV Light Shield handshake diagnostic.

Upload this file to the camera as ``main.py`` while debugging the light.
It deliberately does not import or use ``sensor`` and sends only readable
ASCII diagnostics over USB VCP.
"""

import time

from machine import Pin, PWM
from pyb import USB_VCP


LIGHT_BRIGHTNESS = 50
LIGHT_PWM_FREQUENCY_HZ = 50_000
LIGHT_HEARTBEAT_TIMEOUT_MS = 2000
COMMAND_READ_BYTES = 64

LIGHT_ON_COMMAND = b"LIGHT_ON"
LIGHT_OFF_COMMAND = b"LIGHT_OFF"
LIGHT_HEARTBEAT_COMMAND = b"LIGHT_HEARTBEAT"
LIGHT_BRIGHTNESS_COMMAND_PREFIX = b"LIGHT_BRIGHTNESS="
LIGHT_BRIGHTNESS_MIN = 0
LIGHT_BRIGHTNESS_MAX = 100


usb = USB_VCP()
command_buffer = bytearray()
light_enabled = False
light_brightness = LIGHT_BRIGHTNESS
last_heartbeat_ms = None


def send_log(message):
    """Send a readable diagnostic line without emitting image data."""
    try:
        usb.send(("[FillLight] " + message + "\r\n").encode(), timeout=100)
    except Exception:
        pass


try:
    pwm = PWM(
        Pin("P6"),
        freq=LIGHT_PWM_FREQUENCY_HZ,
        duty_u16=0,
    )
except Exception as exc:
    pwm = None
    send_log("PWM_INIT_ERROR pin=P6 error=" + str(exc))


def apply_fill_light():
    """Apply the current state and brightness to the Light Shield PWM."""
    duty_u16 = (
        (light_brightness * 65535) // 100 if light_enabled else 0
    )
    pwm.duty_u16(duty_u16)


def set_fill_light(enabled):
    """Apply the requested state to the Light Shield PWM output."""
    global light_enabled

    if pwm is None:
        return False

    enabled = bool(enabled)
    light_enabled = enabled
    apply_fill_light()
    return True


def set_light_brightness(value):
    """Update PWM brightness immediately without resetting the camera."""
    global light_brightness

    if pwm is None:
        return False
    if value < LIGHT_BRIGHTNESS_MIN or value > LIGHT_BRIGHTNESS_MAX:
        return False

    light_brightness = value
    apply_fill_light()
    return True


def command_separator():
    """Return the first CR/LF separator in the command buffer, if present."""
    separator_lf = command_buffer.find(b"\n")
    separator_cr = command_buffer.find(b"\r")
    if separator_lf < 0:
        return separator_cr
    if separator_cr < 0:
        return separator_lf
    return min(separator_lf, separator_cr)


def handle_command(command):
    """Handle one complete ASCII command and acknowledge it."""
    global last_heartbeat_ms

    if command.startswith(LIGHT_BRIGHTNESS_COMMAND_PREFIX):
        try:
            brightness = int(
                command[len(LIGHT_BRIGHTNESS_COMMAND_PREFIX):].decode()
            )
        except (ValueError, TypeError):
            brightness = None

        if brightness is None or not set_light_brightness(brightness):
            send_log("RX LIGHT_BRIGHTNESS_ERROR")
        else:
            send_log(
                "RX "
                + command.decode()
                + " -> brightness="
                + str(light_brightness)
                + "% duty_u16="
                + str((light_brightness * 65535) // 100 if light_enabled else 0)
            )
        return

    if command == LIGHT_ON_COMMAND or command == LIGHT_HEARTBEAT_COMMAND:
        last_heartbeat_ms = time.ticks_ms()
        if set_fill_light(True):
            send_log(
                "RX "
                + command.decode()
                + " -> ON duty_u16="
                + str((light_brightness * 65535) // 100)
            )
        return

    if command == LIGHT_OFF_COMMAND:
        last_heartbeat_ms = None
        if set_fill_light(False):
            send_log("RX LIGHT_OFF -> OFF duty_u16=0")
        return

    if command:
        send_log("RX UNKNOWN " + command.decode())


def process_commands():
    """Read all currently available bytes and process complete commands."""
    global command_buffer

    if not usb.any():
        return

    data = usb.recv(COMMAND_READ_BYTES, timeout=0)
    if not data:
        return

    command_buffer.extend(data)
    if len(command_buffer) > 128:
        command_buffer = command_buffer[-128:]

    while True:
        separator = command_separator()
        if separator < 0:
            return

        command = bytes(command_buffer[:separator]).strip()
        command_buffer = command_buffer[separator + 1:]
        handle_command(command)


def expire_heartbeat():
    """Turn the light off when heartbeat traffic stops."""
    global last_heartbeat_ms

    if last_heartbeat_ms is None:
        return
    if time.ticks_diff(time.ticks_ms(), last_heartbeat_ms) <= LIGHT_HEARTBEAT_TIMEOUT_MS:
        return

    last_heartbeat_ms = None
    if set_fill_light(False):
        send_log("HEARTBEAT_TIMEOUT -> OFF duty_u16=0")


if pwm is not None:
    send_log(
        "READY pin=P6 freq="
        + str(LIGHT_PWM_FREQUENCY_HZ)
        + " brightness="
        + str(LIGHT_BRIGHTNESS)
        + "% duty_u16=0"
    )

while True:
    process_commands()
    expire_heartbeat()
    time.sleep_ms(20)
