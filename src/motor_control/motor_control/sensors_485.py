#!/usr/bin/env python3
import os
import shutil
import subprocess

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8

try:
    import gpiod
except ImportError:
    gpiod = None


class Sensors485(Node):
    """Read the four boundary inputs from the RK3576 GPIO expander."""

    def __init__(self):
        super().__init__('sensors_485_node')

        self.declare_parameter('gpio_chip', '/dev/gpiochip6')
        self.declare_parameter('gpio_lines', [9, 8, 7, 6])
        self.declare_parameter('poll_interval', 0.1)
        self.declare_parameter('gpio_use_sudo', False)

        # Keep legacy launch parameters declared while the 485 IO source is removed.
        self.declare_parameter('port', '/dev/ttyS1')
        self.declare_parameter('baud', 9600)

        self.gpio_chip = str(self.get_parameter('gpio_chip').value)
        self.gpio_lines = [
            int(line) for line in self.get_parameter('gpio_lines').value
        ]
        self.poll_interval = float(self.get_parameter('poll_interval').value)
        self.gpio_use_sudo = bool(self.get_parameter('gpio_use_sudo').value)

        if len(self.gpio_lines) != 4:
            raise ValueError('gpio_lines must contain exactly four line offsets')
        if any(line < 0 or line > 23 for line in self.gpio_lines):
            raise ValueError('gpio_lines must be valid gpiochip6 offsets (0-23)')

        self.gpio_request = None
        self.gpio_chip_handle = None
        self.gpio_api = None
        self.gpio_command_prefix = []
        self.last_gpio_error_time = 0.0

        self.io_pub = self.create_publisher(UInt8, '/io_data', 1)
        self.open_gpio_lines()
        self.polling_timer = self.create_timer(
            self.poll_interval, self.polling_callback
        )

    def open_gpio_lines(self):
        if gpiod is None:
            if shutil.which('gpioget') is None:
                raise RuntimeError(
                    'Neither Python gpiod nor the gpioget command is installed'
                )

            if self.gpio_use_sudo and getattr(os, 'geteuid', lambda: 0)() != 0:
                if shutil.which('sudo') is None:
                    raise RuntimeError(
                        'sudo is required for GPIO reads but is not installed'
                    )
                self.gpio_command_prefix = ['sudo', '-n']

            self.gpio_api = 'cli'
            self.get_logger().warning(
                'Python gpiod is unavailable; falling back to gpioget'
            )
            return

        # libgpiod 2.x uses request_lines(); older boards commonly expose 1.x.
        if hasattr(gpiod, 'request_lines'):
            from gpiod.line import Direction, LineSettings

            settings = LineSettings(direction=Direction.INPUT)
            config = {line: settings for line in self.gpio_lines}
            self.gpio_request = gpiod.request_lines(
                self.gpio_chip,
                consumer='sensors_485_node',
                config=config,
            )
            self.gpio_api = 'v2'
        else:
            chip_name = self.gpio_chip.removeprefix('/dev/')
            self.gpio_chip_handle = gpiod.Chip(chip_name)
            self.gpio_request = self.gpio_chip_handle.get_lines(self.gpio_lines)
            self.gpio_request.request(
                consumer='sensors_485_node',
                type=gpiod.LINE_REQ_DIR_IN,
            )
            self.gpio_api = 'v1'

        self.get_logger().info(
            f'Opened {self.gpio_chip} lines {self.gpio_lines} '
            f'using libgpiod {self.gpio_api}'
        )

    def read_gpio_values(self):
        if self.gpio_api == 'cli':
            chip_name = self.gpio_chip.removeprefix('/dev/')
            command = [
                *self.gpio_command_prefix,
                'gpioget',
                chip_name,
                *(str(line) for line in self.gpio_lines),
            ]
            try:
                result = subprocess.run(
                    command,
                    check=True,
                    capture_output=True,
                    text=True,
                    timeout=max(0.2, self.poll_interval),
                )
            except subprocess.CalledProcessError as exc:
                detail = exc.stderr.strip() or 'no error details'
                raise RuntimeError(
                    f'gpioget failed with exit code {exc.returncode}: {detail}'
                ) from exc
            except subprocess.TimeoutExpired as exc:
                raise RuntimeError('gpioget timed out') from exc

            raw_values = result.stdout.split()
            if len(raw_values) != len(self.gpio_lines):
                raise RuntimeError(
                    f'unexpected gpioget output: {result.stdout.strip()}'
                )

            value_map = {
                '0': 0,
                '1': 1,
                'inactive': 0,
                'active': 1,
            }
            try:
                return [value_map[value.lower()] for value in raw_values]
            except KeyError as exc:
                raise RuntimeError(
                    f'unrecognized gpioget value: {exc.args[0]}'
                ) from exc

        if self.gpio_api == 'v2':
            values = []
            for line in self.gpio_lines:
                raw_value = self.gpio_request.get_value(line)
                value_name = getattr(raw_value, 'name', None)
                if value_name is not None:
                    values.append(int(value_name == 'ACTIVE'))
                else:
                    values.append(int(raw_value))
            return values

        return [int(value) for value in self.gpio_request.get_values()]

    def polling_callback(self):
        try:
            gpio_values = self.read_gpio_values()
        except Exception as exc:
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self.last_gpio_error_time >= 1.0:
                self.get_logger().error(f'GPIO read failed: {exc}')
                self.last_gpio_error_time = now
            return

        # line 9/8/7/6 maps to bit 0/1/2/3. The consumer treats low as active.
        io_bitmap = sum(
            (value & 0x01) << bit for bit, value in enumerate(gpio_values)
        )
        msg = UInt8()
        msg.data = io_bitmap
        self.io_pub.publish(msg)

    def destroy_node(self):
        if self.gpio_request is not None:
            try:
                self.gpio_request.release()
            except Exception as exc:
                self.get_logger().warning(f'GPIO release failed: {exc}')

        if self.gpio_chip_handle is not None:
            close = getattr(self.gpio_chip_handle, 'close', None)
            if close is not None:
                close()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Sensors485()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
