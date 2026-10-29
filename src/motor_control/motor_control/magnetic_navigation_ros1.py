#!/usr/bin/env python3
"""ROS1 rospy driver for the W200B-8 RS485 query protocol."""

import time

import serial
import rospy
from std_msgs.msg import Bool, Float32, UInt16


def crc16_modbus(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def build_read_frame(device_id):
    body = bytes((device_id, 0x03, 0x00, 0x01, 0x00, 0x01))
    crc = crc16_modbus(body)
    return body + bytes((crc & 0xFF, crc >> 8))


def decode_response(frame, device_id):
    if len(frame) != 7 or frame[:3] != bytes((device_id, 0x03, 0x02)):
        return None
    if crc16_modbus(frame[:5]) != frame[5] | (frame[6] << 8):
        return None
    return (frame[3] << 8) | frame[4]


def magnetic_error(mask):
    active = [index for index in range(8) if mask & (1 << index)]
    return None if not active else sum(index - 3.5 for index in active) / len(active)


class MagneticNavigationRos1(object):
    def __init__(self):
        port = rospy.get_param("~serial_port", "/dev/magnetic_nav")
        baud = rospy.get_param("~baud_rate", 115200)
        device_id = int(rospy.get_param("~device_id", 8))
        period = float(rospy.get_param("~poll_interval", 0.05))
        self.device_id = device_id
        self.frame = build_read_frame(device_id)
        self.error_pub = rospy.Publisher("/magnetic/error", Float32, queue_size=10)
        self.valid_pub = rospy.Publisher("/magnetic/valid", Bool, queue_size=10)
        self.mask_pub = rospy.Publisher("/magnetic/mask", UInt16, queue_size=10)
        try:
            self.serial = serial.Serial(port, baudrate=baud, timeout=0.05)
        except (serial.SerialException, OSError) as exc:
            rospy.logfatal("磁导航串口打开失败 %s: %s", port, exc)
            raise
        rospy.loginfo("磁导航 ROS1 节点已启动: %s @ %s", port, baud)
        self.timer = rospy.Timer(rospy.Duration(period), self.poll)

    def poll(self, _event):
        try:
            self.serial.reset_input_buffer()
            self.serial.write(self.frame)
            self.serial.flush()
            register = decode_response(self.serial.read(7), self.device_id)
        except (serial.SerialException, OSError) as exc:
            rospy.logerr_throttle(5.0, "磁导航读取失败: %s", exc)
            register = None
        valid = register is not None
        self.valid_pub.publish(valid)
        if not valid:
            return
        self.mask_pub.publish(register)
        error = magnetic_error(register & 0xFF)
        if error is not None:
            self.error_pub.publish(error)

    def close(self):
        if self.serial.is_open:
            self.serial.close()


def main():
    rospy.init_node("magnetic_navigation")
    node = MagneticNavigationRos1()
    rospy.on_shutdown(node.close)
    rospy.spin()


if __name__ == "__main__":
    main()
