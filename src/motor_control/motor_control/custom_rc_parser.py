#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import serial
import struct
import argparse
import sys
import signal
import time

class CustomRCParser:
    def __init__(self, port='/dev/ttyUSB4', baudrate=115200, debug=False):
        self.port = port
        self.baudrate = baudrate
        self.debug = debug
        self.serial_conn = None
        self.frame_header_byte = 0x0f  # 只使用单字节帧头
        self.data_length = 32  # 数据长度为32字节
        self.frame_size = 1 + self.data_length  # 总帧长 = 帧头(1) + 数据(32)
        self.last_print_time = 0
        self.print_interval = 0.02  # 50Hz output rate
        self.sync_loss_count = 0
        self.frame_count = 0
        self.error_count = 0
        self.channels = []
        
    def connect(self):
        """建立串口连接"""
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.005  # 5ms超时
            )
            print(f"成功连接到 {self.port}，波特率 {self.baudrate}")
            return True
        except Exception as e:
            print(f"无法连接到 {self.port}: {e}")
            return False
            
    def disconnect(self):
        """断开串口连接"""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            print("串口连接已关闭")
            
    def parse_frame(self, frame_data):
        """
        解析帧数据
        帧头(1字节) + 数据(32字节)
        """
        if len(frame_data) != self.frame_size:
            if self.debug:
                print(f"帧长度错误: 期望{self.frame_size}字节，实际{len(frame_data)}字节")
            return None
            
        # 检查帧头
        if frame_data[0] != self.frame_header_byte:
            if self.debug:
                print(f"帧头错误: 期望{self.frame_header_byte:02x}，实际{frame_data[0]:02x}")
            return None
            
        # 解析16个通道数据 (大端序)
        channels = []
        for i in range(16):
            # 每个通道数据占2字节，从第1字节开始（跳过帧头）
            offset = 1 + i * 2
            # 使用大端序解析
            channel_value = struct.unpack('>H', frame_data[offset:offset+2])[0]
            channels.append(channel_value)
            
        return channels
        
    def find_valid_frames(self, buffer):
        """
        在缓冲区中寻找所有可能的有效帧
        """
        valid_frames = []
        pos = 0
        
        while pos <= len(buffer) - self.frame_size:
            # 检查当前位置是否为有效帧头
            if buffer[pos] == self.frame_header_byte and len(buffer) >= pos + self.frame_size:
                frame_data = buffer[pos:pos+self.frame_size]
                channels = self.parse_frame(frame_data)
                if channels is not None:
                    valid_frames.append((pos, frame_data, channels))
            pos += 1
            
        return valid_frames
        
    def receive_data(self):
        """接收并处理数据"""
        buffer = bytearray()
        last_data_time = time.time()
        
        print("开始接收遥控器数据...")
        print("按 Ctrl+C 退出")
        if self.debug:
            print("调试模式已启用")
        
        try:
            while True:
                # 检查是否有数据可读
                if self.serial_conn.in_waiting > 0:
                    # 读取所有可用数据
                    data = self.serial_conn.read(self.serial_conn.in_waiting)
                    buffer.extend(data)
                    last_data_time = time.time()
                    
                    # 限制缓冲区大小以防止内存问题
                    if len(buffer) > self.frame_size * 10:
                        # 保留最近的数据
                        buffer = buffer[-(self.frame_size * 5):]
                        if self.debug:
                            print("缓冲区过大，已裁剪")
                    
                    # 寻找有效帧
                    valid_frames = self.find_valid_frames(buffer)
                    
                    if valid_frames:
                        # 使用第一个有效帧
                        pos, frame_data, channels = valid_frames[0]
                        self.frame_count += 1
                        
                        current_time = time.time()
                        # 控制打印频率
                        if current_time - self.last_print_time >= self.print_interval:
                            # 打印前8个通道的值
                            print(f"CH1:{channels[0]:4d} CH2:{channels[1]:4d} CH3:{channels[2]:4d} CH4:{channels[3]:4d} " +
                                  f"CH5:{channels[4]:4d} CH6:{channels[5]:4d} CH7:{channels[6]:4d} CH8:{channels[7]:4d}")
                            self.last_print_time = current_time
                            self.channels = channels
                            # print(self.channels)
                            if self.debug:
                                print(f"[帧#{self.frame_count}] 所有通道: {[f'CH{i+1}:{val}' for i, val in enumerate(channels)]}")
                                print(f"[帧#{self.frame_count}] 原始数据: {' '.join([f'{b:02x}' for b in frame_data])}")
                        
                        # 从缓冲区中移除已处理的数据
                        buffer = buffer[pos + self.frame_size:]
                        
                        # 重置同步丢失计数
                        self.sync_loss_count = 0
                    else:
                        self.error_count += 1
                        self.sync_loss_count += 1
                        
                        # 如果连续多次未能同步，尝试重新同步
                        if self.sync_loss_count > 50:
                            # 在缓冲区中主动寻找下一个可能的帧头
                            header_pos = -1
                            for i in range(len(buffer)):
                                if buffer[i] == self.frame_header_byte:
                                    header_pos = i
                                    break
                                    
                            if header_pos != -1:
                                # 丢弃帧头之前的所有数据
                                buffer = buffer[header_pos:]
                                if self.debug:
                                    print(f"重新同步，丢弃 {header_pos} 字节数据")
                            else:
                                # 如果找不到帧头，清空缓冲区
                                if len(buffer) > self.frame_size:
                                    buffer = bytearray()
                                    if self.debug:
                                        print("无法同步，清空缓冲区")
                            self.sync_loss_count = 0
                else:
                    # 如果长时间没有数据，可以适当休眠
                    if time.time() - last_data_time > 0.05:
                        time.sleep(0.002)  # 2ms的短暂停留
                        
        except KeyboardInterrupt:
            print(f"\n用户中断，正在退出...")
            print(f"总计处理 {self.frame_count} 帧，错误 {self.error_count} 次")
        except Exception as e:
            print(f"接收数据时出错: {e}")
            if self.debug:
                import traceback
                traceback.print_exc()
        finally:
            self.disconnect()


def main():
    parser = argparse.ArgumentParser(description='自定义遥控器接收程序')
    parser.add_argument('--port', default='/dev/ttyUSB4', help='串口端口 (默认: /dev/ttyUSB4)')
    parser.add_argument('--baudrate', type=int, default=115200, help='波特率 (默认: 115200)')
    parser.add_argument('--debug', action='store_true', help='启用调试模式')
    
    args = parser.parse_args()
    
    parser = CustomRCParser(port=args.port, baudrate=args.baudrate, debug=args.debug)
    
    # 处理 Ctrl+C 信号
    signal.signal(signal.SIGINT, lambda sig, frame: sys.exit(0))
    
    if parser.connect():
        parser.receive_data()
    else:
        sys.exit(1)


if __name__ == '__main__':
    main()
