#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import can
import time
import rclpy
from rclpy.node import Node

class CanMotorDriver(Node):
    def __init__(self, node_name='can_motor_driver', channel='can0', interface='socketcan', baudrate=1000000):
        super().__init__(node_name)
        self.channel = channel
        self.interface = interface
        self.baudrate = baudrate  # 波特率1Mbps
        self.bus = self.create_can_bus()
        self.motor_driver_status = True  # 驱动状态标记
        self.send_base_id = 0x140  # 单电机指令发送基地址
        self.reply_base_id = 0x240  # 单电机回复基地址
        self.multi_cmd_base_id = 0x280  # 多电机命令基地址
        self.global_cmd_id = 0x300  # 全局ID指令地址

    # -------------------------- CAN总线连接 --------------------------
    def create_can_bus(self):
        """创建CAN总线连接（1Mbps波特率，自动重试）"""
        while rclpy.ok():
            try:
                # 配置CAN总线波特率为1Mbps
                return can.interface.Bus(
                    channel=self.channel,
                    interface=self.interface,
                    bitrate=self.baudrate
                )
            except (can.CanError, OSError) as e:
                self.get_logger().error(f"CAN总线连接失败: {e}，3秒后重试...")
                time.sleep(3)

    def reconnect_can_bus(self):
        """重连CAN总线"""
        self.get_logger().warn("尝试重连CAN总线...")
        try:
            if self.bus is not None:
                self.bus.shutdown()
        except Exception:
            pass
        self.bus = self.create_can_bus()

    # -------------------------- CAN指令发送 --------------------------
    def send_command(self, motor_id, command_data, is_global=False, is_multi=False):
        """
        发送CAN指令（自动重试3次）
        :param motor_id: 电机ID（1~32）
        :param command_data: 8字节指令数据
        :param is_global: 是否全局指令（使用0x300地址）
        :param is_multi: 是否多电机命令（使用0x280+指令格式）
        :return: 发送成功返回True，失败返回False
        """
        if len(command_data) != 8:
            self.get_logger().error("指令数据必须为8字节")
            return False
        
        if is_global:
            frame_id = self.global_cmd_id
        elif is_multi:
            frame_id = self.multi_cmd_base_id + command_data[0]  # 0x280+指令码
        else:
            if not 1 <= motor_id <= 32:
                self.get_logger().error(f"电机ID{motor_id}超出有效范围（1-32）")
                return False
            frame_id = self.send_base_id + motor_id

        msg = can.Message(arbitration_id=frame_id, data=command_data, is_extended_id=False)
        for attempt in range(3):
            try:
                self.bus.send(msg)
                time.sleep(0.05)
                return True
            except (can.CanError, OSError) as e:
                self.get_logger().error(f"CAN发送失败（电机{motor_id}）: {e}，第{attempt+1}次重试...")
                self.reconnect_can_bus()
        self.get_logger().error(f"电机{motor_id} CAN指令发送失败，已重试3次")
        return False

    # -------------------------- 电机基础控制 --------------------------
    def enable_drive(self, motor_id):
        """使能电机（松开抱闸，电机可运动）"""
        # 使能指令：0x77 00 00 00 00 00 00 00
        return self.send_command(motor_id, [0x77, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])

    def disable_drive(self, motor_id):
        """非使能电机"""
        # 非使能指令：0x80 00 00 00 00 00 00 00
        return self.send_command(motor_id, [0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])

    def stop_motor(self, motor_id):
        """停止电机（不关使能）"""
        # 停止指令：0x81 00 00 00 00 00 00 00
        return self.send_command(motor_id, [0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])

    def brake_lock(self, motor_id):
        """抱闸锁死（关闭系统抱闸）"""
        # 抱闸锁死指令：0x78 00 00 00 00 00 00 00
        return self.send_command(motor_id, [0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])

    def brake_release(self, motor_id):
        """抱闸释放（开启系统抱闸，同使能指令）"""
        return self.enable_drive(motor_id)  # 使能指令同时实现抱闸释放

    def motor_reset(self, motor_id):
        """电机复位（无返回值）"""
        # 复位指令：0x76 00 00 00 00 00 00 00
        return self.send_command(motor_id, [0x76, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])

    # -------------------------- 速度控制相关 --------------------------
    def set_velocity_closed_loop(self, motor_id, target_speed, max_torque=255):
        """
        速度闭环控制
        :param motor_id: 电机ID
        :param target_speed: 目标速度（dps），实际发送时按0.01dps/LSB缩放（乘以100）
        :param max_torque: 最大扭矩（0-255）
        :return: 发送成功返回True
        """
        if not 0 <= max_torque <= 255:
            self.get_logger().error("最大扭矩值必须在0-255之间")
            return False
        
        # 速度值转换：target_speed(dps) = 发送值 * 0.01dps/LSB → 发送值 = target_speed * 100
        send_speed = int(target_speed * 100)
        # 4字节速度数据（低字节在前，高字节在后）
        speed_bytes = [
            send_speed & 0xFF,
            (send_speed >> 8) & 0xFF,
            (send_speed >> 16) & 0xFF,
            (send_speed >> 24) & 0xFF
        ]
        # 指令格式：A2 [最大扭矩] 00 00 [速度低字节] [速度中低字节] [速度中高字节] [速度高字节]
        command_data = [0xA2, max_torque, 0x00, 0x00] + speed_bytes
        return self.send_command(motor_id, command_data)

    # def set_rotation_direction(self, motor_id, is_forward=True):
    #     """
    #     设置正反转（通过速度正负实现，此方法为方向控制封装）
    #     :param motor_id: 电机ID
    #     :param is_forward: True-正转，False-反转
    #     :return: 始终返回True（方向通过速度值正负实际控制）
    #     """
    #     # 正反转通过速度值正负实现，此处仅作为逻辑封装
    #     direction = "正转" if is_forward else "反转"
    #     self.get_logger().info(f"电机{motor_id}设置为{direction}（实际通过速度正负控制）")
    #     return True

    def set_acceleration(self, motor_id, acceleration):
        """
        设置速度规划加速度（单位：dps/s，断电保存）
        :param motor_id: 电机ID
        :param acceleration: 加速度值（dps/s）
        :return: 发送成功返回True
        """
        # 指令格式：43 02 00 00 [加速度低字节] [加速度中低字节] [加速度中高字节] [加速度高字节]
        acc_bytes = [
            acceleration & 0xFF,
            (acceleration >> 8) & 0xFF,
            (acceleration >> 16) & 0xFF,
            (acceleration >> 24) & 0xFF
        ]
        command_data = [0x43, 0x02, 0x00, 0x00] + acc_bytes
        return self.send_command(motor_id, command_data)

    def set_deceleration(self, motor_id, deceleration):
        """
        设置速度规划减速度（单位：dps/s，断电保存）
        :param motor_id: 电机ID
        :param deceleration: 减速度值（dps/s）
        :return: 发送成功返回True
        """
        # 指令格式：43 03 00 00 [减速度低字节] [减速度中低字节] [减速度中高字节] [减速度高字节]
        dec_bytes = [
            deceleration & 0xFF,
            (deceleration >> 8) & 0xFF,
            (deceleration >> 16) & 0xFF,
            (deceleration >> 24) & 0xFF
        ]
        command_data = [0x43, 0x03, 0x00, 0x00] + dec_bytes
        return self.send_command(motor_id, command_data)

    def get_actual_velocity(self, motor_id):
        """
        读取电机实际速度（通过读取状态2获取）
        :return: 实际速度（dps），超时返回0
        """
        # 发送读取状态2指令：0x9C 00 00 00 00 00 00 00
        if not self.send_command(motor_id, [0x9C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]):
            return 0
        
        start_time = time.time()
        while time.time() - start_time < 0.5:
            try:
                msg = self.bus.recv(timeout=0.1)
            except (can.CanError, OSError) as e:
                self.get_logger().error(f"CAN接收失败（电机{motor_id}）: {e}，尝试重连...")
                self.reconnect_can_bus()
                continue
            
            # 校验响应帧：回复地址为0x240+电机ID，指令码为0x9C
            if msg and msg.arbitration_id == (self.reply_base_id + motor_id):
                if len(msg.data) >= 8 and msg.data[0] == 0x9C:
                    # 速度数据：Data[5]（高字节）、Data[4]（低字节），int16_t类型，1dps/LSB
                    speed = msg.data[4] | (msg.data[5] << 8)
                    # 处理负速度
                    if speed > 0x7FFF:
                        speed -= 0x10000
                    return speed
        
        self.get_logger().warn(f"读取电机{motor_id}实际速度超时")
        return 0

    # -------------------------- 状态读取相关 --------------------------
    def parse_error_state(self, error_code):
        """解析错误状态码"""
        error_map = {
            0x0002: "电机堵转",
            0x0004: "低压",
            0x0008: "过压",
            0x0010: "相电流过流",
            0x0040: "功率超限",
            0x0080: "标定参数写入错误",
            0x0100: "超速",
            0x0800: "元器件过温",
            0x1000: "电机温度过温",
            0x2000: "编码器校准错误",
            0x4000: "编码器数据错误"
        }
        errors = []
        for code, desc in error_map.items():
            if error_code & code:
                errors.append(desc)
        return errors if errors else ["无错误"]

    def get_motor_state1(self, motor_id):
        """
        读取电机状态1（温度、抱闸、电压）与错误标志
        :return: 字典包含温度、抱闸状态、电压、错误信息
        """
        # 发送状态1读取指令：0x9A 00 00 00 00 00 00 00
        if not self.send_command(motor_id, [0x9A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]):
            return None
        
        start_time = time.time()
        while time.time() - start_time < 0.5:
            try:
                msg = self.bus.recv(timeout=0.1)
            except (can.CanError, OSError) as e:
                self.get_logger().error(f"CAN接收失败（电机{motor_id}）: {e}，尝试重连...")
                self.reconnect_can_bus()
                continue
            
            if msg and msg.arbitration_id == (self.reply_base_id + motor_id):
                if len(msg.data) >= 8 and msg.data[0] == 0x9A:
                    # 解析数据
                    temperature = msg.data[1]  # int8_t类型，1℃/LSB
                    if temperature > 0x7F:
                        temperature -= 0x100  # 转换为负温度
                    
                    brake_status = (msg.data[3] & 0x01)  # 假设bit0为抱闸状态位（0-锁死，1-释放）
                    brake_desc = "释放" if brake_status else "锁死"
                    
                    voltage = (msg.data[4] | (msg.data[5] << 8)) * 0.1  # 0.1V/LSB
                    
                    error_code = msg.data[6] | (msg.data[7] << 8)  # 错误状态码
                    error_info = self.parse_error_state(error_code)
                    
                    return {
                        "temperature": temperature,
                        "brake_status": brake_desc,
                        "voltage": round(voltage, 1),
                        "error_code": hex(error_code),
                        "error_info": error_info
                    }
        
        self.get_logger().warn(f"读取电机{motor_id}状态1超时")
        return None

    def get_motor_state2(self, motor_id):
        """
        读取电机状态2（转矩、电流、转速、角度）与错误标志
        :return: 字典包含温度、转矩、电流、转速、角度、错误信息
        """
        # 发送状态2读取指令：0x9C 00 00 00 00 00 00 00
        if not self.send_command(motor_id, [0x9C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]):
            return None
        
        start_time = time.time()
        while time.time() - start_time < 0.5:
            try:
                msg = self.bus.recv(timeout=0.1)
            except (can.CanError, OSError) as e:
                self.get_logger().error(f"CAN接收失败（电机{motor_id}）: {e}，尝试重连...")
                self.reconnect_can_bus()
                continue
            
            if msg and msg.arbitration_id == (self.reply_base_id + motor_id):
                if len(msg.data) >= 8 and msg.data[0] == 0x9C:
                    # 解析数据
                    temperature = msg.data[1]  # int8_t类型，1℃/LSB
                    if temperature > 0x7F:
                        temperature -= 0x100
                    
                    torque_current = msg.data[2] | (msg.data[3] << 8)  # iq电流，int16_t，0.01A/LSB
                    if torque_current > 0x7FFF:
                        torque_current -= 0x10000
                    torque = torque_current * 0.01  # 转矩电流值（A）
                    
                    speed = msg.data[4] | (msg.data[5] << 8)  # 转速，int16_t，1dps/LSB
                    if speed > 0x7FFF:
                        speed -= 0x10000
                    
                    angle = msg.data[6] | (msg.data[7] << 8)  # 角度，int16_t，1degree/LSB
                    if angle > 0x7FFF:
                        angle -= 0x10000
                    
                    
                    return {
                        "temperature": temperature,
                        "torque_current": round(torque, 2),
                        "speed": speed,
                        "angle": angle,
                    }
        
        self.get_logger().warn(f"读取电机{motor_id}状态2超时")
        return None

    def get_motor_state3(self, motor_id):
        """
        读取电机状态3（温度、三相电流）
        :return: 字典包含温度、A/B/C相电流
        """
        # 发送状态3读取指令：0x9D 00 00 00 00 00 00 00
        if not self.send_command(motor_id, [0x9D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]):
            return None
        
        start_time = time.time()
        while time.time() - start_time < 0.5:
            try:
                msg = self.bus.recv(timeout=0.1)
            except (can.CanError, OSError) as e:
                self.get_logger().error(f"CAN接收失败（电机{motor_id}）: {e}，尝试重连...")
                self.reconnect_can_bus()
                continue
            
            if msg and msg.arbitration_id == (self.reply_base_id + motor_id):
                if len(msg.data) >= 8 and msg.data[0] == 0x9D:
                    # 解析数据
                    temperature = msg.data[1]  # int8_t类型，1℃/LSB
                    if temperature > 0x7F:
                        temperature -= 0x100
                    
                    # A相电流：Data[2]（低）、Data[3]（高），int16_t，0.01A/LSB
                    current_a = msg.data[2] | (msg.data[3] << 8)
                    if current_a > 0x7FFF:
                        current_a -= 0x10000
                    current_a = current_a * 0.01
                    
                    # B相电流：Data[4]（低）、Data[5]（高）
                    current_b = msg.data[4] | (msg.data[5] << 8)
                    if current_b > 0x7FFF:
                        current_b -= 0x10000
                    current_b = current_b * 0.01
                    
                    # C相电流：Data[6]（低）、Data[7]（高）
                    current_c = msg.data[6] | (msg.data[7] << 8)
                    if current_c > 0x7FFF:
                        current_c -= 0x10000
                    current_c = current_c * 0.01
                    
                    return {
                        "temperature": temperature,
                        "current_a": round(current_a, 2),
                        "current_b": round(current_b, 2),
                        "current_c": round(current_c, 2)
                    }
        
        self.get_logger().warn(f"读取电机{motor_id}状态3超时")
        return None

    def get_motor_temperature(self, motor_id):
        """读取电机温度（通过状态1获取）"""
        state1 = self.get_motor_state1(motor_id)
        return state1["temperature"] if state1 else None

    def get_brake_status(self, motor_id):
        """读取抱闸状态（通过状态1获取）"""
        state1 = self.get_motor_state1(motor_id)
        return state1["brake_status"] if state1 else "未知"

    def get_input_voltage(self, motor_id):
        """读取输入电压（通过状态1获取）"""
        state1 = self.get_motor_state1(motor_id)
        return state1["voltage"] if state1 else 0.0

    def get_actual_torque(self, motor_id):
        """读取实际转矩电流（通过状态2获取）"""
        state2 = self.get_motor_state2(motor_id)
        return state2["torque_current"] if state2 else 0.0

    def get_actual_current(self, motor_id):
        """读取三相电流（通过状态3获取）"""
        state3 = self.get_motor_state3(motor_id)
        if state3:
            return {
                "A相": state3["current_a"],
                "B相": state3["current_b"],
                "C相": state3["current_c"]
            }
        return None

    # -------------------------- 配置参数读写 --------------------------
    def set_comm_protection_time(self, motor_id, protect_time):
        """
        设置通讯中断保护时间（单位：ms）
        :param motor_id: 电机ID
        :param protect_time: 保护时间（ms），4字节数据
        :return: 发送成功返回True
        """
        # 指令格式：B3 00 00 00 [时间低字节] [时间中低字节] [时间中高字节] [时间高字节]
        time_bytes = [
            protect_time & 0xFF,
            (protect_time >> 8) & 0xFF,
            (protect_time >> 16) & 0xFF,
            (protect_time >> 24) & 0xFF
        ]
        command_data = [0xB3, 0x00, 0x00, 0x00] + time_bytes
        return self.send_command(motor_id, command_data)

    def set_baudrate(self, motor_id, is_1m=True):
        """
        设置CAN波特率（断电保存）
        :param motor_id: 电机ID
        :param is_1m: True-1Mbps，False-500kbps
        :return: 发送成功返回True
        """
        baud_byte = 0x01 if is_1m else 0x00
        # 指令格式：B4 00 00 00 00 00 00 [波特率配置字节]
        command_data = [0xB4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, baud_byte]
        success = self.send_command(motor_id, command_data)
        if success:
            baud_str = "1Mbps" if is_1m else "500kbps"
            self.get_logger().info(f"电机{motor_id}波特率设置为{baud_str}，需重启生效")
        return success

    def read_can_id(self, motor_id):
        """
        读取CAN ID
        :param motor_id: 当前电机ID
        :return: 字典包含发送ID、回复ID，超时返回None
        """
        # 指令格式：79 00 01 00 00 00 00 00（Data[2]=1表示读ID）
        command_data = [0x79, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00]
        if not self.send_command(motor_id, command_data, is_global=False):
            return None
        
        start_time = time.time()
        while time.time() - start_time < 0.5:
            try:
                msg = self.bus.recv(timeout=0.1)
            except (can.CanError, OSError) as e:
                self.get_logger().error(f"CAN接收失败（电机{motor_id}）: {e}，尝试重连...")
                self.reconnect_can_bus()
                continue
            
            if msg and msg.arbitration_id == (self.reply_base_id + motor_id):
                print(msg.data)
                if len(msg.data) >= 8 and msg.data[0] == 0x79:
                    # Data[6]和Data[7]组成回复ID：0x24X → 发送ID=0x14X
                    send_id = msg.data[6] | (msg.data[7] << 8)
                    reply_id = send_id + 0x100  # 回复ID = 发送ID + 0x100
                    motor_id_new = send_id - self.send_base_id  # 提取电机ID
                    return {
                        "send_id": hex(send_id),
                        "reply_id": hex(reply_id),
                        "motor_id": motor_id_new
                    }
        
        self.get_logger().warn(f"读取电机{motor_id}CAN ID超时")
        return None

    def write_can_id(self, old_id, new_id):
        """
        写入新CAN ID
        :param old_id: 当前电机ID
        :param new_id: 新电机ID（1~32）
        :return: 发送成功返回True
        """
        if not 1 <= new_id <= 32:
            self.get_logger().error(f"新ID{new_id}超出有效范围（1-32）")
            return False
        
        # 指令格式：79 00 00 00 00 00 00 [新ID]（Data[2]=0表示写ID）
        command_data = [0x79, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, new_id]
        success = self.send_command(old_id, command_data, is_global=True)
        if success:
            new_send_id = self.send_base_id + new_id
            new_reply_id = self.reply_base_id + new_id
            self.get_logger().info(
                f"电机ID更新：旧ID={old_id} → 新ID={new_id}，"
                f"新发送ID={hex(new_send_id)}，新回复ID={hex(new_reply_id)}"
            )
        return success

    # -------------------------- 资源释放 --------------------------
    def shutdown(self):
        """关闭驱动（禁用电机+抱闸锁死+关闭CAN总线）"""
        self.get_logger().info("关闭CAN电机驱动...")
        try:
            # 禁用所有电机并锁死抱闸
            for motor_id in range(1, 33):
                self.disable_drive(motor_id)
                self.brake_lock(motor_id)
            if self.bus is not None:
                self.bus.shutdown()
        except Exception as e:
            self.get_logger().error(f"关闭驱动异常: {e}")
        self.get_logger().info("CAN电机驱动已关闭")

def main(args=None):
    rclpy.init(args=args)
    # 初始化电机驱动（can0接口，1Mbps波特率）
    motor_driver = CanMotorDriver(channel='can0', interface='socketcan', baudrate=1000000)
    
    try:
        # 示例：控制ID=1的电机
        motor_id = 1
        
        # 1. 基础控制示例
        motor_driver.get_logger().info("=== 基础控制测试 ===")
        motor_driver.enable_drive(motor_id)  # 使能电机（抱闸释放）
        time.sleep(0.5)
        # motor_driver.set_rotation_direction(motor_id, is_forward=True)  # 设置正转
        motor_driver.set_acceleration(motor_id, 5000)  # 加速度5000 dps/s
        motor_driver.set_deceleration(motor_id, 5000)  # 减速度5000 dps/s
        motor_driver.set_velocity_closed_loop(motor_id, target_speed=100, max_torque=200)  # 100 dps，最大扭矩200
        time.sleep(3)
        
        # 2. 状态读取示例
        motor_driver.get_logger().info("\n=== 状态读取测试 ===")
        state1 = motor_driver.get_motor_state1(motor_id)
        if state1:
            motor_driver.get_logger().info(
                f"状态1 - 温度: {state1['temperature']}℃, "
                f"抱闸状态: {state1['brake_status']}, "
                f"电压: {state1['voltage']}V, "
                f"错误: {state1['error_info']}"
            )
        
        state2 = motor_driver.get_motor_state2(motor_id)
        if state2:
            motor_driver.get_logger().info(
                f"状态2 - 转矩电流: {state2['torque_current']}A, "
                f"转速: {state2['speed']}dps, "
                f"角度: {state2['angle']}°"
            )
        
        state3 = motor_driver.get_motor_state3(motor_id)
        if state3:
            motor_driver.get_logger().info(
                f"状态3 - A相电流: {state3['current_a']}A, "
                f"B相电流: {state3['current_b']}A, "
                f"C相电流: {state3['current_c']}A"
            )
        
        # 3. 配置参数示例
        motor_driver.get_logger().info("\n=== 配置参数测试 ===")
        motor_driver.set_comm_protection_time(motor_id, 5000)  # 通讯中断保护时间500ms
        # can_id_info = motor_driver.read_can_id(motor_id)
        # if can_id_info:
            # motor_driver.get_logger().info(f"当前CAN ID信息: {can_id_info}")
        # 4. 停止电机
        motor_driver.stop_motor(motor_id)  # 停止电机
        time.sleep(1)
        motor_driver.disable_drive(motor_id)  # 非使能电机
        motor_driver.brake_lock(motor_id)  # 抱闸锁死
        
        rclpy.spin(motor_driver)
    except KeyboardInterrupt:
        motor_driver.get_logger().info("收到中断信号，停止电机...")
    finally:
        motor_driver.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()