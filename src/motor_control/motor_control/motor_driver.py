#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int32, UInt8
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import struct
import can
import time
import math
from typing import Optional, List, Dict
import threading


class CanMotorDriver(Node):
    def __init__(self, node_name='can_motor_driver', channel='can0', interface='socketcan', baudrate=1000000):

        super().__init__(node_name)

        
        # CAN配置
        self.can_interface = "vcan0"  # 根据jifeng系统使用can1
        self.bus: Optional[can.Bus] = None
        self.can_initialized = False
        
        # 电机配置 (基于jifeng系统中的3个电机)
        self.motors = [
            {
                "id": 1,                        # 左轮电机ID
                "velocity": 0.0,                # 目标速度（rad/s）
                "actual_velocity": 0.0,         # 实际速度
                "actual_position": 0.0,         # 实际位置
                "actual_torque": 0.0,           # 实际扭矩
                "actual_temperature": 0.0,      # 实际温度
                "error_code": 0,                # 故障码
                "current_limit": 20.0,          # 电流限制（A）
                "run_mode": 2                   # 运行模式（2速度模式）
            },
            {
                "id": 2,                        # 右轮电机ID
                "velocity": 0.0,
                "actual_velocity": 0.0,
                "actual_position": 0.0,
                "actual_torque": 0.0,
                "actual_temperature": 0.0,
                "error_code": 0,
                "current_limit": 20.0,
                "run_mode": 2
            },
            {
                "id": 3,                        # 前毛刷电机ID
                "velocity": 0.0,
                "actual_velocity": 0.0,
                "actual_position": 0.0,
                "actual_torque": 0.0,
                "actual_temperature": 0.0,
                "error_code": 0,
                "current_limit": 20.0,
                "run_mode": 2
            }
        ]
        
        # 主机ID (与jifeng系统保持一致)
        self.motor_master_id = 99  # 0x63
        
        # 电机参数索引
        self.RUN_MODE_INDEX = 0x7005    # 运行模式索引
        self.SPEED_REF_INDEX = 0x700A   # 速度指令索引
        self.LIMIT_CUR_INDEX = 0x7018   # 电流限制索引
        self.OTHER_PARAM_INDEX = 0x7022 # 其他参数索引 (针对电机3)
        
        # 通信类型
        self.COMM_WRITE_PARAM = 0x12    # 参数写入
        self.COMM_ENABLE_MOTOR = 0x03   # 使能电机
        self.COMM_DISABLE_MOTOR = 0x04  # 失能电机
        self.MC_CMD_SPEED_CLOSED_LOOP = 0xA2  # 速度闭环控制命令
        self.MC_CMD_POS_SPEED_TORQUE_FEEDBACK = 0x02  # 位置、速度、扭矩反馈命令类型
        self.MC_CMD_QUERY_MOTOR = 0x03  # 电机状态查询命令
        
        # 机器人参数
        self.wheel_radius = 0.05  # 轮子半径（米）
        self.wheel_base = 0.3     # 轮距（米）
        self.encoder_resolution = 4096  # 编码器分辨率（每转脉冲数）
        
        # 里程计参数
        self.x = 0.0  # 机器人位置x坐标
        self.y = 0.0  # 机器人位置y坐标
        self.th = 0.0  # 机器人方向角度

        self.BASE_SPEED = 100.0  # 导航目标速度（dps）= self.BASE_SPEED*100
        # Sensor 
        self.front_left = None
        self.front_right = None
        self.mid_left = None
        self.mid_right = None
        self.back_left = None
        self.back_right = None

        
        # 上次时间戳
        self.last_time = self.get_clock().now()
        
        # 初始化CAN总线
        if not self.create_can_bus():
            self.get_logger().warn("Failed to initialize CAN bus, will retry periodically")
            
        # 初始化电机
        self.initialize_motors()
        
        # 创建订阅者，用于接收速度命令
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'motor_speed_commands',
            self.speed_command_callback,
            10)
        # self.io_subscription = self.create_subscription(
        #     UInt8,
        #     "io_data",
        #     self.io_data_callback,
        #     10
        # )
            
        # 创建发布者，用于发布电机速度
        self.velocity_publisher = self.create_publisher(
            Float32MultiArray, 
            'motor_velocities', 
            10)
            
        # 创建发布者，用于发布电机状态（位置、速度、扭矩等）
        self.motor_feedback_publisher = self.create_publisher(
            Float32MultiArray,
            'motor_feedback',
            10)
            
        # 创建发布者，用于发布电机故障信息
        self.error_publisher = self.create_publisher(
            Int32MultiArray,
            'motor_errors',
            10)
            
        # 创建发布者，用于发布整体故障状态
        self.system_error_publisher = self.create_publisher(
            Int32,
            'system_error',
            10)
            
        # 创建发布者，用于发布里程信息
        self.odom_publisher = self.create_publisher(
            Odometry,
            'odom',
            10)
        
        
            
        # 定时器，定期发送速度命令和发布电机状态
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz
        
        # 启动接收线程
        self.receive_thread = None
        self.running = True
        self.start_receive_thread()
        
        self.get_logger().info('Motor Control Node has been started')

    def create_can_bus(self) -> bool:
        """初始化CAN总线"""
        try:
            self.bus = can.Bus(interface='socketcan', channel=self.can_interface, bitrate=1000000)
            self.can_initialized = True
            self.get_logger().info(f'CAN bus {self.can_interface} initialized successfully')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to initialize CAN bus: {e}')
            self.can_initialized = False
            return False

    def reconnect_can_bus(self):
        """重试CAN总线初始化"""
        if not self.can_initialized:
            self.get_logger().info("Retrying CAN bus initialization...")
            self.create_can_bus()

    def send_can_frame(self, can_id: int, data: bytes) -> bool:
        """发送CAN帧"""
        if not self.bus:
            self.get_logger().error("CAN bus not initialized")
            return False
            
        try:
            # 确保数据长度为8字节
            if len(data) < 8:
                data = data.ljust(8, b'\x00')
            elif len(data) > 8:
                data = data[:8]
                
            msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=True)
            self.bus.send(msg)
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to send CAN frame: {e}")
            self.can_initialized = False
            return False

    def motor_set_mode(self, motor_id: int, mode: int) -> bool:
        """设置电机模式"""
        # 构造CAN数据段（8字节）：0x7005索引 + 模式值
        can_data = bytearray(8)
        struct.pack_into("<H", can_data, 0, self.RUN_MODE_INDEX)  # 0x7005（小端）
        struct.pack_into("<B", can_data, 4, mode)                # 模式值存Byte4
        # 构造29位CAN ID（通信类型=0x12=参数写入，主机ID=0x0063，电机ID=motor_id）
        can_id = (self.COMM_WRITE_PARAM << 24) | (self.motor_master_id << 8) | motor_id
        # 发送CAN帧
        return self.send_can_frame(can_id, can_data)

    def motor_set_current_limit(self, motor_id: int, current_limit: float) -> bool:
        """设置电机电流限制"""
        can_data = bytearray(8)
        struct.pack_into("<H", can_data, 0, self.LIMIT_CUR_INDEX)  # 0x7018（小端）
        struct.pack_into("<f", can_data, 4, current_limit)         # 电流值（float）
        can_id = (self.COMM_WRITE_PARAM << 24) | (self.motor_master_id << 8) | motor_id
        return self.send_can_frame(can_id, can_data)

    def motor_set_other_param(self, motor_id: int, param_value: float) -> bool:
        """设置电机其他参数"""
        can_data = bytearray(8)
        struct.pack_into("<H", can_data, 0, self.OTHER_PARAM_INDEX)  # 0x7022（小端）
        struct.pack_into("<f", can_data, 4, param_value)            # 参数值（float）
        can_id = (self.COMM_WRITE_PARAM << 24) | (self.motor_master_id << 8) | motor_id
        return self.send_can_frame(can_id, can_data)

    def motor_set_speed(self, motor_id: int, speed: float) -> bool:
        """设置电机速度"""
        can_data = bytearray(8)
        struct.pack_into("<H", can_data, 0, self.SPEED_REF_INDEX)  # 0x700A（小端）
        struct.pack_into("<f", can_data, 4, speed)                 # 速度值（float）
        can_id = (self.COMM_WRITE_PARAM << 24) | (self.motor_master_id << 8) | motor_id
        return self.send_can_frame(can_id, can_data)

    def motor_query_feedback(self, motor_id: int) -> bool:
        """主动查询电机反馈数据"""
        can_data = bytearray(8)
        can_data[0] = self.MC_CMD_QUERY_MOTOR  # 查询命令
        can_data[1] = 0x00  # 命令子类型，0表示查询位置、速度、扭矩
        # 构造29位CAN ID（通信类型=0x03=查询命令，主机ID=0x0063，电机ID=motor_id）
        can_id = (self.MC_CMD_QUERY_MOTOR << 24) | (self.motor_master_id << 8) | motor_id
        return self.send_can_frame(can_id, can_data)

    def motor_enable(self, motor_id: int) -> bool:
        """使能电机"""
        can_data = b'\x00' * 8  # 使能指令数据段全零
        can_id = (self.COMM_ENABLE_MOTOR << 24) | (self.motor_master_id << 8) | motor_id
        return self.send_can_frame(can_id, can_data)
        
    def motor_disable(self, motor_id: int) -> bool:
        """停止单个电机"""
        can_data = b'\x80'+b'\x00' * 7
        can_id = (self.COMM_DISABLE_MOTOR << 24) | (self.motor_master_id << 8) | motor_id
        return self.send_can_frame(can_id, can_data)

    def initialize_motors(self):
        """初始化所有电机"""
        self.get_logger().info("Initializing motors...")
        time.sleep(3.0)  # 等待CAN接口就绪，与jifeng系统保持一致
        
        # 初始化左轮电机 (ID=1)
        self.get_logger().info("Initializing left wheel motor (ID=1)...")
        self.motor_set_mode(1, 2)  # 设置速度模式
        time.sleep(0.01)
        self.motor_enable(1)  # 使能电机
        time.sleep(0.01)
        self.motor_set_current_limit(1, 20.0)  # 设置电流限制
        time.sleep(0.01)

        # 初始化右轮电机 (ID=2)
        self.get_logger().info("Initializing right wheel motor (ID=2)...")
        self.motor_set_mode(2, 2)  # 设置速度模式
        time.sleep(0.01)
        self.motor_enable(2)  # 使能电机
        time.sleep(0.01)
        self.motor_set_current_limit(2, 20.0)  # 设置电流限制
        time.sleep(0.01)

        # 初始化前毛刷电机 (ID=3)
        self.get_logger().info("Initializing front brush motor (ID=3)...")
        self.motor_set_mode(3, 2)  # 设置速度模式
        time.sleep(0.01)
        self.motor_set_other_param(3, 15.0)  # 设置特定参数
        time.sleep(0.01)
        self.motor_enable(3)  # 使能电机
        time.sleep(0.01)
        self.motor_set_current_limit(3, 20.0)  # 设置电流限制
        time.sleep(0.01)

    def speed_command_callback(self, msg: Float32MultiArray):
        """处理速度命令回调函数"""
        if len(msg.data) != 3:  # 3个电机的速度命令
            self.get_logger().warn(f"Received speed command with incorrect length: {len(msg.data)}, expected: 3")
            return
            
        # 更新电机速度目标值
        for i in range(min(len(self.motors), len(msg.data))):
            self.motors[i]["velocity"] = float(msg.data[i])
        
        self.get_logger().debug(f"Updated motor velocity targets: {[m['velocity'] for m in self.motors]}")

    # def io_data_callback(self, msg: UInt8):
    #     """处理IO数据回调（可根据需要扩展功能）"""

    #     # 位0 (1<<0 = 0x01)：前左
    #     self.front_left = (msg.data & 0x01) == 0x01
        
    #     # 位1 (1<<1 = 0x02)：前右
    #     self.front_right = (msg.data & 0x02) == 0x02
        
    #     # 位2 (1<<2 = 0x04)：中左
    #     self.mid_left = (msg.data & 0x04) == 0x04
        
    #     # 位3 (1<<3 = 0x08)：中右  8
    #     self.mid_right = (msg.data & 0x08) == 0x08
        
    #     # 位4 (1<<4 = 0x10)：后左 16
    #     self.back_left = (msg.data & 0x10) == 0x10
        
    #     # 位5 (1<<5 = 0x20)：后右  32
    #     self.back_right = (msg.data & 0x20) == 0x20
    #     if msg.data is not 0:
    #         self.get_logger().info(f"--------------Boundary Detected--------------")
    #         self.motor_set_speed(1, 0)
    #         self.motor_set_speed(2, 0) 
            # for m in self.motors:
                # self.motor_set_speed(m["id"], 0)
        # if self.front_left or self.front_right:
        #     self.motor_set_speed(1, -0.3 * self.BASE_SPEED)
        #     self.motor_set_speed(2, 0.3 * self.BASE_SPEED)
        #     if self.mid_left or self.mid_right:
        #         self.motor_set_speed(1, 0)
        #         self.motor_set_speed(2, 0)
        #         while self.front_left or self.front_right and rclpy.ok():
        #             self.motor_set_speed(1, 0.3 * self.BASE_SPEED)
        #             self.motor_set_speed(2, -0.3 * self.BASE_SPEED)
        # if self.back_left or self.back_right:
        #     self.motor_set_speed(1, 0)
        #     self.motor_set_speed(2, 0)
        #     while self.front_left or self.front_right and rclpy.ok():
        #         self.motor_set_speed(1, 0.3 * self.BASE_SPEED)
        #         self.motor_set_speed(2, -0.3 * self.BASE_SPEED)

        # self.get_logger().info(f"IO状态: {msg.data}, front_left={self.front_left}, front_right={self.front_right}, "
        # f"mid_left={self.mid_left}, mid_right={self.mid_right}, "
        # f"back_left={self.back_left}, back_right={self.back_right}"
        # )

    def send_speed_commands(self):
        """发送速度命令给所有电机"""
        current_time = time.time()
        for motor in self.motors:
            result = self.motor_set_speed(motor["id"], motor["velocity"])
            if not result:
                self.get_logger().error(f"Failed to set motor {motor['id']} speed")
                # 记录错误
                motor["error_code"] = 1

    def query_motor_feedback(self):
        """主动查询所有电机的反馈数据"""
        for motor in self.motors:
            result = self.motor_query_feedback(motor["id"])
            if not result:
                self.get_logger().error(f"Failed to query motor {motor['id']} feedback")
                # 记录错误
                motor["error_code"] = 3  # 查询错误

    def parse_motor_feedback(self, can_id: int, data: bytearray):
        """解析电机反馈数据（基于图片中定义的协议）"""
        # 根据图片中的29位ID结构，从bit8~bit15提取电机ID
        # Bit8~Bit15: 当前电机CAN ID
        motor_id = (can_id >> 8) & 0xFF
        
        # 查找对应的电机
        motor = None
        for m in self.motors:
            if m["id"] == motor_id:
                motor = m
                break
                
        if motor is None:
            return
            
        try:
            # 解析数据区（Byte0~Byte7）
            # Byte0~1: 当前角度 [0~65535] 对应 (-12.57f~12.57f)
            position_raw = (data[0] << 8) | data[1]  # 低字节在前
            
            # Byte2~3: 当前角速度 [0~65535] 对应 (-20rad/s~20rad/s)
            speed_raw = (data[2] << 8) | data[3]  # 低字节在前
            
            # Byte4~5: 当前力矩 [0~65535] 对应 (-60Nm~60Nm)
            torque_raw = (data[4] << 8) | data[5]  # 低字节在前
            
            # Byte6~7: 当前温度：Temp(摄氏度)*10
            temp_raw = (data[6] << 8) | data[7]  # 低字节在前
            temp = temp_raw / 10.0  # 转换为实际温度值
            
            # 提取故障信息和模式状态
            # Bit21~16: 故障信息
            fault_info = (can_id >> 16) & 0x3F
            # Bit22~23: 模式状态
            mode_status = (can_id >> 22) & 0x03
            
            # 根据电机ID使用不同的转换范围
            if motor_id in [1, 2]:  # 左右轮电机
                # 位置范围: -12.57 ~ 12.57
                position = self.uint16_to_float(position_raw, -12.57, 12.57, 16)
                # 速度范围: -20 ~ 20 rad/s
                speed = self.uint16_to_float(speed_raw, -20.0, 20.0, 16)
                # 扭矩范围: -60 ~ 60 Nm
                torque = self.uint16_to_float(torque_raw, -60.0, 60.0, 16)
            else:  # 前毛刷电机 (ID=3)
                # 位置范围: -4π ~ 4π
                position = self.uint16_to_float(position_raw, -4 * math.pi, 4 * math.pi, 16)
                # 速度范围: -44 ~ 44 rad/s
                speed = self.uint16_to_float(speed_raw, -44.0, 44.0, 16)
                # 扭矩范围: -17 ~ 17 Nm
                torque = self.uint16_to_float(torque_raw, -17.0, 17.0, 16)
                
            # 更新电机实际参数
            motor["actual_position"] = position
            motor["actual_velocity"] = speed
            motor["actual_torque"] = torque
            motor["actual_temperature"] = temp
            
            # 设置错误码和模式状态
            motor["error_code"] = fault_info
            motor["run_mode"] = mode_status
            
        except Exception as e:
            self.get_logger().warn(f"Error parsing motor {motor_id} feedback: {str(e)}")
            motor["error_code"] = 2  # 解析错误

    def uint16_to_float(self, x, x_min, x_max, bits):
        """将16位整数转换为浮点数"""
        span = (1 << bits) - 1
        offset = x_max - x_min
        return offset * x / span + x_min

    def update_odometry(self):
        """更新里程信息"""
        current_time = self.get_clock().now()
        dt = (current_time.nanoseconds - self.last_time.nanoseconds) / 1e9
        self.last_time = current_time
        
        if dt <= 0:
            return

        # 获取左右轮的实际速度（rad/s）
        left_vel = self.motors[0]["actual_velocity"]  # 左轮电机在索引0
        right_vel = self.motors[1]["actual_velocity"]  # 右轮电机在索引1

        # 将角速度转换为线速度（m/s）
        left_vel_linear = left_vel * self.wheel_radius
        right_vel_linear = right_vel * self.wheel_radius

        # 计算机器人线速度和角速度
        linear_velocity = (right_vel_linear + left_vel_linear) / 2.0
        angular_velocity = (right_vel_linear - left_vel_linear) / self.wheel_base

        # 计算位移和角度变化
        delta_x = linear_velocity * dt * 10  # 增加一个缩放因子来调整里程
        delta_y = 0.0
        delta_th = angular_velocity * dt

        # 更新机器人位置（基于差动驱动模型）
        self.x += delta_x * math.cos(self.th) - delta_y * math.sin(self.th)
        self.y += delta_x * math.sin(self.th) + delta_y * math.cos(self.th)
        self.th += delta_th

        # 发布里程计消息
        self.publish_odometry(linear_velocity, angular_velocity)

    def publish_odometry(self, linear_velocity, angular_velocity):
        """发布里程计消息"""
        current_time = self.get_clock().now()
        
        # 创建Odometry消息
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # 设置位置
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = self.quaternion_from_euler(0, 0, self.th)

        # 设置速度
        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = angular_velocity

        # 设置协方差矩阵（暂时设置为0，可以根据实际传感器精度调整）
        odom.pose.covariance = [0.0] * 36
        odom.twist.covariance = [0.0] * 36

        # 发布里程计消息
        self.odom_publisher.publish(odom)

    def quaternion_from_euler(self, roll, pitch, yaw):
        """从欧拉角创建四元数"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        
        return q

    def receive_can_frames(self):
        """接收CAN帧的线程函数"""
        self.get_logger().info("Starting CAN frame receiving thread...")
        while self.running:
            try:
                # 如果CAN未初始化，尝试重新初始化
                if not self.can_initialized:
                    self.reconnect_can_bus()
                    time.sleep(1.0)  # 等待一段时间再重试
                    continue
                    
                # 使用100ms超时接收CAN帧
                if self.bus:
                    msg = self.bus.recv(timeout=0.1)
                    if msg is not None:
                        self.parse_motor_feedback(msg.arbitration_id, msg.data)
                else:
                    time.sleep(0.1)  # 如果没有CAN总线，短暂休眠
            except can.CanError as e:
                self.get_logger().error(f"CAN error while receiving frames: {str(e)}")
                self.can_initialized = False
                time.sleep(1.0)  # 出现错误后等待更长时间
            except Exception as e:
                if self.running:  # 只在运行时记录错误
                    self.get_logger().error(f"Error receiving CAN frames: {str(e)}")
                time.sleep(1.0)  # 出现错误后等待更长时间
        self.get_logger().info("Stopped CAN frame receiving thread")

    def start_receive_thread(self):
        """启动接收线程"""
        self.receive_thread = threading.Thread(target=self.receive_can_frames, daemon=True)
        self.receive_thread.start()

    def timer_callback(self):
        """定时器回调函数，发送速度命令并发布电机状态"""
        # 发送速度命令给所有电机
        self.send_speed_commands()
        
        # # 主动查询电机反馈数据
        # self.query_motor_feedback()
        
        # 更新里程信息
        self.update_odometry()
        
        # 发布电机速度
        velocity_msg = Float32MultiArray()
        velocity_msg.data = [float(m["actual_velocity"]) for m in self.motors]
        self.velocity_publisher.publish(velocity_msg)
        
        # 发布电机反馈信息（位置、速度、扭矩、温度）- 按电机ID分组
        feedback_msg = Float32MultiArray()
        # 每个电机的数据按顺序：[电机ID, 位置, 速度, 扭矩, 温度]
        feedback_data = []
        for motor in self.motors:
            feedback_data.extend([
                float(motor["id"]),               # 电机ID
                motor["actual_position"],         # 位置
                motor["actual_velocity"],         # 速度
                motor["actual_torque"],           # 扭矩
                motor["actual_temperature"]       # 温度
            ])
        feedback_msg.data = feedback_data
        self.motor_feedback_publisher.publish(feedback_msg)
        
        # 发布电机错误信息
        error_msg = Int32MultiArray()
        error_msg.data = [int(m["error_code"]) for m in self.motors]
        self.error_publisher.publish(error_msg)
        
        # 发布系统整体错误状态（如果有任何电机出错）
        system_error = 0
        for motor in self.motors:
            if motor["error_code"] != 0:
                system_error = 1
                break
                
        system_error_msg = Int32()
        system_error_msg.data = system_error
        self.system_error_publisher.publish(system_error_msg)

    def destroy_node(self):
        """节点销毁时停止所有电机"""
        self.get_logger().info("Stopping all motors...")
        self.running = False  # 停止接收线程
        
        # 发送速度为0的命令
        for motor in self.motors:
            motor["velocity"] = 0.0
        self.send_speed_commands()
        time.sleep(0.01)  # 短暂延迟确保命令发送
            
        # 发送关闭命令
        for motor in self.motors:
            self.motor_disable(motor["id"])
            time.sleep(0.01)  # 短暂延迟确保命令发送
            
        # 等待接收线程结束
        if self.receive_thread:
            self.receive_thread.join(timeout=1.0)
            
        # 关闭CAN总线
        if self.bus:
            self.bus.shutdown()
            self.get_logger().info("CAN bus shutdown")
            
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        motor_driver = CanMotorDriver()
        rclpy.spin(motor_driver)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Unexpected error: {e}")
    finally:
        # Check if rclpy is still OK before shutdown
        if rclpy.ok():
            if 'motor_driver' in locals():
                motor_driver.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()