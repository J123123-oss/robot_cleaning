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
        self.can_interface = channel  # can0
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
                "current_limit": 20.0,          # 电流限制（A）
                "run_mode": 2,                  # 运行模式（2速度模式）
                "fault_code": 0                 # 故障码
            },
            {
                "id": 2,                        # 右轮电机ID
                "velocity": 0.0,
                "actual_velocity": 0.0,
                "actual_position": 0.0,
                "actual_torque": 0.0,
                "actual_temperature": 0.0,
                "current_limit": 20.0,
                "run_mode": 2,
                "fault_code": 0
            },
            {
                "id": 3,                        # 前毛刷电机ID
                "velocity": 0.0,
                "actual_velocity": 0.0,
                "actual_position": 0.0,
                "actual_torque": 0.0,
                "actual_temperature": 0.0,
                "current_limit": 20.0,
                "run_mode": 2,
                "fault_code": 0
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
        self.COMM_DISABLE_MOTOR = 0x04  # 失能 / 清除故障
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

        self.BASE_SPEED = 2.0 # 导航目标速度（dps）= self.BASE_SPEED*10
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
        # self.initialize_motors()
        
        # 创建订阅者，用于接收速度命令
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'motor_speed_commands',
            self.speed_command_callback,
            10)
            
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
            
        # 创建发布者，用于发布三路电机故障码
        self.motor_fault_publisher = self.create_publisher(
            Float32MultiArray,
            'motor_fault_codes',
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
            time.sleep(0.01) 
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to send CAN frame: {e}")
            self.can_initialized = False
            return False

    # -------------------------------------------------------------------------
    # 【新增】清除电机故障（官方指令：04 + 数据01）
    # -------------------------------------------------------------------------
    def motor_clear_fault(self, motor_id: int) -> bool:
        can_id = (0x04 << 24) | (self.motor_master_id << 8) | motor_id
        data = b'\x01\x00\x00\x00\x00\x00\x00\x00'
        ret = self.send_can_frame(can_id, data)
        if ret:
            self.get_logger().info(f"✅ 电机{motor_id} 清除故障指令已发送")
        return ret

    # -------------------------------------------------------------------------
    # 【新增】解析故障帧 0x15006301 ~ 0x15006303
    # -------------------------------------------------------------------------
    def parse_motor_fault(self, can_id: int, data: bytes):
        motor_id = can_id & 0xFF
        fault = data[0]

        for motor in self.motors:
            if motor["id"] == motor_id:
                motor["fault_code"] = fault
                break

        self.get_logger().error("======================================")
        self.get_logger().error(f"电机 {motor_id} 故障帧：0x{can_id:08X}")
        self.get_logger().error(f"故障码：0x{fault:02X}")

        if fault == 0x00:
            self.get_logger().info("✅ 无故障")
        else:
            if fault & (1 << 3):
                self.get_logger().error("🚨 过压故障")
            if fault & (1 << 2):
                self.get_logger().error("🚨 欠压故障")
            if fault & (1 << 1):
                self.get_logger().error("🚨 驱动芯片故障")
            if fault & (1 << 0):
                self.get_logger().error("🚨 电机过温故障")
            if fault & (1 << 7):
                self.get_logger().error("🚨 编码器未标定")
            if fault & (1 << 14):
                self.get_logger().error("🚨 堵转/过载故障")
        self.get_logger().error("======================================")

        self.publish_motor_fault_codes()

    def publish_motor_fault_codes(self):
        fault_codes = [float(m["fault_code"]) for m in self.motors]
        fault_msg = Float32MultiArray()
        fault_msg.data = fault_codes
        self.motor_fault_publisher.publish(fault_msg)

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

    def send_speed_commands(self):
        """发送速度命令给所有电机"""
        current_time = time.time()
        for motor in self.motors:
            result = self.motor_set_speed(motor["id"], motor["velocity"])
            if not result:
                self.get_logger().error(f"Failed to set motor {motor['id']} speed")

    def query_motor_feedback(self):
        """主动查询所有电机的反馈数据"""
        for motor in self.motors:
            result = self.motor_query_feedback(motor["id"])
            if not result:
                self.get_logger().error(f"Failed to query motor {motor['id']} feedback")

    def parse_motor_feedback(self, can_id: int, data: bytearray):
        """解析电机反馈数据（RS02协议 type2）"""
        motor_id = (can_id >> 8) & 0xFF

        motor = None
        for m in self.motors:
            if m["id"] == motor_id:
                motor = m
                break

        if motor is None:
            return

        try:
            # Byte0~1: 当前角度 [0~65535] → -12.57~12.57 rad
            position_raw = (data[0] << 8) | data[1]

            # Byte2~3: 当前角速度 [0~65535] → -44~44 rad/s
            speed_raw = (data[2] << 8) | data[3]

            # Byte4~5: 当前力矩 [0~65535] → -17~17 Nm
            torque_raw = (data[4] << 8) | data[5]

            # Byte6~7: 当前温度 = raw / 10 (摄氏度)
            temp_raw = (data[6] << 8) | data[7]
            temp = temp_raw / 10.0

            position = self.uint16_to_float(position_raw, -12.57, 12.57, 16)
            speed = self.uint16_to_float(speed_raw, -44.0, 44.0, 16)
            torque = self.uint16_to_float(torque_raw, -17.0, 17.0, 16)

            motor["actual_position"] = position
            motor["actual_velocity"] = speed
            motor["actual_torque"] = torque
            motor["actual_temperature"] = temp

        except Exception as e:
            self.get_logger().warn(f"Error parsing motor {motor_id} feedback: {str(e)}")

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
                if not self.can_initialized:
                    self.reconnect_can_bus()
                    time.sleep(1.0)
                    continue
                    
                if self.bus:
                    msg = self.bus.recv(timeout=0.1)
                    if msg is not None:
                        can_id = msg.arbitration_id
                        cmd_type = (can_id >> 24) & 0xFF
                        
                        # 故障帧 0x15
                        if cmd_type == 0x15:
                            self.parse_motor_fault(can_id, msg.data)
                        elif cmd_type == 0x2:
                            self.parse_motor_feedback(can_id, msg.data)
                else:
                    time.sleep(0.1)
            except Exception as e:
                time.sleep(1.0)
        self.get_logger().info("Stopped CAN frame receiving thread")

    def start_receive_thread(self):
        """启动接收线程"""
        self.receive_thread = threading.Thread(target=self.receive_can_frames, daemon=True)
        self.receive_thread.start()

    def timer_callback(self):
        """定时器回调函数，发送速度命令并发布电机状态"""
        # 发送速度命令给所有电机
        self.send_speed_commands()
        
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

    def destroy_node(self):
        """节点销毁时停止所有电机"""
        self.get_logger().info("Stopping all motors...")
        self.running = False  # 停止接收线程
        
        # 发送速度为0的命令
        for motor in self.motors:
            motor["velocity"] = 0.0
        self.send_speed_commands()
        time.sleep(0.01)
            
        # 发送关闭命令
        for motor in self.motors:
            self.motor_disable(motor["id"])
            time.sleep(0.01)
            
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
        if rclpy.ok():
            if 'motor_driver' in locals():
                motor_driver.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()