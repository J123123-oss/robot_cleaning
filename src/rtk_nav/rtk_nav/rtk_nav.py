#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import math
from typing import Optional, List, Dict, Tuple, Generator
import rclpy
import re
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3  # 用于发布左右轮速度
from std_msgs.msg import String       # 用于发布控制模式和导航状态
from custom_msgs.msg import WTRTK

# -------------------------- 全局配置与枚举 --------------------------
# RTK导航配置
RTK_WAYPOINT_TOLERANCE = 0.2
RTK_HEADING_TOLERANCE = 0.5
LINEAR_SPEED_BASE = 100.0       # origin 0.0124
TURN_SPEED = 50.0      # origin 0.1
INITIAL_MOVE_TOLERANCE = 0.5
IMU_CALIBRATION_TIMEOUT = 3.0
HEADING_CALIBRATION_TIMEOUT = 5.0

# 控制模式（与电机节点保持一致）
class ControlMode:
    REMOTE = "REMOTE"
    NORMAL = "NORMAL"
    RTK_NAV = "RTK_NAV"

# 导航状态枚举
class NavState:
    IDLE = "IDLE"
    INITIAL_MOVE = "INITIAL_MOVE"
    WAYPOINT_MOVE = "WAYPOINT_MOVE"
    WAYPOINT_CALIB = "WAYPOINT_CALIB"
    COMPLETED = "COMPLETED"
# -------------------------- 合并后的RTK控制+导航节点 --------------------------
class RTKNavControlNode(Node):
    def __init__(self):
        super().__init__('rtk_nav_control_node')

        self.process_percent = 0.0  # 路径文件处理进度百分比
        # ================== 原有RTKNavigator属性 ==================
        self.waypoints: List[Tuple[float, float, float]] = []
        self.current_waypoint_idx = 0
        self.current_gps: Optional[Tuple[float, float]] = None
        self.imu_yaw = 0.0
        self.imu_initialized = False
        self.imu_calibration_offset = 0.0
        self.current_control_mode = ControlMode.NORMAL

        # 声明RTK路径参数
        self.rtk_path_file = self.declare_parameter(
            'rtk_path_file',
            # "/home/forlinx/robot_cleaning/src/rtk_nav/rtk_nav/cleaning_path/cleaning_path_20251121_173149.txt"
            "/home/ubuntu/robot_cleaning/src/rtk_nav/rtk_nav/cleaning_path/cleaning_path_20251121_173149.txt"
        ).value
        
        self.path_dir = os.path.dirname(self.rtk_path_file)  # 获取路径文件所在目录


        # 导航上下文
        self.nav_context = {
            "nav_state": NavState.IDLE,
            "target_waypoint": None,
            "calib_generator": None,
            "last_distance": 0.0,
            "last_target_heading": 0.0
        }

        # ================== 原有RTKControlNode属性 ==================
        self.rate = self.create_rate(4)
        self.nav_generator: Optional[Generator] = None
        self.nav_running = False
        self.multi_waypoint_generator = None  # 多点导航生成器

        # ROS2发布器/订阅器
        self.motor_speed_pub = self.create_publisher(Vector3, "/rtk/motor_speed", 10)
        self.nav_state_pub = self.create_publisher(String, "/rtk/nav_state", 10)
        self.control_mode_sub = self.create_subscription(String, "/control/mode", self.mode_callback, 10)
        self.gps_sub = self.create_subscription(NavSatFix, '/fix', self.gps_callback, 10)
        self.heading_sub = self.create_subscription(WTRTK, '/wtrtk_data', self.heading_callback, 10)

        # 定时器（10Hz驱动导航逻辑）
        self.rtk_nav_timer = self.create_timer(0.1, self.rtk_timer_callback)

        # 加载航点
        self.load_rtk_path()
        # 加载初始路径文件
        self.load_waypoints_from_file(self.rtk_path_file)
    
    def load_waypoints_from_file(self, file_path: str) -> bool:
        """从文件加载航点数据"""
        try:
            self.waypoints = []
            self.current_waypoint_idx = 0
              
            with open(file_path, 'r', encoding='utf-8') as f:
                lines = f.readlines()[1:]  # 跳过表头
                for line in lines:
                    line = line.strip()
                    if not line or line.startswith('#'):
                        continue
                    seq, lon, lat, heading_deg = line.split(',')
                    self.waypoints.append((float(lon), float(lat), float(heading_deg)))
            
            self.get_logger().info(f"[RTKNav] 成功加载路径文件: {file_path}, 共 {len(self.waypoints)} 个航点")
            self.rtk_path_file = file_path  # 更新当前路径文件
            return True
            
        except Exception as e:
            self.get_logger().error(f"[RTKNav] 加载路径文件失败: {e}")
            return False
    
    def get_next_path_file(self) -> Optional[str]:
        """获取下一个路径文件（按文件名时间戳排序）"""
        try:
            # 1. 获取目录下所有符合命名规则的路径文件
            file_pattern = re.compile(r'.*_\d{8}_\d{6}\.txt')
            all_files = [f for f in os.listdir(self.path_dir) if file_pattern.match(f)]
            
            if not all_files:
                self.get_logger().warn("[RTKNav] 路径目录下未找到符合规则的路径文件")
                return None
            
            # 2. 按文件名中的时间戳排序（提取YYYYMMDD_HHMMSS部分）
            def extract_timestamp(filename: str) -> str:
                match = re.search(r'(\d{8}_\d{6})', filename)
                return match.group(1) if match else ''
            
            all_files.sort(key=extract_timestamp)
            total_files = len(all_files)  # 总文件数
            current_file = os.path.basename(self.rtk_path_file)
            
            # 3. 找到当前文件的索引
            try:
                current_idx = all_files.index(current_file)
            except ValueError:
                self.get_logger().warn(f"[RTKNav] 当前文件 {current_file} 不在路径目录中，使用第一个文件")
                # 首次使用第一个文件，进度 1/总数量
                progress_num = 1
                progress_percent = round((progress_num / total_files) * 100, 1)
                self.get_logger().info(f"[RTKNav] 路径文件进度：{progress_num}/{total_files}，{progress_percent}%")
                return os.path.join(self.path_dir, all_files[0])
            
            # 4. 计算并输出进度（当前文件索引+1 为已执行/待执行的序号）
            current_progress = current_idx + 1
            progress_percent = round((current_progress / total_files) * 100, 1)
            # 更新进度百分比
            self.process_percent = progress_percent
            self.get_logger().info(f"[RTKNav] 路径文件进度：{current_progress}/{total_files}，{progress_percent}%")
            
            # 5. 最后一个文件时结束循环（不再返回新文件）
            if current_idx >= total_files - 1:
                self.get_logger().info("[RTKNav] 已执行到最后一个路径文件（{current_file}），执行返回")
                return None
            
            # 6. 获取下一个文件（非最后一个时）
            next_idx = current_idx + 1
            next_file = all_files[next_idx]
            self.get_logger().info(f"[RTKNav] 准备切换到下一个路径文件：{next_file}")
            
            return os.path.join(self.path_dir, next_file)
            
        except Exception as e:
            self.get_logger().error(f"[RTKNav] 获取下一个路径文件失败: {e}")
            return None

    # ================== 原有RTKNavigator方法 ==================
    def load_rtk_path(self) -> bool:
        if not os.path.exists(self.rtk_path_file):
            self.get_logger().error(f"RTK路径文件不存在：{self.rtk_path_file}")
            return False

        try:
            with open(self.rtk_path_file, 'r', encoding='utf-8') as f:
                lines = f.readlines()[1:]  # 跳过表头
                for line in lines:
                    line = line.strip()
                    if not line:
                        continue
                    seq, lon, lat, heading_deg = line.split(',')
                    self.waypoints.append((float(lon), float(lat), float(heading_deg)))
            self.get_logger().info(f"成功加载RTK航点{len(self.waypoints)}个")
            return True
        except Exception as e:
            self.get_logger().error(f"解析RTK文件失败：{str(e)}")
            return False

    def gps_callback(self, msg: NavSatFix) -> None:
        if msg.status.status < 0:
            self.get_logger().warn("GPS信号无效")
            return

        status_map = {1: "弱", 2: "差分", 3: "RTK", 4: "RTK固定解"}
        if msg.status.status in status_map:
            self.get_logger().debug(f"GPS状态：{status_map[msg.status.status]}")

        self.current_gps = (msg.longitude, msg.latitude)

    def heading_callback(self, msg: WTRTK) -> None:
        ins_heading_deg = msg.ins_heading
        ins_heading_rad = math.radians(ins_heading_deg)
        ins_heading_rad = math.fmod(ins_heading_rad + math.pi, 2 * math.pi) - math.pi
        self.get_logger().info(f"self.imu_initialized: {self.imu_initialized}")
        if not self.imu_initialized:
            self.imu_calibration_offset = -ins_heading_rad
            self.imu_initialized = True
            self.get_logger().info(
                f"IMU校准完成！初始偏移：{math.degrees(self.imu_calibration_offset):.2f}°")

        self.imu_yaw = ins_heading_rad + self.imu_calibration_offset
        self.imu_yaw = math.fmod(self.imu_yaw + math.pi, 2 * math.pi) - math.pi

    def get_target_waypoint(self, current_waypoint_idx: int = None) -> Optional[Tuple[float, float, float]]:
        """获取当前目标航点（含航向角），到达最后一个航点时自动切换路径文件"""
        idx = current_waypoint_idx if current_waypoint_idx is not None else self.current_waypoint_idx
        
        # 检查是否到达最后一个航点
        if idx >= len(self.waypoints):
            self.get_logger().info("[RTKNav] 已到达当前路径文件的最后一个航点，准备切换路径文件")
            
            # 获取下一个路径文件
            next_file = self.get_next_path_file()
            if next_file and self.load_waypoints_from_file(next_file):
                # 切换文件成功，返回新文件的第一个航点
                self.current_waypoint_idx = 0
                return self.waypoints[0]
            else:
                # 没有下一个文件，返回None表示结束
                self.get_logger().info("[RTKNav] 没有更多路径文件，导航结束")
                self.current_control_mode = ControlMode.NORMAL
                return None
        
        # 返回当前目标航点
        return self.waypoints[idx]

    def calc_distance_to_waypoint(self, waypoint: Tuple[float, float, float]) -> float:
        if not self.current_gps:
            return float('inf')

        R = 6371000.0  # 地球半径（米）
        lon1, lat1 = self.current_gps
        lon2, lat2, _ = waypoint

        # 转换为弧度
        lon1_rad = math.radians(lon1)
        lat1_rad = math.radians(lat1)
        lon2_rad = math.radians(lon2)
        lat2_rad = math.radians(lat2)

        # Haversine公式计算距离
        delta_lon = lon2_rad - lon1_rad
        delta_lat = lat2_rad - lat1_rad

        a = math.sin(delta_lat / 2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(delta_lon / 2)** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        return R * c

    def get_path_heading(self, waypoint: Tuple[float, float, float]) -> float:
        """获取目标航点的路径航向角（转换为rad并归一化，与IMU基准一致）"""
        # 修正：航点航向角是绝对角度，需叠加IMU校准偏移（让路径航向与IMU基准对齐）
        heading_deg = waypoint[2] + math.degrees(self.imu_calibration_offset)
        heading_rad = math.radians(heading_deg)
        return math.fmod(heading_rad + math.pi, 2 * math.pi) - math.pi

    def get_heading_error(self, target_heading: float) -> float:
        """新增：计算当前航向与目标航向的误差（归一化到[-π, π]，单位：rad）"""
        heading_error = target_heading - self.imu_yaw
        return math.fmod(heading_error + math.pi, 2 * math.pi) - math.pi

    def get_speed_correction(self, target_heading: float) -> float:
        yaw_error = self.get_heading_error(target_heading)
        yaw_error_deg = math.degrees(abs(yaw_error))
        # 待测试，需要调整参数
        # 动态kp：大误差用大kp（快速修正），小误差用小kp（避免震荡）
        # 动态比例系数
        if yaw_error_deg > 10:
            kp = 20
        elif yaw_error_deg > 3:
            kp = 10
        else:
            kp = 5

        correction = kp * yaw_error
        max_correction = 50
        return max(min(correction, max_correction), -max_correction)

    def calibrate_heading_at_waypoint(self, target_heading: float) -> Generator[Tuple[float, float], None, bool]:
        self.get_logger().info(
            f"开始航向校准：目标{math.degrees(target_heading):.2f}°，当前{math.degrees(self.imu_yaw):.2f}°")
        start_time = self.get_clock().now()

        while rclpy.ok():
            heading_error_rad = self.get_heading_error(target_heading)
            heading_error_deg = math.degrees(abs(heading_error_rad))

            # 校准达标
            if heading_error_deg <= RTK_HEADING_TOLERANCE:
                self.get_logger().info(f"航向校准完成！误差：{heading_error_deg:.2f}°")
                return True

            # 超时处理
            elapsed_time = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed_time > HEADING_CALIBRATION_TIMEOUT:
                self.get_logger().warn(f"航向校准超时！误差：{heading_error_deg:.2f}°")
                return True

            # 计算转向速度
            turn_speed = TURN_SPEED
            # 修正误差：若误差超过180°，反向旋转（缩短路径）
            if abs(heading_error_rad) > math.pi:
                if heading_error_rad > 0:
                    heading_error_rad -= 2 * math.pi
                else:
                    heading_error_rad += 2 * math.pi
            # 根据修正后的误差计算转向方向
            if heading_error_rad > 0:
                # 顺时针旋转（根据你的电机控制逻辑调整，若反向则互换左右速度）
                left_speed = turn_speed
                right_speed = -turn_speed
            else:
                # 逆时针旋转
                left_speed = -turn_speed
                right_speed = turn_speed

            yield (left_speed, right_speed)

        return False

    def move_to_first_waypoint(self) -> Generator[Tuple[float, float], None, bool]:
        if not self.waypoints:
            self.get_logger().error("无航点数据，无法执行初始移动")
            return False

        # 等待IMU校准
        if not self.imu_initialized:
            self.get_logger().info(f"等待IMU校准（超时{IMU_CALIBRATION_TIMEOUT}秒）")
            start_time = self.get_clock().now()
            while not self.imu_initialized and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)
                elapsed_time = (self.get_clock().now() - start_time).nanoseconds / 1e9
                if elapsed_time > IMU_CALIBRATION_TIMEOUT:
                    self.get_logger().warn("IMU校准超时，使用默认偏航角")
                    break

        first_waypoint = self.waypoints[0]
        self.nav_context["target_waypoint"] = first_waypoint
        self.get_logger().info(f"开始移动到第一个航点：{first_waypoint[:2]}")

        last_distance = 0
        while rclpy.ok():
            distance = self.calc_distance_to_waypoint(first_waypoint)
            # 距离变化显著时打印
            if abs(last_distance - distance) > 0.1:
                self.get_logger().info(f"到第一个航点距离：{distance:.2f} m")
                last_distance = distance

            # 到达距离阈值，开始航向校准
            if distance < INITIAL_MOVE_TOLERANCE:
                self.get_logger().info(f"已到达第一个航点距离阈值：{distance:.2f} m")
                target_heading = self.get_path_heading(first_waypoint)
                self.nav_context["calib_generator"] = self.calibrate_heading_at_waypoint(target_heading)
                self.nav_context["nav_state"] = NavState.WAYPOINT_CALIB

                while rclpy.ok():
                    try:
                        left_speed, right_speed = next(self.nav_context["calib_generator"])
                        yield (left_speed, right_speed)
                    except StopIteration:
                        self.nav_context["calib_generator"] = None
                        self.nav_context["target_waypoint"] = None
                        return True
                return True

            # 未到达，直线行驶并纠偏
            target_heading = self.get_path_heading(first_waypoint)
            correction = self.get_speed_correction(target_heading)

            base_speed = LINEAR_SPEED_BASE
            left_speed_rad = -base_speed - correction
            right_speed_rad = base_speed + correction

            yield (left_speed_rad, right_speed_rad)

        return False

    def rad_from_linear(self, linear_speed: float) -> float:
        """线速度转电机角速度（适配电机节点参数）"""
        reduction_ratio = 1.0 #7.75
        wheel_diameter = 1.0 #0.04874
        return (linear_speed * reduction_ratio) / wheel_diameter

    def reset_nav_context(self):
        """重置导航状态"""
        self.current_waypoint_idx = 0
        self.nav_context = {
            "nav_state": NavState.IDLE,
            "target_waypoint": None,
            "calib_generator": None,
            "last_distance": 0.0,
            "last_target_heading": 0.0
        }
        # self.get_logger().info("RTK导航状态已重置")

    # ================== 原有控制模式检查 + 多点导航生成器 ==================
    def check_control_mode(self) -> bool:
        """
        检查当前控制模式，若切换为遥控器模式，暂停导航
        返回：True=保持RTK模式，False=已切换为遥控器模式
        """
        if self.current_control_mode == ControlMode.REMOTE:
            self.get_logger().info("[ROSNode] 切换到遥控器控制模式，暂停RTK导航（保存上下文）")
            # 发布停止速度
            stop_speed = Vector3()
            stop_speed.x = 0.0
            stop_speed.y = 0.0
            self.motor_speed_pub.publish(stop_speed)
            self.nav_running = False
            return False
        return True

    def multi_waypoint_nav_generator(self, resume: bool = False):
        """
        多点RTK导航生成器（适配ROS2定时器回调）
        每次yield返回左右轮速度，支持中断恢复
        """
        # 1. 初始化/恢复导航状态
        if resume:
            current_nav_state = self.nav_context["nav_state"]
            self.get_logger().info(f"[ROSNode] 从状态{current_nav_state}恢复RTK多点导航")
        else:
            # 只有导航状态为IDLE时，才重新初始化初始移动（解决重复进入第一个航点）
            if self.nav_context["nav_state"] == NavState.IDLE:
                current_nav_state = NavState.INITIAL_MOVE
                self.nav_context["nav_state"] = current_nav_state
                self.current_waypoint_idx = 0  # 新导航从0开始
                self.nav_context["target_waypoint"] = None
                self.nav_context["calib_generator"] = None
            else:
                current_nav_state = self.nav_context["nav_state"]

        # 2. 阶段1：初始移动（初始点→第一个航点）- 仅首次启动且非恢复时执行
        if current_nav_state == NavState.INITIAL_MOVE and not resume:
            self.get_logger().info("[ROSNode] 进入初始移动阶段：初始点→第一个航点")
            initial_move_generator = self.move_to_first_waypoint()
            while True:
                # 检查控制模式，若切换则退出
                if not self.check_control_mode():
                    yield (0.0, 0.0)  # 返回停止速度
                    return
                # 获取初始移动速度
                try:
                    left_speed, right_speed = next(initial_move_generator)
                    yield (left_speed, right_speed)  # 向定时器回调返回速度
                except StopIteration:
                    # 初始移动完成，切换到第一个航点
                    self.current_waypoint_idx = 1
                    current_nav_state = NavState.WAYPOINT_MOVE
                    self.nav_context["nav_state"] = current_nav_state
                    self.get_logger().info("[ROSNode] 初始移动完成，进入航点导航阶段")
                    break
                except Exception as e:
                    self.get_logger().error(f"[ROSNav] 初始移动失败：{str(e)}")
                    self.nav_running = False
                    self.publish_stop_speed()
                    yield (0.0, 0.0)
                    return

        # 3. 阶段2：多点航点循环（依次导航到所有航点）
        last_waypoint_idx = self.current_waypoint_idx
        while True:
            # 检查退出条件：控制模式切换/航点全部完成
            if not self.check_control_mode():
                yield (0.0, 0.0)
                return
            if self.current_waypoint_idx >= len(self.waypoints):
                break  # 所有航点导航完成

            # 关键修复：若航点索引切换（新航点），强制重置为WAYPOINT_MOVE状态
            if self.current_waypoint_idx != last_waypoint_idx:
                current_nav_state = NavState.WAYPOINT_MOVE
                self.nav_context["nav_state"] = current_nav_state
                last_waypoint_idx = self.current_waypoint_idx  # 更新上一个航点索引
                # self.get_logger().info(f"[ROSNode] 检测到航点切换，强制进入移动阶段（当前航点{self.current_waypoint_idx}）")

            # 3.1 获取目标航点（原有逻辑保留）
            if self.nav_context["target_waypoint"]:
                target_waypoint = self.nav_context["target_waypoint"]
            else:
                target_waypoint = self.get_target_waypoint(self.current_waypoint_idx)
                print("target_waypoint:",target_waypoint)
                if not target_waypoint:
                    self.get_logger().warn("[ROSNode] 未获取到目标航点，退出导航")
                    yield (0.0, 0.0)
                    return
                self.nav_context["target_waypoint"] = target_waypoint

            # 3.2 发布当前导航状态
            self.publish_nav_state(current_nav_state)

            # 3.3 子阶段A：航向校准
            if current_nav_state == NavState.WAYPOINT_CALIB:
                calib_generator = self.nav_context["calib_generator"]
                # 若校准生成器不存在，重新创建
                if not calib_generator:
                    self.get_logger().warn("[ROSNode] 校准生成器不存在，重新初始化航向校准")
                    target_heading = self.get_path_heading(target_waypoint)
                    calib_generator = self.calibrate_heading_at_waypoint(target_heading)
                    self.nav_context["calib_generator"] = calib_generator

                # 执行航向校准
                try:
                    left_speed, right_speed = next(calib_generator)
                    yield (left_speed, right_speed)
                except StopIteration as e:
                    # 校准完成，切换到下一个航点
                    calib_result = e.value if hasattr(e, 'value') else False
                    self.get_logger().info(
                        f"[ROSNode] 航点{self.current_waypoint_idx}航向校准完成，结果：{calib_result}"
                    )
                    # 更新索引和状态
                    self.current_waypoint_idx += 1
                    self.nav_context["calib_generator"] = None
                    self.nav_context["target_waypoint"] = None
                    self.nav_context["last_distance"] = 0.0  # 重置距离缓存
                    self.nav_context["last_target_heading"] = 0.0  # 重置航向缓存

                    # 关键：主动触发一次“获取新航点”的逻辑（提前校验航点2是否存在）
                    new_waypoint = self.get_target_waypoint(self.current_waypoint_idx)
                    if new_waypoint:
                        self.get_logger().info(
                            f"[ROSNode] 切换到新航点{self.current_waypoint_idx}，准备进入移动阶段"
                        )
                    else:
                        self.get_logger().warn(f"[ROSNode] 未获取到航点{self.current_waypoint_idx}")
                    yield (0.0, 0.0)  # 停顿期间停止电机
                    
                except Exception as e:
                    self.get_logger().error(f"[ROSNav] 航向校准失败：{str(e)}")
                    yield (0.0, 0.0)
                    return

            # 3.4 子阶段B：移动到当前航点
            if current_nav_state == NavState.WAYPOINT_MOVE:
                # 关键修复：强制校验并获取目标航点（确保航点2被正确加载）
                if not self.nav_context["target_waypoint"]:
                    target_waypoint = self.get_target_waypoint(self.current_waypoint_idx)
                    if not target_waypoint:
                        self.get_logger().warn(f"[ROSNode] 未获取到航点{self.current_waypoint_idx}，退出导航")
                        yield (0.0, 0.0)
                        return
                    self.nav_context["target_waypoint"] = target_waypoint
                # 计算到目标航点的距离和航向
                distance = self.calc_distance_to_waypoint(target_waypoint)
                target_heading = self.get_path_heading(target_waypoint)

                # 打印距离和航向（仅当变化较大时，减少日志冗余）
                if (abs(self.nav_context["last_distance"] - distance) > 0.1 or
                    abs(self.nav_context["last_target_heading"] - target_heading) > 0.1):
                    self.nav_context["last_distance"] = distance
                    self.nav_context["last_target_heading"] = target_heading
                    self.get_logger().info(
                        f"[ROSNode] 目标航点{self.current_waypoint_idx}：({self.waypoints[self.current_waypoint_idx][0]}，{self.waypoints[self.current_waypoint_idx][1]})，航向角{self.waypoints[self.current_waypoint_idx][2]}°，距离{distance:.2f}m，目标航向{math.degrees(target_heading):.2f}°"
                        # f"[ROSNode] 目标航点{self.current_waypoint_idx}：{self.current_waypoint_idx[0][:2]}距离{distance:.2f}m，目标航向{math.degrees(target_heading):.2f}°"
                    )

                # 距离未达标：直线行驶+实时纠偏
                if distance >= RTK_WAYPOINT_TOLERANCE:
                    correction = self.get_speed_correction(target_heading)
                    base_speed_rad = self.rad_from_linear(LINEAR_SPEED_BASE)
                    left_speed = base_speed_rad - correction
                    right_speed = base_speed_rad + correction
                    yield (left_speed, right_speed)
                # 距离达标：切换到航向校准阶段
                else:
                    self.get_logger().info(
                        f"[ROSNode] 已到达航点{self.current_waypoint_idx}距离阈值（{distance:.2f}m ≤ {RTK_WAYPOINT_TOLERANCE}m）"
                    )
                    target_heading = self.get_path_heading(target_waypoint)
                    calib_generator = self.calibrate_heading_at_waypoint(target_heading)
                    self.nav_context["calib_generator"] = calib_generator
                    current_nav_state = NavState.WAYPOINT_CALIB
                    self.nav_context["nav_state"] = current_nav_state
                    yield (0.0, 0.0)  # 到达航点后先停止

        # 4. 所有航点导航完成
        self.get_logger().info("[ROSNode] RTK多点导航全部完成")
        self.nav_context["nav_state"] = NavState.COMPLETED
        self.nav_running = False
        self.publish_stop_speed()  # 停止电机
        self.reset_nav_context()
        yield (0.0, 0.0)  # 最终返回停止速度

    # ================== 原有RTKControlNode核心方法 ==================
    def publish_stop_speed(self):
        """发布停止电机速度指令"""
        stop_speed = Vector3()
        stop_speed.x = 0.0
        stop_speed.y = 0.0
        stop_speed.z = 0.0
        self.motor_speed_pub.publish(stop_speed)

    def publish_nav_state(self, state: NavState):
        """发布当前导航状态"""
        state_msg = String()
        state_msg.data = state if isinstance(state, str) else state.value
        self.nav_state_pub.publish(state_msg)

    def mode_callback(self, msg: String):
        """接收电机节点的控制模式，更新自身状态"""
        self.current_control_mode = msg.data
        # 切换非RTK模式时，重置导航状态
        if self.current_control_mode != ControlMode.RTK_NAV:
            if self.multi_waypoint_generator:
                self.multi_waypoint_generator = None
            self.nav_running = False
            self.publish_stop_speed()
            self.reset_nav_context()

    def rtk_timer_callback(self):
        """10Hz定时器回调，驱动多点导航逻辑"""
        # 仅在RTK导航模式下执行导航逻辑
        if self.current_control_mode == ControlMode.RTK_NAV:
            # 初始化多点导航生成器（首次进入/导航完成后重新初始化，解决重复进入初始点）
            if not self.multi_waypoint_generator and not self.nav_running:
                # 判断是否需要恢复导航
                resume = (self.nav_context["nav_state"] != NavState.IDLE)
                self.multi_waypoint_generator = self.multi_waypoint_nav_generator(resume=resume)
                self.nav_running = True
                self.publish_nav_state(self.nav_context["nav_state"])
                self.get_logger().info("[ROSNode] 启动/恢复RTK多点导航")

            # 获取多点导航速度并发布
            try:
                if self.multi_waypoint_generator and self.nav_running:
                    left_speed_rad, right_speed_rad = next(self.multi_waypoint_generator)
                    # 构造速度消息并发布
                    speed_msg = Vector3()
                    speed_msg.x = float(left_speed_rad)
                    speed_msg.y = float(right_speed_rad)
                    speed_msg.z = 0.0
                    self.motor_speed_pub.publish(speed_msg)
            except StopIteration:
                # 导航生成器执行完毕（全部航点完成/主动退出）
                self.get_logger().info("[ROSNode] 多点导航生成器执行完毕")
                self.publish_nav_state(NavState.COMPLETED)
                self.multi_waypoint_generator = None
                self.nav_running = False
            except Exception as e:
                self.get_logger().error(f"[ROSNode] RTK多点导航错误：{str(e)}")
                # 发布停止指令
                self.publish_stop_speed()
                # 重置导航状态
                self.multi_waypoint_generator = None
                self.nav_running = False
                self.nav_context["nav_state"] = NavState.IDLE
                self.publish_nav_state(NavState.IDLE)
        else:
            # 非RTK模式：重置生成器
            if self.multi_waypoint_generator:
                self.multi_waypoint_generator = None
                self.nav_running = False
                self.publish_stop_speed()
            pass

# -------------------------- 主函数入口 --------------------------
def main(args=None):
    rclpy.init(args=args)
    rtk_node = RTKNavControlNode()

    try:
        rclpy.spin(rtk_node)
    except KeyboardInterrupt:
        rtk_node.get_logger().info("RTK控制节点收到中断信号，即将退出")
    except Exception as e:
        rtk_node.get_logger().fatal(f"RTK控制节点异常：{str(e)}")
    finally:
        # 发布停止速度
        rtk_node.publish_stop_speed()
        rtk_node.destroy_node()
        rclpy.shutdown()
        print("RTK控制节点退出完成")

if __name__ == "__main__":
    main()