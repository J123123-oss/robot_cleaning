#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import math
from typing import Optional, List, Dict, Tuple, Generator
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3  # 用于发布左右轮速度
from std_msgs.msg import String       # 用于发布控制模式和导航状态
from custom_msgs.msg import WTRTK

# -------------------------- 全局配置与枚举 --------------------------
# RTK导航配置
RTK_WAYPOINT_TOLERANCE = 0.5
RTK_HEADING_TOLERANCE = 0.5
LINEAR_SPEED_BASE = 100       # origin 0.0124
ANGULAR_SPEED_BASE = 100      # origin 0.1
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

# -------------------------- RTK导航核心类 --------------------------
class RTKNavigator:
    def __init__(self, node: Node):
        self.node = node
        self.waypoints: List[Tuple[float, float, float]] = []
        self.current_waypoint_idx = 0
        self.current_gps: Optional[Tuple[float, float]] = None
        self.imu_yaw = 0.0
        self.imu_initialized = False
        self.imu_calibration_offset = 0.0

        # 声明RTK路径参数
        self.rtk_path_file = self.node.declare_parameter(
            'rtk_path_file',
            "/home/forlinx/robot_cleaning/src/rtk_nav/rtk_nav/cleaning_path/cleaning_path_20251121_173149.txt"
        ).value

        # 加载航点
        self.load_rtk_path()

        self.nav_context = {
            "nav_state": NavState.IDLE,
            "target_waypoint": None,
            "calib_generator": None
        }

        # ROS2订阅器：GPS+IMU
        self.gps_sub = self.node.create_subscription(
            NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.heading_sub = self.node.create_subscription(
            WTRTK, '/gps/wtrtk', self.heading_callback, 10)

    def load_rtk_path(self) -> bool:
        if not os.path.exists(self.rtk_path_file):
            self.node.get_logger().error(f"RTK路径文件不存在：{self.rtk_path_file}")
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
            self.node.get_logger().info(f"成功加载RTK航点{len(self.waypoints)}个")
            return True
        except Exception as e:
            self.node.get_logger().error(f"解析RTK文件失败：{str(e)}")
            return False

    def gps_callback(self, msg: NavSatFix) -> None:
        if msg.status.status < 0:
            self.node.get_logger().warn("GPS信号无效")
            return

        status_map = {1: "弱", 2: "差分", 3: "RTK", 4: "RTK固定解"}
        if msg.status.status in status_map:
            self.node.get_logger().debug(f"GPS状态：{status_map[msg.status.status]}")

        self.current_gps = (msg.longitude, msg.latitude)

    def heading_callback(self, msg: WTRTK) -> None:
        ins_heading_deg = msg.ins_heading
        ins_heading_rad = math.radians(ins_heading_deg)
        ins_heading_rad = math.fmod(ins_heading_rad + math.pi, 2 * math.pi) - math.pi

        if not self.imu_initialized:
            self.imu_calibration_offset = -ins_heading_rad
            self.imu_initialized = True
            self.node.get_logger().info(
                f"IMU校准完成！初始偏移：{math.degrees(self.imu_calibration_offset):.2f}°")

        self.imu_yaw = ins_heading_rad + self.imu_calibration_offset
        self.imu_yaw = math.fmod(self.imu_yaw + math.pi, 2 * math.pi) - math.pi

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
        heading_deg = waypoint[2] + math.degrees(self.imu_calibration_offset)
        heading_rad = math.radians(heading_deg)
        return math.fmod(heading_rad + math.pi, 2 * math.pi) - math.pi

    def get_heading_error(self, target_heading: float) -> float:
        heading_error = target_heading - self.imu_yaw
        return math.fmod(heading_error + math.pi, 2 * math.pi) - math.pi

    def get_speed_correction(self, target_heading: float) -> float:
        yaw_error = self.get_heading_error(target_heading)
        yaw_error_deg = math.degrees(abs(yaw_error))

        # 动态比例系数
        if yaw_error_deg > 10:
            kp = 0.8
        elif yaw_error_deg > 3:
            kp = 0.5
        else:
            kp = 0.2

        correction = kp * yaw_error
        max_correction = 0.3
        return max(min(correction, max_correction), -max_correction)

    def calibrate_heading_at_waypoint(self, target_heading: float) -> Generator[Tuple[float, float], None, bool]:
        self.node.get_logger().info(
            f"开始航向校准：目标{math.degrees(target_heading):.2f}°，当前{math.degrees(self.imu_yaw):.2f}°")
        start_time = self.node.get_clock().now()

        while rclpy.ok():
            heading_error_rad = self.get_heading_error(target_heading)
            heading_error_deg = math.degrees(abs(heading_error_rad))

            # 校准达标
            if heading_error_deg <= RTK_HEADING_TOLERANCE:
                self.node.get_logger().info(f"航向校准完成！误差：{heading_error_deg:.2f}°")
                return True

            # 超时处理
            elapsed_time = (self.node.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed_time > HEADING_CALIBRATION_TIMEOUT:
                self.node.get_logger().warn(f"航向校准超时！误差：{heading_error_deg:.2f}°")
                return True

            # 计算转向速度
            angular_speed = ANGULAR_SPEED_BASE
            if heading_error_rad > 0:
                left_speed = angular_speed
                right_speed = -angular_speed
            else:
                left_speed = -angular_speed
                right_speed = angular_speed

            yield (left_speed, right_speed)

        return False

    def move_to_first_waypoint(self) -> Generator[Tuple[float, float], None, bool]:
        if not self.waypoints:
            self.node.get_logger().error("无航点数据，无法执行初始移动")
            return False

        # 等待IMU校准
        if not self.imu_initialized:
            self.node.get_logger().info(f"等待IMU校准（超时{IMU_CALIBRATION_TIMEOUT}秒）")
            start_time = self.node.get_clock().now()
            while not self.imu_initialized and rclpy.ok():
                rclpy.spin_once(self.node, timeout_sec=0.05)
                elapsed_time = (self.node.get_clock().now() - start_time).nanoseconds / 1e9
                if elapsed_time > IMU_CALIBRATION_TIMEOUT:
                    self.node.get_logger().warn("IMU校准超时，使用默认偏航角")
                    break

        first_waypoint = self.waypoints[0]
        self.nav_context["target_waypoint"] = first_waypoint
        self.node.get_logger().info(f"开始移动到第一个航点：{first_waypoint[:2]}")

        last_distance = 0
        while rclpy.ok():
            distance = self.calc_distance_to_waypoint(first_waypoint)
            # 距离变化显著时打印
            if abs(last_distance - distance) > 0.1:
                self.node.get_logger().info(f"到第一个航点距离：{distance:.2f} m")
                last_distance = distance

            # 到达距离阈值，开始航向校准
            if distance < INITIAL_MOVE_TOLERANCE:
                self.node.get_logger().info(f"已到达第一个航点距离阈值：{distance:.2f} m")
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
            left_speed_rad = -base_speed + correction
            right_speed_rad = base_speed + correction
            # left_speed_rad = self.rad_from_linear(base_speed) - correction
            # right_speed_rad = self.rad_from_linear(base_speed) + correction

            yield (left_speed_rad, right_speed_rad)

        return False

    def rad_from_linear(self, linear_speed: float) -> float:
        """线速度转电机角速度（适配电机节点参数）"""
        reduction_ratio = 7.75
        wheel_diameter = 0.04874
        return (linear_speed * reduction_ratio) / wheel_diameter

    def reset_nav_context(self):
        """重置导航状态"""
        self.current_waypoint_idx = 0
        self.nav_context = {
            "nav_state": NavState.IDLE,
            "target_waypoint": None,
            "calib_generator": None
        }
        self.node.get_logger().info("RTK导航状态已重置")

# -------------------------- RTK控制节点（独立ROS2节点） --------------------------
class RTKControlNode(Node):
    def __init__(self):
        super().__init__('rtk_control_node')

        # 循环频率
        self.rate = self.create_rate(10)

        # 1. 初始化RTK导航器
        self.rtk_navigator = RTKNavigator(self)
        self.nav_generator: Optional[Generator] = None

        # 2. ROS2发布器：发布电机速度指令（给电机节点）
        self.motor_speed_pub = self.create_publisher(
            Vector3, "/rtk/motor_speed", 10
        )

        # 3. ROS2订阅器：订阅当前控制模式（来自电机节点）
        self.control_mode_sub = self.create_subscription(
            String, "/control/mode", self.mode_callback, 10
        )

        # 4. ROS2发布器：发布导航状态
        self.nav_state_pub = self.create_publisher(
            String, "/rtk/nav_state", 10
        )

        # 当前控制模式
        self.current_control_mode = ControlMode.NORMAL

        # 启动主循环
        self.rtk_nav_timer = self.create_timer(0.1, self.rtk_timer_callback)  # 0.1秒 = 10Hz


    def rtk_timer_callback(self):
        # 仅在RTK导航模式下执行导航逻辑
        if self.current_control_mode == ControlMode.RTK_NAV:
            # 初始化导航生成器
            if not self.nav_generator:
                self.nav_generator = self.rtk_navigator.move_to_first_waypoint()
                self.publish_nav_state(NavState.INITIAL_MOVE)

            # 获取导航速度并发布给电机节点
            try:
                if self.nav_generator:
                    left_speed_rad, right_speed_rad = next(self.nav_generator)
                    # 构造速度消息（x=左轮速度，y=右轮速度，z=预留）
                    speed_msg = Vector3()
                    speed_msg.x = left_speed_rad
                    speed_msg.y = right_speed_rad
                    speed_msg.z = 0.0
                    self.motor_speed_pub.publish(speed_msg)
            except StopIteration:
                self.get_logger().info("第一个航点导航完成")
                self.publish_nav_state(NavState.IDLE)
                self.nav_generator = None
            except Exception as e:
                self.get_logger().error(f"RTK导航错误：{str(e)}")
                # 发布停止指令
                stop_speed = Vector3()
                stop_speed.x = 0.0
                stop_speed.y = 0.0
                self.motor_speed_pub.publish(stop_speed)
                self.nav_generator = None
                self.publish_nav_state(NavState.IDLE)
        else:
            # 非RTK模式，发布停止速度（可选，防止电机误动）
            pass    

    def mode_callback(self, msg: String):
        """接收电机节点的控制模式，更新自身状态"""
        self.current_control_mode = msg.data
        # 切换非RTK模式时，重置导航状态
        if self.current_control_mode != ControlMode.RTK_NAV and self.nav_generator:
            self.rtk_navigator.reset_nav_context()
            self.nav_generator = None
            # 发布停止速度
            stop_speed = Vector3()
            stop_speed.x = 0.0
            stop_speed.y = 0.0
            stop_speed.z = 0.0
            self.motor_speed_pub.publish(stop_speed)

    def publish_nav_state(self, state: str):
        """发布当前导航状态"""
        state_msg = String()
        state_msg.data = state
        self.nav_state_pub.publish(state_msg)

    def run(self):
        """RTK节点主循环"""
        while rclpy.ok():
            # 仅在RTK导航模式下执行导航逻辑
            if self.current_control_mode == ControlMode.RTK_NAV:
                # 初始化导航生成器
                if not self.nav_generator:
                    self.nav_generator = self.rtk_navigator.move_to_first_waypoint()
                    self.publish_nav_state(NavState.INITIAL_MOVE)

                # 获取导航速度并发布给电机节点
                try:
                    if self.nav_generator:
                        left_speed_rad, right_speed_rad = next(self.nav_generator)
                        # 构造速度消息（x=左轮速度，y=右轮速度，z=预留）
                        speed_msg = Vector3()
                        speed_msg.x = left_speed_rad
                        speed_msg.y = right_speed_rad
                        speed_msg.z = 0.0
                        self.motor_speed_pub.publish(speed_msg)
                except StopIteration:
                    self.get_logger().info("第一个航点导航完成")
                    self.publish_nav_state(NavState.IDLE)
                    self.nav_generator = None
                except Exception as e:
                    self.get_logger().error(f"RTK导航错误：{str(e)}")
                    # 发布停止指令
                    stop_speed = Vector3()
                    stop_speed.x = 0.0
                    stop_speed.y = 0.0
                    self.motor_speed_pub.publish(stop_speed)
                    self.nav_generator = None
                    self.publish_nav_state(NavState.IDLE)
            else:
                # 非RTK模式，发布停止速度（可选，防止电机误动）
                pass

            # 处理回调并延时
            # rclpy.spin_once(self, timeout_sec=0.01)
            # self.rate.sleep()

# -------------------------- 主函数入口 --------------------------
def main(args=None):
    rclpy.init(args=args)
    rtk_node = RTKControlNode()

    try:
        rclpy.spin(rtk_node)
    except KeyboardInterrupt:
        rtk_node.get_logger().info("RTK控制节点收到中断信号，即将退出")
    except Exception as e:
        rtk_node.get_logger().fatal(f"RTK控制节点异常：{str(e)}")
    finally:
        # 发布停止速度
        stop_speed = Vector3()
        stop_speed.x = 0.0
        stop_speed.y = 0.0
        rtk_node.motor_speed_pub.publish(stop_speed)
        rtk_node.destroy_node()
        rclpy.shutdown()
        print("RTK控制节点退出完成")

if __name__ == "__main__":
    main()