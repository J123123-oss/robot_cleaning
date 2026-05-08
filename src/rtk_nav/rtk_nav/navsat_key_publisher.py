import sys
import select
import tty
import termios
import rclpy
import os
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header
from custom_msgs.msg import WTRTK
import random

# 预设的GPS数据字典（键：数字字符串，值：(经度, 纬度, X高度)）
GPS_PRESETS = {

    '1': (120.07130780, 30.32172521, 192.02),
    '2': (120.07129290, 30.32166478, 102.13),
    '3': (120.07130323, 30.32166287, 12.02),
    '4': (120.07131813, 30.32172329, 102.13),
    '5': (120.07132846, 30.32172137, 192.02),
    '6': (120.07131356, 30.32166095, 102.13),
    '7': (120.07132388, 30.32165903, 12.02),
    '8': (120.07133878, 30.32171946, 12.02),
    # '9': (120.07150170, 30.32131988, 223.65),
    # '10': (120.07141551, 30.32124189, 313.93),
    # '11': (120.07141285, 30.32124410, 43.65),
    # '12': (120.07149904, 30.32132210, 43.65)
}

class NavSatKeyPublisher(Node):
    def __init__(self):
        super().__init__('navsat_key_publisher')
        
        # 发布者：仅在按键时发布单条 NavSatFix
        self.publisher_ = self.create_publisher(NavSatFix, '/fix', 10)
        self.wtrtk_publisher_ = self.create_publisher(WTRTK, '/wtrtk_data', 0)
        

        # 终端设置保存与恢复
        self.old_settings = termios.tcgetattr(sys.stdin)
        # 设置为非阻塞无回车读取（原逻辑保留）
        self.set_terminal_raw_mode()

        # 读取点列表（优先参数指定文件，否则从 cleaning_path 中选最新文件）
        # points: list of (lon, lat, alt, heading)
        self.points = []
        self.current_index = 0
        # 是否处于持续发布状态（按键启动后开始持续1Hz发布）
        self.publish_active = False
        self.points_file = None
        # 尝试自动发现最新路径文件
        try:
            self.points_file = self.find_latest_points_file()
        except Exception:
            self.points_file = None
        if self.points_file:
            try:
                self.points = self.load_points_from_file(self.points_file)
                self.get_logger().info(f'Loaded {len(self.points)} points from {self.points_file}')
            except Exception as e:
                self.get_logger().error(f'Failed to load points file {self.points_file}: {e}')
                self.points = []
        else:
            # 回退到内置预设（尽量保证程序可用）
            self.get_logger().warn('未找到点文件，回退到内置GPS_PRESETS')
            for k in sorted(GPS_PRESETS.keys()):
                lon, lat, alt = GPS_PRESETS[k]
                self.points.append((lon, lat, alt, 0.0))

        if not self.points:
            self.get_logger().fatal('没有可用的GPS点，程序将退出')

        self.get_logger().info('=== GPS按键发布器（文件加载版）已启动 ===')
        self.get_logger().info('按键：数字1-9 立即发布对应序号点；n 发布下一点；p 发布上一点；Ctrl+C 退出')
        # 定时器：每1秒检查是否需要发布（由 publish_active 控制）
        self.pub_timer = self.create_timer(1.0, self.timer_callback)
    
    def set_terminal_raw_mode(self):
        """将终端设置为原始模式，按键立即可读（不需要回车）。"""
        settings = termios.tcgetattr(sys.stdin)
        # lflags: turn off ICANON and ECHO for raw mode
        settings[3] = settings[3] & ~termios.ICANON & ~termios.ECHO
        settings[6][termios.VMIN] = 1
        settings[6][termios.VTIME] = 0
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    
    def publish_point(self, index: int):
        """按索引（0-base）发布指定点一次，并打印日志。"""
        if index < 0 or index >= len(self.points):
            self.get_logger().warn(f'索引越界: {index+1}')
            return
        lon, lat, alt, heading = self.points[index]
        nav_sat_msg = NavSatFix()
        nav_sat_msg.header.stamp = self.get_clock().now().to_msg()
        nav_sat_msg.header.frame_id = ''
        nav_sat_msg.status.status = 4
        nav_sat_msg.status.service = 0
        nav_sat_msg.latitude = lat
        nav_sat_msg.longitude = lon
        nav_sat_msg.altitude = alt
        nav_sat_msg.position_covariance = [0.0] * 9
        nav_sat_msg.position_covariance_type = 0
        self.publisher_.publish(nav_sat_msg)
        # 发布 WTRTK 消息：ins_heading 字段
        try:
            wmsg = WTRTK()
            wmsg.header = Header()
            wmsg.header.stamp = self.get_clock().now().to_msg()
            wmsg.header.frame_id = 'wtrtk_link'
            # 保证 heading 为 float
            try:
                # ins_heading+随机1-30
                wmsg.ins_heading = float(heading + random.uniform(1.0, 5.0))
            except Exception:
                wmsg.ins_heading = 0.0
            # self.wtrtk_publisher_.publish(wmsg)
        except Exception as e:
            self.get_logger().warn(f'发布 WTRTK 失败: {e}')
        self.get_logger().info(f'Published idx={index+1}: lon={lon:.8f}, lat={lat:.8f}, heading={wmsg.ins_heading:.2f}')

    def timer_callback(self):
        # 每秒发布当前索引的点（同时发布 /fix 和 /wtrtk_data）
        if self.publish_active and self.points:
            self.publish_point(self.current_index)
    
    def check_keypress(self):
        """检测按键并在按键时发布点：
        - 数字 1-9: 直接发布对应序号点
        - n: 发布下一点
        - p: 发布上一点
        - Ctrl+C: 返回 False 用于退出
        """
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            key = sys.stdin.read(1)
            # 退出
            if ord(key) == 3:
                return False

            if key in [str(i) for i in range(1, 10)]:
                idx = int(key) - 1
                if idx < len(self.points):
                    self.current_index = idx
                    # 启动持续发布并立即触发一次
                    self.publish_active = True
                    self.publish_point(self.current_index)
                else:
                    self.get_logger().warn(f'没有第{idx+1}个点')

            elif key.lower() == 'n':
                # next
                self.current_index = (self.current_index + 1) % len(self.points)
                self.publish_active = True
                self.publish_point(self.current_index)

            elif key.lower() == 'p':
                # prev
                self.current_index = (self.current_index - 1) % len(self.points)
                self.publish_active = True
                self.publish_point(self.current_index)

            elif key.lower() == 's':
                # stop continuous publish
                if self.publish_active:
                    self.publish_active = False
                    self.get_logger().info('已停止持续发布')
                else:
                    self.get_logger().info('当前未在发布状态')

            else:
                self.get_logger().info(f'未绑定按键: {repr(key)}')
        return True

    def find_latest_points_file(self):
        """在 cleaning_path 目录查找最新生成的 three_path 文件并返回路径。"""
        base_dir = os.path.join(os.path.dirname(__file__), 'cleaning_path')
        if not os.path.isdir(base_dir):
            return None
        candidates = []
        for fn in os.listdir(base_dir):
            if fn.startswith('three_path_') and (fn.endswith('.txt') or fn.endswith('.csv')):
                candidates.append(os.path.join(base_dir, fn))
        if not candidates:
            return None
        candidates.sort(key=lambda p: os.path.getmtime(p), reverse=True)
        return candidates[0]

    def load_points_from_file(self, filepath):
        """从文件读取每行 lon,lat[,alt] 格式的点，忽略注释和空行。"""
        pts = []
        with open(filepath, 'r', encoding='utf-8') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                parts = [p.strip() for p in line.split(',')]
                # 支持带序号行（如: 1,lon,lat,heading）和不带序号行
                # 找到其中包含经度和纬度的两个连续浮点数
                nums = []
                for p in parts:
                    try:
                        nums.append(float(p))
                    except Exception:
                        continue
                if len(nums) >= 2:
                    # 支持两种常见格式：
                    # 1) lon,lat[,alt,...]
                    # 2) idx,lon,lat[,heading|alt,...]
                    lon = None
                    lat = None
                    alt = 0.0
                    if len(nums) >= 3:
                        # 如果第一项看起来像序号（整数且较小），且第二/三项在经纬度合法范围，则按 idx,lon,lat 解析
                        first_is_index = abs(nums[0] - round(nums[0])) < 1e-8 and 0 < nums[0] < 1e6
                        second_is_lon = -180.0 <= nums[1] <= 180.0
                        third_is_lat = -90.0 <= nums[2] <= 90.0
                        if first_is_index and second_is_lon and third_is_lat:
                            lon = nums[1]
                            lat = nums[2]
                            # 第四列通常为 heading_deg（dense 文件格式），我们把它保存到 heading 字段，alt 默认为 0
                            heading = nums[3] if len(nums) >= 4 else 0.0
                            alt = 0.0
                        else:
                            lon = nums[0]
                            lat = nums[1]
                            alt = nums[2] if len(nums) >= 3 else 0.0
                            heading = nums[3] if len(nums) >= 4 else 0.0
                    else:
                        lon = nums[0]
                        lat = nums[1]
                        alt = nums[2] if len(nums) >= 3 else 0.0
                        heading = nums[3] if len(nums) >= 4 else 0.0
                    pts.append((lon, lat, alt, heading))
        return pts

    def destroy_node(self):
        """
        节点销毁时恢复终端设置，避免终端异常
        """
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        super().destroy_node()

def main(args=None):
    # 1. 初始化ROS2上下文
    rclpy.init(args=args)
    
    # 2. 创建节点实例
    navsat_publisher = NavSatKeyPublisher()
    
    # 3. 循环运行：检测按键 + 处理ROS2回调
    try:
        while rclpy.ok():
            # 检测按键（非阻塞）
            if not navsat_publisher.check_keypress():
                break
            # 处理定时器回调（发布消息）
            rclpy.spin_once(navsat_publisher, timeout_sec=0.01)
    except KeyboardInterrupt:
        pass
    finally:
        # 4. 销毁节点，关闭ROS2上下文
        navsat_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()