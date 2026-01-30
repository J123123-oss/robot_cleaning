import sys
import select
import tty
import termios
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

# 预设的GPS数据字典（键：数字字符串，值：(经度, 纬度, X高度)）
GPS_PRESETS = {
    '1': (120.07151235, 30.32131103, 223.65),
    '2': (120.07142616, 30.32123303, 313.93),
    '3': (120.07142350, 30.32123525, 43.65),
    '4': (120.07150969, 30.32131325, 313.93),
    '5': (120.07150703, 30.32131546, 223.65),
    '6': (120.07142084, 30.32123746, 313.93),
    '7': (120.07141817, 30.32123967, 43.65),
    '8': (120.07150437, 30.32131767, 313.93),
    '9': (120.07150170, 30.32131988, 223.65),
    '10': (120.07141551, 30.32124189, 313.93),
    '11': (120.07141285, 30.32124410, 43.65),
    '12': (120.07149904, 30.32132210, 43.65)
}

class NavSatKeyPublisher(Node):
    def __init__(self):
        super().__init__('navsat_key_publisher')
        
        # 1. 创建发布者，发布 /fix 话题，消息类型 NavSatFix，队列大小10
        self.publisher_ = self.create_publisher(NavSatFix, '/fix', 10)
        
        # 2. 初始化当前使用的GPS数据（默认使用12号数据，和你原始命令一致）
        self.current_lon, self.current_lat, self.current_alt = GPS_PRESETS['4']
        
        # 3. 创建定时器，实现1Hz持续发布（对应 -r 1）
        self.timer_period = 1.0  # 周期1秒
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        # 4. 保存终端原始设置，用于后续恢复（实现无回车按键检测）
        self.old_settings = termios.tcgetattr(sys.stdin)
        # 这里修改终端模式，支持行缓冲（方便读取两位数）
        self.set_terminal_line_buffer()
        
        # 5. 初始化输入缓冲区，用于拼接两位数
        self.input_buffer = ""
        
        # 6. 打印提示信息（明确输入方式）
        self.get_logger().info("=== GPS按键发布器已启动（修复10/11/12发布问题）===")
        self.get_logger().info("输入方式：")
        self.get_logger().info("  1-9：直接按键（无回车）切换对应GPS数据")
        self.get_logger().info("  10/11/12：输入数字后按【空格】或【回车】切换")
        self.get_logger().info("按下Ctrl+C退出程序")
    
    def set_terminal_line_buffer(self):
        """设置终端为行缓冲模式，支持读取多字符输入（两位数）"""
        settings = termios.tcgetattr(sys.stdin)
        settings[3] = (settings[3] & ~termios.ICANON) | termios.ICANON  # 启用行缓冲
        settings[6][termios.VMIN] = 1
        settings[6][termios.VTIME] = 0
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    
    def timer_callback(self):
        """
        定时器回调函数：每秒构造并发布一次NavSatFix消息
        """
        # 1. 构造NavSatFix消息
        nav_sat_msg = NavSatFix()
        
        # 2. 填充消息头（stamp自动使用当前节点时间，frame_id留空和你原始命令一致）
        nav_sat_msg.header.stamp = self.get_clock().now().to_msg()
        nav_sat_msg.header.frame_id = ''
        
        # 3. 填充status（和你原始命令一致：status=4, service=0）
        nav_sat_msg.status.status = 4
        nav_sat_msg.status.service = 0
        
        # 4. 填充经纬度、高度（使用当前选中的数值）
        nav_sat_msg.latitude = self.current_lat
        nav_sat_msg.longitude = self.current_lon
        nav_sat_msg.altitude = self.current_alt
        
        # 5. 填充位置协方差（和你原始命令一致，全0）
        nav_sat_msg.position_covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        nav_sat_msg.position_covariance_type = 0
        
        # 6. 发布消息
        self.publisher_.publish(nav_sat_msg)
        
        # 7. 打印当前发布状态（可选，方便调试查看）
        self.get_logger().info(
            f"当前发布：纬度={self.current_lat:.8f}, 经度={self.current_lon:.8f}, 高度={self.current_alt:.2f}"
        )
    
    def check_keypress(self):
        """
        检测按键输入（无回车），切换对应的GPS数据
        """
        # 检测是否有按键输入
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            key = sys.stdin.read(1)
            
            # 处理数字键1-9
            if key in [str(i) for i in range(1, 10)]:
                self.current_lon, self.current_lat, self.current_alt = GPS_PRESETS[key]
                self.get_logger().info(f"已切换到GPS预设{key}")
            
            # 处理10/11/12（明确映射，修复无法触发问题）
            elif key == '0':
                self.current_lon, self.current_lat, self.current_alt = GPS_PRESETS['10']
                self.get_logger().info(f"已切换到GPS预设10（按键0触发）")
            elif key == '-':
                self.current_lon, self.current_lat, self.current_alt = GPS_PRESETS['11']
                self.get_logger().info(f"已切换到GPS预设11（按键-触发）")
            elif key == '=':
                self.current_lon, self.current_lat, self.current_alt = GPS_PRESETS['12']
                self.get_logger().info(f"已切换到GPS预设12（按键=触发）")
            
            # 处理退出（Ctrl+C）
            elif ord(key) == 3:
                return False
        return True

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