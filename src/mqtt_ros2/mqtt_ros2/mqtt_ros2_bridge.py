#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import paho.mqtt.client as mqtt
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import subprocess
import threading
import traceback
import os

# 全局线程锁 保证MQTT发布线程安全
lock = threading.Lock()

def check_network(broker, timeout=3):
    """
    检测网络是否连通且MQTT Broker可达 (原逻辑完全保留)
    Args:
        broker (str): MQTT Broker地址（IP或域名）
        timeout (int): 超时时间（秒）
    Returns:
        bool: 网络可用且Broker可达返回True，否则False
    """
    ping_param = "-n" if os.name == "nt" else "-c"
    ping_command = ["ping", ping_param, "1", "-W", str(timeout), broker]

    try:
        subprocess.run(
            ping_command,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=True,
            timeout=timeout + 1
        )
        return True
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired):
        public_dns = ["8.8.8.8", "114.114.114.114"]
        for dns in public_dns:
            try:
                subprocess.run(
                    ["ping", ping_param, "1", "-W", str(timeout), dns],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    check=True,
                    timeout=timeout + 1
                )
                print(f"[网络检测] 基础网络连通（DNS:{dns}），但MQTT Broker({broker})不可达")
                return False
            except:
                continue
        return False


class MQTTRos2Bridge(Node):
    """ROS2版本 MQTT-ROS桥接核心类 继承ROS2 Node"""
    def __init__(self):
        super().__init__('mqtt')  # ROS2节点名称
        
        # ===================== ROS2 参数声明与获取 (替代rospy.get_param) =====================
        # 格式: self.declare_parameter(参数名, 默认值)  ROS2必须先声明再获取
        self.broker = self.declare_parameter('broker', '121.40.57.48').value
        self.port = int(self.declare_parameter('port', 1883).value)
        self.user = self.declare_parameter('user', 'gf-mounted').value
        self.password = str(self.declare_parameter('password', '20230810').value)
        self.topic_status = self.declare_parameter('topic_status', 'robot/HANGZHOU/status').value
        self.topic_dock_status = self.declare_parameter('topic_dock_status', 'dock/HANGZHOU/status').value
        self.topic_cmd = self.declare_parameter('topic_cmd', 'robot/HANGZHOU/cmd').value
        self.topic_command = self.declare_parameter('topic_command', 'robot/HANGZHOU/command').value
        self.topic_result = self.declare_parameter('topic_result', 'robot/HANGZHOU/result').value
        self.client_id = self.declare_parameter('client_id', 'python-mqtt-client-ID').value
        self.ca_cert = self.declare_parameter('ca_cert', None).value

        # 打印最终配置
        self.config = {
            "broker":self.broker, "port":self.port, "user":self.user, "password":self.password,
            "topic_status":self.topic_status, "topic_dock_status":self.topic_dock_status, "topic_cmd":self.topic_cmd,
            "topic_command":self.topic_command, "topic_result":self.topic_result,
            "client_id":self.client_id, "ca_cert":self.ca_cert
        }
        # self.get_logger().info(f"✅ 初始化完成，最终配置: {self.config}")

        # ===================== ROS2 发布者/订阅者创建 (替代rospy的pub/sub) =====================
        # ROS2 发布: 给ROS2内部发指令 /robot_cmd
        self.ros_cmd_pub = self.create_publisher(String, 'robot_cmd', 10)
        # ROS2 订阅: 监听ROS2内部状态 /robot_state 并转发到MQTT
        self.create_subscription(String, 'robot_state', self.ros_robot_state_callback, 10)
        self.create_subscription(String, 'dock_state', self.ros_dock_state_callback, 10)

        # ===================== MQTT客户端初始化 =====================
        self.client = mqtt.Client(
            client_id=self.client_id, 
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2
        )
        # MQTT回调函数绑定
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.on_message = self.on_message
        self.client.on_subscribe = self.on_subscribe
        self.client.on_publish = self.on_publish

        # MQTT TLS加密配置
        if self.ca_cert:
            self.client.tls_set(ca_certs=self.ca_cert, cert_reqs=mqtt.ssl.CERT_NONE)
        
        # MQTT账号密码配置
        self.client.username_pw_set(self.user, self.password)
        
        # MQTT自动重连配置
        self.client.reconnect_delay_set(min_delay=1, max_delay=60)

    # =========================================================
    # MQTT V2 回调函数 (原逻辑完全保留)
    # =========================================================
    def on_connect(self, client, userdata, flags, reason_code, properties):
        self.get_logger().info(f"\n[状态] 服务器连接结果: {mqtt.connack_string(reason_code)}")
        if reason_code == mqtt.MQTT_ERR_SUCCESS:
            self.get_logger().info(f"  ├─ 订阅控制主题: {self.topic_cmd}")
            self.get_logger().info(f"  ├─ 订阅命令主题: {self.topic_command}")
            client.subscribe(self.topic_cmd, qos=0)
            client.subscribe(self.topic_command, qos=0)

    def on_disconnect(self, client, userdata, disconnect_flags, reason_code, properties):
        self.get_logger().warn(f"\n[状态] 与服务器断开连接: {mqtt.error_string(reason_code)}")
        self.get_logger().warn(f"  ├─ 标志: {disconnect_flags}")
        self.get_logger().warn(f"  └─ 原因代码: {reason_code}")

    def on_message(self, client, userdata, msg):
        self.get_logger().info(f"\n[收到消息] \n  ├─ 主题: {msg.topic}\n  ├─ QoS: {msg.qos}\n  └─ 内容: {msg.payload.decode()}")
        
        try:
            if msg.topic == self.topic_cmd and self.ros_cmd_pub:
                ros_msg = String()
                try:
                    cmd_obj = json.loads(msg.payload.decode())
                    ros_msg.data = json.dumps(cmd_obj)
                except json.JSONDecodeError:
                    ros_msg.data = msg.payload.decode()
                    
                self.ros_cmd_pub.publish(ros_msg)
                self.get_logger().info(f"[MQTT->ROS2] 已发布到 /robot_cmd: {ros_msg.data}")
            
            elif msg.topic == self.topic_command:
                self.handle_terminal_command(msg.payload.decode())
                
        except Exception as e:
            error_msg = f"处理消息时出错: {str(e)}\n{traceback.format_exc()}"
            self.get_logger().error(error_msg)

    def on_subscribe(self, client, userdata, mid, reason_codes, properties):
        if reason_codes and len(reason_codes) > 0:
            qos = reason_codes[0].value
            self.get_logger().info(f"\n[状态] 订阅成功 (消息ID: {mid}, QoS: {qos})")
        else:
            self.get_logger().info(f"\n[状态] 订阅成功 (消息ID: {mid})")

    def on_publish(self, client, userdata, mid, reason_code, properties):
        self.get_logger().info(f"\n[状态] 消息发布成功 (消息ID: {mid}, 原因代码: {reason_code})")

    # =========================================================
    # 终端命令处理 + 执行 (原逻辑完全保留)
    # =========================================================
    def handle_terminal_command(self, command_str):
        try:
            command_id = f"cmd_{int(time.time() * 1000)}"
            self.get_logger().info(f"[命令执行] 开始执行命令: {command_str}")
            
            result = self.execute_command(command_str, timeout=60)
            
            response = {
                "id": command_id,
                "command": command_str,
                "success": result["returncode"] == 0,
                "stdout": result["stdout"],
                "stderr": result["stderr"],
                "returncode": result["returncode"],
                "timestamp": time.time()
            }
            
            with lock:
                self.client.publish(
                    self.topic_result,
                    json.dumps(response, ensure_ascii=False),
                    qos=0
                )
                self.get_logger().info(f"[命令执行] 已发送结果到 {self.topic_result}")
                
        except Exception as e:
            error_response = {
                "id": command_id if 'command_id' in locals() else f"cmd_err_{int(time.time()*1000)}",
                "command": command_str,
                "success": False,
                "error": str(e),
                "timestamp": time.time()
            }
            with lock:
                self.client.publish(
                    self.topic_result,
                    json.dumps(error_response, ensure_ascii=False),
                    qos=0
                )
            self.get_logger().error(f"[命令执行] 错误: {str(e)}")

    def execute_command(self, command, timeout=60):
        try:
            self.get_logger().info(f"[执行命令] 执行: {command}")
            self.get_logger().info(f"[执行命令] 超时: {timeout}秒")
            
            process = subprocess.Popen(
                command,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                universal_newlines=True
            )
            
            stdout, stderr = process.communicate(timeout=timeout)
            
            result = {
                "stdout": stdout.strip(),
                "stderr": stderr.strip(),
                "returncode": process.returncode
            }
            
            self.get_logger().info(f"[执行命令] 完成, 返回码: {process.returncode}")
            if stdout.strip():
                self.get_logger().info(f"[执行命令] stdout: {stdout.strip()}")
            if stderr.strip():
                self.get_logger().warn(f"[执行命令] stderr: {stderr.strip()}")
                
            return result
            
        except subprocess.TimeoutExpired:
            self.get_logger().error(f"[执行命令] 超时, 终止进程")
            process.kill()
            stdout, stderr = process.communicate()
            return {
                "stdout": stdout.strip(),
                "stderr": f"命令执行超时 ({timeout}秒)\n{stderr.strip()}",
                "returncode": -1
            }
        except Exception as e:
            error_msg = f"执行命令时发生异常: {str(e)}"
            self.get_logger().error(f"[执行命令] 异常: {error_msg}")
            return {
                "stdout": "",
                "stderr": error_msg,
                "returncode": -1
            }

    # =========================================================
    # MQTT连接 + 启动 + 关闭 核心方法
    # =========================================================
    def connect_mqtt(self):
        """MQTT连接 + 前置网络检测"""
        keepalive = 60
        while not check_network(self.broker):
            self.get_logger().warn(f"[网络检测] 网络不可用或Broker({self.broker})不可达，3秒后重试...")
            time.sleep(3)
        
        self.get_logger().info(f"\n⏳ 网络已连通，尝试连接MQTT服务器: {self.broker}:{self.port}")
        
        try:
            self.client.connect(self.broker, self.port, keepalive)
            self.client.loop_start()
            self.get_logger().info("✅ MQTT连接成功!")
            self.get_logger().info(f"  ├─ 客户端ID: {self.client_id}")
            self.get_logger().info(f"  ├─ 控制主题: {self.topic_cmd} (ROS2指令)")
            self.get_logger().info(f"  ├─ 命令主题: {self.topic_command} (终端命令)")
            self.get_logger().info(f"  ├─ 结果主题: {self.topic_result} (执行结果)")
            self.get_logger().info(f"  ├─ 状态主题: {self.topic_status} (ROS2状态)")
            self.get_logger().info(f"  └─ 停靠状态主题: {self.topic_dock_status} (ROS2停靠状态)")
            self.get_logger().info("=" * 50)
            return True
        except Exception as e:
            self.get_logger().error(f"❌ 连接Broker失败: {str(e)}，3秒后重试...")
            time.sleep(3)
            return self.connect_mqtt()

    def publish_mqtt_msg(self, topic, payload, qos=0):
        """封装MQTT发布方法"""
        with lock:
            self.client.publish(topic, payload, qos)

    def ros_robot_state_callback(self, msg):
        """ROS2回调: 监听/robot_state 话题 转发到MQTT (替代原rospy回调)"""
        self.get_logger().info(f"[ROS2] 收到robot_state: {msg.data}")
        self.publish_mqtt_msg(self.topic_status, msg.data)

    def ros_dock_state_callback(self, msg):
        """ROS2回调: 监听/dock_state 话题 转发到MQTT (替代原rospy回调)"""
        self.get_logger().info(f"[ROS2] 收到dock_state: {msg.data}")
        self.publish_mqtt_msg(self.topic_dock_status, msg.data)

    def stop_bridge(self):
        """关闭桥接 释放资源"""
        self.get_logger().info("\n🛑 开始断开连接...")
        self.client.disconnect()
        self.client.loop_stop()
        self.get_logger().info("✅ MQTT已断开连接，节点关闭完成")

# =========================================================
# ROS2 程序主入口 (核心替换 rospy的main)
# =========================================================
def main(args=None):
    # ROS2节点初始化
    rclpy.init(args=args)
    # 创建桥接节点实例
    mqtt_ros2_bridge = MQTTRos2Bridge()
    
    try:
        # 连接MQTT并启动
        if mqtt_ros2_bridge.connect_mqtt():
            mqtt_ros2_bridge.get_logger().info("🚀 MQTT-ROS2桥接节点运行中 (CTRL+C 退出)...")
            mqtt_ros2_bridge.get_logger().info(f"📨 发送终端命令到MQTT主题: {mqtt_ros2_bridge.topic_command}")
            mqtt_ros2_bridge.get_logger().info(f"📩 接收命令结果从MQTT主题: {mqtt_ros2_bridge.topic_result}")
            rclpy.spin(mqtt_ros2_bridge) # ROS2自旋 阻塞运行
    except KeyboardInterrupt:
        # CTRL+C 优雅退出
        mqtt_ros2_bridge.stop_bridge()
    except Exception as e:
        mqtt_ros2_bridge.get_logger().error(f"运行异常: {str(e)}")
        mqtt_ros2_bridge.stop_bridge()
    finally:
        # 销毁节点 关闭ROS2
        mqtt_ros2_bridge.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()