#!/bin/bash
set -o pipefail
# export ROS_LOCALHOST_ONLY=1
# GPIO_3控制开启RTK
gpioset gpiochip3 3=0

gpioset gpiochip3 2=0
# 指定需要ping通的目标地址，可根据需要修改
TARGET_IP="121.40.57.48"

# 循环ping目标地址，直到ping通为止
echo "等待目标地址 $TARGET_IP 可达..."
while ! ping -c 1 -W 2 $TARGET_IP > /dev/null 2>&1; do
    sleep 1
done
echo "已成功ping通 $TARGET_IP"
# 日志按实际日期滚动；单文件达到 10 MiB 后切分为 runYYYYMMDD.1.log、.2.log。
# 可通过 systemd Environment=MOTOR_START_LOG_MAX_BYTES=... 覆盖分卷大小。
source /opt/ros/humble/setup.bash && source /home/ztl/robot_cleaning/install/setup.bash && ros2 launch rtk_nav run.launch.py 2>&1 | /usr/bin/python3 /home/ztl/robot_cleaning/rotate_motor_start_log.py
