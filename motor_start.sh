#!/bin/bash
export ROS_LOCALHOST_ONLY=1
# 指定需要ping通的目标地址，可根据需要修改
TARGET_IP="121.40.57.48"

# 循环ping目标地址，直到ping通为止
echo "等待目标地址 $TARGET_IP 可达..."
while ! ping -c 1 -W 2 $TARGET_IP > /dev/null 2>&1; do
    sleep 1
done
echo "已成功ping通 $TARGET_IP"
source /home/ztl/robot_cleaning/install/setup.bash && ros2 launch rtk_nav run.launch.py >> /home/ztl/robot_cleaning/motor_start_log/run$(date +%Y%m%d).log 2>&1
