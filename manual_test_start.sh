#!/bin/bash
# 手动测试脚本：开机 60s 后后台运行
sleep 60
source /opt/ros/humble/setup.bash
source /home/ztl/robot_cleaning/install/setup.bash
python3 /home/ztl/robot_cleaning/src/motor_control/motor_control/manual_test.py >> /home/ztl/robot_cleaning/motor_start_log/manual_test_$(date +%Y%m%d).log 2>&1
