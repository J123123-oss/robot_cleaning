#!/bin/bash
# start can0
# ip link set can0 down
nmcli r wifi on
ip link set can0 type can bitrate 1000000 sample-point 0.8 dbitrate 2000000 sample-point 0.8 fd on
ip link set can0 up
# 设置CAN0发送队列长度
sh -c 'echo 4096 > /sys/class/net/can0/tx_queue_len'
# start ch340
insmod /home/ztl/robot_cleaning/ch341.ko
chmod +x /home/ztl/robot_cleaning/motor_start.sh
