#!/bin/bash
# start can1
# ip link set can1 down
nmcli r wifi on
ip link set can1 type can bitrate 1000000 sample-point 0.8 dbitrate 2000000 sample-point 0.8 fd on
ip link set can1 up
# 设置can1发送队列长度
sh -c 'echo 4096 > /sys/class/net/can1/tx_queue_len'
# start ch340
# insmod /home/forlinx/robot_cleaning/ch341.ko
if ! pgrep -x MediaServer >/dev/null 2>&1; then
    /home/forlinx/ZLMediaKit/release/linux/Release/MediaServer -d >/home/forlinx/ZLMediaKit/release/linux/Release/mediaserver-start.log 2>&1 &
fi
chmod +x /home/forlinx/robot_cleaning/motor_start.sh
