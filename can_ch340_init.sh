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
# 待修改：降低eth0优先级或者暂时删除
ip route del default via 192.168.3.1 dev eth0

# ===================== 设置 WiFi 优先（自适应网关，永不报错） =====================
# 等待网络完全就绪
sleep 10

# 循环等待 wwan0 拿到默认路由（最多等100次）
for i in {1..100}; do
    GW=$(ip route show default dev wwan0 2>/dev/null | awk '{print $3}')
    if [ -n "$GW" ]; then
        ip route del default dev wwan0 2>/dev/null
        ip route add default via $GW dev wwan0 metric 200
        break
    fi
    sleep 1
done