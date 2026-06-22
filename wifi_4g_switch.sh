#!/bin/bash

# 等待 4G 网卡 wwan0 获取 IP 地址（最多等60秒，避免卡死）
MAX_WAIT=60
WAIT_TIME=0

echo "等待 4G(wwan0) 拨号获取IP..."
while ! ip addr show wwan0 | grep -q 'inet\s'; do
    sleep 1
    WAIT_TIME=$((WAIT_TIME + 1))
    if [ $WAIT_TIME -ge $MAX_WAIT ]; then
        echo "4G 超时未获取IP，仅设置WiFi路由"
        break
    fi
done

# 清理所有默认路由，避免重复
ip route del default 2>/dev/null

# 设置 WiFi 优先（metric 100 最高优先级）
ip route add default via 192.168.137.1 dev wlan0 metric 100

# 正确获取 4G 网关并设置备用路由
if ip addr show wwan0 | grep -q 'inet\s'; then
    # 正确方法：直接从路由表读取 4G 网关地址
    WWAN_GW=$(ip route show 0.0.0.0/0 dev wwan0 2>/dev/null | awk '/default/ {print $3}')
    
    # 如果上面没读到，就从网段获取网关（4G点对点网关是网段第2个IP）
    if [ -z "$WWAN_GW" ]; then
        WWAN_GW=$(ip route show dev wwan0 | awk '/scope link/ {print $1}' | cut -d'/' -f1 | awk -F. '{print $1"."$2"."$3"."$4+1}')
    fi

    # 添加 4G 默认路由，metric 200 备用
    ip route add default via $WWAN_GW dev wwan0 metric 200
    echo "已设置：WiFi优先(100)，4G备用(200) | 4G网关：$WWAN_GW"
else
    echo "已设置：仅WiFi优先"
fi