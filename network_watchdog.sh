#!/bin/bash
# 网络看门狗：检测 usb0 RNDIS 假活并自动重置接口
#   假活特征：链路 UP、网关可达、但出口不通
#   恢复手段：down/up usb0 + DHCP 续租，触发手机重建数据连接

TARGET_IP="121.40.57.48"
INTERFACE="usb0"
MAX_FAIL=3
CHECK_INTERVAL=30
LOG_TAG="net-watchdog"

log() { echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1"; }

wait_for_ip() {
    # 等待接口获取 IP，最多等 15 秒
    local waited=0
    while [ "$waited" -lt 15 ]; do
        if ip addr show "$INTERFACE" | grep -q 'inet\s'; then
            return 0
        fi
        sleep 1
        waited=$((waited + 1))
    done
    return 1
}

fail_count=0

while true; do
    # 检查接口是否存在且 UP（RNDIS 的 state 是 UNKNOWN，用 flags 中的 ,UP 判断）
    if ! ip link show "$INTERFACE" 2>/dev/null | grep -q ",UP" ; then
        log "$INTERFACE 接口不存在或未 UP，跳过本轮"
        sleep "$CHECK_INTERVAL"
        continue
    fi

    if ping -c 2 -W 3 "$TARGET_IP" > /dev/null 2>&1; then
        if [ "$fail_count" -gt 0 ]; then
            log "网络恢复，fail_count=$fail_count → 0"
        fi
        fail_count=0
    else
        fail_count=$((fail_count + 1))
        log "ping $TARGET_IP 失败 (fail_count=$fail_count/$MAX_FAIL)"

        if [ "$fail_count" -ge "$MAX_FAIL" ]; then
            log "连续 $MAX_FAIL 次失败，重置 $INTERFACE"

            # 记录当前 IP，用于对比
            OLD_IP=$(ip addr show "$INTERFACE" | grep -oP 'inet \K[\d.]+' | head -1)
            log "当前 IP: ${OLD_IP:-无}"

            # Step 1: 释放 DHCP 租约
            dhclient -r "$INTERFACE" 2>/dev/null || true
            sleep 1

            # Step 2: down/up 接口
            ip link set "$INTERFACE" down
            sleep 2
            ip link set "$INTERFACE" up
            sleep 2

            # Step 3: 重新获取 DHCP
            dhclient "$INTERFACE" 2>/dev/null || true
            sleep 2

            # Step 4: 等 IP 到位
            if wait_for_ip; then
                NEW_IP=$(ip addr show "$INTERFACE" | grep -oP 'inet \K[\d.]+' | head -1)
                log "新 IP: $NEW_IP"
            else
                log "DHCP 超时，接口未获取 IP"
            fi

            # Step 5: 验证
            sleep 2
            if ping -c 3 -W 5 "$TARGET_IP" > /dev/null 2>&1; then
                log "重置成功，网络已恢复"
            else
                log "重置后仍不通，等待下次检测"
            fi
            fail_count=0
        fi
    fi

    sleep "$CHECK_INTERVAL"
done
