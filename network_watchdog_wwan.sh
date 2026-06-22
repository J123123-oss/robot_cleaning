#!/bin/bash
# 网络看门狗 (wwan0/4G版 - Quectel EC25 + quectel-CM)
#   假活特征：接口 UP、有 IP、但出口不通
#   恢复手段：杀 quectel-CM → down wwan0 → 重启 quectel-CM → 等 IP → 验证

TARGET_IP="121.40.57.48"
INTERFACE="wwan0"
MAX_FAIL=3
CHECK_INTERVAL=30
QUECTEL_CM="/usr/local/bin/quectel-CM"

log() { echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1"; }

# 等待接口获取 IP
wait_for_ip() {
    local waited=0
    while [ "$waited" -lt 15 ]; do
        if ip addr show "$INTERFACE" 2>/dev/null | grep -q 'inet\s'; then
            return 0
        fi
        sleep 1
        waited=$((waited + 1))
    done
    return 1
}

# 恢复流程：杀掉 quectel-CM，down 接口，重新拨号
do_recovery() {
    log "杀掉所有 quectel-CM 进程"
    pkill -f "quectel-CM" 2>/dev/null || true
    sleep 2

    # 确认死透
    if pgrep -f "quectel-CM" > /dev/null; then
        log "强制杀掉残留进程"
        pkill -9 -f "quectel-CM" 2>/dev/null || true
        sleep 1
    fi

    log "down $INTERFACE"
    ip link set "$INTERFACE" down 2>/dev/null || true
    sleep 1

    log "启动 quectel-CM 重新拨号"
    "$QUECTEL_CM" &
    sleep 5

    if wait_for_ip; then
        NEW_IP=$(ip addr show "$INTERFACE" | grep -oP 'inet \K[\d.]+' | head -1)
        log "拨号成功，IP: $NEW_IP"
        return 0
    else
        log "等待 IP 超时"
        return 1
    fi
}

log "看门狗启动，监控 $INTERFACE → $TARGET_IP (间隔=${CHECK_INTERVAL}s, 阈值=${MAX_FAIL}次)"

fail_count=0

while true; do
    if ! ip link show "$INTERFACE" 2>/dev/null | grep -q ",UP" ; then
        # 接口 DOWN，也计入失败（可能是 quectel-CM 挂了）
        fail_count=$((fail_count + 1))
        log "$INTERFACE 未 UP (fail_count=$fail_count/$MAX_FAIL)"

        if [ "$fail_count" -ge "$MAX_FAIL" ]; then
            log "接口持续 DOWN，尝试重启 quectel-CM"
            do_recovery
            fail_count=0
        fi
        sleep "$CHECK_INTERVAL"
        continue
    fi

    if ping -c 2 -W 3 -I "$INTERFACE" "$TARGET_IP" > /dev/null 2>&1; then
        if [ "$fail_count" -gt 0 ]; then
            log "网络恢复，fail_count=$fail_count → 0"
        fi
        fail_count=0
    else
        fail_count=$((fail_count + 1))
        OLD_IP=$(ip addr show "$INTERFACE" 2>/dev/null | grep -oP 'inet \K[\d.]+' | head -1)
        log "ping $TARGET_IP 失败，IP: ${OLD_IP:-无} (fail_count=$fail_count/$MAX_FAIL)"

        if [ "$fail_count" -ge "$MAX_FAIL" ]; then
            log "开始恢复流程"
            do_recovery
            if ping -c 3 -W 5 -I "$INTERFACE" "$TARGET_IP" > /dev/null 2>&1; then
                log "恢复成功，网络已通"
            else
                log "恢复后仍不通，等待下次检测"
            fi
            fail_count=0
        fi
    fi

    sleep "$CHECK_INTERVAL"
done
