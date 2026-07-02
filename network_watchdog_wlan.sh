#!/bin/bash
# 网络看门狗 (wlan0/WiFi版)
#   假活特征：接口 UP、有 IP、但出口不通
#   恢复手段（逐级升级）：
#     L1: nmcli device wifi rescan → 重新扫描 AP
#     L2: nmcli connection up → 用已保存 profile 重连
#     L3: ip link down/up → 内核层重置接口
#     L4: rfkill block/unblock → 硬件层重置（最后手段）

TARGET_IP="121.40.57.48"
INTERFACE="wlan0"
MAX_FAIL=3
CHECK_INTERVAL=30

# 各级恢复的冷却计数（避免频繁核弹级恢复）
L4_COOLDOWN=6       # 6 轮 × 30s = 3 分钟后才允许再次 L4
l4_cooldown_remain=0

log() { echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1"; }

get_ip() {
    ip addr show "$INTERFACE" 2>/dev/null | grep -oP 'inet \K[\d.]+' | head -1
}

get_ssid() {
    nmcli -t -f GENERAL.CONNECTION device show "$INTERFACE" 2>/dev/null | cut -d: -f2
}

# 等待 WiFi 完全就绪：接口 UP → 已关联 SSID → 拿到 IP → ping 通
# 参数: $1 = 最大等待秒数 (默认 60)
wait_for_wifi_ready() {
    local max_wait="${1:-60}"
    local waited=0

    log "等待 WiFi 就绪 (最多 ${max_wait}s)..."

    # Step 1: 等接口 UP
    while [ "$waited" -lt "$max_wait" ]; do
        if ip link show "$INTERFACE" 2>/dev/null | grep -q ",UP" ; then
            log "接口 $INTERFACE 已 UP (+${waited}s)"
            break
        fi
        sleep 2
        waited=$((waited + 2))
    done

    if [ "$waited" -ge "$max_wait" ]; then
        log "超时: 接口未 UP"
        return 1
    fi

    # Step 2: 等 NM 关联 AP（SSID 非空）
    while [ "$waited" -lt "$max_wait" ]; do
        local ssid
        ssid=$(get_ssid)
        if [ -n "$ssid" ] && [ "$ssid" != "--" ]; then
            log "已关联 SSID: $ssid (+${waited}s)"
            break
        fi
        sleep 2
        waited=$((waited + 2))
    done

    if [ "$waited" -ge "$max_wait" ]; then
        log "超时: 未关联 AP"
        return 1
    fi

    # Step 3: 等拿到 IP
    while [ "$waited" -lt "$max_wait" ]; do
        local ip
        ip=$(get_ip)
        if [ -n "$ip" ]; then
            log "已获取 IP: $ip (+${waited}s)"
            break
        fi
        sleep 2
        waited=$((waited + 2))
    done

    if [ "$waited" -ge "$max_wait" ]; then
        log "超时: 未获取 IP"
        return 1
    fi

    return 0
}

# L1: 软恢复 —— 重新扫描 WiFi
do_l1_recovery() {
    log "[L1] 触发 WiFi 重新扫描"
    nmcli device wifi rescan 2>/dev/null || true
    # rescan 只是触发扫描，NM 需要时间处理结果
    wait_for_wifi_ready 30
}

# L2: 中恢复 —— 用已保存的连接配置重新激活
do_l2_recovery() {
    local conn_name
    conn_name=$(nmcli -t -f GENERAL.CONNECTION device show "$INTERFACE" 2>/dev/null | cut -d: -f2)

    log "[L2] 断开 $INTERFACE (连接: ${conn_name:-未知})"
    nmcli device disconnect "$INTERFACE" 2>/dev/null || true
    sleep 2

    if [ -n "$conn_name" ]; then
        log "[L2] 使用已保存配置重新连接: $conn_name"
        nmcli connection up "$conn_name" 2>/dev/null || true
    else
        log "[L2] 无已保存连接，由 NM 自动连接"
        nmcli device connect "$INTERFACE" 2>/dev/null || true
    fi

    wait_for_wifi_ready 30
}

# L3: 硬恢复 —— 内核层 down/up
do_l3_recovery() {
    log "[L3] down/up $INTERFACE"
    ip link set "$INTERFACE" down 2>/dev/null || true
    sleep 3
    ip link set "$INTERFACE" up 2>/dev/null || true
    # down/up 后 NM 需要重新扫描和关联，给够时间
    sleep 3
    nmcli device wifi rescan 2>/dev/null || true
    wait_for_wifi_ready 60
}

# L4: 核恢复 —— rfkill 硬件重置
do_l4_recovery() {
    if [ "$l4_cooldown_remain" -gt 0 ]; then
        log "[L4] 冷却中 (剩余 $l4_cooldown_remain 轮)，跳过，回退到 L3"
        do_l3_recovery
        return
    fi

    log "[L4] rfkill 硬件重置 WiFi"
    rfkill block wifi 2>/dev/null || true
    sleep 3
    rfkill unblock wifi 2>/dev/null || true

    # rfkill 后驱动需要重新加载，给够时间
    sleep 5
    nmcli device wifi rescan 2>/dev/null || true
    wait_for_wifi_ready 90

    l4_cooldown_remain=$L4_COOLDOWN
}

# 验证恢复结果
verify() {
    if ping -c 3 -W 5 -I "$INTERFACE" "$TARGET_IP" > /dev/null 2>&1; then
        log "恢复成功，网络已通"
        return 0
    else
        local ip ssid
        ip=$(get_ip)
        ssid=$(get_ssid)
        log "恢复后仍不通 (SSID: ${ssid:-无}, IP: ${ip:-无})"
        return 1
    fi
}

log "看门狗启动，监控 $INTERFACE → $TARGET_IP (间隔=${CHECK_INTERVAL}s, 阈值=${MAX_FAIL}次)"

fail_count=0
level=1

while true; do
    # 冷却递减
    if [ "$l4_cooldown_remain" -gt 0 ]; then
        l4_cooldown_remain=$((l4_cooldown_remain - 1))
    fi

    # 检查接口是否存在且 UP
    if ! ip link show "$INTERFACE" 2>/dev/null | grep -q ",UP" ; then
        fail_count=$((fail_count + 1))
        SSID=$(get_ssid)
        log "$INTERFACE 未 UP，SSID: ${SSID:-无} (fail_count=$fail_count/$MAX_FAIL)"

        if [ "$fail_count" -ge "$MAX_FAIL" ]; then
            log "接口持续 DOWN，尝试 L3 恢复"
            do_l3_recovery
            verify || log "L3 未恢复，下次尝试升级"
            fail_count=0
            level=1
        fi
        sleep "$CHECK_INTERVAL"
        continue
    fi

    # 尝试 ping 目标
    if ping -c 2 -W 3 -I "$INTERFACE" "$TARGET_IP" > /dev/null 2>&1; then
        if [ "$fail_count" -gt 0 ]; then
            log "网络恢复，fail_count=$fail_count → 0"
        fi
        fail_count=0
        level=1
    else
        fail_count=$((fail_count + 1))
        OLD_IP=$(get_ip)
        SSID=$(get_ssid)
        log "ping $TARGET_IP 失败，SSID: ${SSID:-无}, IP: ${OLD_IP:-无} (fail_count=$fail_count/$MAX_FAIL, level=$level)"

        if [ "$fail_count" -ge "$MAX_FAIL" ]; then
            log "连续 $MAX_FAIL 次失败，开始恢复流程 (当前等级: L$level)"

            case $level in
                1) do_l1_recovery ;;
                2) do_l2_recovery ;;
                *) do_l3_recovery ;;
            esac

            if verify; then
                level=1
            else
                # 逐级升级
                case $level in
                    1) level=2 ; log "L1 未恢复，下次升级至 L2" ;;
                    2) level=3 ; log "L2 未恢复，下次升级至 L3" ;;
                    *) log "L3 未恢复，尝试 L4（如有冷却则回退 L3）"
                       do_l4_recovery
                       verify && level=1 || log "L4 仍未恢复，继续监控"
                       level=1 ;;
                esac
            fi
            fail_count=0
        fi
    fi

    sleep "$CHECK_INTERVAL"
done
