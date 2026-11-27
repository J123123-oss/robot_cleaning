#!/bin/bash
set -u

PROFILE="${WWAN_PROFILE:-4G-CMNET}"
MODEM_DEVICE="${WWAN_MODEM_DEVICE:-cdc-wdm0}"
TARGET_IP="${WWAN_TARGET_IP:-121.40.57.48}"
TARGET_PORT="${WWAN_TARGET_PORT:-7000}"
CHECK_INTERVAL="${WWAN_CHECK_INTERVAL:-15}"
MAX_FAIL="${WWAN_MAX_FAIL:-3}"
RECOVERY_WAIT="${WWAN_RECOVERY_WAIT:-90}"
RECOVERY_ATTEMPTS="${WWAN_RECOVERY_ATTEMPTS:-2}"
MODEM_RESET_AFTER="${WWAN_MODEM_RESET_AFTER:-3}"

log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1"
}

get_data_interface() {
    local iface
    iface=$(nmcli -g GENERAL.IP-IFACE device show "$MODEM_DEVICE" 2>/dev/null | tail -n 1)
    if [ -n "$iface" ] && [ "$iface" != "--" ]; then
        printf '%s\n' "$iface"
        return 0
    fi

    iface=$(nmcli -t -f DEVICE,TYPE device status 2>/dev/null | awk -F: '$2 == "gsm" {print $1; exit}')
    if [ -n "$iface" ] && [ "$iface" != "--" ]; then
        printf '%s\n' "$iface"
        return 0
    fi

    return 1
}

get_ip() {
    local iface="$1"
    ip -4 -o addr show dev "$iface" 2>/dev/null | awk '{split($4, parts, "/"); print parts[1]; exit}'
}

has_route() {
    local iface="$1"
    ip route get "$TARGET_IP" oif "$iface" 2>/dev/null | grep -q "dev $iface"
}

probe() {
    local iface ip
    iface=$(get_data_interface) || return 1
    ip=$(get_ip "$iface")
    [ -n "$ip" ] || return 1
    has_route "$iface" || return 1
    ping -c 1 -W 3 -I "$iface" "$TARGET_IP" >/dev/null 2>&1 || return 1
    timeout 4 bash -c "</dev/tcp/${TARGET_IP}/${TARGET_PORT}" >/dev/null 2>&1
}

wait_for_ready() {
    local waited=0 iface ip
    while [ "$waited" -lt "$RECOVERY_WAIT" ]; do
        iface=$(get_data_interface 2>/dev/null || true)
        if [ -n "$iface" ]; then
            ip=$(get_ip "$iface")
            if [ -n "$ip" ] && has_route "$iface" && ping -c 1 -W 3 -I "$iface" "$TARGET_IP" >/dev/null 2>&1 && timeout 4 bash -c "</dev/tcp/${TARGET_IP}/${TARGET_PORT}" >/dev/null 2>&1; then
                log "4G 恢复成功: profile=$PROFILE, iface=$iface, ip=$ip, target=$TARGET_IP:$TARGET_PORT"
                return 0
            fi
        fi
        sleep 3
        waited=$((waited + 3))
    done
    return 1
}

activate_profile() {
    nmcli connection up id "$PROFILE" >/dev/null 2>&1
}

reset_modem() {
    local udi modem_id
    udi=$(nmcli -g GENERAL.UDI device show "$MODEM_DEVICE" 2>/dev/null | tail -n 1)
    modem_id="${udi##*/}"
    if [ -z "$modem_id" ] || [ "$modem_id" = "--" ] || [ "$modem_id" = "$udi" ]; then
        log "无法确定 ModemManager 模块编号，跳过模块复位"
        return 1
    fi

    log "连续恢复失败，复位 ModemManager 模块: $modem_id"
    mmcli -m "$modem_id" --reset >/dev/null 2>&1
}

recover() {
    local attempt
    for attempt in $(seq 1 "$RECOVERY_ATTEMPTS"); do
        log "第 $attempt/$RECOVERY_ATTEMPTS 次重新激活 $PROFILE"
        activate_profile || true
        if wait_for_ready; then
            return 0
        fi

        if [ "$attempt" -lt "$RECOVERY_ATTEMPTS" ]; then
            nmcli connection down id "$PROFILE" >/dev/null 2>&1 || true
            sleep 3
        fi
    done
    return 1
}

log "WWAN 看门狗启动: profile=$PROFILE, modem=$MODEM_DEVICE, target=$TARGET_IP:$TARGET_PORT, interval=${CHECK_INTERVAL}s"
fail_count=0
recovery_failures=0
last_state="unknown"

while true; do
    if probe; then
        if [ "$last_state" != "healthy" ]; then
            iface=$(get_data_interface 2>/dev/null || true)
            ip=$(get_ip "$iface" 2>/dev/null || true)
            log "4G 网络正常: profile=$PROFILE, iface=${iface:-无}, ip=${ip:-无}, target=$TARGET_IP:$TARGET_PORT"
            last_state="healthy"
        fi
        fail_count=0
        recovery_failures=0
    else
        fail_count=$((fail_count + 1))
        iface=$(get_data_interface 2>/dev/null || true)
        ip=$(get_ip "$iface" 2>/dev/null || true)
        state="$(nmcli -g GENERAL.STATE device show "$MODEM_DEVICE" 2>/dev/null | tail -n 1)"
        log "4G 检查失败: state=${state:-未知}, iface=${iface:-无}, ip=${ip:-无}, fail_count=$fail_count/$MAX_FAIL"
        last_state="unhealthy"

        if [ "$fail_count" -ge "$MAX_FAIL" ]; then
            log "连续 $MAX_FAIL 次失败，开始通过 NetworkManager 恢复 $PROFILE"
            if recover; then
                fail_count=0
                recovery_failures=0
                last_state="healthy"
            else
                recovery_failures=$((recovery_failures + 1))
                log "NetworkManager 恢复失败: recovery_failures=$recovery_failures/$MODEM_RESET_AFTER"
                if [ "$recovery_failures" -ge "$MODEM_RESET_AFTER" ]; then
                    if reset_modem; then
                        sleep 10
                        if wait_for_ready; then
                            fail_count=0
                            recovery_failures=0
                            last_state="healthy"
                        fi
                    fi
                    recovery_failures=0
                fi
                fail_count=0
            fi
        fi
    fi

    sleep "$CHECK_INTERVAL"
done
