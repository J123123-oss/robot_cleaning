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
ROUTE_METRIC="${WWAN_TARGET_ROUTE_METRIC:-50}"
ROUTE_FAIL_MAX="${WWAN_ROUTE_FAIL_MAX:-$MAX_FAIL}"
FRPC_SERVICE="${WWAN_FRPC_SERVICE:-frpc.service}"
FRPC_RESTART_COOLDOWN="${WWAN_FRPC_RESTART_COOLDOWN:-10}"
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

get_gateway() {
    local iface="$1" gateway
    gateway=$(ip route show default dev "$iface" 2>/dev/null | awk 'NR == 1 {for (i = 1; i <= NF; i++) if ($i == "via") {print $(i + 1); exit}}')
    if [ -z "$gateway" ]; then
        gateway=$(nmcli -g IP4.GATEWAY device show "$MODEM_DEVICE" 2>/dev/null | tail -n 1)
    fi
    [ -n "$gateway" ] && [ "$gateway" != "--" ] || return 1
    printf '%s\n' "$gateway"
}

get_target_route() {
    ip -4 route show "$TARGET_IP/32" 2>/dev/null | head -n 1
}

ensure_target_route() {
    local iface="$1" ip gateway
    ip=$(get_ip "$iface")
    [ -n "$ip" ] || return 1
    gateway=$(get_gateway "$iface") || return 1
    ip route replace "$TARGET_IP/32" via "$gateway" dev "$iface" src "$ip" metric "$ROUTE_METRIC"
}

remove_target_route() {
    ip route del "$TARGET_IP/32" 2>/dev/null || true
}

restart_frpc() {
    local reason="$1" now
    now=$(date +%s)

    if ! systemctl is-active --quiet "$FRPC_SERVICE"; then
        return 0
    fi

    if [ "$last_frpc_restart" -ne 0 ] && [ $((now - last_frpc_restart)) -lt "$FRPC_RESTART_COOLDOWN" ]; then
        log "跳过 FRP 重启，仍在冷却时间内: reason=$reason"
        return 0
    fi

    log "FRP 路径发生变化，重建 $FRPC_SERVICE: reason=$reason"
    if systemctl restart "$FRPC_SERVICE"; then
        last_frpc_restart="$now"
    else
        log "重建 $FRPC_SERVICE 失败"
    fi
}

install_target_route() {
    local iface="$1" before after
    before=$(get_target_route)
    if ! ensure_target_route "$iface"; then
        return 1
    fi

    after=$(get_target_route)
    if [ "$before" != "$after" ]; then
        restart_frpc "4G 路由更新"
    fi
}

remove_target_route_for_failover() {
    local before after
    before=$(get_target_route)
    remove_target_route
    after=$(get_target_route)
    if [ "$before" != "$after" ]; then
        restart_frpc "4G 不可用，回退系统默认路由"
    fi
}

probe_tcp_on_interface() {
    local iface="$1" ip="$2"
    python3 -c 'import socket, sys
iface, source, target, port = sys.argv[1:]
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.settimeout(4)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BINDTODEVICE, iface.encode() + b"\0")
sock.bind((source, 0))
sock.connect((target, int(port)))
sock.close()' "$iface" "$ip" "$TARGET_IP" "$TARGET_PORT" >/dev/null 2>&1
}

probe() {
    local iface ip
    iface=$(get_data_interface) || return 1
    ip=$(get_ip "$iface")
    [ -n "$ip" ] || return 1
    ping -c 1 -W 3 -I "$iface" "$TARGET_IP" >/dev/null 2>&1 || return 1
    probe_tcp_on_interface "$iface" "$ip"
}

wait_for_ready() {
    local waited=0 iface ip
    while [ "$waited" -lt "$RECOVERY_WAIT" ]; do
        iface=$(get_data_interface 2>/dev/null || true)
        if [ -n "$iface" ]; then
            ip=$(get_ip "$iface")
            if [ -n "$ip" ] && ping -c 1 -W 3 -I "$iface" "$TARGET_IP" >/dev/null 2>&1 && probe_tcp_on_interface "$iface" "$ip"; then
                install_target_route "$iface" || log "4G 已恢复，但无法安装 FRP 目标动态路由"
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
remove_target_route
last_frpc_restart=0
fail_count=0
recovery_failures=0
last_state="unknown"

while true; do
    if probe; then
        iface=$(get_data_interface 2>/dev/null || true)
        if [ -n "$iface" ] && ! install_target_route "$iface"; then
            log "4G 探测成功，但无法安装 FRP 目标动态路由"
        fi
        if [ "$last_state" != "healthy" ]; then
            ip=$(get_ip "$iface" 2>/dev/null || true)
            log "4G 网络正常，FRP 目标路由走当前 4G: profile=$PROFILE, iface=${iface:-无}, ip=${ip:-无}, target=$TARGET_IP:$TARGET_PORT"
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

        if [ "$fail_count" -ge "$ROUTE_FAIL_MAX" ]; then
            remove_target_route_for_failover
            log "4G 目标探测失败，移除专用路由，回退系统默认路由"
        fi

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
