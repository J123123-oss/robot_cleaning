#!/usr/bin/env python3
"""Keep the configured Wi-Fi and Quectel LTE links recoverable via NetworkManager."""

from __future__ import annotations

import logging
import os
import socket
import subprocess
import time


LOG = logging.getLogger("network_failover_watchdog")


def env_int(name: str, default: int) -> int:
    try:
        return int(os.environ.get(name, default))
    except (TypeError, ValueError):
        LOG.warning("Invalid %s; using %d", name, default)
        return default


class NetworkFailoverWatchdog:
    def __init__(self) -> None:
        self.target_ip = os.environ.get("NETWORK_PROBE_IP", "121.40.57.48")
        self.target_port = env_int("NETWORK_PROBE_PORT", 1883)
        self.wifi_interface = os.environ.get("WIFI_INTERFACE", "wlan0")
        self.wwan_interface = os.environ.get("WWAN_INTERFACE", "wwan0")
        self.wifi_profile = os.environ.get("WIFI_PROFILE", "GiiFen-5G")
        self.wwan_profile = os.environ.get("WWAN_PROFILE", "4G-CMNET")
        self.check_interval = max(3, env_int("CHECK_INTERVAL_SEC", 10))
        self.failure_threshold = max(1, env_int("FAILURE_THRESHOLD", 3))
        self.success_threshold = max(1, env_int("SUCCESS_THRESHOLD", 3))
        self.force_wifi_cycle_threshold = max(
            self.failure_threshold,
            env_int("FORCE_WIFI_CYCLE_THRESHOLD", 6),
        )
        self.recovery_cooldown = max(10, env_int("RECOVERY_COOLDOWN_SEC", 60))
        self.connect_timeout = max(1, env_int("PROBE_TIMEOUT_SEC", 3))
        self.wifi_preferred_metric = env_int("WIFI_PREFERRED_METRIC", 600)
        self.wifi_standby_metric = env_int("WIFI_STANDBY_METRIC", 800)

        self.wifi_failures = 0
        self.wifi_successes = 0
        self.wwan_failures = 0
        self.last_wifi_recovery = -self.recovery_cooldown
        self.last_wwan_recovery = -self.recovery_cooldown
        self.last_health: tuple[bool, bool] | None = None
        self.applied_wifi_metric = self.profile_metric(self.wifi_profile)

    @staticmethod
    def run(args: list[str], timeout: int = 20) -> subprocess.CompletedProcess[str]:
        try:
            return subprocess.run(
                args,
                check=False,
                capture_output=True,
                text=True,
                timeout=timeout,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            LOG.warning("Command failed (%s): %s", " ".join(args), exc)
            return subprocess.CompletedProcess(args, 1, "", str(exc))

    def has_ipv4(self, interface: str) -> bool:
        result = self.run(["ip", "-4", "-o", "addr", "show", "dev", interface], timeout=5)
        return result.returncode == 0 and " inet " in result.stdout

    def tcp_probe(self, interface: str) -> bool:
        if not self.has_ipv4(interface):
            return False

        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                sock.setsockopt(
                    socket.SOL_SOCKET,
                    getattr(socket, "SO_BINDTODEVICE", 25),
                    interface.encode("ascii") + b"\0",
                )
                sock.settimeout(self.connect_timeout)
                sock.connect((self.target_ip, self.target_port))
            return True
        except (OSError, UnicodeError):
            return False

    def active_profile(self, interface: str) -> str:
        result = self.run(
            ["nmcli", "-g", "GENERAL.CONNECTION", "device", "show", interface],
            timeout=5,
        )
        value = result.stdout.strip()
        return "" if value in ("", "--", "(externally)") else value

    def profile_metric(self, profile: str) -> int | None:
        result = self.run(
            ["nmcli", "-g", "ipv4.route-metric", "connection", "show", profile],
            timeout=5,
        )
        try:
            return int(result.stdout.strip()) if result.returncode == 0 else None
        except ValueError:
            return None

    def set_wifi_metric(self, metric: int) -> bool:
        current = self.profile_metric(self.wifi_profile)
        changed = current != metric or self.applied_wifi_metric != metric
        if current != metric:
            result = self.run(
                [
                    "nmcli", "connection", "modify", self.wifi_profile,
                    "ipv4.route-metric", str(metric),
                    "ipv6.route-metric", str(metric),
                ],
                timeout=10,
            )
            if result.returncode != 0:
                LOG.error(
                    "Cannot set Wi-Fi profile %s route metric to %d: %s",
                    self.wifi_profile,
                    metric,
                    result.stderr.strip(),
                )
                return False

        if changed and self.active_profile(self.wifi_interface) == self.wifi_profile:
            result = self.run(["nmcli", "device", "reapply", self.wifi_interface], timeout=10)
            if result.returncode != 0:
                LOG.warning(
                    "Updated Wi-Fi profile metric to %d, but NetworkManager could not reapply it: %s",
                    metric,
                    result.stderr.strip(),
                )
                return False

        self.applied_wifi_metric = metric
        if changed:
            LOG.info("Wi-Fi route metric is %d; LTE standby metric is 700", metric)
        return True

    def activate_profile(self, profile: str, interface: str, timeout: int = 90) -> bool:
        result = self.run(
            ["nmcli", "connection", "up", profile, "ifname", interface],
            timeout=timeout,
        )
        if result.returncode == 0:
            LOG.info("Requested NetworkManager to activate %s on %s", profile, interface)
            return True
        LOG.warning(
            "NetworkManager could not activate %s on %s: %s",
            profile,
            interface,
            result.stderr.strip() or result.stdout.strip(),
        )
        return False

    def reconnect_wifi(self, force_cycle: bool) -> None:
        LOG.warning("Requesting Wi-Fi recovery for profile %s", self.wifi_profile)
        if force_cycle and self.active_profile(self.wifi_interface) == self.wifi_profile:
            down = self.run(["nmcli", "connection", "down", self.wifi_profile], timeout=20)
            if down.returncode != 0:
                LOG.warning("Could not disconnect Wi-Fi profile before retry: %s", down.stderr.strip())
            time.sleep(2)
        self.activate_profile(self.wifi_profile, self.wifi_interface, timeout=60)

    def reconnect_wwan(self) -> None:
        LOG.warning("Requesting LTE recovery for profile %s", self.wwan_profile)
        if self.active_profile("cdc-wdm0") == self.wwan_profile:
            down = self.run(["nmcli", "connection", "down", self.wwan_profile], timeout=25)
            if down.returncode != 0:
                LOG.warning("Could not disconnect LTE profile before retry: %s", down.stderr.strip())
            time.sleep(2)
        self.activate_profile(self.wwan_profile, "cdc-wdm0", timeout=90)

    def step(self) -> None:
        wifi_ok = self.tcp_probe(self.wifi_interface)
        wwan_ok = self.tcp_probe(self.wwan_interface)
        health = (wifi_ok, wwan_ok)
        if health != self.last_health:
            LOG.info(
                "MQTT TCP probe %s:%d: Wi-Fi=%s, LTE=%s",
                self.target_ip,
                self.target_port,
                "OK" if wifi_ok else "FAILED",
                "OK" if wwan_ok else "FAILED",
            )
            self.last_health = health

        if wifi_ok:
            self.wifi_successes += 1
            self.wifi_failures = 0
            self.last_wifi_recovery = -self.recovery_cooldown
            if self.wifi_successes >= self.success_threshold:
                self.set_wifi_metric(self.wifi_preferred_metric)
        else:
            self.wifi_successes = 0
            self.wifi_failures += 1
            if self.wifi_failures >= self.failure_threshold and wwan_ok:
                self.set_wifi_metric(self.wifi_standby_metric)

            now = time.monotonic()
            if (
                self.wifi_failures >= self.failure_threshold
                and now - self.last_wifi_recovery >= self.recovery_cooldown
            ):
                # Prefer a Wi-Fi cycle when LTE is confirmed as a fallback.
                # If both paths remain unhealthy, escalate after a longer
                # failure streak so a stuck Wi-Fi association is not left alone.
                force_cycle = wwan_ok or self.wifi_failures >= self.force_wifi_cycle_threshold
                self.reconnect_wifi(force_cycle=force_cycle)
                self.last_wifi_recovery = time.monotonic()

        if wwan_ok:
            self.wwan_failures = 0
            self.last_wwan_recovery = -self.recovery_cooldown
        else:
            self.wwan_failures += 1
            now = time.monotonic()
            if (
                self.wwan_failures >= self.failure_threshold
                and now - self.last_wwan_recovery >= self.recovery_cooldown
            ):
                self.reconnect_wwan()
                self.last_wwan_recovery = time.monotonic()

    def run_forever(self) -> None:
        LOG.info(
            "Started: target=%s:%d, Wi-Fi=%s/%s, LTE=%s/%s, interval=%ds, failure=%d, force-Wi-Fi-cycle=%d, recovery=%ds",
            self.target_ip,
            self.target_port,
            self.wifi_interface,
            self.wifi_profile,
            self.wwan_interface,
            self.wwan_profile,
            self.check_interval,
            self.failure_threshold,
            self.force_wifi_cycle_threshold,
            self.recovery_cooldown,
        )
        while True:
            try:
                self.step()
            except Exception:
                LOG.exception("Unexpected watchdog error; monitoring will continue")
            time.sleep(self.check_interval)


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )
    NetworkFailoverWatchdog().run_forever()


if __name__ == "__main__":
    main()
