"""Pure calculations for the short RTK-loss IMU/odometry fallback."""

import math


def _require_finite(*values: float) -> None:
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("fallback calculation inputs must be finite")


def normalize_heading_error(target_deg: float, current_deg: float) -> float:
    """Return target minus current heading in [-180, 180)."""
    _require_finite(target_deg, current_deg)
    return (float(target_deg) - float(current_deg) + 180.0) % 360.0 - 180.0


def calculate_heading_hold_wheel_speeds(
    direction: str,
    base_rpm: float,
    target_heading_deg: float,
    current_heading_deg: float,
    kp: float = 0.90,
    max_correction_rpm: float = 10.0,
    deadband_deg: float = 0.30,
    correction_base_ratio: float = 0.50,
) -> tuple[float, float]:
    """Compute the field-tested differential heading correction in wheel RPM."""
    if direction not in ("FORWARD", "BACKWARD"):
        raise ValueError(f"unknown fallback direction: {direction!r}")
    _require_finite(
        base_rpm,
        target_heading_deg,
        current_heading_deg,
        kp,
        max_correction_rpm,
        deadband_deg,
        correction_base_ratio,
    )
    base_rpm = float(base_rpm)
    kp = float(kp)
    max_correction_rpm = float(max_correction_rpm)
    deadband_deg = float(deadband_deg)
    correction_base_ratio = float(correction_base_ratio)
    if base_rpm < 0.0 or kp < 0.0 or max_correction_rpm < 0.0:
        raise ValueError("base speed and correction limits must be non-negative")
    if deadband_deg < 0.0 or correction_base_ratio < 0.0:
        raise ValueError("deadband and correction ratio must be non-negative")

    heading_error = normalize_heading_error(target_heading_deg, current_heading_deg)
    correction = 0.0
    if abs(heading_error) > deadband_deg:
        correction_angle = max(-45.0, min(45.0, -heading_error))
        correction_limit = min(max_correction_rpm, base_rpm * correction_base_ratio)
        correction = max(
            -correction_limit,
            min(correction_limit, correction_angle * kp),
        )

    if direction == "FORWARD":
        return -base_rpm + correction, base_rpm + correction
    return base_rpm + correction, -base_rpm + correction


def integrate_local_enu_delta(
    east_m: float,
    north_m: float,
    linear_velocity_mps: float,
    imu_heading_deg: float,
    dt_sec: float,
) -> tuple[float, float, float]:
    """Integrate signed forward speed using compass heading (0=N, clockwise)."""
    _require_finite(east_m, north_m, linear_velocity_mps, imu_heading_deg, dt_sec)
    dt_sec = float(dt_sec)
    if dt_sec <= 0.0:
        raise ValueError("dt_sec must be positive")
    distance = abs(float(linear_velocity_mps)) * dt_sec
    heading_rad = math.radians(float(imu_heading_deg))
    east = float(east_m) + float(linear_velocity_mps) * math.sin(heading_rad) * dt_sec
    north = float(north_m) + float(linear_velocity_mps) * math.cos(heading_rad) * dt_sec
    _require_finite(east, north, distance)
    return east, north, distance


def fallback_budget_exhausted(
    distance_m: float,
    elapsed_sec: float,
    max_distance_m: float = 1.0,
    max_duration_sec: float = 3.0,
) -> bool:
    """Return true once either conservative fallback travel limit is reached."""
    _require_finite(distance_m, elapsed_sec, max_distance_m, max_duration_sec)
    if max_distance_m <= 0.0 or max_duration_sec <= 0.0:
        raise ValueError("fallback limits must be positive")
    if distance_m < 0.0 or elapsed_sec < 0.0:
        raise ValueError("fallback distance and elapsed time must be non-negative")
    return distance_m >= max_distance_m or elapsed_sec >= max_duration_sec


def position_residual_m(
    predicted_xy: tuple[float, float], measured_xy: tuple[float, float]
) -> float:
    """Return Euclidean map-frame residual between predicted and measured XY."""
    if len(predicted_xy) != 2 or len(measured_xy) != 2:
        raise ValueError("positions must each contain east/x and north/y")
    _require_finite(*predicted_xy, *measured_xy)
    residual = math.hypot(
        float(measured_xy[0]) - float(predicted_xy[0]),
        float(measured_xy[1]) - float(predicted_xy[1]),
    )
    _require_finite(residual)
    return residual
