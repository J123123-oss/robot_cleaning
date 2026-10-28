"""Pure total-current protection logic for the motor control node."""


class TotalCurrentMonitor:
    """Detect total battery current above a limit for a sustained duration."""

    def __init__(self, threshold: float = 4.0, duration: float = 5.0) -> None:
        if threshold <= 0 or duration < 0:
            raise ValueError("threshold must be positive and duration non-negative")
        self.threshold = float(threshold)
        self.duration = float(duration)
        self._over_current_since = None

    def reset(self) -> None:
        """Clear the pending over-current interval."""

        self._over_current_since = None

    def update(self, current: float, enabled: bool, now: float) -> bool:
        """Return ``True`` once enabled current stays above the threshold."""

        if not enabled or abs(float(current)) <= self.threshold:
            self.reset()
            return False
        if self._over_current_since is None:
            self._over_current_since = float(now)
            return self.duration == 0.0
        return float(now) - self._over_current_since >= self.duration
