"""Pure current-threshold monitoring logic for motor stall protection."""

import math


class CurrentStallMonitor:
    """Detect enabled current above a limit for a sustained duration."""

    def __init__(self, threshold: float = 4.0, duration: float = 5.0) -> None:
        threshold = float(threshold)
        duration = float(duration)
        if not math.isfinite(threshold) or threshold <= 0:
            raise ValueError("threshold must be finite and positive")
        if not math.isfinite(duration) or duration < 0:
            raise ValueError("duration must be finite and non-negative")
        self.threshold = threshold
        self.duration = duration
        self._over_current_since = None

    def reset(self) -> None:
        """Clear the pending over-current interval."""

        self._over_current_since = None

    def elapsed(self, now: float) -> float:
        """Return the current continuous over-threshold interval in seconds."""

        if self._over_current_since is None:
            return 0.0
        try:
            elapsed = float(now) - self._over_current_since
        except (TypeError, ValueError, OverflowError):
            return 0.0
        return max(0.0, elapsed)

    def update(
        self,
        current: float,
        enabled: bool,
        now: float,
        sample_fresh: bool = True,
    ) -> bool:
        """Return ``True`` once a fresh, enabled current exceeds the limit long enough."""

        try:
            current = float(current)
            now = float(now)
        except (TypeError, ValueError, OverflowError):
            self.reset()
            return False

        if (
            not enabled
            or not sample_fresh
            or not math.isfinite(current)
            or not math.isfinite(now)
            or abs(current) <= self.threshold
        ):
            self.reset()
            return False
        if self._over_current_since is None:
            self._over_current_since = now
            return self.duration == 0.0
        return self.elapsed(now) >= self.duration


class TotalCurrentMonitor(CurrentStallMonitor):
    """Detect battery output current above a limit for a sustained duration."""
