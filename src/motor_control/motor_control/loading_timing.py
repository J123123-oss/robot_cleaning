def advance_active_elapsed(elapsed: float, interval: float, was_active: bool) -> float:
    """Accumulate elapsed time only while the requested motion is active."""
    if not was_active:
        return elapsed
    return elapsed + max(0.0, interval)
