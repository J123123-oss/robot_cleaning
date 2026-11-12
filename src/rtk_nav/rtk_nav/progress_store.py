"""Atomic persistence for the RTK cleaning progress snapshot."""

from __future__ import annotations

import json
import os
import tempfile
from pathlib import Path
from typing import Any, Mapping, Optional


REQUIRED_FIELDS = (
    "version",
    "route_id",
    "route_path",
    "waypoint_idx",
    "area",
    "nav_state",
    "target_waypoint",
    "updated_at",
)


def _validate(payload: Any) -> Optional[dict]:
    if not isinstance(payload, dict) or any(field not in payload for field in REQUIRED_FIELDS):
        return None
    try:
        version = int(payload["version"])
        waypoint_idx = int(payload["waypoint_idx"])
    except (TypeError, ValueError, OverflowError):
        return None
    if version != 1 or waypoint_idx < 0:
        return None
    if not isinstance(payload["route_id"], str) or not payload["route_id"].strip():
        return None
    if not isinstance(payload["route_path"], str) or not payload["route_path"].strip():
        return None
    if not isinstance(payload["nav_state"], str) or not payload["nav_state"].strip():
        return None
    target = payload["target_waypoint"]
    if target is not None:
        if not isinstance(target, (list, tuple)) or len(target) != 3:
            return None
        try:
            target = [float(value) for value in target]
        except (TypeError, ValueError, OverflowError):
            return None
    normalized = dict(payload)
    normalized["version"] = version
    normalized["waypoint_idx"] = waypoint_idx
    normalized["target_waypoint"] = target
    return normalized


def save_progress(path: os.PathLike[str] | str, payload: Mapping[str, Any]) -> None:
    """Write a validated snapshot using an atomic same-directory replace."""
    normalized = _validate(dict(payload))
    if normalized is None:
        raise ValueError("invalid RTK progress snapshot")
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(
        prefix=f".{destination.name}.", suffix=".tmp", dir=str(destination.parent)
    )
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump(normalized, stream, ensure_ascii=False, separators=(",", ":"))
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, destination)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def load_progress(path: os.PathLike[str] | str) -> Optional[dict]:
    """Load a valid snapshot, returning ``None`` for missing/corrupt data."""
    try:
        with Path(path).open("r", encoding="utf-8") as stream:
            return _validate(json.load(stream))
    except (OSError, UnicodeError, ValueError, TypeError, json.JSONDecodeError):
        return None
