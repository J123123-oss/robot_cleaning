"""Append RTK point records in a YAML-compatible Markdown format."""
from datetime import datetime
from pathlib import Path
import math
import re


def record_rtk_point(path, lon, lat, position_status, orientation_status, timestamp=None):
    """Append one point; return its sequence number, or None when invalid."""
    try:
        lon, lat = float(lon), float(lat)
    except (TypeError, ValueError):
        return None
    if not (math.isfinite(lon) and math.isfinite(lat) and -180 <= lon <= 180 and -90 <= lat <= 90):
        return None
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    text = target.read_text(encoding="utf-8") if target.exists() else ""
    seqs = [int(value) for value in re.findall(r"^\s*-\s*seq:\s*(\d+)", text, re.MULTILINE)]
    seq = max(seqs, default=0) + 1
    stamp = timestamp or datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    valid = int(position_status) == 4 and int(orientation_status) == 4
    with target.open("a", encoding="utf-8") as stream:
        stream.write(
            f'- seq: {seq}\n  point: {{lon: {lon!r}, lat: {lat!r}}}\n'
            f'  valid: {str(valid).lower()}\n  position_status: {int(position_status)}\n'
            f'  orientation_status: {int(orientation_status)}\n  time: "{stamp}"\n'
        )
    return seq
