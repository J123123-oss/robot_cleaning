"""Small, dependency-free geometry helpers for RTK geofences."""

from __future__ import annotations

import json
import math
from typing import Iterable, Sequence

Point = tuple[float, float]


def _signed_area(polygon: Sequence[Point]) -> float:
    return 0.5 * sum(
        polygon[index][0] * polygon[(index + 1) % len(polygon)][1]
        - polygon[(index + 1) % len(polygon)][0] * polygon[index][1]
        for index in range(len(polygon))
    )


def _line_intersection(
    first_point: Point,
    first_direction: Point,
    second_point: Point,
    second_direction: Point,
) -> Point:
    denominator = (
        first_direction[0] * second_direction[1]
        - first_direction[1] * second_direction[0]
    )
    if abs(denominator) < 1e-9:
        raise ValueError("parallel polygon edges cannot be offset")
    delta = (
        second_point[0] - first_point[0],
        second_point[1] - first_point[1],
    )
    scale = (
        delta[0] * second_direction[1]
        - delta[1] * second_direction[0]
    ) / denominator
    return (
        first_point[0] + scale * first_direction[0],
        first_point[1] + scale * first_direction[1],
    )


def expand_convex_polygon(polygon: Sequence[Point], radius_m: float) -> list[Point]:
    """Expand a convex polygon by a Euclidean radius in the same XY frame."""
    if len(polygon) < 3:
        raise ValueError("polygon must contain at least three points")
    if radius_m < 0:
        raise ValueError("radius_m must be non-negative")
    ccw = _signed_area(polygon) > 0
    offset_lines: list[tuple[Point, Point]] = []
    for index, point in enumerate(polygon):
        next_point = polygon[(index + 1) % len(polygon)]
        direction = (
            next_point[0] - point[0],
            next_point[1] - point[1],
        )
        length = math.hypot(*direction)
        if length < 1e-9:
            raise ValueError("polygon contains a duplicate point")
        if ccw:
            normal = (direction[1] / length, -direction[0] / length)
        else:
            normal = (-direction[1] / length, direction[0] / length)
        offset_point = (
            point[0] + radius_m * normal[0],
            point[1] + radius_m * normal[1],
        )
        offset_lines.append((offset_point, direction))

    expanded: list[Point] = []
    for index in range(len(offset_lines)):
        previous = offset_lines[index - 1]
        current = offset_lines[index]
        expanded.append(
            _line_intersection(
                previous[0],
                previous[1],
                current[0],
                current[1],
            )
        )
    return expanded


def point_in_convex_polygon(point: Point, polygon: Sequence[Point]) -> bool:
    """Return whether a point is inside or on the boundary of a convex polygon."""
    if len(polygon) < 3:
        return False
    signs = []
    for index, vertex in enumerate(polygon):
        next_vertex = polygon[(index + 1) % len(polygon)]
        cross = (
            (next_vertex[0] - vertex[0]) * (point[1] - vertex[1])
            - (next_vertex[1] - vertex[1]) * (point[0] - vertex[0])
        )
        if abs(cross) > 1e-7:
            signs.append(cross > 0)
    return not signs or all(sign == signs[0] for sign in signs)


def _point_to_segment_distance(point: Point, start: Point, end: Point) -> float:
    direction = (end[0] - start[0], end[1] - start[1])
    length_squared = direction[0] ** 2 + direction[1] ** 2
    if length_squared < 1e-12:
        return math.hypot(point[0] - start[0], point[1] - start[1])
    projection = (
        (point[0] - start[0]) * direction[0]
        + (point[1] - start[1]) * direction[1]
    ) / length_squared
    projection = max(0.0, min(1.0, projection))
    closest = (
        start[0] + projection * direction[0],
        start[1] + projection * direction[1],
    )
    return math.hypot(point[0] - closest[0], point[1] - closest[1])


def point_to_polyline_distance(point: Point, polyline: Sequence[Point]) -> float:
    if len(polyline) < 2:
        raise ValueError("polyline must contain at least two points")
    return min(
        _point_to_segment_distance(point, polyline[index], polyline[index + 1])
        for index in range(len(polyline) - 1)
    )


def build_polyline_corridor(
    polyline: Sequence[Point],
    radius_m: float,
) -> list[list[Point]]:
    """Return one expanded rectangle per polyline segment.

    Segment rectangles overlap at shared vertices; runtime checks can treat
    the returned list as a logical union.
    """
    if len(polyline) < 2:
        raise ValueError("polyline must contain at least two points")
    if radius_m < 0:
        raise ValueError("radius_m must be non-negative")
    corridors: list[list[Point]] = []
    for start, end in zip(polyline, polyline[1:]):
        direction = (end[0] - start[0], end[1] - start[1])
        length = math.hypot(*direction)
        if length < 1e-9:
            continue
        normal = (-direction[1] / length, direction[0] / length)
        corridors.append(
            [
                (start[0] + normal[0] * radius_m, start[1] + normal[1] * radius_m),
                (end[0] + normal[0] * radius_m, end[1] + normal[1] * radius_m),
                (end[0] - normal[0] * radius_m, end[1] - normal[1] * radius_m),
                (start[0] - normal[0] * radius_m, start[1] - normal[1] * radius_m),
            ]
        )
    return corridors


def build_geofence_payload(
    area_names: Sequence[str],
    inner_corners_utm: Sequence[Sequence[Point]],
    bridge_tracks: Sequence[dict],
    utm_zone: Sequence[object],
    expansion_radius_m: float = 0.2,
) -> dict:
    """Build JSON-compatible fence metadata from legacy planner geometry."""
    areas = []
    placeholder_tracks = []
    for name, corners in zip(area_names, inner_corners_utm):
        if abs(_signed_area(corners)) < 1e-6:
            # A=C collapses a cleaning rectangle into a passage. It still
            # needs its own corridor, even when no other region is connected.
            if corners:
                start, end = max(
                    ((a, b) for a in corners for b in corners),
                    key=lambda pair: math.hypot(
                        pair[1][0] - pair[0][0], pair[1][1] - pair[0][1]
                    ),
                )
                placeholder_tracks.append({
                    "name": name,
                    "from_area": name,
                    "to_area": name,
                    "centerline_utm": [start, end],
                })
            continue
        boundary = expand_convex_polygon(corners, expansion_radius_m)
        areas.append({
            "name": name,
            "boundary_utm": [list(point) for point in boundary],
        })

    bridges = []
    for bridge in [*placeholder_tracks, *bridge_tracks]:
        centerline = bridge["centerline_utm"]
        corridors = build_polyline_corridor(centerline, expansion_radius_m)
        bridges.append({
            "name": bridge["name"],
            "from_area": bridge["from_area"],
            "to_area": bridge["to_area"],
            "centerline_utm": [list(point) for point in centerline],
            "corridors_utm": [
                [list(point) for point in corridor]
                for corridor in corridors
            ],
        })
    return {
        "version": 1,
        "expansion_radius_m": expansion_radius_m,
        "utm_zone": list(utm_zone),
        "areas": areas,
        "bridges": bridges,
    }


def load_geofence_payload(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as fence_file:
        payload = json.load(fence_file)
    if not isinstance(payload, dict):
        raise ValueError("geofence sidecar must contain a JSON object")
    return payload


def point_allowed_by_payload(point: Point, payload: dict) -> bool:
    for area in payload.get("areas", []):
        if point_in_convex_polygon(point, area.get("boundary_utm", [])):
            return True
    for bridge in payload.get("bridges", []):
        centerline = bridge.get("centerline_utm", [])
        radius = float(payload.get("expansion_radius_m", 0.0))
        if len(centerline) >= 2 and point_to_polyline_distance(point, centerline) <= radius + 1e-6:
            return True
        for corridor in bridge.get("corridors_utm", []):
            if point_in_convex_polygon(point, corridor):
                return True
    return False


def validate_convex_polygon(polygon: Sequence[Point]) -> None:
    if len(polygon) < 3 or not all(
        len(point) == 2 and all(math.isfinite(float(value)) for value in point)
        for point in polygon
    ):
        raise ValueError("area requires at least three finite XY coordinates")
    turns = []
    for index, point in enumerate(polygon):
        next_point = polygon[(index + 1) % len(polygon)]
        following = polygon[(index + 2) % len(polygon)]
        cross = ((next_point[0] - point[0]) * (following[1] - next_point[1])
                 - (next_point[1] - point[1]) * (following[0] - next_point[0]))
        if abs(cross) < 1e-9:
            raise ValueError("area has duplicate or collinear vertices")
        turns.append(cross > 0)
    if not all(turn == turns[0] for turn in turns):
        raise ValueError("area must be a simple convex polygon")
    edge_count = len(polygon)
    for first in range(edge_count):
        a, b = polygon[first], polygon[(first + 1) % edge_count]
        for second in range(first + 1, edge_count):
            if second in (first, (first + 1) % edge_count) or (second + 1) % edge_count == first:
                continue
            c, d = polygon[second], polygon[(second + 1) % edge_count]
            ab_c = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
            ab_d = (b[0] - a[0]) * (d[1] - a[1]) - (b[1] - a[1]) * (d[0] - a[0])
            cd_a = (d[0] - c[0]) * (a[1] - c[1]) - (d[1] - c[1]) * (a[0] - c[0])
            cd_b = (d[0] - c[0]) * (b[1] - c[1]) - (d[1] - c[1]) * (b[0] - c[0])
            if ab_c * ab_d < 0 and cd_a * cd_b < 0:
                raise ValueError("area must be a simple convex polygon")


def _segment_polygon_interval(start: Point, end: Point, polygon: Sequence[Point]):
    if len(polygon) < 3 or abs(_signed_area(polygon)) < 1e-9:
        return None
    direction = (end[0] - start[0], end[1] - start[1])
    orientation = 1.0 if _signed_area(polygon) > 0 else -1.0
    lower, upper = 0.0, 1.0
    for vertex, following in zip(polygon, list(polygon[1:]) + [polygon[0]]):
        edge = (following[0] - vertex[0], following[1] - vertex[1])
        initial = orientation * (edge[0] * (start[1] - vertex[1])
                                 - edge[1] * (start[0] - vertex[0]))
        slope = orientation * (edge[0] * direction[1] - edge[1] * direction[0])
        if abs(slope) < 1e-12:
            if initial < -1e-8:
                return None
        elif slope > 0:
            lower = max(lower, -initial / slope)
        else:
            upper = min(upper, -initial / slope)
        if lower > upper:
            return None
    return lower, upper


def _segment_circle_interval(start: Point, end: Point, center: Point, radius: float):
    direction = (end[0] - start[0], end[1] - start[1])
    offset = (start[0] - center[0], start[1] - center[1])
    squared = direction[0] ** 2 + direction[1] ** 2
    if squared < 1e-18:
        return (0.0, 1.0) if math.hypot(*offset) <= radius else None
    projection = -(offset[0] * direction[0] + offset[1] * direction[1]) / squared
    closest_squared = (offset[0] + projection * direction[0]) ** 2 + (offset[1] + projection * direction[1]) ** 2
    remainder = radius ** 2 - closest_squared
    if remainder < 0:
        return None
    span = math.sqrt(remainder / squared)
    lower, upper = max(0.0, projection - span), min(1.0, projection + span)
    return (lower, upper) if lower <= upper else None


def segment_allowed_by_payload(start: Point, end: Point, payload: dict) -> bool:
    """Check complete line coverage by the union of fence polygons and corridors."""
    intervals = []
    for area in payload.get("areas", []):
        intervals.append(_segment_polygon_interval(start, end, area.get("boundary_utm", [])))
    radius = float(payload.get("expansion_radius_m", 0.0))
    for bridge in payload.get("bridges", []):
        for corridor in bridge.get("corridors_utm", []):
            intervals.append(_segment_polygon_interval(start, end, corridor))
        centerline = bridge.get("centerline_utm", [])
        for first, second in zip(centerline, centerline[1:]):
            for corridor in build_polyline_corridor([first, second], radius):
                intervals.append(_segment_polygon_interval(start, end, corridor))
            intervals.append(_segment_circle_interval(start, end, first, radius))
            intervals.append(_segment_circle_interval(start, end, second, radius))
    covered = 0.0
    for lower, upper in sorted(interval for interval in intervals if interval is not None):
        if lower > covered + 1e-9:
            return False
        covered = max(covered, upper)
        if covered >= 1.0 - 1e-9:
            return True
    return False


def nearest_recovery_target(point: Point, payload: dict) -> Point | None:
    candidates = []
    for area in payload.get("areas", []):
        polygon = area.get("boundary_utm", [])
        if len(polygon) >= 3:
            center = (sum(p[0] for p in polygon) / len(polygon),
                      sum(p[1] for p in polygon) / len(polygon))
            for start, end in zip(polygon, polygon[1:] + polygon[:1]):
                delta = (end[0] - start[0], end[1] - start[1])
                length_squared = delta[0] ** 2 + delta[1] ** 2
                if length_squared <= 1e-12:
                    continue
                ratio = max(0.0, min(1.0, ((point[0] - start[0]) * delta[0]
                                           + (point[1] - start[1]) * delta[1]) / length_squared))
                nearest = (start[0] + ratio * delta[0], start[1] + ratio * delta[1])
                inward = (center[0] - nearest[0], center[1] - nearest[1])
                distance = math.hypot(*inward)
                if distance > 1e-9:
                    scale = min(0.1 / distance, 1.0)
                    candidates.append((nearest[0] + scale * inward[0],
                                       nearest[1] + scale * inward[1]))
    for bridge in payload.get("bridges", []):
        line = bridge.get("centerline_utm", [])
        for start, end in zip(line, line[1:]):
            delta = (end[0] - start[0], end[1] - start[1])
            length_squared = delta[0] ** 2 + delta[1] ** 2
            if length_squared <= 1e-12:
                continue
            ratio = max(0.0, min(1.0, ((point[0] - start[0]) * delta[0]
                                       + (point[1] - start[1]) * delta[1]) / length_squared))
            candidates.append((start[0] + ratio * delta[0], start[1] + ratio * delta[1]))
    candidates = [candidate for candidate in candidates
                  if all(math.isfinite(value) for value in candidate)
                  and point_allowed_by_payload(candidate, payload)]
    return min(candidates, key=lambda candidate: math.hypot(
        candidate[0] - point[0], candidate[1] - point[1]), default=None)
