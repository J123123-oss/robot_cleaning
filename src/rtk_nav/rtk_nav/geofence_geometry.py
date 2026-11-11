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
    for name, corners in zip(area_names, inner_corners_utm):
        boundary = expand_convex_polygon(corners, expansion_radius_m)
        areas.append({
            "name": name,
            "boundary_utm": [list(point) for point in boundary],
        })

    bridges = []
    for bridge in bridge_tracks:
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
