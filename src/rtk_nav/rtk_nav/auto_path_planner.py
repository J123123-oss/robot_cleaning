#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Generate a safe serpentine route from a small set of RTK traces."""

import argparse
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Optional, Sequence, Tuple, Union


Point = Tuple[float, float]
LocalPoint = Tuple[float, float]
JsonSource = Union[str, Path, Mapping[str, Any]]

METERS_PER_DEGREE_LAT = 110540.0
METERS_PER_DEGREE_LON = 111320.0
CONNECTOR_SAMPLE_SPACING_M = 0.25
EPSILON = 1e-9


class PlanningError(ValueError):
    """Raised when the input map cannot produce a safe route."""


@dataclass(frozen=True)
class AutoMap:
    """Validated map data in longitude/latitude coordinates."""

    boundary: Tuple[Point, ...]
    guides: Tuple[Tuple[Point, ...], ...]
    no_go: Tuple[Tuple[Point, ...], ...]
    start: Optional[Point]


@dataclass(frozen=True)
class Segment:
    """One ordered coverage or travel segment."""

    kind: str
    points: Tuple[Point, ...]
    length_m: float


@dataclass(frozen=True)
class Route:
    """Generated route and summary metrics."""

    axis_angle_rad: float
    segments: Tuple[Segment, ...]
    total_length_m: float
    max_connector_length_m: float


@dataclass(frozen=True)
class _LocalFrame:
    origin: Point
    lon_scale: float

    @classmethod
    def from_map(cls, map_data: AutoMap) -> "_LocalFrame":
        origin = map_data.boundary[0]
        latitude_scale = math.cos(math.radians(origin[1]))
        return cls(
            origin=origin,
            lon_scale=METERS_PER_DEGREE_LON * latitude_scale,
        )

    def to_xy(self, point: Point) -> LocalPoint:
        return (
            (point[0] - self.origin[0]) * self.lon_scale,
            (point[1] - self.origin[1]) * METERS_PER_DEGREE_LAT,
        )

    def from_xy(self, point: LocalPoint) -> Point:
        return (
            point[0] / self.lon_scale + self.origin[0],
            point[1] / METERS_PER_DEGREE_LAT + self.origin[1],
        )


def _read_json_source(source: JsonSource) -> Mapping[str, Any]:
    if isinstance(source, Mapping):
        return source
    try:
        payload = json.loads(Path(source).read_text(encoding="utf-8"))
    except OSError as exc:
        raise PlanningError(f"cannot read input map: {source}") from exc
    except json.JSONDecodeError as exc:
        raise PlanningError(f"invalid JSON input: {exc}") from exc
    if not isinstance(payload, Mapping):
        raise PlanningError("input JSON must contain an object")
    return payload


def _coerce_point(raw: Any, field: str) -> Point:
    if not isinstance(raw, Sequence) or isinstance(raw, (str, bytes)) or len(raw) < 2:
        raise PlanningError(f"{field} point must be [longitude, latitude]")
    try:
        point = (float(raw[0]), float(raw[1]))
    except (TypeError, ValueError) as exc:
        raise PlanningError(f"{field} point must contain numbers") from exc
    if not all(math.isfinite(value) for value in point):
        raise PlanningError(f"{field} point must contain finite numbers")
    if not -180.0 <= point[0] <= 180.0 or not -90.0 <= point[1] <= 90.0:
        raise PlanningError(f"{field} point is outside longitude/latitude bounds")
    return point


def _close_ring(raw: Any, field: str) -> Tuple[Point, ...]:
    if not isinstance(raw, Sequence) or isinstance(raw, (str, bytes)):
        raise PlanningError(f"{field} must be a list of points")
    if len(raw) < 3:
        raise PlanningError(f"{field} needs at least three points")
    points = [_coerce_point(point, field) for point in raw]
    if points[0] != points[-1]:
        points.append(points[0])
    distinct = set(points[:-1])
    if len(distinct) < 3:
        raise PlanningError(f"{field} is degenerate")
    return tuple(points)


def _parse_guides(raw: Any) -> Tuple[Tuple[Point, ...], ...]:
    if not isinstance(raw, Sequence) or isinstance(raw, (str, bytes)) or not raw:
        raise PlanningError("guides must contain at least one track")
    guides = []
    for index, guide in enumerate(raw):
        if not isinstance(guide, Sequence) or isinstance(guide, (str, bytes)):
            raise PlanningError(f"guides[{index}] must be a list of points")
        points = tuple(_coerce_point(point, f"guides[{index}]") for point in guide)
        if len(points) < 2 or _distance(points[0], points[-1]) <= EPSILON:
            raise PlanningError(f"guides[{index}] must have two distinct endpoints")
        guides.append(points)
    return tuple(guides)


def load_map(source: JsonSource) -> AutoMap:
    """Load and validate a JSON map object or JSON file path."""
    payload = _read_json_source(source)
    boundary = _close_ring(payload.get("boundary"), "boundary")
    guides = _parse_guides(payload.get("guides"))

    raw_no_go = payload.get("no_go", [])
    if not isinstance(raw_no_go, Sequence) or isinstance(raw_no_go, (str, bytes)):
        raise PlanningError("no_go must be a list of polygon rings")
    no_go = tuple(
        _close_ring(ring, f"no_go[{index}]")
        for index, ring in enumerate(raw_no_go)
    )

    raw_start = payload.get("start")
    start = None if raw_start is None else _coerce_point(raw_start, "start")
    return AutoMap(boundary=boundary, guides=guides, no_go=no_go, start=start)


def _distance(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _local_distance(a: LocalPoint, b: LocalPoint) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def estimate_axis_angle(guides: Sequence[Sequence[Point]]) -> float:
    """Estimate an undirected dominant guide angle in radians."""
    sum_cos = 0.0
    sum_sin = 0.0
    first_angle = None
    for guide in guides:
        if len(guide) < 2:
            continue
        start = guide[0]
        end = guide[-1]
        angle = math.atan2(end[1] - start[1], end[0] - start[0])
        if _distance(start, end) <= EPSILON:
            continue
        if first_angle is None:
            first_angle = angle
        sum_cos += math.cos(2.0 * angle)
        sum_sin += math.sin(2.0 * angle)
    if first_angle is None:
        raise PlanningError("guides do not contain a usable direction")
    if math.hypot(sum_cos, sum_sin) <= EPSILON:
        axis = first_angle
    else:
        axis = 0.5 * math.atan2(sum_sin, sum_cos)
    axis %= math.pi
    return axis


def _rotate(point: LocalPoint, angle: float) -> LocalPoint:
    cosine = math.cos(angle)
    sine = math.sin(angle)
    return (
        point[0] * cosine + point[1] * sine,
        -point[0] * sine + point[1] * cosine,
    )


def _unrotate(point: LocalPoint, angle: float) -> LocalPoint:
    cosine = math.cos(angle)
    sine = math.sin(angle)
    return (
        point[0] * cosine - point[1] * sine,
        point[0] * sine + point[1] * cosine,
    )


def _rotated_geometry(
    map_data: AutoMap, axis_angle: float
) -> Tuple[_LocalFrame, Tuple[LocalPoint, ...], Tuple[Tuple[LocalPoint, ...], ...]]:
    frame = _LocalFrame.from_map(map_data)

    def rotate_ring(ring: Sequence[Point]) -> Tuple[LocalPoint, ...]:
        return tuple(_rotate(frame.to_xy(point), axis_angle) for point in ring)

    return (
        frame,
        rotate_ring(map_data.boundary),
        tuple(rotate_ring(ring) for ring in map_data.no_go),
    )


def _line_intersections(ring: Sequence[LocalPoint], sweep_value: float) -> list[float]:
    intersections = []
    for first, second in zip(ring, ring[1:]):
        first_v = first[1]
        second_v = second[1]
        if abs(second_v - first_v) <= EPSILON:
            continue
        if (first_v <= sweep_value < second_v) or (
            second_v <= sweep_value < first_v
        ):
            ratio = (sweep_value - first_v) / (second_v - first_v)
            intersections.append(first[0] + ratio * (second[0] - first[0]))
    return intersections


def extract_scanline_intervals(
    map_data: AutoMap,
    axis_angle: float,
    sweep_value: float,
    edge_clearance: float,
) -> list[Tuple[float, float]]:
    """Return safe local U intervals for one local V scanline."""
    if edge_clearance < 0.0:
        raise PlanningError("edge_clearance must be non-negative")
    _, boundary, no_go = _rotated_geometry(map_data, axis_angle)
    intersections = _line_intersections(boundary, sweep_value)
    for ring in no_go:
        intersections.extend(_line_intersections(ring, sweep_value))
    intersections.sort()
    if len(intersections) % 2:
        raise PlanningError("scanline intersects invalid map geometry")

    intervals = []
    for left, right in zip(intersections[::2], intersections[1::2]):
        left += edge_clearance
        right -= edge_clearance
        if right - left > EPSILON:
            intervals.append((left, right))
    return intervals


def _scan_values(
    boundary: Sequence[LocalPoint], sweep_spacing: float, edge_clearance: float
) -> list[float]:
    values = [point[1] for point in boundary]
    lower = min(values) + edge_clearance
    upper = max(values) - edge_clearance
    if upper <= lower + EPSILON:
        raise PlanningError("edge_clearance leaves no usable scan area")
    result = []
    current = lower
    while current <= upper + EPSILON:
        result.append(current)
        current += sweep_spacing
    if not result:
        raise PlanningError("no scanline fits inside the boundary")
    return result


def _point_on_segment(point: LocalPoint, first: LocalPoint, second: LocalPoint) -> bool:
    cross = (point[0] - first[0]) * (second[1] - first[1]) - (
        point[1] - first[1]
    ) * (second[0] - first[0])
    if abs(cross) > 1e-7:
        return False
    return (
        min(first[0], second[0]) - 1e-7 <= point[0] <= max(first[0], second[0]) + 1e-7
        and min(first[1], second[1]) - 1e-7
        <= point[1]
        <= max(first[1], second[1]) + 1e-7
    )


def _point_in_ring(point: LocalPoint, ring: Sequence[LocalPoint]) -> bool:
    inside = False
    for first, second in zip(ring, ring[1:]):
        if _point_on_segment(point, first, second):
            return True
        if (first[1] > point[1]) != (second[1] > point[1]):
            crossing_x = (second[0] - first[0]) * (point[1] - first[1]) / (
                second[1] - first[1]
            ) + first[0]
            if point[0] < crossing_x:
                inside = not inside
    return inside


def _point_is_allowed(
    point: LocalPoint,
    boundary: Sequence[LocalPoint],
    no_go: Sequence[Sequence[LocalPoint]],
) -> bool:
    if not _point_in_ring(point, boundary):
        return False
    return not any(_point_in_ring(point, ring) for ring in no_go)


def _line_is_allowed(
    start: LocalPoint,
    end: LocalPoint,
    boundary: Sequence[LocalPoint],
    no_go: Sequence[Sequence[LocalPoint]],
) -> bool:
    length = _local_distance(start, end)
    sample_count = max(1, int(math.ceil(length / CONNECTOR_SAMPLE_SPACING_M)))
    for index in range(sample_count + 1):
        ratio = index / sample_count
        point = (
            start[0] + (end[0] - start[0]) * ratio,
            start[1] + (end[1] - start[1]) * ratio,
        )
        if not _point_is_allowed(point, boundary, no_go):
            return False
    return True


def _connector_candidates(start: LocalPoint, end: LocalPoint) -> list[list[LocalPoint]]:
    candidates = [[start, end]]
    first_corner = (end[0], start[1])
    second_corner = (start[0], end[1])
    candidates.extend([[start, first_corner, end], [start, second_corner, end]])
    return candidates


def _path_length(points: Sequence[LocalPoint]) -> float:
    return sum(_local_distance(first, second) for first, second in zip(points, points[1:]))


def _find_connector(
    start: LocalPoint,
    end: LocalPoint,
    boundary: Sequence[LocalPoint],
    no_go: Sequence[Sequence[LocalPoint]],
    max_connector: float,
) -> Tuple[LocalPoint, ...]:
    choices = []
    for candidate in _connector_candidates(start, end):
        if _line_is_allowed(candidate[0], candidate[1], boundary, no_go) and (
            len(candidate) == 2
            or _line_is_allowed(candidate[1], candidate[2], boundary, no_go)
        ):
            choices.append(candidate)
    if not choices:
        raise PlanningError(
            "no safe connector between consecutive coverage segments"
        )
    choices.sort(key=_path_length)
    connector = tuple(choices[0])
    connector_length = _path_length(connector)
    if connector_length > max_connector + EPSILON:
        raise PlanningError(
            "safe connector length {:.2f}m exceeds max_connector {:.2f}m".format(
                connector_length, max_connector
            )
        )
    return connector


def _local_coverage_segments(
    map_data: AutoMap,
    axis_angle: float,
    sweep_spacing: float,
    edge_clearance: float,
) -> list[Tuple[LocalPoint, LocalPoint]]:
    _, boundary, _ = _rotated_geometry(map_data, axis_angle)
    segments = []
    for row_index, sweep_value in enumerate(
        _scan_values(boundary, sweep_spacing, edge_clearance)
    ):
        intervals = extract_scanline_intervals(
            map_data, axis_angle, sweep_value, edge_clearance
        )
        if row_index % 2:
            intervals = list(reversed(intervals))
        for left, right in intervals:
            if row_index % 2:
                segments.append(((right, sweep_value), (left, sweep_value)))
            else:
                segments.append(((left, sweep_value), (right, sweep_value)))
    if not segments:
        raise PlanningError("no usable coverage segment was generated")
    return segments


def _segments_from_start(
    map_data: AutoMap,
    axis_angle: float,
    local_segments: Sequence[Tuple[LocalPoint, LocalPoint]],
) -> list[Tuple[LocalPoint, LocalPoint]]:
    if map_data.start is None:
        return list(local_segments)
    frame = _LocalFrame.from_map(map_data)
    start_local = _rotate(frame.to_xy(map_data.start), axis_angle)
    distances = [
        min(_local_distance(start_local, segment[0]), _local_distance(start_local, segment[1]))
        for segment in local_segments
    ]
    first_index = min(range(len(local_segments)), key=distances.__getitem__)
    reordered = list(local_segments[first_index:]) + list(local_segments[:first_index])
    first_segment = reordered[0]
    if _local_distance(start_local, first_segment[1]) < _local_distance(
        start_local, first_segment[0]
    ):
        reordered[0] = (first_segment[1], first_segment[0])
    return reordered


def _route_from_local_segments(
    map_data: AutoMap,
    axis_angle: float,
    local_segments: Sequence[Tuple[LocalPoint, LocalPoint]],
    max_connector: float,
) -> Route:
    frame, boundary, no_go = _rotated_geometry(map_data, axis_angle)
    output = []
    previous_end = None
    max_connector_length = 0.0

    for index, (start, end) in enumerate(local_segments):
        if previous_end is not None:
            connector = _find_connector(
                previous_end, start, boundary, no_go, max_connector
            )
            connector_points = tuple(
                frame.from_xy(_unrotate(point, axis_angle)) for point in connector
            )
            connector_length = _path_length(connector)
            max_connector_length = max(max_connector_length, connector_length)
            output.append(
                Segment("connector", connector_points, connector_length)
            )

        coverage_local = (start, end)
        coverage_points = tuple(
            frame.from_xy(_unrotate(point, axis_angle)) for point in coverage_local
        )
        coverage_length = _path_length(coverage_local)
        output.append(Segment("coverage", coverage_points, coverage_length))
        previous_end = end

    total_length = sum(segment.length_m for segment in output)
    return Route(
        axis_angle_rad=axis_angle,
        segments=tuple(output),
        total_length_m=total_length,
        max_connector_length_m=max_connector_length,
    )


def plan_route(
    map_data: AutoMap,
    sweep_spacing: float = 1.0,
    edge_clearance: float = 0.3,
    max_connector: float = 8.0,
) -> Route:
    """Generate a safe serpentine route from validated map data."""
    if sweep_spacing <= 0.0:
        raise PlanningError("sweep_spacing must be positive")
    if edge_clearance < 0.0:
        raise PlanningError("edge_clearance must be non-negative")
    if max_connector < 0.0:
        raise PlanningError("max_connector must be non-negative")

    axis_angle = estimate_axis_angle(map_data.guides)
    local_segments = _local_coverage_segments(
        map_data, axis_angle, sweep_spacing, edge_clearance
    )
    if map_data.start is not None:
        frame = _LocalFrame.from_map(map_data)
        start_local = _rotate(frame.to_xy(map_data.start), axis_angle)
        _, boundary, no_go = _rotated_geometry(map_data, axis_angle)
        if not _point_is_allowed(start_local, boundary, no_go):
            raise PlanningError("start is outside the usable map area")
        start_segments = _segments_from_start(map_data, axis_angle, local_segments)
        return _route_from_local_segments(
            map_data, axis_angle, start_segments, max_connector
        )
    return _route_from_local_segments(
        map_data, axis_angle, local_segments, max_connector
    )


def _segment_dict(segment: Segment) -> dict[str, Any]:
    return {
        "kind": segment.kind,
        "points": [[point[0], point[1]] for point in segment.points],
        "length_m": round(segment.length_m, 6),
    }


def route_to_json(route: Route) -> str:
    """Serialize a route as the auto planner JSON format."""
    payload = {
        "format": "rtk_auto_route_v1",
        "axis_angle_deg": round(math.degrees(route.axis_angle_rad) % 180.0, 6),
        "segments": [_segment_dict(segment) for segment in route.segments],
        "metrics": {
            "coverage_segments": sum(
                segment.kind == "coverage" for segment in route.segments
            ),
            "connector_segments": sum(
                segment.kind == "connector" for segment in route.segments
            ),
            "total_length_m": round(route.total_length_m, 6),
            "max_connector_length_m": round(route.max_connector_length_m, 6),
        },
    }
    return json.dumps(payload, ensure_ascii=False, indent=2)


def route_to_geojson(route: Route) -> str:
    """Serialize a route as a GeoJSON FeatureCollection."""
    features = []
    for index, segment in enumerate(route.segments):
        features.append(
            {
                "type": "Feature",
                "properties": {
                    "index": index,
                    "kind": segment.kind,
                    "length_m": round(segment.length_m, 6),
                },
                "geometry": {
                    "type": "LineString",
                    "coordinates": [list(point) for point in segment.points],
                },
            }
        )
    return json.dumps(
        {"type": "FeatureCollection", "features": features},
        ensure_ascii=False,
        indent=2,
    )


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, help="auto map JSON file")
    parser.add_argument("--output", required=True, help="route JSON output file")
    parser.add_argument(
        "--sweep-spacing", type=float, default=1.0, help="coverage line spacing in metres"
    )
    parser.add_argument(
        "--edge-clearance", type=float, default=0.3, help="coverage edge clearance in metres"
    )
    parser.add_argument(
        "--max-connector", type=float, default=8.0, help="maximum connector length in metres"
    )
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    """Run the auto planner command line interface."""
    args = _build_parser().parse_args(argv)
    output_path = Path(args.output)
    try:
        map_data = load_map(args.input)
        route = plan_route(
            map_data,
            sweep_spacing=args.sweep_spacing,
            edge_clearance=args.edge_clearance,
            max_connector=args.max_connector,
        )
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(route_to_json(route) + "\n", encoding="utf-8")
        output_path.with_suffix(".geojson").write_text(
            route_to_geojson(route) + "\n", encoding="utf-8"
        )
        print(
            "generated {} coverage segments and {} connector segments".format(
                sum(segment.kind == "coverage" for segment in route.segments),
                sum(segment.kind == "connector" for segment in route.segments),
            )
        )
        return 0
    except (PlanningError, OSError) as exc:
        print(f"auto planner error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
