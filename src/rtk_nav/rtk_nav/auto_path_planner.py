#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Generate a safe serpentine route from a small set of RTK traces."""

import argparse
import heapq
import json
import math
import sys
from itertools import permutations
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Optional, Sequence, Tuple, Union

try:
    import yaml
except ImportError:  # pragma: no cover - exercised only without PyYAML
    yaml = None


Point = Tuple[float, float]
LocalPoint = Tuple[float, float]
JsonSource = Union[str, Path, Mapping[str, Any]]

METERS_PER_DEGREE_LAT = 110540.0
METERS_PER_DEGREE_LON = 111320.0
CONNECTOR_SAMPLE_SPACING_M = 0.25
EPSILON = 1e-9
DEFAULT_TURN_PENALTY_M = 1.0
DEFAULT_MAX_CONNECTOR_PENALTY = 1.0
DEFAULT_TXT_SPACING_M = 15.0
VALID_GUIDES = {"horizontal", "vertical"}


class PlanningError(ValueError):
    """Raised when the input map cannot produce a safe route."""


@dataclass(frozen=True)
class Polygon:
    """One outer ring and zero or more excluded inner rings."""

    boundary: Tuple[Point, ...]
    holes: Tuple[Tuple[Point, ...], ...] = ()
    edge_distance_lon: Tuple[float, float] = (0.0, 0.0)
    edge_distance_lat: Tuple[float, float] = (0.0, 0.0)


@dataclass(frozen=True)
class PlannerDefaults:
    """Legacy-compatible defaults inherited by unconfigured map regions."""

    interval: float = 1.0
    start_corner: str = "top_left"
    swap_wh_select: bool = False
    edge_distance_lon: Tuple[float, float] = (0.1, 0.1)
    edge_distance_lat: Tuple[float, float] = (0.1, 0.1)


@dataclass(frozen=True)
class Region:
    """A logical cleaning region made from one or more polygons."""

    id: str
    polygons: Tuple[Polygon, ...]
    start: Optional[Point] = None
    connection_tolerance_m: float = 0.0
    guide: Optional[str] = None


@dataclass(frozen=True)
class Connector:
    """An explicit traversable path between two named regions."""

    id: str
    from_region: Optional[str]
    to_region: Optional[str]
    path: Tuple[Point, ...]
    edge_distance_lon: Optional[Tuple[float, float]] = None
    edge_distance_lat: Optional[Tuple[float, float]] = None


@dataclass(frozen=True)
class AutoMap:
    """Validated map data in longitude/latitude coordinates."""

    boundary: Tuple[Point, ...]
    guides: Tuple[Tuple[Point, ...], ...]
    no_go: Tuple[Tuple[Point, ...], ...]
    start: Optional[Point]
    defaults: PlannerDefaults = PlannerDefaults()
    regions: Tuple[Region, ...] = ()
    connectors: Tuple[Connector, ...] = ()
    order: Tuple[str, ...] = ()
    legacy: bool = False


@dataclass(frozen=True)
class Segment:
    """One ordered coverage or travel segment."""

    kind: str
    points: Tuple[Point, ...]
    length_m: float
    region_id: Optional[str] = None
    connector_id: Optional[str] = None
    from_region: Optional[str] = None
    to_region: Optional[str] = None


@dataclass(frozen=True)
class Route:
    """Generated route and summary metrics."""

    axis_angle_rad: float
    segments: Tuple[Segment, ...]
    total_length_m: float
    max_connector_length_m: float
    turn_count: int = 0
    order: Tuple[str, ...] = ()


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
    if isinstance(raw, Mapping):
        lon = raw.get("lon", raw.get("longitude", raw.get("x")))
        lat = raw.get("lat", raw.get("latitude", raw.get("y")))
        if lon is None or lat is None:
            raise PlanningError(f"{field} point must contain longitude and latitude")
        raw_values = (lon, lat)
    else:
        if not isinstance(raw, Sequence) or isinstance(raw, (str, bytes)) or len(raw) < 2:
            raise PlanningError(f"{field} point must be [longitude, latitude]")
        raw_values = (raw[0], raw[1])
    try:
        point = (float(raw_values[0]), float(raw_values[1]))
    except (TypeError, ValueError) as exc:
        raise PlanningError(f"{field} point must contain numbers") from exc
    if not all(math.isfinite(value) for value in point):
        raise PlanningError(f"{field} point must contain finite numbers")
    if not -180.0 <= point[0] <= 180.0 or not -90.0 <= point[1] <= 90.0:
        raise PlanningError(f"{field} point is outside longitude/latitude bounds")
    return point


def _ring_values(raw: Any, field: str) -> Any:
    """Accept list rings and common named-corner JSON representations."""
    if not isinstance(raw, Mapping):
        return raw
    for key in ("points", "coordinates", "corners"):
        if key in raw:
            return raw[key]

    named_orders = (
        ("top_left", "top_right", "bottom_right", "bottom_left"),
        ("top-left", "top-right", "bottom-right", "bottom-left"),
        ("p1", "p2", "p3", "p4"),
    )
    for names in named_orders:
        if all(name in raw for name in names):
            return [raw[name] for name in names]

    values = list(raw.values())
    if values and all(isinstance(value, Mapping) for value in values):
        return values
    raise PlanningError(f"{field} must be a list of points")


def _close_ring(raw: Any, field: str) -> Tuple[Point, ...]:
    raw = _ring_values(raw, field)
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


def _cross(first: LocalPoint, second: LocalPoint) -> float:
    return first[0] * second[1] - first[1] * second[0]


def _subtract(first: LocalPoint, second: LocalPoint) -> LocalPoint:
    return (first[0] - second[0], first[1] - second[1])


def _segments_intersect(
    first_start: LocalPoint,
    first_end: LocalPoint,
    second_start: LocalPoint,
    second_end: LocalPoint,
) -> bool:
    first_vector = _subtract(first_end, first_start)
    second_vector = _subtract(second_end, second_start)
    start_delta = _subtract(second_start, first_start)
    denominator = _cross(first_vector, second_vector)
    if abs(denominator) > 1e-12:
        first_ratio = _cross(start_delta, second_vector) / denominator
        second_ratio = _cross(start_delta, first_vector) / denominator
        return (
            -1e-9 <= first_ratio <= 1.0 + 1e-9
            and -1e-9 <= second_ratio <= 1.0 + 1e-9
        )
    if abs(_cross(start_delta, first_vector)) > 1e-12:
        return False
    first_length_squared = first_vector[0] ** 2 + first_vector[1] ** 2
    if first_length_squared <= 1e-18:
        return _local_distance(first_start, second_start) <= 1e-9
    second_start_ratio = (
        (second_start[0] - first_start[0]) * first_vector[0]
        + (second_start[1] - first_start[1]) * first_vector[1]
    ) / first_length_squared
    second_end_ratio = (
        (second_end[0] - first_start[0]) * first_vector[0]
        + (second_end[1] - first_start[1]) * first_vector[1]
    ) / first_length_squared
    return max(0.0, min(second_start_ratio, second_end_ratio)) <= min(
        1.0, max(second_start_ratio, second_end_ratio)
    ) + 1e-9


def _properly_cross(
    first_start: LocalPoint,
    first_end: LocalPoint,
    second_start: LocalPoint,
    second_end: LocalPoint,
) -> bool:
    first_vector = _subtract(first_end, first_start)
    second_vector = _subtract(second_end, second_start)
    first_to_second_start = _subtract(second_start, first_start)
    first_to_second_end = _subtract(second_end, first_start)
    second_to_first_start = _subtract(first_start, second_start)
    second_to_first_end = _subtract(first_end, second_start)
    return (
        _cross(first_vector, first_to_second_start)
        * _cross(first_vector, first_to_second_end)
        < -1e-12
        and _cross(second_vector, second_to_first_start)
        * _cross(second_vector, second_to_first_end)
        < -1e-12
    )


def _ring_edges(ring: Sequence[LocalPoint]):
    return zip(ring, ring[1:])


def _validate_simple_ring(ring: Sequence[Point], field: str) -> None:
    local_ring = tuple(ring)
    edge_count = len(local_ring) - 1
    for first_index in range(edge_count):
        first_start, first_end = local_ring[first_index], local_ring[first_index + 1]
        for second_index in range(first_index + 1, edge_count):
            if second_index in (first_index + 1, first_index - 1):
                continue
            if first_index == 0 and second_index == edge_count - 1:
                continue
            second_start, second_end = local_ring[second_index], local_ring[second_index + 1]
            if _segments_intersect(first_start, first_end, second_start, second_end):
                raise PlanningError(f"{field} self-intersects")


def _strictly_inside(point: Point, ring: Sequence[Point]) -> bool:
    return _point_in_ring(point, ring) and not any(
        _point_on_segment(point, first, second)
        for first, second in _ring_edges(ring)
    )


def _rings_overlap(first: Sequence[Point], second: Sequence[Point]) -> bool:
    if any(_strictly_inside(point, second) for point in first[:-1]):
        return True
    if any(_strictly_inside(point, first) for point in second[:-1]):
        return True
    return any(
        _properly_cross(first_start, first_end, second_start, second_end)
        for first_start, first_end in _ring_edges(first)
        for second_start, second_end in _ring_edges(second)
    )


def _validate_polygon_geometry(polygon: Polygon, field: str) -> None:
    _validate_simple_ring(polygon.boundary, f"{field}.boundary")
    for hole_index, hole in enumerate(polygon.holes):
        hole_field = f"{field}.holes[{hole_index}]"
        _validate_simple_ring(hole, hole_field)
        if not _strictly_inside(hole[0], polygon.boundary):
            raise PlanningError(f"{hole_field} must be inside {field}.boundary")
        if any(
            _segments_intersect(first_start, first_end, second_start, second_end)
            for first_start, first_end in _ring_edges(polygon.boundary)
            for second_start, second_end in _ring_edges(hole)
        ):
            raise PlanningError(f"{hole_field} intersects {field}.boundary")
        for previous_hole in polygon.holes[:hole_index]:
            if _rings_overlap(hole, previous_hole) or _strictly_inside(hole[0], previous_hole):
                raise PlanningError(f"{hole_field} overlaps another hole")


def _validate_region_geometry(polygons: Sequence[Polygon], field: str) -> None:
    for index, polygon in enumerate(polygons):
        _validate_polygon_geometry(polygon, f"{field}.polygons[{index}]")
    for first_index, first in enumerate(polygons):
        for second in polygons[first_index + 1 :]:
            if _rings_overlap(first.boundary, second.boundary):
                raise PlanningError(f"{field} contains overlapping polygons")


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


def _parse_id(raw: Any, field: str) -> str:
    if not isinstance(raw, str) or not raw.strip():
        raise PlanningError(f"{field} must be a non-empty string")
    return raw.strip()


def _parse_guide(raw: Any, field: str) -> Optional[str]:
    if raw is None:
        return None
    if not isinstance(raw, str) or raw.strip() not in VALID_GUIDES:
        raise PlanningError(f"{field} must be horizontal or vertical")
    return raw.strip()


def _parse_distance_pair(raw: Any, field: str) -> Tuple[float, float]:
    if isinstance(raw, Sequence) and not isinstance(raw, (str, bytes)):
        if len(raw) != 2:
            raise PlanningError(f"{field} must contain exactly two distances")
        values = (raw[0], raw[1])
    else:
        values = (raw, raw)
    try:
        distances = (float(values[0]), float(values[1]))
    except (TypeError, ValueError) as exc:
        raise PlanningError(f"{field} must contain numbers") from exc
    if not all(math.isfinite(value) for value in distances):
        raise PlanningError(f"{field} must contain finite numbers")
    return distances


def _parse_defaults(raw: Any) -> PlannerDefaults:
    if raw is None:
        return PlannerDefaults()
    if not isinstance(raw, Mapping):
        raise PlanningError("defaults must be an object")

    raw_interval = raw.get("interval", PlannerDefaults.interval)
    try:
        interval = float(raw_interval)
    except (TypeError, ValueError) as exc:
        raise PlanningError("defaults.interval must be a positive number") from exc
    if not math.isfinite(interval) or interval <= 0.0:
        raise PlanningError("defaults.interval must be a positive number")

    start_corner = raw.get("start_corner", PlannerDefaults.start_corner)
    if not isinstance(start_corner, str) or start_corner not in {
        "top_left",
        "top_right",
        "bottom_left",
        "bottom_right",
    }:
        raise PlanningError(
            "defaults.start_corner must be top_left, top_right, bottom_left, or bottom_right"
        )

    swap_wh_select = raw.get("swap_wh_select", PlannerDefaults.swap_wh_select)
    if not isinstance(swap_wh_select, bool):
        raise PlanningError("defaults.swap_wh_select must be a boolean")

    return PlannerDefaults(
        interval=interval,
        start_corner=start_corner,
        swap_wh_select=swap_wh_select,
        edge_distance_lon=_parse_distance_pair(
            raw.get("edge_distance_lon", PlannerDefaults.edge_distance_lon),
            "defaults.edge_distance_lon",
        ),
        edge_distance_lat=_parse_distance_pair(
            raw.get("edge_distance_lat", PlannerDefaults.edge_distance_lat),
            "defaults.edge_distance_lat",
        ),
    )


def _parse_polygon(
    raw: Any,
    field: str,
    default_edge_distance_lon: Tuple[float, float] = PlannerDefaults.edge_distance_lon,
    default_edge_distance_lat: Tuple[float, float] = PlannerDefaults.edge_distance_lat,
) -> Polygon:
    if not isinstance(raw, Mapping):
        raise PlanningError(f"{field} must be an object")
    boundary = _close_ring(raw.get("boundary"), f"{field}.boundary")
    raw_holes = raw.get("holes", [])
    if not isinstance(raw_holes, Sequence) or isinstance(raw_holes, (str, bytes)):
        raise PlanningError(f"{field}.holes must be a list of polygon rings")
    holes = tuple(
        _close_ring(hole, f"{field}.holes[{index}]")
        for index, hole in enumerate(raw_holes)
    )
    inherited_edge_distance_lon = (
        default_edge_distance_lon if len(boundary) == 5 else (0.0, 0.0)
    )
    inherited_edge_distance_lat = (
        default_edge_distance_lat if len(boundary) == 5 else (0.0, 0.0)
    )
    edge_distance_lon = _parse_distance_pair(
        raw.get("edge_distance_lon", inherited_edge_distance_lon),
        f"{field}.edge_distance_lon",
    )
    edge_distance_lat = _parse_distance_pair(
        raw.get("edge_distance_lat", inherited_edge_distance_lat),
        f"{field}.edge_distance_lat",
    )
    polygon = Polygon(
        boundary=boundary,
        holes=holes,
        edge_distance_lon=edge_distance_lon,
        edge_distance_lat=edge_distance_lat,
    )
    _validate_polygon_geometry(polygon, field)
    return polygon


def _parse_regions(
    raw: Any,
    defaults: PlannerDefaults = PlannerDefaults(),
) -> Tuple[Region, ...]:
    if isinstance(raw, Mapping):
        normalized = []
        for region_id, value in raw.items():
            if isinstance(value, Mapping):
                item = dict(value)
                item.setdefault("id", region_id)
            else:
                item = {"id": region_id, "boundary": value}
            normalized.append(item)
        raw = normalized
    if not isinstance(raw, Sequence) or isinstance(raw, (str, bytes)) or not raw:
        raise PlanningError("regions must contain at least one region")
    regions = []
    seen = set()
    for index, value in enumerate(raw):
        if not isinstance(value, Mapping):
            raise PlanningError(f"regions[{index}] must be an object")
        region_id = _parse_id(value.get("id", value.get("name")), f"regions[{index}].id")
        if region_id in seen:
            raise PlanningError(f"duplicate region id: {region_id}")
        seen.add(region_id)
        guide = _parse_guide(value.get("guide"), f"regions[{index}].guide")

        region_edge_distance_lon = _parse_distance_pair(
            value.get("edge_distance_lon", defaults.edge_distance_lon),
            f"regions[{index}].edge_distance_lon",
        )
        region_edge_distance_lat = _parse_distance_pair(
            value.get("edge_distance_lat", defaults.edge_distance_lat),
            f"regions[{index}].edge_distance_lat",
        )

        raw_polygons = value.get("polygons")
        if raw_polygons is None:
            if "boundary" not in value:
                raise PlanningError(f"regions[{index}] needs polygons or boundary")
            raw_polygons = [
                {"boundary": value.get("boundary"), "holes": value.get("holes", [])}
            ]
        if (
            not isinstance(raw_polygons, Sequence)
            or isinstance(raw_polygons, (str, bytes))
            or not raw_polygons
        ):
            raise PlanningError(f"regions[{index}].polygons must not be empty")
        has_explicit_polygon_edges = any(
            isinstance(polygon, Mapping)
            and (
                "edge_distance_lon" in polygon
                or "edge_distance_lat" in polygon
            )
            for polygon in raw_polygons
        )
        inherited_edge_distance_lon = region_edge_distance_lon
        inherited_edge_distance_lat = region_edge_distance_lat
        if (
            len(raw_polygons) > 1
            and "edge_distance_lon" not in value
            and "edge_distance_lat" not in value
            and not has_explicit_polygon_edges
        ):
            # A composite region often contains touching polygons. Applying
            # an independent inset to every polygon would create artificial
            # gaps and duplicate coverage along their shared edge.
            inherited_edge_distance_lon = (0.0, 0.0)
            inherited_edge_distance_lat = (0.0, 0.0)
        polygons = tuple(
            _parse_polygon(
                polygon,
                f"regions[{index}].polygons[{polygon_index}]",
                inherited_edge_distance_lon,
                inherited_edge_distance_lat,
            )
            for polygon_index, polygon in enumerate(raw_polygons)
        )
        _validate_region_geometry(polygons, f"regions[{index}]")
        raw_start = value.get("start")
        start = None if raw_start is None else _coerce_point(raw_start, f"regions[{index}].start")
        raw_tolerance = value.get("connection_tolerance_m", 0.0)
        try:
            connection_tolerance_m = float(raw_tolerance)
        except (TypeError, ValueError) as exc:
            raise PlanningError(
                f"regions[{index}].connection_tolerance_m must be a number"
            ) from exc
        if not math.isfinite(connection_tolerance_m) or connection_tolerance_m < 0.0:
            raise PlanningError(
                f"regions[{index}].connection_tolerance_m must be non-negative"
            )
        regions.append(
            Region(
                id=region_id,
                polygons=polygons,
                start=start,
                connection_tolerance_m=connection_tolerance_m,
                guide=guide,
            )
        )
    return tuple(regions)


def _parse_connectors(raw: Any, region_ids: set[str]) -> Tuple[Connector, ...]:
    if raw is None:
        return ()
    if isinstance(raw, Mapping):
        normalized = []
        for connector_id, value in raw.items():
            if isinstance(value, Mapping):
                item = dict(value)
                item.setdefault("id", connector_id)
            else:
                item = {"id": connector_id, "path": value}
            normalized.append(item)
        raw = normalized
    if not isinstance(raw, Sequence) or isinstance(raw, (str, bytes)):
        raise PlanningError("connectors must be a list")
    connectors = []
    seen = set()
    for index, value in enumerate(raw):
        if not isinstance(value, Mapping):
            raise PlanningError(f"connectors[{index}] must be an object")
        connector_id = _parse_id(
            value.get("id", value.get("name")), f"connectors[{index}].id"
        )
        if connector_id in seen or connector_id in region_ids:
            raise PlanningError(f"duplicate map item id: {connector_id}")
        seen.add(connector_id)
        raw_from = value.get("from", value.get("from_region"))
        raw_to = value.get("to", value.get("to_region"))
        if (raw_from is None) != (raw_to is None):
            raise PlanningError(
                f"connector {connector_id} must omit both from and to together"
            )
        if raw_from is None:
            from_region = None
            to_region = None
        else:
            from_region = _parse_id(raw_from, f"connectors[{index}].from")
            to_region = _parse_id(raw_to, f"connectors[{index}].to")
            if from_region not in region_ids or to_region not in region_ids:
                raise PlanningError(
                    f"connector {connector_id} references an unknown region"
                )
        raw_path = value.get("path")
        if not isinstance(raw_path, Sequence) or isinstance(raw_path, (str, bytes)):
            raise PlanningError(f"connectors[{index}].path must be a list of points")
        path = tuple(
            _coerce_point(point, f"connectors[{index}].path") for point in raw_path
        )
        if len(path) < 2 or _distance(path[0], path[-1]) <= EPSILON:
            raise PlanningError(f"connectors[{index}].path needs two distinct endpoints")
        edge_distance_lon = (
            None
            if "edge_distance_lon" not in value
            else _parse_distance_pair(
                value["edge_distance_lon"],
                f"connectors[{index}].edge_distance_lon",
            )
        )
        edge_distance_lat = (
            None
            if "edge_distance_lat" not in value
            else _parse_distance_pair(
                value["edge_distance_lat"],
                f"connectors[{index}].edge_distance_lat",
            )
        )
        connectors.append(
            Connector(
                id=connector_id,
                from_region=from_region,
                to_region=to_region,
                path=path,
                edge_distance_lon=edge_distance_lon,
                edge_distance_lat=edge_distance_lat,
            )
        )
    return tuple(connectors)


def _parse_order(
    raw: Any,
    regions: Sequence[Region],
    connectors: Sequence[Connector],
) -> Tuple[str, ...]:
    region_ids = {region.id for region in regions}
    connector_by_id = {connector.id: connector for connector in connectors}
    if raw is None:
        if len(regions) == 1 and not connectors:
            return (regions[0].id,)
        if (
            len(connectors) == len(regions) - 1
            and all(
                connector.from_region is not None and connector.to_region is not None
                for connector in connectors
            )
        ):
            incoming = {connector.to_region for connector in connectors}
            starts = [region.id for region in regions if region.id not in incoming]
            if len(starts) == 1:
                generated = []
                current = starts[0]
                remaining = list(connectors)
                generated.append(current)
                while remaining:
                    next_connector = next(
                        (item for item in remaining if item.from_region == current),
                        None,
                    )
                    if next_connector is None:
                        break
                    generated.extend((next_connector.id, next_connector.to_region))
                    current = next_connector.to_region
                    remaining.remove(next_connector)
                if not remaining and len(generated) == 2 * len(regions) - 1:
                    return tuple(generated)
        raise PlanningError("order is required for multiple regions or connectors")
    if not isinstance(raw, Sequence) or isinstance(raw, (str, bytes)) or not raw:
        raise PlanningError("order must be a non-empty list")

    order = tuple(_parse_id(item, "order item") for item in raw)
    seen_regions = set()
    seen_connectors = set()
    for item in order:
        if item in region_ids:
            if item in seen_regions:
                raise PlanningError(f"region appears more than once in order: {item}")
            seen_regions.add(item)
        elif item in connector_by_id:
            if item in seen_connectors:
                raise PlanningError(f"connector appears more than once in order: {item}")
            seen_connectors.add(item)
        else:
            raise PlanningError(f"order references unknown map item: {item}")

    connector_ids = set(connector_by_id)
    if seen_regions != region_ids:
        missing = sorted(region_ids - seen_regions)
        raise PlanningError(f"order does not include regions: {', '.join(missing)}")
    if seen_connectors != connector_ids:
        missing = sorted(connector_ids - seen_connectors)
        raise PlanningError(f"order does not include connectors: {', '.join(missing)}")

    for previous, current in zip(order, order[1:]):
        if previous in region_ids and current in region_ids:
            raise PlanningError(
                f"regions {previous} and {current} need an explicit connector"
            )

    first_region_index = next(
        index for index, item in enumerate(order) if item in region_ids
    )
    last_region_index = max(
        index for index, item in enumerate(order) if item in region_ids
    )
    for index, item in enumerate(order):
        if item not in connector_by_id:
            continue
        connector = connector_by_id[item]
        if connector.from_region is None:
            if not (index < first_region_index or index > last_region_index):
                raise PlanningError(
                    f"unbound connector must be before the first or after the last region: {item}"
                )
            continue
        if index == 0 or index == len(order) - 1:
            raise PlanningError(f"connector must be between two regions: {item}")
        if order[index - 1] != connector.from_region or order[index + 1] != connector.to_region:
            raise PlanningError(
                f"connector {item} does not connect {order[index - 1]} to {order[index + 1]}"
            )
    return order


def load_map(source: JsonSource) -> AutoMap:
    """Load and validate a JSON map object or JSON file path."""
    payload = _read_json_source(source)
    if payload.get("type") == "FeatureCollection":
        raise PlanningError(
            "input is a GeoJSON route output, not an auto map; "
            "use the original auto_map_*.json input file"
        )
    if payload.get("format") == "rtk_auto_map_v2" or "regions" in payload:
        raw_defaults = payload.get("defaults", payload.get("default"))
        defaults = _parse_defaults(raw_defaults)
        regions = _parse_regions(payload.get("regions"), defaults)
        raw_guides = payload.get("guides")
        if raw_guides is None:
            longest_edge = _longest_edge(
                polygon.boundary
                for region in regions
                for polygon in region.polygons
            )
            guides = (longest_edge,)
        else:
            guides = _parse_guides(raw_guides)
        region_ids = {region.id for region in regions}
        connectors = _parse_connectors(
            payload.get("connectors", payload.get("bridges", [])), region_ids
        )
        order = _parse_order(payload.get("order"), regions, connectors)
        boundary = regions[0].polygons[0].boundary
        no_go = regions[0].polygons[0].holes
        raw_start = payload.get("start")
        start = None if raw_start is None else _coerce_point(raw_start, "start")
        return AutoMap(
            boundary=boundary,
            guides=guides,
            no_go=no_go,
            start=start,
            defaults=defaults,
            regions=regions,
            connectors=connectors,
            order=order,
        )

    defaults = _parse_defaults(payload.get("defaults", payload.get("default")))
    guides = _parse_guides(payload.get("guides"))
    boundary = _close_ring(payload.get("boundary"), "boundary")

    raw_no_go = payload.get("no_go", [])
    if not isinstance(raw_no_go, Sequence) or isinstance(raw_no_go, (str, bytes)):
        raise PlanningError("no_go must be a list of polygon rings")
    no_go = tuple(
        _close_ring(ring, f"no_go[{index}]")
        for index, ring in enumerate(raw_no_go)
    )

    raw_start = payload.get("start")
    start = None if raw_start is None else _coerce_point(raw_start, "start")
    return AutoMap(
        boundary=boundary,
        guides=guides,
        no_go=no_go,
        start=start,
        defaults=defaults,
        regions=(Region("legacy", (Polygon(boundary, no_go),), start),),
        order=("legacy",),
        legacy=True,
    )


def _read_yaml_source(source: JsonSource) -> Mapping[str, Any]:
    if isinstance(source, Mapping):
        payload = source
    else:
        if yaml is None:
            raise PlanningError("legacy YAML input requires PyYAML")
        try:
            text = Path(source).read_text(encoding="utf-8")
        except OSError as exc:
            raise PlanningError(f"cannot read input YAML: {source}") from exc
        try:
            payload = yaml.safe_load(text)
        except Exception as exc:
            raise PlanningError(f"invalid input YAML: {exc}") from exc
    if not isinstance(payload, Mapping):
        raise PlanningError("input YAML must contain an object")
    return payload


def _yaml_area_point(area: Mapping[str, Any], area_name: str, field: str) -> Point:
    if field not in area:
        raise PlanningError(f"area {area_name} is missing {field}")
    return _coerce_point(area[field], f"area {area_name}.{field}")


def _approximate_fourth_corner(a: Point, b: Point, c: Point) -> Point:
    """Complete three ordered calibration points without flattening the slope."""
    return (a[0] + c[0] - b[0], a[1] + c[1] - b[1])


def _yaml_rectangle_polygon(
    area: Mapping[str, Any],
    area_name: str,
    defaults: PlannerDefaults,
    orthogonalize: bool = False,
) -> Tuple[dict[str, Any], Tuple[Point, ...]]:
    a = _yaml_area_point(area, area_name, "calib_point_a")
    b = _yaml_area_point(area, area_name, "calib_point_b")
    c = _yaml_area_point(area, area_name, "calib_point_c")
    if orthogonalize:
        # Match full_path_planner_dense: AB fixes the height and AC fixes the
        # signed width after projection onto the perpendicular direction.
        lon_scale = METERS_PER_DEGREE_LON * math.cos(math.radians(a[1]))
        lat_scale = METERS_PER_DEGREE_LAT
        ab = ((b[0] - a[0]) * lon_scale, (b[1] - a[1]) * lat_scale)
        ac = ((c[0] - a[0]) * lon_scale, (c[1] - a[1]) * lat_scale)
        ab_length = math.hypot(*ab)
        if ab_length <= EPSILON:
            raise PlanningError(f"area {area_name} has a degenerate height edge")
        perpendicular = (-ab[1] / ab_length, ab[0] / ab_length)
        width = ac[0] * perpendicular[0] + ac[1] * perpendicular[1]
        d_xy = (perpendicular[0] * width, perpendicular[1] * width)
        rect_c_xy = (d_xy[0] + ab[0], d_xy[1] + ab[1])
        d = (a[0] + d_xy[0] / lon_scale, a[1] + d_xy[1] / lat_scale)
        rect_c = (
            a[0] + rect_c_xy[0] / lon_scale,
            a[1] + rect_c_xy[1] / lat_scale,
        )
        boundary = [list(point) for point in (d, a, b, rect_c, d)]
        corners = (a, b, rect_c, d)
        boundary_point_annotations = [
            {
                "boundary_index": 0,
                "label": "D",
                "map_position": "southwest",
                "source": "automatic",
                "calculation": "A + C - B, after orthogonalizing C against AB",
                "note_zh": "西南角；由 A、B、C 自动正交化计算",
            },
            {
                "boundary_index": 1,
                "label": "A",
                "map_position": "southeast",
                "source": "manual",
                "input_field": "calib_point_a",
                "note_zh": "东南角；需要手动标记",
            },
            {
                "boundary_index": 2,
                "label": "B",
                "map_position": "northeast",
                "source": "manual",
                "input_field": "calib_point_b",
                "note_zh": "东北角；需要手动标记",
            },
            {
                "boundary_index": 3,
                "label": "C_prime",
                "map_position": "northwest",
                "source": "automatic",
                "manual_input_field": "calib_point_c",
                "calculation": "orthogonal projection derived from A, B, and C",
                "note_zh": "西北角；不是原始 C，按 A/B/C 自动正交化计算",
            },
            {
                "boundary_index": 4,
                "label": "D",
                "map_position": "closure",
                "source": "automatic",
                "calculation": "copy boundary[0]",
                "note_zh": "闭合点；自动复制第 1 个点，不需要手动标记",
            },
        ]
        boundary_order = ["D", "A", "B", "C_prime", "D"]
    else:
        d = _approximate_fourth_corner(a, b, c)
        boundary = [list(point) for point in (d, a, b, c, d)]
        corners = (a, b, c, d)
        boundary_point_annotations = [
            {
                "boundary_index": 0,
                "label": "D",
                "map_position": "southwest",
                "source": "automatic",
                "calculation": "A + C - B",
                "note_zh": "西南角；由 A、B、C 自动计算",
            },
            {
                "boundary_index": 1,
                "label": "A",
                "map_position": "southeast",
                "source": "manual",
                "input_field": "calib_point_a",
                "note_zh": "东南角；需要手动标记",
            },
            {
                "boundary_index": 2,
                "label": "B",
                "map_position": "northeast",
                "source": "manual",
                "input_field": "calib_point_b",
                "note_zh": "东北角；需要手动标记",
            },
            {
                "boundary_index": 3,
                "label": "C",
                "map_position": "northwest",
                "source": "manual",
                "input_field": "calib_point_c",
                "note_zh": "西北角；需要手动标记",
            },
            {
                "boundary_index": 4,
                "label": "D",
                "map_position": "closure",
                "source": "automatic",
                "calculation": "copy boundary[0]",
                "note_zh": "闭合点；自动复制第 1 个点，不需要手动标记",
            },
        ]
        boundary_order = ["D", "A", "B", "C", "D"]
    for annotation in boundary_point_annotations:
        annotation["point_number"] = annotation["boundary_index"] + 1
        annotation["coordinate"] = list(boundary[annotation["boundary_index"]])
    polygon: dict[str, Any] = {
        "source_area": area_name,
        "boundary": boundary,
        "boundary_order": boundary_order,
        "boundary_point_annotations": boundary_point_annotations,
        "manual_calibration_points": {
            "A": list(a),
            "B": list(b),
            "C": list(c),
        },
        "holes": [],
    }
    for key in ("edge_distance_lon", "edge_distance_lat"):
        if key in area:
            polygon[key] = area[key]
    _parse_polygon(
        polygon,
        f"area {area_name}",
        defaults.edge_distance_lon,
        defaults.edge_distance_lat,
    )
    return polygon, corners


def _nearest_corner(point: Point, corners: Sequence[Point]) -> Point:
    if not corners:
        raise PlanningError("cannot attach a connector to an empty region")
    return min(corners, key=lambda candidate: _distance(point, candidate))


def _yaml_connector_offsets(
    area: Mapping[str, Any],
    area_name: str,
    defaults: PlannerDefaults,
) -> Tuple[Tuple[float, float], Tuple[float, float]]:
    """Resolve bridge offsets with the same defaults as the dense planner."""
    return (
        _parse_distance_pair(
            area.get("edge_distance_lon", defaults.edge_distance_lon),
            f"area {area_name}.edge_distance_lon",
        ),
        _parse_distance_pair(
            area.get("edge_distance_lat", defaults.edge_distance_lat),
            f"area {area_name}.edge_distance_lat",
        ),
    )


def convert_legacy_yaml_to_map(source: JsonSource) -> dict[str, Any]:
    """Convert the legacy area YAML into an ``rtk_auto_map_v2`` payload."""
    payload = _read_yaml_source(source)
    raw_areas = payload.get("areas")
    if not isinstance(raw_areas, Sequence) or isinstance(raw_areas, (str, bytes)):
        raise PlanningError("input YAML areas must be a list")
    if not raw_areas:
        raise PlanningError("input YAML areas must not be empty")

    defaults = _parse_defaults(payload.get("default"))
    region_polygons: dict[str, list[dict[str, Any]]] = {}
    region_corners: dict[str, list[Point]] = {}
    region_guides: dict[str, str] = {}
    region_order: list[str] = []
    area_sequence: list[tuple[str, str]] = []
    pre_connectors: list[dict[str, Any]] = []
    post_connectors: list[dict[str, Any]] = []
    bridge_areas: list[dict[str, Any]] = []
    internal_bridge_areas: list[dict[str, Any]] = []

    def area_region_id(area_name: str) -> Optional[str]:
        """Return the logical region represented by a cleaning area name."""
        prefix, separator, suffix = area_name.partition("_")
        if not separator or not prefix.startswith("E") or not prefix[1:].isdigit():
            if area_name.startswith("E") and area_name[1:].isdigit():
                return area_name
            return None
        if area_name == "E17_upB_downC":
            return None
        if area_name in {"E17_up", "E17_down"}:
            return area_name
        return prefix

    def add_region_area(
        area_name: str,
        raw_area: Mapping[str, Any],
        index: int,
    ) -> None:
        region_id = area_region_id(area_name)
        if region_id is None:
            raise PlanningError(f"unsupported area name: {area_name}")
        if region_id not in region_polygons:
            region_polygons[region_id] = []
            region_corners[region_id] = []
            region_order.append(region_id)
        guide = _parse_guide(raw_area.get("guide"), f"areas[{index}].guide")
        existing_guide = region_guides.get(region_id)
        if (
            guide is not None
            and existing_guide is not None
            and guide != existing_guide
        ):
            raise PlanningError(
                f"cleaning region {region_id} has conflicting guides: "
                f"{existing_guide} and {guide}"
            )
        if guide is not None:
            region_guides[region_id] = guide
        polygon, corners = _yaml_rectangle_polygon(
            raw_area,
            area_name,
            defaults,
            orthogonalize=area_name == "E16",
        )
        region_polygons[region_id].append(polygon)
        region_corners[region_id].extend(corners)
        if not area_sequence or area_sequence[-1] != ("region", region_id):
            area_sequence.append(("region", region_id))

    for index, raw_area in enumerate(raw_areas):
        if not isinstance(raw_area, Mapping):
            raise PlanningError(f"areas[{index}] must be an object")
        raw_name = raw_area.get("name")
        if not isinstance(raw_name, str) or not raw_name.strip():
            raise PlanningError(f"areas[{index}].name must be a non-empty string")
        area_name = raw_name.strip()

        if area_name == "E17_upB_downC":
            a = _yaml_area_point(raw_area, area_name, "calib_point_a")
            b = _yaml_area_point(raw_area, area_name, "calib_point_b")
            internal_bridge_areas.append(
                {
                    "id": area_name,
                    "from": "E17_up",
                    "to": "E17_down",
                    "path": [list(a), list(b)],
                    "edge_distance_lon": list(
                        _yaml_connector_offsets(
                            raw_area, area_name, defaults
                        )[0]
                    ),
                    "edge_distance_lat": list(
                        _yaml_connector_offsets(
                            raw_area, area_name, defaults
                        )[1]
                    ),
                }
            )
            area_sequence.append(("connector", area_name))
            continue

        if area_region_id(area_name) is not None:
            add_region_area(area_name, raw_area, index)
            continue

        if area_name.startswith("bridge_"):
            a = _yaml_area_point(raw_area, area_name, "calib_point_a")
            b = _yaml_area_point(raw_area, area_name, "calib_point_b")
            edge_distance_lon, edge_distance_lat = _yaml_connector_offsets(
                raw_area, area_name, defaults
            )
            bridge_areas.append(
                {
                    "id": area_name,
                    "a": a,
                    "b": b,
                    "edge_distance_lon": list(edge_distance_lon),
                    "edge_distance_lat": list(edge_distance_lat),
                }
            )
            area_sequence.append(("connector_candidate", area_name))
            continue

        if area_name.startswith("back_"):
            a = _yaml_area_point(raw_area, area_name, "calib_point_a")
            b = _yaml_area_point(raw_area, area_name, "calib_point_b")
            edge_distance_lon, edge_distance_lat = _yaml_connector_offsets(
                raw_area, area_name, defaults
            )
            post_connectors.append(
                {
                    "id": area_name,
                    "path": [list(a), list(b)],
                    "edge_distance_lon": list(edge_distance_lon),
                    "edge_distance_lat": list(edge_distance_lat),
                }
            )
            continue

        raise PlanningError(f"unsupported area name: {area_name}")

    missing_regions = [
        region_id for region_id, polygons in region_polygons.items() if not polygons
    ]
    if missing_regions:
        raise PlanningError(
            "input YAML is missing cleaning regions: " + ", ".join(missing_regions)
        )

    def token_region(token: str, point: Point) -> Optional[str]:
        digits = ""
        for character in token:
            if not character.isdigit():
                break
            digits += character
        if not digits:
            return None
        base = f"E{digits}"
        candidates = [
            region_id
            for region_id in region_polygons
            if region_id == base or region_id.startswith(f"{base}_")
        ]
        if not candidates:
            return None
        suffix = token[len(digits) :].lower()
        if "up" in suffix:
            up_candidates = [item for item in candidates if item.endswith("_up")]
            if up_candidates:
                return up_candidates[0]
        if "down" in suffix:
            down_candidates = [
                item for item in candidates if item.endswith("_down")
            ]
            if down_candidates:
                return down_candidates[0]
        if len(candidates) == 1:
            return candidates[0]
        return min(
            candidates,
            key=lambda item: _distance(
                point, _nearest_corner(point, region_corners[item])
            ),
        )

    inter_connectors: list[dict[str, Any]] = []
    internal_by_id = {item["id"]: item for item in internal_bridge_areas}
    for bridge in bridge_areas:
        body = bridge["id"][len("bridge_") :]
        if "-" not in body:
            from_region = to_region = None
        else:
            from_token, to_token = body.split("-", 1)
            from_region = token_region(from_token, bridge["a"])
            to_region = token_region(to_token, bridge["b"])
        path = [list(bridge["a"]), list(bridge["b"])]
        if from_region is not None and to_region is not None:
            if bridge["id"] == "bridge_16A-17downB" and "E17_up" in region_polygons:
                internal = internal_by_id.get("E17_upB_downC")
                if internal is not None:
                    path.extend(
                        [internal["path"][1], internal["path"][0]]
                    )
                    to_region = "E17_up"
            nearest = _nearest_corner(path[-1], region_corners[to_region])
            if _distance(path[-1], nearest) > EPSILON:
                path.append(list(nearest))
            inter_connectors.append(
                {
                    "id": bridge["id"],
                    "from": from_region,
                    "to": to_region,
                    "path": path,
                    "edge_distance_lon": bridge["edge_distance_lon"],
                    "edge_distance_lat": bridge["edge_distance_lat"],
                }
            )
        else:
            pre_connectors.append(
                {
                    "id": bridge["id"],
                    "path": path,
                    "edge_distance_lon": bridge["edge_distance_lon"],
                    "edge_distance_lat": bridge["edge_distance_lat"],
                }
            )

    inter_connectors.extend(internal_bridge_areas)
    # Keep the source order for the inter-region connectors, including the
    # explicit E17 gap crossing, rather than grouping by connector type.
    inter_by_id = {item["id"]: item for item in inter_connectors}
    ordered_inter_connectors = [
        inter_by_id[item_id]
        for kind, item_id in area_sequence
        if kind in {"connector", "connector_candidate"}
        and item_id in inter_by_id
    ]
    inter_connectors = ordered_inter_connectors

    connectors = pre_connectors + inter_connectors + post_connectors
    ordered_regions_and_inter = [
        item_id
        for kind, item_id in area_sequence
        if (kind == "region" and item_id in region_polygons)
        or (
            kind in {"connector", "connector_candidate"}
            and item_id in inter_by_id
        )
    ]
    seen_order_items = set()
    physical_order = []
    for item_id in ordered_regions_and_inter:
        if item_id not in seen_order_items:
            physical_order.append(item_id)
            seen_order_items.add(item_id)

    default_payload = {
        "interval": defaults.interval,
        "start_corner": defaults.start_corner,
        "swap_wh_select": defaults.swap_wh_select,
        "edge_distance_lon": list(defaults.edge_distance_lon),
        "edge_distance_lat": list(defaults.edge_distance_lat),
    }
    regions = []
    for region_id in region_order:
        region: dict[str, Any] = {
            "id": region_id,
            "polygons": region_polygons[region_id],
        }
        if len(region_polygons[region_id]) > 1:
            region["connection_tolerance_m"] = 3.0 if region_id == "E13" else 2.0
        if region_id in region_guides:
            region["guide"] = region_guides[region_id]
        regions.append(region)

    raw_guides = payload.get("guides")
    if raw_guides is None:
        longest_edge = _longest_edge(
            polygon["boundary"]
            for polygons in region_polygons.values()
            for polygon in polygons
        )
        guides = [[list(longest_edge[0]), list(longest_edge[1])]]
    else:
        guides = [
            [list(point) for point in guide]
            for guide in _parse_guides(raw_guides)
        ]

    return {
        "format": "rtk_auto_map_v2",
        "guides": guides,
        "defaults": default_payload,
        "regions": regions,
        "connectors": connectors,
        "order": [item["id"] for item in pre_connectors]
        + physical_order
        + [item["id"] for item in post_connectors],
    }


def _distance(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _geo_distance_m(first: Point, second: Point) -> float:
    latitude = math.radians((first[1] + second[1]) / 2.0)
    delta_lon = (second[0] - first[0]) * METERS_PER_DEGREE_LON * math.cos(latitude)
    delta_lat = (second[1] - first[1]) * METERS_PER_DEGREE_LAT
    return math.hypot(delta_lon, delta_lat)


def _geo_axis_angle(first: Point, second: Point) -> float:
    latitude = math.radians((first[1] + second[1]) / 2.0)
    delta_lon = (second[0] - first[0]) * METERS_PER_DEGREE_LON * math.cos(latitude)
    delta_lat = (second[1] - first[1]) * METERS_PER_DEGREE_LAT
    if math.hypot(delta_lon, delta_lat) <= EPSILON:
        raise PlanningError("boundary edge has no usable direction")
    return math.atan2(delta_lat, delta_lon) % math.pi


def _longest_edge(rings: Sequence[Sequence[Point]]) -> Tuple[Point, Point]:
    edges = [edge for ring in rings for edge in _ring_edges(ring)]
    if not edges:
        raise PlanningError("regions do not contain a usable boundary edge")
    return max(edges, key=lambda edge: _geo_distance_m(edge[0], edge[1]))


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


def _region_axis_angle(
    region: Region,
    default_axis: float,
    use_region_fallback: bool,
) -> float:
    if region.guide == "horizontal":
        return 0.0
    if region.guide == "vertical":
        return math.pi / 2.0
    if not use_region_fallback:
        return default_axis
    longest = _longest_edge(
        polygon.boundary for polygon in region.polygons
    )
    return _geo_axis_angle(longest[0], longest[1])


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


RotatedPolygon = Tuple[
    Tuple[LocalPoint, ...], Tuple[Tuple[LocalPoint, ...], ...]
]
RotatedRegion = Tuple[RotatedPolygon, ...]
CoverageSegment = Tuple[LocalPoint, LocalPoint]
CoverageGroup = Tuple[CoverageSegment, ...]


def _edge_adjusted_ring(
    ring: Sequence[LocalPoint],
    edge_distance_lon: Tuple[float, float],
    edge_distance_lat: Tuple[float, float],
) -> Tuple[LocalPoint, ...]:
    if all(abs(value) <= EPSILON for value in (*edge_distance_lon, *edge_distance_lat)):
        return tuple(ring)
    if len(ring) != 5:
        raise PlanningError(
            "edge_distance_lon/lat require a four-corner polygon boundary"
        )

    signed_area = sum(
        first[0] * second[1] - second[0] * first[1]
        for first, second in _ring_edges(ring)
    )
    if abs(signed_area) <= EPSILON:
        raise PlanningError("edge distances require a non-degenerate polygon")

    side_directions = {
        "bottom": (0.0, -1.0),
        "right": (1.0, 0.0),
        "top": (0.0, 1.0),
        "left": (-1.0, 0.0),
    }

    def inward_normal(first: LocalPoint, second: LocalPoint) -> LocalPoint:
        direction = _subtract(second, first)
        length = math.hypot(direction[0], direction[1])
        if length <= EPSILON:
            raise PlanningError("edge distances require non-zero polygon edges")
        if signed_area > 0.0:
            return (-direction[1] / length, direction[0] / length)
        return (direction[1] / length, -direction[0] / length)

    edge_normals = tuple(
        inward_normal(first, second) for first, second in _ring_edges(ring)
    )

    # Identify sides by their outward normal rather than assuming that the
    # ring starts at a particular corner.  This preserves the legacy
    # [right, left] and [bottom, top] distance order for either ring
    # orientation, including slightly skewed measured edges.
    side_names = tuple(side_directions)
    side_assignment = max(
        permutations(side_names),
        key=lambda assignment: sum(
            -edge_normals[index][0] * side_directions[assignment[index]][0]
            - edge_normals[index][1] * side_directions[assignment[index]][1]
            for index in range(4)
        ),
    )
    offset_by_side = {
        "bottom": edge_distance_lat[0],
        "right": edge_distance_lon[0],
        "top": edge_distance_lat[1],
        "left": edge_distance_lon[1],
    }

    def offset_line(
        first: LocalPoint, second: LocalPoint, distance: float
    ) -> Tuple[LocalPoint, LocalPoint]:
        direction = _subtract(second, first)
        inward = inward_normal(first, second)
        return (
            (
                first[0] + inward[0] * distance,
                first[1] + inward[1] * distance,
            ),
            direction,
        )

    offset_lines = tuple(
        offset_line(
            first,
            second,
            offset_by_side[side_assignment[index]],
        )
        for index, (first, second) in enumerate(_ring_edges(ring))
    )

    def intersect_lines(
        first: Tuple[LocalPoint, LocalPoint],
        second: Tuple[LocalPoint, LocalPoint],
    ) -> LocalPoint:
        first_point, first_direction = first
        second_point, second_direction = second
        denominator = _cross(first_direction, second_direction)
        if abs(denominator) <= 1e-12:
            raise PlanningError("edge distances produce parallel polygon edges")
        delta = _subtract(second_point, first_point)
        ratio = _cross(delta, second_direction) / denominator
        point = (
            first_point[0] + ratio * first_direction[0],
            first_point[1] + ratio * first_direction[1],
        )
        if not all(math.isfinite(value) for value in point):
            raise PlanningError("edge distances produce an invalid polygon")
        return point

    adjusted_vertices = tuple(
        intersect_lines(offset_lines[(index - 1) % 4], offset_lines[index])
        for index in range(4)
    )
    adjusted_ring = adjusted_vertices + (adjusted_vertices[0],)
    _validate_simple_ring(adjusted_ring, "edge-adjusted polygon")
    adjusted_area = sum(
        first[0] * second[1] - second[0] * first[1]
        for first, second in _ring_edges(adjusted_ring)
    )
    if signed_area * adjusted_area <= EPSILON:
        raise PlanningError("edge distances leave no usable polygon")
    return adjusted_ring


def _rotated_region_geometry(
    map_data: AutoMap,
    region: Region,
    axis_angle: float,
    apply_edge_distance: bool = True,
) -> Tuple[_LocalFrame, RotatedRegion]:
    frame = _LocalFrame.from_map(map_data)

    def local_ring(ring: Sequence[Point]) -> Tuple[LocalPoint, ...]:
        return tuple(frame.to_xy(point) for point in ring)

    def rotate_ring(ring: Sequence[LocalPoint]) -> Tuple[LocalPoint, ...]:
        return tuple(_rotate(point, axis_angle) for point in ring)

    geometry = tuple(
        (
            rotate_ring(
                _edge_adjusted_ring(
                    local_ring(polygon.boundary),
                    polygon.edge_distance_lon,
                    polygon.edge_distance_lat,
                )
                if apply_edge_distance
                else local_ring(polygon.boundary)
            ),
            tuple(rotate_ring(local_ring(hole)) for hole in polygon.holes),
        )
        for polygon in region.polygons
    )
    return frame, geometry


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


def _ring_intervals(
    ring: Sequence[LocalPoint], sweep_value: float
) -> list[Tuple[float, float]]:
    intersections = _line_intersections(ring, sweep_value)
    intersections.sort()
    if len(intersections) % 2:
        raise PlanningError("scanline intersects invalid map geometry")
    return [
        (left, right)
        for left, right in zip(intersections[::2], intersections[1::2])
        if right - left > EPSILON
    ]


def _merge_intervals(intervals: Sequence[Tuple[float, float]]) -> list[Tuple[float, float]]:
    if not intervals:
        return []
    ordered = sorted(intervals)
    merged = [ordered[0]]
    for left, right in ordered[1:]:
        previous_left, previous_right = merged[-1]
        if left <= previous_right + EPSILON:
            merged[-1] = (previous_left, max(previous_right, right))
        else:
            merged.append((left, right))
    return merged


def _subtract_intervals(
    intervals: Sequence[Tuple[float, float]],
    cuts: Sequence[Tuple[float, float]],
) -> list[Tuple[float, float]]:
    remaining = list(intervals)
    for cut_left, cut_right in _merge_intervals(cuts):
        next_remaining = []
        for left, right in remaining:
            if cut_right <= left + EPSILON or cut_left >= right - EPSILON:
                next_remaining.append((left, right))
                continue
            if left < cut_left - EPSILON:
                next_remaining.append((left, min(right, cut_left)))
            if cut_right < right - EPSILON:
                next_remaining.append((max(left, cut_right), right))
        remaining = next_remaining
    return [(left, right) for left, right in remaining if right - left > EPSILON]


def _polygon_scanline_intervals(
    polygon: RotatedPolygon, sweep_value: float
) -> list[Tuple[float, float]]:
    boundary, holes = polygon
    allowed = _ring_intervals(boundary, sweep_value)
    hole_intervals = [
        interval
        for hole in holes
        for interval in _ring_intervals(hole, sweep_value)
    ]
    return _subtract_intervals(allowed, hole_intervals)


def _clip_scanline_intervals(
    intervals: Sequence[Tuple[float, float]], edge_clearance: float
) -> list[Tuple[float, float]]:
    clipped = []
    for left, right in _merge_intervals(intervals):
        left += edge_clearance
        right -= edge_clearance
        if right - left > EPSILON:
            clipped.append((left, right))
    return clipped


def _region_scanline_intervals(
    geometry: RotatedRegion,
    sweep_value: float,
    edge_clearance: float,
) -> list[Tuple[float, float]]:
    intervals = [
        interval
        for polygon in geometry
        for interval in _polygon_scanline_intervals(polygon, sweep_value)
    ]
    return _clip_scanline_intervals(intervals, edge_clearance)


def _region_scan_values(
    geometry: RotatedRegion, sweep_spacing: float, edge_clearance: float
) -> list[float]:
    values = [point[1] for boundary, _ in geometry for point in boundary]
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
        raise PlanningError("no scanline fits inside the region")
    return result


def extract_scanline_intervals(
    map_data: AutoMap,
    axis_angle: float,
    sweep_value: float,
    edge_clearance: float,
    region_id: Optional[str] = None,
) -> list[Tuple[float, float]]:
    """Return safe local U intervals for one local V scanline."""
    if edge_clearance < 0.0:
        raise PlanningError("edge_clearance must be non-negative")
    if region_id is not None:
        region = next((item for item in map_data.regions if item.id == region_id), None)
        if region is None:
            raise PlanningError(f"unknown region: {region_id}")
        _, geometry = _rotated_region_geometry(map_data, region, axis_angle)
        return _region_scanline_intervals(geometry, sweep_value, edge_clearance)

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


def _point_on_ring(point: LocalPoint, ring: Sequence[LocalPoint]) -> bool:
    return any(_point_on_segment(point, first, second) for first, second in zip(ring, ring[1:]))


def _point_is_allowed(
    point: LocalPoint,
    boundary: Sequence[LocalPoint],
    no_go: Sequence[Sequence[LocalPoint]],
) -> bool:
    if not _point_in_ring(point, boundary):
        return False
    return not any(_point_in_ring(point, ring) for ring in no_go)


def _point_is_allowed_region(
    point: LocalPoint,
    geometry: RotatedRegion,
    allow_hole_boundary: bool = False,
) -> bool:
    for boundary, holes in geometry:
        if not _point_in_ring(point, boundary):
            continue
        blocked = False
        for hole in holes:
            if _point_in_ring(point, hole):
                if allow_hole_boundary and _point_on_ring(point, hole):
                    continue
                blocked = True
                break
        if not blocked:
            return True
    return False


def _distance_to_segment(
    point: LocalPoint, first: LocalPoint, second: LocalPoint
) -> float:
    direction = _subtract(second, first)
    length_squared = direction[0] ** 2 + direction[1] ** 2
    if length_squared <= EPSILON:
        return _local_distance(point, first)
    ratio = (
        (point[0] - first[0]) * direction[0]
        + (point[1] - first[1]) * direction[1]
    ) / length_squared
    ratio = max(0.0, min(1.0, ratio))
    projection = (
        first[0] + ratio * direction[0],
        first[1] + ratio * direction[1],
    )
    return _local_distance(point, projection)


def _distance_to_ring(point: LocalPoint, ring: Sequence[LocalPoint]) -> float:
    return min(
        _distance_to_segment(point, first, second)
        for first, second in _ring_edges(ring)
    )


def _point_is_allowed_region_with_tolerance(
    point: LocalPoint,
    geometry: RotatedRegion,
    connection_tolerance_m: float,
) -> bool:
    if _point_is_allowed_region(point, geometry, allow_hole_boundary=False):
        return True
    if connection_tolerance_m <= EPSILON:
        return False

    # A tolerance may bridge a small gap between polygons, but never permits
    # travel through a hole or an arbitrary point far outside the region.
    boundary_distance = math.inf
    for boundary, holes in geometry:
        if any(_point_in_ring(point, hole) for hole in holes):
            return False
        boundary_distance = min(boundary_distance, _distance_to_ring(point, boundary))
    return boundary_distance <= connection_tolerance_m + EPSILON


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


def _segment_intersection_parameters(
    start: LocalPoint,
    end: LocalPoint,
    ring: Sequence[LocalPoint],
) -> list[float]:
    direction = _subtract(end, start)
    direction_length_squared = direction[0] ** 2 + direction[1] ** 2
    if direction_length_squared <= 1e-18:
        return [0.0]
    parameters = []
    for edge_start, edge_end in _ring_edges(ring):
        edge = _subtract(edge_end, edge_start)
        offset = _subtract(edge_start, start)
        denominator = _cross(direction, edge)
        if abs(denominator) > 1e-12:
            first_ratio = _cross(offset, edge) / denominator
            second_ratio = _cross(offset, direction) / denominator
            if -1e-9 <= first_ratio <= 1.0 + 1e-9 and -1e-9 <= second_ratio <= 1.0 + 1e-9:
                parameters.append(max(0.0, min(1.0, first_ratio)))
            continue
        if abs(_cross(offset, direction)) > 1e-12:
            continue
        for point in (edge_start, edge_end):
            ratio = (
                (point[0] - start[0]) * direction[0]
                + (point[1] - start[1]) * direction[1]
            ) / direction_length_squared
            if -1e-9 <= ratio <= 1.0 + 1e-9:
                parameters.append(max(0.0, min(1.0, ratio)))
    parameters.extend((0.0, 1.0))
    return sorted(set(round(parameter, 12) for parameter in parameters))


def _line_enters_ring(
    start: LocalPoint,
    end: LocalPoint,
    ring: Sequence[LocalPoint],
) -> bool:
    parameters = _segment_intersection_parameters(start, end, ring)
    for parameter in parameters:
        point = (
            start[0] + (end[0] - start[0]) * parameter,
            start[1] + (end[1] - start[1]) * parameter,
        )
        if _point_in_ring(point, ring):
            return True
    for first_parameter, second_parameter in zip(parameters, parameters[1:]):
        if second_parameter - first_parameter <= 1e-12:
            continue
        midpoint_parameter = (first_parameter + second_parameter) / 2.0
        midpoint = (
            start[0] + (end[0] - start[0]) * midpoint_parameter,
            start[1] + (end[1] - start[1]) * midpoint_parameter,
        )
        if _point_in_ring(midpoint, ring):
            return True
    return False


def _line_is_allowed_region(
    start: LocalPoint,
    end: LocalPoint,
    geometry: RotatedRegion,
    connection_tolerance_m: float = 0.0,
) -> bool:
    parameters = {0.0, 1.0}
    for boundary, holes in geometry:
        parameters.update(_segment_intersection_parameters(start, end, boundary))
        for hole in holes:
            parameters.update(_segment_intersection_parameters(start, end, hole))
    ordered_parameters = sorted(parameters)
    for ratio in ordered_parameters:
        point = (
            start[0] + (end[0] - start[0]) * ratio,
            start[1] + (end[1] - start[1]) * ratio,
        )
        if not _point_is_allowed_region_with_tolerance(
            point, geometry, connection_tolerance_m
        ):
            return False
    for first_ratio, second_ratio in zip(ordered_parameters, ordered_parameters[1:]):
        if second_ratio - first_ratio <= 1e-12:
            continue
        ratio = (first_ratio + second_ratio) / 2.0
        point = (
            start[0] + (end[0] - start[0]) * ratio,
            start[1] + (end[1] - start[1]) * ratio,
        )
        if not _point_is_allowed_region_with_tolerance(
            point, geometry, connection_tolerance_m
        ):
            return False
    return True


def _connector_candidates(start: LocalPoint, end: LocalPoint) -> list[list[LocalPoint]]:
    candidates = [[start, end]]
    first_corner = (end[0], start[1])
    second_corner = (start[0], end[1])
    candidates.extend([[start, first_corner, end], [start, second_corner, end]])
    return candidates


def _safe_connector_path(
    start: LocalPoint,
    end: LocalPoint,
    geometry: RotatedRegion,
    max_connector: float,
    connection_tolerance_m: float = 0.0,
) -> Optional[Tuple[LocalPoint, ...]]:
    """Return a safe connector path, preferring short paths and few turns."""
    if _local_distance(start, end) <= EPSILON:
        return (start,)

    direct_candidates: list[Tuple[Tuple[LocalPoint, ...], float]] = []
    for candidate in _connector_candidates(start, end)[1:]:
        if all(
            _line_is_allowed_region(
                first, second, geometry, connection_tolerance_m
            )
            for first, second in zip(candidate, candidate[1:])
        ):
            length = _path_length(candidate)
            if length <= max_connector + EPSILON:
                direct_candidates.append((tuple(candidate), length))
    if direct_candidates:
        return min(
            direct_candidates,
            key=lambda item: (item[1], _turn_count(item[0])),
        )[0]

    try:
        return _find_region_connector(
            start,
            end,
            geometry,
            max_connector,
            connection_tolerance_m,
        )
    except PlanningError:
        return None


def _safe_connector_length(
    start: LocalPoint,
    end: LocalPoint,
    geometry: RotatedRegion,
    max_connector: float,
    connection_tolerance_m: float = 0.0,
) -> Optional[float]:
    """Return the length of the shortest safe connector."""
    path = _safe_connector_path(
        start,
        end,
        geometry,
        max_connector,
        connection_tolerance_m,
    )
    return None if path is None else _path_length(path)


def _bridge_attachment_connector(
    start: LocalPoint,
    end: LocalPoint,
    geometry: RotatedRegion,
    max_connector: float,
    connection_tolerance_m: float = 0.0,
) -> Tuple[LocalPoint, ...]:
    """Prefer one safe direct bridge attachment, then use the grid fallback."""
    if _local_distance(start, end) <= EPSILON:
        return (start,)

    direct_length = _local_distance(start, end)
    if direct_length <= max_connector + EPSILON and _line_is_allowed_region(
        start, end, geometry, connection_tolerance_m
    ):
        return (start, end)

    return _find_region_connector(
        start,
        end,
        geometry,
        max_connector,
        connection_tolerance_m,
    )


def _project_connector_endpoint(
    point: LocalPoint,
    raw_geometry: RotatedRegion,
    adjusted_geometry: RotatedRegion,
    region_id: str,
    endpoint_name: str,
) -> LocalPoint:
    """Move a raw bridge endpoint onto the effective inset region boundary."""
    for raw_polygon, adjusted_polygon in zip(raw_geometry, adjusted_geometry):
        if not _point_is_allowed_region(
            point,
            (raw_polygon,),
            allow_hole_boundary=False,
        ):
            continue
        if _point_is_allowed_region(
            point,
            (adjusted_polygon,),
            allow_hole_boundary=False,
        ):
            return point

        raw_boundary, _ = raw_polygon
        adjusted_boundary, _ = adjusted_polygon
        if len(raw_boundary) != 5 or len(adjusted_boundary) != 5:
            continue

        adjusted_x = (
            min(point[0] for point in adjusted_boundary[:-1]),
            max(point[0] for point in adjusted_boundary[:-1]),
        )
        adjusted_y = (
            min(point[1] for point in adjusted_boundary[:-1]),
            max(point[1] for point in adjusted_boundary[:-1]),
        )
        candidate = (
            min(max(point[0], adjusted_x[0]), adjusted_x[1]),
            min(max(point[1], adjusted_y[0]), adjusted_y[1]),
        )
        if _point_is_allowed_region(
            candidate,
            (adjusted_polygon,),
            allow_hole_boundary=False,
        ):
            return candidate

    raise PlanningError(
        f"connector {endpoint_name} is not projectable into region {region_id}"
    )


def _path_endpoint_direction(
    path: Sequence[LocalPoint],
    from_start: bool,
) -> LocalPoint:
    """Return a unit direction for one end of a connector polyline."""
    indices = (
        range(1, len(path))
        if from_start
        else range(len(path) - 2, -1, -1)
    )
    endpoint = path[0] if from_start else path[-1]
    for index in indices:
        other = path[index]
        delta = (
            (other[0] - endpoint[0]) if from_start else (endpoint[0] - other[0]),
            (other[1] - endpoint[1]) if from_start else (endpoint[1] - other[1]),
        )
        length = math.hypot(delta[0], delta[1])
        if length > EPSILON:
            return (delta[0] / length, delta[1] / length)
    raise PlanningError("connector path has no non-zero endpoint direction")


def _connector_offset_endpoints(
    raw_local_path: Sequence[LocalPoint],
    source_anchor: LocalPoint,
    destination_anchor: LocalPoint,
    connector: Connector,
) -> Tuple[LocalPoint, LocalPoint]:
    """Apply dense-planner bridge offsets to the two connector endpoints."""
    edge_distance_lon = connector.edge_distance_lon or (0.0, 0.0)
    edge_distance_lat = connector.edge_distance_lat or (0.0, 0.0)
    start_direction = _path_endpoint_direction(raw_local_path, from_start=True)
    end_direction = _path_endpoint_direction(raw_local_path, from_start=False)

    # A bridge with A=C has no measurable width.  In that common case the
    # longitude pair cannot identify a lateral side, so only its asymmetric
    # midpoint shift is meaningful; symmetric defaults leave the line intact.
    lateral_shift = (edge_distance_lon[1] - edge_distance_lon[0]) / 2.0
    start_normal = (-start_direction[1], start_direction[0])
    end_normal = (-end_direction[1], end_direction[0])
    start_offset = (
        start_direction[0] * edge_distance_lat[0]
        + start_normal[0] * lateral_shift,
        start_direction[1] * edge_distance_lat[0]
        + start_normal[1] * lateral_shift,
    )
    end_offset = (
        -end_direction[0] * edge_distance_lat[1]
        + end_normal[0] * lateral_shift,
        -end_direction[1] * edge_distance_lat[1]
        + end_normal[1] * lateral_shift,
    )
    return (
        (source_anchor[0] + start_offset[0], source_anchor[1] + start_offset[1]),
        (
            destination_anchor[0] + end_offset[0],
            destination_anchor[1] + end_offset[1],
        ),
    )


def _clamp_connector_endpoint(
    point: LocalPoint,
    anchor: LocalPoint,
    raw_geometry: RotatedRegion,
) -> LocalPoint:
    """Keep lateral endpoint drift inside a rectangular destination boundary."""
    for raw_polygon in raw_geometry:
        if not _point_is_allowed_region(
            anchor,
            (raw_polygon,),
            allow_hole_boundary=False,
        ):
            continue
        raw_boundary, _ = raw_polygon
        if len(raw_boundary) != 5:
            continue
        min_x = min(value[0] for value in raw_boundary[:-1])
        max_x = max(value[0] for value in raw_boundary[:-1])
        min_y = min(value[1] for value in raw_boundary[:-1])
        max_y = max(value[1] for value in raw_boundary[:-1])
        candidate = (
            min(max(point[0], min_x), max_x),
            min(max(point[1], min_y), max_y),
        )
        if _point_is_allowed_region(
            candidate,
            (raw_polygon,),
            allow_hole_boundary=False,
        ):
            return candidate
        boundary_candidates = []
        for first, second in _ring_edges(raw_boundary):
            direction = (second[0] - first[0], second[1] - first[1])
            length_squared = direction[0] ** 2 + direction[1] ** 2
            if length_squared <= EPSILON:
                continue
            ratio = (
                (point[0] - first[0]) * direction[0]
                + (point[1] - first[1]) * direction[1]
            ) / length_squared
            ratio = max(0.0, min(1.0, ratio))
            projection = (
                first[0] + ratio * direction[0],
                first[1] + ratio * direction[1],
            )
            if _point_is_allowed_region(
                projection,
                (raw_polygon,),
                allow_hole_boundary=False,
            ):
                boundary_candidates.append(
                    (_local_distance(point, projection), projection)
                )
        if boundary_candidates:
            return min(boundary_candidates, key=lambda item: item[0])[1]
    return point


def _effective_connector_path(
    map_data: AutoMap,
    connector: Connector,
    regions: Mapping[str, Region],
    axis_angle: float,
) -> Tuple[Tuple[Point, ...], Tuple[LocalPoint, ...]]:
    """Apply connector offsets first, then region offsets as a fallback."""
    frame = _LocalFrame.from_map(map_data)
    source_region = regions[connector.from_region]
    destination_region = regions[connector.to_region]
    _, source_raw = _rotated_region_geometry(
        map_data,
        source_region,
        axis_angle,
        apply_edge_distance=False,
    )
    _, source_adjusted = _rotated_region_geometry(
        map_data,
        source_region,
        axis_angle,
    )
    _, destination_raw = _rotated_region_geometry(
        map_data,
        destination_region,
        axis_angle,
        apply_edge_distance=False,
    )
    _, destination_adjusted = _rotated_region_geometry(
        map_data,
        destination_region,
        axis_angle,
    )

    raw_local_path = tuple(
        _rotate(frame.to_xy(point), axis_angle) for point in connector.path
    )
    connector_has_offsets = (
        connector.edge_distance_lon is not None
        or connector.edge_distance_lat is not None
    )
    source_anchor = _project_connector_endpoint(
        raw_local_path[0],
        source_raw,
        source_raw if connector_has_offsets else source_adjusted,
        connector.from_region,
        f"{connector.id} source endpoint",
    )
    destination_anchor = _project_connector_endpoint(
        raw_local_path[-1],
        destination_raw,
        destination_raw if connector_has_offsets else destination_adjusted,
        connector.to_region,
        f"{connector.id} destination endpoint",
    )
    if connector_has_offsets:
        source_endpoint, destination_endpoint = _connector_offset_endpoints(
            raw_local_path,
            source_anchor,
            destination_anchor,
            connector,
        )
        destination_endpoint = _clamp_connector_endpoint(
            destination_endpoint,
            destination_anchor,
            destination_raw,
        )
    else:
        source_endpoint = source_anchor
        destination_endpoint = destination_anchor
    source_delta = (
        source_endpoint[0] - raw_local_path[0][0],
        source_endpoint[1] - raw_local_path[0][1],
    )
    destination_delta = (
        destination_endpoint[0] - raw_local_path[-1][0],
        destination_endpoint[1] - raw_local_path[-1][1],
    )
    raw_path_length = _path_length(raw_local_path)
    effective_local_path = []
    travelled = 0.0
    for index, point in enumerate(raw_local_path):
        if index == 0:
            effective_local_path.append(source_endpoint)
            continue
        if index == len(raw_local_path) - 1:
            effective_local_path.append(destination_endpoint)
            continue
        travelled += _local_distance(raw_local_path[index - 1], point)
        ratio = travelled / raw_path_length
        effective_local_path.append(
            (
                point[0]
                + source_delta[0] * (1.0 - ratio)
                + destination_delta[0] * ratio,
                point[1]
                + source_delta[1] * (1.0 - ratio)
                + destination_delta[1] * ratio,
            )
        )

    effective_geo_path = []
    for raw_geo, raw_local, effective_local in zip(
        connector.path, raw_local_path, effective_local_path
    ):
        if _local_distance(raw_local, effective_local) <= EPSILON:
            effective_geo_path.append(raw_geo)
        else:
            effective_geo_path.append(
                frame.from_xy(_unrotate(effective_local, axis_angle))
            )
    return tuple(effective_geo_path), tuple(effective_local_path)


def _path_length(points: Sequence[LocalPoint]) -> float:
    return sum(_local_distance(first, second) for first, second in zip(points, points[1:]))


def _turn_count(points: Sequence[LocalPoint]) -> int:
    """Count heading changes in a local polyline, including U-turns."""
    previous_direction: Optional[LocalPoint] = None
    turns = 0
    for first, second in zip(points, points[1:]):
        delta = (second[0] - first[0], second[1] - first[1])
        length = math.hypot(delta[0], delta[1])
        if length <= EPSILON:
            continue
        direction = (delta[0] / length, delta[1] / length)
        if previous_direction is not None:
            cross = (
                previous_direction[0] * direction[1]
                - previous_direction[1] * direction[0]
            )
            dot = (
                previous_direction[0] * direction[0]
                + previous_direction[1] * direction[1]
            )
            if abs(cross) > 1e-9 or dot < 0.0:
                turns += 1
        previous_direction = direction
    return turns


def _route_turn_count(
    segments: Sequence[Segment], frame: _LocalFrame, axis_angle: float
) -> int:
    """Count heading changes after converting route segments to local metres."""
    points: list[LocalPoint] = []
    for segment in segments:
        for point in segment.points:
            local_point = _rotate(frame.to_xy(point), axis_angle)
            if not points or _local_distance(points[-1], local_point) > EPSILON:
                points.append(local_point)
    return _turn_count(points)


def _objective_key(
    total_length_m: float,
    turn_count: int,
    max_connector_length_m: float,
    turn_penalty_m: float,
    max_connector_penalty: float,
    terminal_connector_length_m: float = 0.0,
) -> Tuple[float, float, float, int, float, float]:
    """Return the deterministic weighted route objective.

    The terminal connector is an operational priority: when a region has a
    known next bridge, prefer an exit that reaches it directly. The requested
    weighted score is still calculated without charging that connector twice.
    """
    score = (
        total_length_m
        + turn_penalty_m * turn_count
        + max_connector_penalty * max_connector_length_m
    )
    return (
        terminal_connector_length_m,
        score,
        total_length_m,
        turn_count,
        max_connector_length_m,
        terminal_connector_length_m,
    )


def _unique_local_points(points: Sequence[LocalPoint]) -> list[LocalPoint]:
    result = []
    for point in points:
        if not any(_local_distance(point, existing) <= 1e-7 for existing in result):
            result.append(point)
    return result


def _orthogonal_connector_nodes(
    start: LocalPoint,
    end: LocalPoint,
    geometry: RotatedRegion,
    connection_tolerance_m: float = 0.0,
) -> list[LocalPoint]:
    """Return allowed grid intersections used for turn-only travel."""
    x_values = {start[0], end[0]}
    y_values = {start[1], end[1]}
    for boundary, holes in geometry:
        for point in boundary[:-1]:
            x_values.add(point[0])
            y_values.add(point[1])
        for hole in holes:
            for point in hole[:-1]:
                x_values.add(point[0])
                y_values.add(point[1])

    nodes = [start, end]
    for x_value in x_values:
        for y_value in y_values:
            point = (x_value, y_value)
            if _point_is_allowed_region_with_tolerance(
                point, geometry, connection_tolerance_m
            ):
                nodes.append(point)
    return _unique_local_points(nodes)


def _find_region_connector(
    start: LocalPoint,
    end: LocalPoint,
    geometry: RotatedRegion,
    max_connector: float,
    connection_tolerance_m: float = 0.0,
) -> Tuple[LocalPoint, ...]:
    if _local_distance(start, end) <= EPSILON:
        return (start,)
    if not _point_is_allowed_region(start, geometry, allow_hole_boundary=False):
        raise PlanningError("connector start is outside the region")
    if not _point_is_allowed_region(end, geometry, allow_hole_boundary=False):
        raise PlanningError("connector end is outside the region")

    nodes = _orthogonal_connector_nodes(
        start, end, geometry, connection_tolerance_m
    )
    start_index = min(range(len(nodes)), key=lambda index: _local_distance(nodes[index], start))
    end_index = min(range(len(nodes)), key=lambda index: _local_distance(nodes[index], end))

    graph = [[] for _ in nodes]
    for first_index in range(len(nodes)):
        for second_index in range(first_index + 1, len(nodes)):
            first = nodes[first_index]
            second = nodes[second_index]
            if (
                abs(first[0] - second[0]) > EPSILON
                and abs(first[1] - second[1]) > EPSILON
            ):
                continue
            if not _line_is_allowed_region(
                first, second, geometry, connection_tolerance_m
            ):
                continue
            weight = _local_distance(first, second)
            graph[first_index].append((weight, second_index))
            graph[second_index].append((weight, first_index))

    distances = [math.inf] * len(nodes)
    previous = [None] * len(nodes)
    distances[start_index] = 0.0
    queue = [(0.0, start_index)]
    while queue:
        distance, current = heapq.heappop(queue)
        if distance > distances[current] + EPSILON:
            continue
        if current == end_index:
            break
        for weight, neighbor in graph[current]:
            candidate = distance + weight
            if candidate + EPSILON < distances[neighbor]:
                distances[neighbor] = candidate
                previous[neighbor] = current
                heapq.heappush(queue, (candidate, neighbor))

    if not math.isfinite(distances[end_index]):
        raise PlanningError("no safe connector between consecutive coverage segments")
    if distances[end_index] > max_connector + EPSILON:
        raise PlanningError(
            "safe connector length {:.2f}m exceeds max_connector {:.2f}m".format(
                distances[end_index], max_connector
            )
        )

    path_indices = []
    current = end_index
    while current is not None:
        path_indices.append(current)
        current = previous[current]
    path_indices.reverse()
    return tuple(nodes[index] for index in path_indices)


def _find_connector(
    start: LocalPoint,
    end: LocalPoint,
    boundary: Sequence[LocalPoint],
    no_go: Sequence[Sequence[LocalPoint]],
    max_connector: float,
) -> Tuple[LocalPoint, ...]:
    return _find_region_connector(start, end, ((tuple(boundary), tuple(no_go)),), max_connector)


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


def _local_region_coverage_segments(
    map_data: AutoMap,
    region: Region,
    axis_angle: float,
    sweep_spacing: float,
    edge_clearance: float,
) -> Tuple[RotatedRegion, list[Tuple[LocalPoint, LocalPoint]]]:
    geometry, groups = _local_region_coverage_groups(
        map_data,
        region,
        axis_angle,
        sweep_spacing,
        edge_clearance,
    )
    return geometry, [segment for group in groups for segment in group]


def _local_region_coverage_groups(
    map_data: AutoMap,
    region: Region,
    axis_angle: float,
    sweep_spacing: float,
    edge_clearance: float,
) -> Tuple[RotatedRegion, Tuple[CoverageGroup, ...]]:
    _, geometry = _rotated_region_geometry(map_data, region, axis_angle)
    _, original_geometry = _rotated_region_geometry(
        map_data, region, axis_angle, apply_edge_distance=False
    )

    def polygons_touch(first: RotatedPolygon, second: RotatedPolygon) -> bool:
        first_boundary = first[0]
        second_boundary = second[0]
        return any(
            _point_on_segment(point, edge_start, edge_end)
            for point in first_boundary[:-1]
            for edge_start, edge_end in _ring_edges(second_boundary)
        ) or any(
            _point_on_segment(point, edge_start, edge_end)
            for point in second_boundary[:-1]
            for edge_start, edge_end in _ring_edges(first_boundary)
        )

    component_by_polygon = list(range(len(geometry)))

    def find_component(index: int) -> int:
        while component_by_polygon[index] != index:
            component_by_polygon[index] = component_by_polygon[
                component_by_polygon[index]
            ]
            index = component_by_polygon[index]
        return index

    def join_components(first: int, second: int) -> None:
        first_root = find_component(first)
        second_root = find_component(second)
        if first_root != second_root:
            component_by_polygon[second_root] = first_root

    for first_index in range(len(geometry)):
        for second_index in range(first_index + 1, len(geometry)):
            if polygons_touch(
                original_geometry[first_index], original_geometry[second_index]
            ):
                join_components(first_index, second_index)

    components = []
    component_indices = {}
    for polygon_index in range(len(geometry)):
        root = find_component(polygon_index)
        component_index = component_indices.setdefault(root, len(components))
        if component_index == len(components):
            components.append([])
        components[component_index].append(polygon_index)

    scan_values = _region_scan_values(geometry, sweep_spacing, edge_clearance)
    groups = []
    for component in components:
        component_geometry = tuple(geometry[index] for index in component)
        segments = []
        for row_index, sweep_value in enumerate(scan_values):
            intervals = _region_scanline_intervals(
                component_geometry, sweep_value, edge_clearance
            )
            if row_index % 2:
                intervals = list(reversed(intervals))
            for left, right in intervals:
                if row_index % 2:
                    segments.append(((right, sweep_value), (left, sweep_value)))
                else:
                    segments.append(((left, sweep_value), (right, sweep_value)))
        if segments:
            groups.append(tuple(segments))
    if not groups:
        raise PlanningError(f"region {region.id} has no usable coverage segment")
    return geometry, tuple(groups)


def _serpentine_candidates(
    local_segments: Sequence[Tuple[LocalPoint, LocalPoint]],
) -> list[list[Tuple[LocalPoint, LocalPoint]]]:
    """Return the four complete sweep directions without breaking turn order."""
    forward = list(local_segments)
    reverse_direction = [(end, start) for start, end in forward]
    reverse_order = list(reversed(forward))
    reverse_order_and_direction = [
        (end, start) for start, end in reverse_order
    ]
    candidates = [
        forward,
        reverse_direction,
        reverse_order,
        reverse_order_and_direction,
    ]
    unique_candidates = []
    for candidate in candidates:
        if candidate not in unique_candidates:
            unique_candidates.append(candidate)
    return unique_candidates


def _beam_optimized_multi_polygon_segments(
    map_data: AutoMap,
    axis_angle: float,
    local_segments: Sequence[Tuple[LocalPoint, LocalPoint]],
    geometry: RotatedRegion,
    max_connector: float,
    connection_tolerance_m: float,
    start_point: Optional[Point],
    exit_point: Optional[Point],
    turn_penalty_m: float,
    max_connector_penalty: float,
) -> Tuple[Tuple[LocalPoint, LocalPoint], ...]:
    """Find a safe large-region ordering while keeping the search bounded."""
    frame = _LocalFrame.from_map(map_data)
    local_start = (
        None
        if start_point is None
        else _rotate(frame.to_xy(start_point), axis_angle)
    )
    local_exit = (
        None
        if exit_point is None
        else _rotate(frame.to_xy(exit_point), axis_angle)
    )
    segment_count = len(local_segments)
    connector_cache: dict[
        Tuple[LocalPoint, LocalPoint], Optional[Tuple[LocalPoint, ...]]
    ] = {}

    def connector_path(
        start: LocalPoint, end: LocalPoint
    ) -> Optional[Tuple[LocalPoint, ...]]:
        key = (start, end)
        if key not in connector_cache:
            connector_cache[key] = _safe_connector_path(
                start,
                end,
                geometry,
                max_connector,
                connection_tolerance_m,
            )
        return connector_cache[key]

    def connector_length(
        start: LocalPoint, end: LocalPoint
    ) -> Optional[float]:
        path = connector_path(start, end)
        return None if path is None else _path_length(path)

    def segment_start(index: int, orientation: int) -> LocalPoint:
        start, end = local_segments[index]
        return start if orientation == 0 else end

    def segment_end(index: int, orientation: int) -> LocalPoint:
        start, end = local_segments[index]
        return end if orientation == 0 else start

    def selected_points(
        selected: Sequence[Tuple[LocalPoint, LocalPoint]],
    ) -> list[LocalPoint]:
        points: list[LocalPoint] = []
        if not selected:
            return points
        if local_start is not None:
            entry = connector_cache[(local_start, selected[0][0])]
            if entry is None:
                return []
            points.extend(entry)
        points.extend(selected[0])
        for start, end in selected[1:]:
            connector = connector_cache[(points[-1], start)]
            if connector is None:
                return []
            points.extend(connector[1:])
            points.append(end)
        return points

    # State is (visited mask, last segment, orientation, max connector,
    # total connector, selected segments). Keep enough alternatives to retain
    # an exit-friendly ordering without turning this into factorial search.
    beam_width = max(256, min(2048, segment_count * 64))
    beam = []
    for index in range(segment_count):
        for orientation in (0, 1):
            first = segment_start(index, orientation)
            cost = 0.0 if local_start is None else connector_length(local_start, first)
            if cost is None:
                continue
            beam.append(
                (
                    1 << index,
                    index,
                    orientation,
                    cost,
                    cost,
                    ((first, segment_end(index, orientation)),),
                )
            )

    if not beam:
        raise PlanningError(
            "no safe connector ordering between multi-polygon coverage segments"
        )

    def partial_ranking(state):
        _, last, orientation, longest, total, selected = state
        points = selected_points(selected)
        return _objective_key(
            total,
            _turn_count(points),
            longest,
            turn_penalty_m,
            max_connector_penalty,
        )

    def final_ranking(state):
        _, last, orientation, longest, total, selected = state
        tail = (
            0.0
            if local_exit is None
            else connector_length(segment_end(last, orientation), local_exit)
        )
        if tail is None:
            return (math.inf, math.inf, math.inf, math.inf, math.inf, math.inf)
        points = selected_points(selected)
        if local_exit is not None:
            tail_path = connector_cache[(points[-1], local_exit)]
            if tail_path is None:
                return (math.inf, math.inf, math.inf, math.inf, math.inf, math.inf)
            points.extend(tail_path[1:])
        return _objective_key(
            total + tail,
            _turn_count(points),
            max(longest, tail),
            turn_penalty_m,
            max_connector_penalty,
            terminal_connector_length_m=tail,
        )

    beam.sort(key=partial_ranking)
    beam = beam[:beam_width]
    for _ in range(1, segment_count):
        next_states = {}
        for mask, last, orientation, longest, total, selected in beam:
            current_end = segment_end(last, orientation)
            for next_index in range(segment_count):
                if mask & (1 << next_index):
                    continue
                for next_orientation in (0, 1):
                    next_start = segment_start(next_index, next_orientation)
                    cost = connector_length(current_end, next_start)
                    if cost is None:
                        continue
                    next_longest = max(longest, cost)
                    next_total = total + cost
                    next_mask = mask | (1 << next_index)
                    next_state_key = (next_mask, next_index, next_orientation)
                    candidate = (
                        next_mask,
                        next_index,
                        next_orientation,
                        next_longest,
                        next_total,
                        selected
                        + ((next_start, segment_end(next_index, next_orientation)),),
                    )
                    previous = next_states.get(next_state_key)
                    if previous is None or partial_ranking(candidate) < partial_ranking(previous):
                        next_states[next_state_key] = candidate

        if not next_states:
            raise PlanningError(
                "no safe connector ordering between multi-polygon coverage segments"
            )
        beam = sorted(next_states.values(), key=partial_ranking)[:beam_width]

    best = None
    best_metric = None
    for state in beam:
        _, last, orientation, longest, total, selected = state
        tail = (
            0.0
            if local_exit is None
            else connector_length(segment_end(last, orientation), local_exit)
        )
        if tail is None:
            continue
        metric = final_ranking(state)
        if best_metric is None or metric < best_metric:
            best_metric = metric
            best = selected

    if best is None:
        raise PlanningError(
            "no safe connector ordering between multi-polygon coverage segments"
        )
    return tuple(best)


def _optimized_multi_polygon_groups(
    map_data: AutoMap,
    axis_angle: float,
    groups: Sequence[CoverageGroup],
    geometry: RotatedRegion,
    max_connector: float,
    connection_tolerance_m: float,
    start_point: Optional[Point],
    exit_point: Optional[Point],
    turn_penalty_m: float,
    max_connector_penalty: float,
    preserve_polygon_order: bool = False,
) -> Tuple[CoverageSegment, ...]:
    """Order multi-polygon coverage with a weighted route objective.

    Disconnected polygons are interleaved by default so nearby scanlines can
    be chained across a small gap. Set ``preserve_polygon_order`` to keep the
    legacy one-polygon-at-a-time behavior.
    """
    group_count = len(groups)
    if group_count == 0:
        raise PlanningError("multi-polygon region has no coverage groups")
    if group_count > 1 and not preserve_polygon_order:
        local_segments = tuple(segment for group in groups for segment in group)
        return tuple(
            _beam_optimized_multi_polygon_segments(
                map_data,
                axis_angle,
                local_segments,
                geometry,
                max_connector,
                connection_tolerance_m,
                start_point,
                exit_point,
                turn_penalty_m,
                max_connector_penalty,
            )
        )
    if group_count == 1:
        # A touching polygon component behaves like one continuous sweep
        # area. Keep the existing segment-level exit optimization for it;
        # only disconnected components need the no-reentry group search.
        return tuple(
            _optimized_multi_polygon_segments(
                map_data,
                axis_angle,
                groups[0],
                geometry,
                max_connector,
                connection_tolerance_m,
                start_point,
                exit_point,
                turn_penalty_m,
                max_connector_penalty,
            )
        )

    frame = _LocalFrame.from_map(map_data)
    local_start = (
        None
        if start_point is None
        else _rotate(frame.to_xy(start_point), axis_angle)
    )
    local_exit = (
        None
        if exit_point is None
        else _rotate(frame.to_xy(exit_point), axis_angle)
    )
    group_candidates = tuple(
        tuple(tuple(candidate) for candidate in _serpentine_candidates(group))
        for group in groups
    )
    connector_cache: dict[
        Tuple[LocalPoint, LocalPoint], Optional[Tuple[LocalPoint, ...]]
    ] = {}
    candidate_cache: dict[Tuple[int, int], Optional[dict[str, Any]]] = {}

    def connector_path(
        start: LocalPoint, end: LocalPoint
    ) -> Optional[Tuple[LocalPoint, ...]]:
        key = (start, end)
        if key not in connector_cache:
            connector_cache[key] = _safe_connector_path(
                start,
                end,
                geometry,
                max_connector,
                connection_tolerance_m,
            )
        return connector_cache[key]

    def candidate_details(
        group_index: int, candidate_index: int
    ) -> Optional[dict[str, Any]]:
        key = (group_index, candidate_index)
        if key in candidate_cache:
            return candidate_cache[key]
        candidate = group_candidates[group_index][candidate_index]
        polyline: list[LocalPoint] = []
        total_length = 0.0
        max_connector_length = 0.0
        for start, end in candidate:
            if not polyline:
                polyline.extend((start, end))
            else:
                connector = connector_path(polyline[-1], start)
                if connector is None:
                    candidate_cache[key] = None
                    return None
                connector_length = _path_length(connector)
                total_length += connector_length
                max_connector_length = max(max_connector_length, connector_length)
                polyline.extend(connector[1:])
                polyline.append(end)
            total_length += _local_distance(start, end)
        details = {
            "start": polyline[0],
            "end": polyline[-1],
            "polyline": tuple(polyline),
            "total_length": total_length,
            "turns": _turn_count(polyline),
            "max_connector": max_connector_length,
        }
        candidate_cache[key] = details
        return details

    def transition_turns(
        previous: dict[str, Any],
        connector: Tuple[LocalPoint, ...],
        current: dict[str, Any],
    ) -> int:
        points = list(previous["polyline"][-2:])
        points.extend(connector[1:])
        points.extend(current["polyline"][1:2])
        return _turn_count(points)

    def initial_cost(
        group_index: int, candidate_index: int
    ) -> Optional[Tuple[float, int, float]]:
        details = candidate_details(group_index, candidate_index)
        if details is None:
            return None
        if local_start is None:
            return (
                details["total_length"],
                details["turns"],
                details["max_connector"],
            )
        connector = connector_path(local_start, details["start"])
        if connector is None:
            return None
        connector_length = _path_length(connector)
        entry_points = list(connector) + list(details["polyline"][1:])
        return (
            details["total_length"] + connector_length,
            _turn_count(entry_points),
            max(details["max_connector"], connector_length),
        )

    def tail_cost(
        group_index: int, candidate_index: int
    ) -> Optional[Tuple[float, int, float]]:
        details = candidate_details(group_index, candidate_index)
        if details is None:
            return None
        if local_exit is None:
            return (0.0, 0, 0.0)
        connector = connector_path(details["end"], local_exit)
        if connector is None:
            return None
        connector_length = _path_length(connector)
        tail_points = list(details["polyline"][-2:]) + list(connector[1:])
        return (
            connector_length,
            _turn_count(tail_points),
            connector_length,
        )

    def add_cost(
        first: Tuple[float, int, float], second: Tuple[float, int, float]
    ) -> Tuple[float, int, float]:
        return (
            first[0] + second[0],
            first[1] + second[1],
            max(first[2], second[2]),
        )

    def is_better(
        candidate: Tuple[float, int, float],
        previous: Optional[Tuple[float, int, float]],
    ) -> bool:
        return previous is None or _objective_key(
            *candidate,
            turn_penalty_m,
            max_connector_penalty,
        ) < _objective_key(
            *previous,
            turn_penalty_m,
            max_connector_penalty,
        )

    def final_key(
        route_cost: Tuple[float, int, float],
        tail: Tuple[float, int, float],
    ) -> Tuple[float, float, float, int, float, float]:
        return _objective_key(
            *add_cost(route_cost, tail),
            turn_penalty_m,
            max_connector_penalty,
            terminal_connector_length_m=tail[0],
        )

    def flatten_selection(
        selection: Sequence[Tuple[int, int]],
    ) -> Tuple[CoverageSegment, ...]:
        return tuple(
            segment
            for group_index, candidate_index in selection
            for segment in group_candidates[group_index][candidate_index]
        )

    if group_count <= 12:
        metrics: dict[Tuple[int, int, int], Tuple[float, int, float]] = {}
        parents: dict[
            Tuple[int, int, int], Optional[Tuple[int, int, int]]
        ] = {}
        for group_index in range(group_count):
            for candidate_index in range(len(group_candidates[group_index])):
                cost = initial_cost(group_index, candidate_index)
                if cost is None:
                    continue
                state = (1 << group_index, group_index, candidate_index)
                metrics[state] = cost
                parents[state] = None

        full_mask = (1 << group_count) - 1
        for mask in range(1, full_mask + 1):
            current_states = [
                (state, metric)
                for state, metric in metrics.items()
                if state[0] == mask
            ]
            for state, current_metric in current_states:
                _, last_group, last_candidate = state
                previous = candidate_details(last_group, last_candidate)
                if previous is None:
                    continue
                for next_group in range(group_count):
                    if mask & (1 << next_group):
                        continue
                    for next_candidate in range(
                        len(group_candidates[next_group])
                    ):
                        current = candidate_details(next_group, next_candidate)
                        if current is None:
                            continue
                        connector = connector_path(previous["end"], current["start"])
                        if connector is None:
                            continue
                        connector_length = _path_length(connector)
                        transition_cost = (
                            connector_length + current["total_length"],
                            transition_turns(previous, connector, current)
                            + current["turns"],
                            max(current["max_connector"], connector_length),
                        )
                        next_state = (
                            mask | (1 << next_group),
                            next_group,
                            next_candidate,
                        )
                        candidate_metric = add_cost(
                            current_metric,
                            transition_cost,
                        )
                        if is_better(candidate_metric, metrics.get(next_state)):
                            metrics[next_state] = candidate_metric
                            parents[next_state] = state

        best_state = None
        best_metric = None
        best_objective = None
        for group_index in range(group_count):
            for candidate_index in range(len(group_candidates[group_index])):
                state = (full_mask, group_index, candidate_index)
                current_metric = metrics.get(state)
                if current_metric is None:
                    continue
                tail = tail_cost(group_index, candidate_index)
                if tail is None:
                    continue
                candidate_metric = add_cost(current_metric, tail)
                candidate_objective = final_key(current_metric, tail)
                if best_objective is None or candidate_objective < best_objective:
                    best_metric = candidate_metric
                    best_objective = candidate_objective
                    best_state = state

        if best_state is None:
            raise PlanningError(
                "no safe connector ordering between multi-polygon coverage groups"
            )

        selected = []
        state = best_state
        while state is not None:
            _, group_index, candidate_index = state
            selected.append((group_index, candidate_index))
            state = parents[state]
        selected.reverse()
        return flatten_selection(selected)

    beam_width = max(256, min(2048, group_count * 64))
    beam = []
    for group_index in range(group_count):
        for candidate_index in range(len(group_candidates[group_index])):
            cost = initial_cost(group_index, candidate_index)
            if cost is None:
                continue
            beam.append(
                (
                    1 << group_index,
                    group_index,
                    candidate_index,
                    cost,
                    ((group_index, candidate_index),),
                )
            )

    def ranking(state) -> Tuple[float, float, float, int, float, float]:
        _, group_index, candidate_index, cost, _ = state
        tail = tail_cost(group_index, candidate_index)
        if tail is None:
            return (math.inf, math.inf, math.inf, math.inf)
        return final_key(cost, tail)

    beam.sort(key=ranking)
    beam = beam[:beam_width]
    for _ in range(1, group_count):
        next_states = {}
        for (
            mask,
            last_group,
            last_candidate,
            current_cost,
            selected,
        ) in beam:
            previous = candidate_details(last_group, last_candidate)
            if previous is None:
                continue
            for next_group in range(group_count):
                if mask & (1 << next_group):
                    continue
                for next_candidate in range(
                    len(group_candidates[next_group])
                ):
                    current = candidate_details(next_group, next_candidate)
                    if current is None:
                        continue
                    connector = connector_path(previous["end"], current["start"])
                    if connector is None:
                        continue
                    connector_length = _path_length(connector)
                    transition_cost = (
                        connector_length + current["total_length"],
                        transition_turns(previous, connector, current)
                        + current["turns"],
                        max(current["max_connector"], connector_length),
                    )
                    next_mask = mask | (1 << next_group)
                    next_state_key = (next_mask, next_group, next_candidate)
                    candidate = (
                        next_mask,
                        next_group,
                        next_candidate,
                        add_cost(current_cost, transition_cost),
                        selected + ((next_group, next_candidate),),
                    )
                    previous = next_states.get(next_state_key)
                    if previous is None or (
                        ranking(candidate) < ranking(previous)
                    ):
                        next_states[next_state_key] = candidate

        if not next_states:
            raise PlanningError(
                "no safe connector ordering between multi-polygon coverage groups"
            )
        beam = sorted(next_states.values(), key=ranking)[:beam_width]

    best = None
    best_metric = None
    best_objective = None
    for state in beam:
        _, group_index, candidate_index, current_cost, selected = state
        tail = tail_cost(group_index, candidate_index)
        if tail is None:
            continue
        metric = add_cost(current_cost, tail)
        candidate_objective = final_key(current_cost, tail)
        if best_objective is None or candidate_objective < best_objective:
            best_metric = metric
            best_objective = candidate_objective
            best = selected
    if best is None:
        raise PlanningError(
            "no safe connector ordering between multi-polygon coverage groups"
        )
    return flatten_selection(best)


def _optimized_multi_polygon_segments(
    map_data: AutoMap,
    axis_angle: float,
    local_segments: Sequence[Tuple[LocalPoint, LocalPoint]],
    geometry: RotatedRegion,
    max_connector: float,
    connection_tolerance_m: float,
    start_point: Optional[Point],
    exit_point: Optional[Point],
    turn_penalty_m: float,
    max_connector_penalty: float,
) -> Tuple[Tuple[LocalPoint, LocalPoint], ...]:
    """Order a multi-polygon sweep set by safe connector cost.

    A polygon that extends from the main sweep body can otherwise become the
    final scanline, forcing a long return to the next bridge. The exact
    bitmask search is retained for small regions and a bounded beam search
    handles larger regions.
    """
    segment_count = len(local_segments)
    if segment_count > 16:
        try:
            return _beam_optimized_multi_polygon_segments(
                map_data,
                axis_angle,
                local_segments,
                geometry,
                max_connector,
                connection_tolerance_m,
                start_point,
                exit_point,
                turn_penalty_m,
                max_connector_penalty,
            )
        except PlanningError:
            # Preserve the proven linear route when beam pruning cannot retain
            # a complete Hamiltonian ordering for a highly fragmented region.
            return tuple(local_segments)

    frame = _LocalFrame.from_map(map_data)
    local_start = (
        None
        if start_point is None
        else _rotate(frame.to_xy(start_point), axis_angle)
    )
    local_exit = (
        None
        if exit_point is None
        else _rotate(frame.to_xy(exit_point), axis_angle)
    )
    connector_cache: dict[Tuple[LocalPoint, LocalPoint], Optional[float]] = {}

    def connector_length(
        start: LocalPoint, end: LocalPoint
    ) -> Optional[float]:
        key = (start, end)
        if key in connector_cache:
            return connector_cache[key]
        length = _safe_connector_length(
            start,
            end,
            geometry,
            max_connector,
            connection_tolerance_m,
        )
        connector_cache[key] = length
        return length

    def segment_start(index: int, orientation: int) -> LocalPoint:
        start, end = local_segments[index]
        return start if orientation == 0 else end

    def segment_end(index: int, orientation: int) -> LocalPoint:
        start, end = local_segments[index]
        return end if orientation == 0 else start

    metrics: dict[Tuple[int, int, int], Tuple[float, float]] = {}
    parents: dict[Tuple[int, int, int], Optional[Tuple[int, int, int]]] = {}
    for index in range(segment_count):
        for orientation in (0, 1):
            first = segment_start(index, orientation)
            cost = 0.0 if local_start is None else connector_length(local_start, first)
            if cost is None:
                continue
            state = (1 << index, index, orientation)
            metrics[state] = (cost, cost)
            parents[state] = None

    full_mask = (1 << segment_count) - 1
    for mask in range(1, full_mask + 1):
        for last in range(segment_count):
            if not mask & (1 << last):
                continue
            for orientation in (0, 1):
                state = (mask, last, orientation)
                current_metric = metrics.get(state)
                if current_metric is None:
                    continue
                current_end = segment_end(last, orientation)
                for next_index in range(segment_count):
                    if mask & (1 << next_index):
                        continue
                    for next_orientation in (0, 1):
                        next_start = segment_start(next_index, next_orientation)
                        cost = connector_length(current_end, next_start)
                        if cost is None:
                            continue
                        next_state = (
                            mask | (1 << next_index),
                            next_index,
                            next_orientation,
                        )
                        candidate_metric = (
                            max(current_metric[0], cost),
                            current_metric[1] + cost,
                        )
                        if candidate_metric < metrics.get(
                            next_state, (math.inf, math.inf)
                        ):
                            metrics[next_state] = candidate_metric
                            parents[next_state] = state

    best_state = None
    best_metric = (math.inf, math.inf, math.inf)
    for last in range(segment_count):
        for orientation in (0, 1):
            state = (full_mask, last, orientation)
            current_metric = metrics.get(state)
            if current_metric is None:
                continue
            tail = (
                0.0
                if local_exit is None
                else connector_length(segment_end(last, orientation), local_exit)
            )
            if tail is None:
                continue
            candidate_metric = (
                max(current_metric[0], tail),
                current_metric[1] + tail,
                tail,
            )
            if candidate_metric < best_metric:
                best_metric = candidate_metric
                best_state = state

    if best_state is None:
        raise PlanningError(
            "no safe connector ordering between multi-polygon coverage segments"
        )

    selected = []
    state = best_state
    while state is not None:
        _, index, orientation = state
        selected.append((segment_start(index, orientation), segment_end(index, orientation)))
        state = parents[state]
    selected.reverse()
    return tuple(selected)


def _segments_from_start(
    map_data: AutoMap,
    axis_angle: float,
    local_segments: Sequence[Tuple[LocalPoint, LocalPoint]],
    start_point: Optional[Point] = None,
    exit_point: Optional[Point] = None,
) -> list[Tuple[LocalPoint, LocalPoint]]:
    selected_start = map_data.start if start_point is None else start_point
    frame = _LocalFrame.from_map(map_data)
    start_local = (
        _rotate(frame.to_xy(selected_start), axis_angle)
        if selected_start is not None
        else None
    )
    exit_local = (
        _rotate(frame.to_xy(exit_point), axis_angle)
        if exit_point is not None
        else None
    )

    def attachment_cost(
        candidate: Sequence[Tuple[LocalPoint, LocalPoint]],
    ) -> Union[float, Tuple[float, float]]:
        start_cost = (
            0.0
            if start_local is None
            else _local_distance(start_local, candidate[0][0])
        )
        if exit_local is None:
            return start_cost

        # An explicit next-region connector defines where this region must
        # finish. Prefer that endpoint first; otherwise a small saving at the
        # entry can create a long return across the last polygon.
        exit_cost = _local_distance(candidate[-1][1], exit_local)
        return (exit_cost, start_cost)

    return min(_serpentine_candidates(local_segments), key=attachment_cost)


def _route_from_local_segments(
    map_data: AutoMap,
    axis_angle: float,
    local_segments: Sequence[Tuple[LocalPoint, LocalPoint]],
    max_connector: float,
    region_id: Optional[str] = None,
    geometry: Optional[RotatedRegion] = None,
    start_point: Optional[Point] = None,
    exit_point: Optional[Point] = None,
    order: Tuple[str, ...] = (),
    preserve_segment_order: bool = False,
    connection_tolerance_m: float = 0.0,
    attachment_geometry: Optional[RotatedRegion] = None,
    prefer_diagonal_attachment: bool = False,
) -> Route:
    if geometry is None:
        frame, boundary, no_go = _rotated_geometry(map_data, axis_angle)
        geometry = ((boundary, tuple(no_go)),)
    else:
        frame = _LocalFrame.from_map(map_data)
    if attachment_geometry is None:
        attachment_geometry = geometry

    selected_start = map_data.start if start_point is None else start_point
    ordered_segments = (
        list(local_segments)
        if preserve_segment_order
        else _segments_from_start(
            map_data, axis_angle, local_segments, selected_start, exit_point
        )
    )
    output = []
    previous_end = None
    max_connector_length = 0.0

    output_region_id = None if map_data.legacy else region_id
    if selected_start is not None:
        start_local = _rotate(frame.to_xy(selected_start), axis_angle)
        if not _point_is_allowed_region(
            start_local, attachment_geometry, allow_hole_boundary=False
        ):
            raise PlanningError("start is outside the usable map area")
        first_start = ordered_segments[0][0]
        if _local_distance(start_local, first_start) > EPSILON:
            connector = (
                _bridge_attachment_connector(
                    start_local,
                    first_start,
                    attachment_geometry,
                    max_connector,
                    connection_tolerance_m,
                )
                if prefer_diagonal_attachment
                else _find_region_connector(
                    start_local,
                    first_start,
                    attachment_geometry,
                    max_connector,
                    connection_tolerance_m,
                )
            )
            connector_points = tuple(
                frame.from_xy(_unrotate(point, axis_angle)) for point in connector
            )
            connector_length = _path_length(connector)
            max_connector_length = max(max_connector_length, connector_length)
            output.append(
                Segment(
                    "connector",
                    connector_points,
                    connector_length,
                    region_id=output_region_id,
                )
            )

    for start, end in ordered_segments:
        if previous_end is not None:
            connector = _find_region_connector(
                previous_end,
                start,
                geometry,
                max_connector,
                connection_tolerance_m,
            )
            connector_points = tuple(
                frame.from_xy(_unrotate(point, axis_angle)) for point in connector
            )
            connector_length = _path_length(connector)
            max_connector_length = max(max_connector_length, connector_length)
            output.append(
                Segment(
                    "connector",
                    connector_points,
                    connector_length,
                    region_id=output_region_id,
                )
            )

        coverage_local = (start, end)
        coverage_points = tuple(
            frame.from_xy(_unrotate(point, axis_angle)) for point in coverage_local
        )
        coverage_length = _path_length(coverage_local)
        output.append(
            Segment(
                "coverage",
                coverage_points,
                coverage_length,
                region_id=output_region_id,
            )
        )
        previous_end = end

    total_length = sum(segment.length_m for segment in output)
    return Route(
        axis_angle_rad=axis_angle,
        segments=tuple(output),
        total_length_m=total_length,
        max_connector_length_m=max_connector_length,
        order=order,
    )


def _merge_ordered_inter_region_connector_spans(
    segments: Sequence[Segment],
    order: Sequence[str],
    connectors: Mapping[str, Connector],
) -> Tuple[Segment, ...]:
    """Join each ordered inter-region bridge with its two region tails."""
    output = list(segments)
    order_positions = {item: index for index, item in enumerate(order)}

    for item in reversed(order):
        connector = connectors.get(item)
        if (
            connector is None
            or connector.from_region is None
            or connector.to_region is None
        ):
            continue

        order_index = order_positions[item]
        if (
            order_index == 0
            or order_index == len(order) - 1
            or order[order_index - 1] != connector.from_region
            or order[order_index + 1] != connector.to_region
        ):
            continue

        bridge_indexes = [
            index
            for index, segment in enumerate(output)
            if segment.connector_id == connector.id
        ]
        if len(bridge_indexes) != 1:
            raise PlanningError(
                f"ordered connector {connector.id} must produce one route segment"
            )
        bridge_index = bridge_indexes[0]

        start = bridge_index
        while start > 0:
            candidate = output[start - 1]
            if not (
                candidate.kind == "connector"
                and candidate.connector_id is None
                and candidate.region_id == connector.from_region
            ):
                break
            start -= 1

        end = bridge_index + 1
        while end < len(output):
            candidate = output[end]
            if not (
                candidate.kind == "connector"
                and candidate.connector_id is None
                and candidate.region_id == connector.to_region
            ):
                break
            end += 1

        if end - start == 1:
            continue

        merged_points: list[Point] = []
        for segment in output[start:end]:
            for point in segment.points:
                if merged_points and point == merged_points[-1]:
                    continue
                merged_points.append(point)

        output[start:end] = [
            Segment(
                kind="connector",
                points=tuple(merged_points),
                length_m=sum(segment.length_m for segment in output[start:end]),
                connector_id=connector.id,
                from_region=connector.from_region,
                to_region=connector.to_region,
            )
        ]

    return tuple(output)


def plan_route(
    map_data: AutoMap,
    sweep_spacing: float = 1.0,
    edge_clearance: float = 0.3,
    max_connector: float = 8.0,
    turn_penalty_m: float = DEFAULT_TURN_PENALTY_M,
    max_connector_penalty: float = DEFAULT_MAX_CONNECTOR_PENALTY,
    preserve_polygon_order: bool = False,
) -> Route:
    """Generate a safe route using length, turns, and connector risk together."""
    if sweep_spacing <= 0.0:
        raise PlanningError("sweep_spacing must be positive")
    if edge_clearance < 0.0:
        raise PlanningError("edge_clearance must be non-negative")
    if max_connector < 0.0:
        raise PlanningError("max_connector must be non-negative")
    if not math.isfinite(turn_penalty_m) or turn_penalty_m < 0.0:
        raise PlanningError("turn_penalty_m must be non-negative")
    if not math.isfinite(max_connector_penalty) or max_connector_penalty < 0.0:
        raise PlanningError("max_connector_penalty must be non-negative")

    axis_angle = estimate_axis_angle(map_data.guides)
    regions = {region.id: region for region in map_data.regions}
    connectors = {connector.id: connector for connector in map_data.connectors}
    if not regions:
        legacy_region = Region(
            "legacy", (Polygon(map_data.boundary, map_data.no_go),), map_data.start
        )
        regions = {legacy_region.id: legacy_region}
    region_axes = {
        region.id: _region_axis_angle(
            region,
            axis_angle,
            use_region_fallback=not map_data.legacy,
        )
        for region in regions.values()
    }
    order = map_data.order or tuple(regions)
    effective_connector_paths = {
        connector.id: _effective_connector_path(
            map_data,
            connector,
            regions,
            axis_angle,
        )
        for connector in map_data.connectors
        if connector.from_region is not None
    }

    output = []
    max_connector_length = 0.0
    previous_region: Optional[Region] = None
    pending_entry: Optional[Point] = None
    pending_entry_uses_raw_geometry = False

    for index, item in enumerate(order):
        if item in regions:
            region = regions[item]
            region_axis = region_axes[region.id]
            start_point = pending_entry
            prefer_diagonal_attachment = pending_entry is not None
            if start_point is None:
                start_point = region.start
            if index == 0 and map_data.start is not None:
                start_point = map_data.start
            next_item = order[index + 1] if index + 1 < len(order) else None
            exit_point = (
                effective_connector_paths[next_item][0][0]
                if next_item in connectors
                and connectors[next_item].from_region is not None
                else None
            )
            geometry, coverage_groups = _local_region_coverage_groups(
                map_data, region, region_axis, sweep_spacing, edge_clearance
            )
            local_segments = [
                segment
                for group in coverage_groups
                for segment in group
            ]
            # Coverage endpoints are generated in edge-adjusted geometry;
            # retain that geometry for internal segment connections. A region
            # entered from a connector also uses it for the incoming bridge
            # attachment, while an explicit map start keeps legacy behavior.
            routing_geometry = geometry
            _, attachment_geometry = _rotated_region_geometry(
                map_data,
                region,
                region_axis,
                apply_edge_distance=(
                    pending_entry is not None
                    and not pending_entry_uses_raw_geometry
                ),
            )
            preserve_segment_order = False
            if len(region.polygons) > 1:
                local_segments = _optimized_multi_polygon_groups(
                    map_data,
                    region_axis,
                    coverage_groups,
                    routing_geometry,
                    max_connector,
                    region.connection_tolerance_m,
                    start_point,
                    exit_point,
                    turn_penalty_m,
                    max_connector_penalty,
                    preserve_polygon_order,
                )
                preserve_segment_order = True
            region_route = _route_from_local_segments(
                map_data,
                region_axis,
                local_segments,
                max_connector,
                region_id=region.id,
                geometry=routing_geometry,
                start_point=start_point,
                exit_point=exit_point,
                order=order,
                preserve_segment_order=preserve_segment_order,
                connection_tolerance_m=region.connection_tolerance_m,
                attachment_geometry=attachment_geometry,
                prefer_diagonal_attachment=prefer_diagonal_attachment,
            )
            output.extend(region_route.segments)
            max_connector_length = max(
                max_connector_length, region_route.max_connector_length_m
            )
            previous_region = region
            pending_entry = None
            pending_entry_uses_raw_geometry = False
            continue

        connector = connectors.get(item)
        if connector is None:
            raise PlanningError(f"order references unknown connector: {item}")
        if connector.from_region is None:
            frame = _LocalFrame.from_map(map_data)
            local_path = tuple(
                _rotate(frame.to_xy(point), axis_angle) for point in connector.path
            )
            connector_length = _path_length(local_path)
            output.append(
                Segment(
                    kind="connector",
                    points=connector.path,
                    length_m=connector_length,
                    connector_id=connector.id,
                )
            )
            max_connector_length = max(max_connector_length, connector_length)
            continue
        if previous_region is None or previous_region.id != connector.from_region:
            raise PlanningError(f"connector {item} has no matching source region")
        if index == len(order) - 1 or order[index + 1] != connector.to_region:
            raise PlanningError(f"connector {item} has no matching destination region")

        effective_geo_path, effective_local_path = effective_connector_paths[connector.id]
        connector_has_offsets = (
            connector.edge_distance_lon is not None
            or connector.edge_distance_lat is not None
        )
        frame, _ = _rotated_region_geometry(
            map_data,
            previous_region,
            axis_angle,
            apply_edge_distance=False,
        )
        _, source_attachment_geometry = _rotated_region_geometry(
            map_data,
            previous_region,
            axis_angle,
            apply_edge_distance=not connector_has_offsets,
        )
        all_region_geometries = [
            _rotated_region_geometry(
                map_data, region, axis_angle, apply_edge_distance=False
            )[1]
            for region in regions.values()
        ]
        connector_start_local = effective_local_path[0]
        source_anchor_local = connector_start_local
        if connector_has_offsets:
            source_anchor_local = _project_connector_endpoint(
                _rotate(frame.to_xy(connector.path[0]), axis_angle),
                source_attachment_geometry,
                source_attachment_geometry,
                connector.from_region,
                f"{connector.id} source anchor",
            )
        elif not _point_is_allowed_region(
            connector_start_local,
            source_attachment_geometry,
            allow_hole_boundary=False,
        ):
            raise PlanningError(
                f"connector {item} starts outside usable region {connector.from_region}"
            )
        source_start_inside = _point_is_allowed_region(
            connector_start_local,
            source_attachment_geometry,
            allow_hole_boundary=False,
        )
        source_attachment_point = (
            connector_start_local if source_start_inside else source_anchor_local
        )
        previous_end = output[-1].points[-1]
        previous_end_local = _rotate(frame.to_xy(previous_end), axis_angle)
        if _local_distance(previous_end_local, source_attachment_point) > EPSILON:
            attach = _bridge_attachment_connector(
                previous_end_local,
                source_attachment_point,
                source_attachment_geometry,
                max_connector,
                previous_region.connection_tolerance_m,
            )
            attach_points = tuple(
                frame.from_xy(_unrotate(point, axis_angle)) for point in attach
            )
            attach_length = _path_length(attach)
            output.append(
                Segment(
                    "connector",
                    attach_points,
                    attach_length,
                    region_id=None if map_data.legacy else previous_region.id,
                )
            )
            max_connector_length = max(max_connector_length, attach_length)

        if (
            connector_has_offsets
            and not source_start_inside
            and _local_distance(source_anchor_local, connector_start_local) > EPSILON
        ):
            source_extension_length = _local_distance(
                source_anchor_local, connector_start_local
            )
            if source_extension_length > max_connector + EPSILON or any(
                _line_enters_ring(source_anchor_local, connector_start_local, hole)
                for geometry in all_region_geometries
                for _, holes in geometry
                for hole in holes
            ):
                raise PlanningError(
                    f"connector {connector.id} has no safe source offset attachment"
                )
            source_extension_points = tuple(
                frame.from_xy(_unrotate(point, axis_angle))
                for point in (source_anchor_local, connector_start_local)
            )
            output.append(
                Segment(
                    "connector",
                    source_extension_points,
                    source_extension_length,
                    region_id=None if map_data.legacy else previous_region.id,
                )
            )
            max_connector_length = max(
                max_connector_length, source_extension_length
            )

        destination_region = regions[connector.to_region]
        _, destination_geometry = _rotated_region_geometry(
            map_data,
            destination_region,
            axis_angle,
            apply_edge_distance=not connector_has_offsets,
        )
        destination_end_local = effective_local_path[-1]
        destination_anchor_local = destination_end_local
        if connector_has_offsets:
            destination_anchor_local = _project_connector_endpoint(
                _rotate(frame.to_xy(connector.path[-1]), axis_angle),
                destination_geometry,
                destination_geometry,
                connector.to_region,
                f"{connector.id} destination anchor",
            )
        if not connector_has_offsets and not _point_is_allowed_region(
            destination_end_local, destination_geometry, allow_hole_boundary=False
        ):
            raise PlanningError(
                f"connector {item} ends outside usable region {connector.to_region}"
            )

        connector_local_path = effective_local_path
        for path_start, path_end in zip(connector_local_path, connector_local_path[1:]):
            if any(
                _line_enters_ring(path_start, path_end, hole)
                for geometry in all_region_geometries
                for _, holes in geometry
                for hole in holes
            ):
                raise PlanningError(f"connector {connector.id} intersects a hole")
        connector_length = _path_length(connector_local_path)
        if connector_length > max_connector + EPSILON:
            raise PlanningError(
                "connector {} length {:.2f}m exceeds max_connector {:.2f}m".format(
                    connector.id, connector_length, max_connector
                )
            )
        output.append(
            Segment(
                "connector",
                effective_geo_path,
                connector_length,
                connector_id=connector.id,
                from_region=connector.from_region,
                to_region=connector.to_region,
            )
        )
        max_connector_length = max(max_connector_length, connector_length)
        destination_end_inside = _point_is_allowed_region(
            destination_end_local,
            destination_geometry,
            allow_hole_boundary=False,
        )
        if (
            connector_has_offsets
            and not destination_end_inside
            and _local_distance(destination_end_local, destination_anchor_local)
            > EPSILON
        ):
            destination_extension_length = _local_distance(
                destination_end_local, destination_anchor_local
            )
            if destination_extension_length > max_connector + EPSILON or any(
                _line_enters_ring(destination_end_local, destination_anchor_local, hole)
                for geometry in all_region_geometries
                for _, holes in geometry
                for hole in holes
            ):
                raise PlanningError(
                    f"connector {connector.id} has no safe destination offset attachment"
                )
            destination_extension_points = tuple(
                frame.from_xy(_unrotate(point, axis_angle))
                for point in (destination_end_local, destination_anchor_local)
            )
            output.append(
                Segment(
                    "connector",
                    destination_extension_points,
                    destination_extension_length,
                    region_id=None if map_data.legacy else destination_region.id,
                )
            )
            max_connector_length = max(
                max_connector_length, destination_extension_length
            )
            pending_entry = destination_extension_points[-1]
        else:
            pending_entry = effective_geo_path[-1]
        pending_entry_uses_raw_geometry = connector_has_offsets

    output = list(
        _merge_ordered_inter_region_connector_spans(output, order, connectors)
    )
    total_length = sum(segment.length_m for segment in output)
    max_connector_length = max(
        (
            segment.length_m
            for segment in output
            if segment.kind == "connector"
        ),
        default=0.0,
    )
    frame = _LocalFrame.from_map(map_data)
    return Route(
        axis_angle_rad=axis_angle,
        segments=tuple(output),
        total_length_m=total_length,
        max_connector_length_m=max_connector_length,
        turn_count=_route_turn_count(output, frame, axis_angle),
        order=order,
    )


def _segment_dict(segment: Segment) -> dict[str, Any]:
    payload = {
        "kind": segment.kind,
        "points": [[point[0], point[1]] for point in segment.points],
        "length_m": round(segment.length_m, 6),
    }
    if segment.region_id is not None:
        payload["region_id"] = segment.region_id
    if segment.connector_id is not None:
        payload["connector_id"] = segment.connector_id
    if segment.from_region is not None:
        payload["from_region"] = segment.from_region
    if segment.to_region is not None:
        payload["to_region"] = segment.to_region
    return payload


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
            "turn_count": route.turn_count,
            "order": list(route.order),
        },
    }
    return json.dumps(payload, ensure_ascii=False, indent=2)


def _geo_segment_distance_m(first: Point, second: Point) -> float:
    """Approximate a short longitude/latitude segment length in metres."""
    latitude = math.radians((first[1] + second[1]) / 2.0)
    delta_lon_m = (second[0] - first[0]) * METERS_PER_DEGREE_LON * math.cos(
        latitude
    )
    delta_lat_m = (second[1] - first[1]) * METERS_PER_DEGREE_LAT
    return math.hypot(delta_lon_m, delta_lat_m)


def _interpolate_geo_segment(
    first: Point, second: Point, spacing_m: float
) -> Tuple[Point, ...]:
    """Return both segment corners and points no farther than ``spacing_m``."""
    distance_m = _geo_segment_distance_m(first, second)
    if distance_m <= EPSILON:
        return (first,)
    count = max(1, int(math.ceil(distance_m / spacing_m)))
    return tuple(
        (
            first[0] + (second[0] - first[0]) * index / count,
            first[1] + (second[1] - first[1]) * index / count,
        )
        for index in range(count + 1)
    )


def _txt_segment_label(segment: Segment) -> str:
    """Return a legacy TXT comment identifying one route segment."""
    if segment.region_id is not None:
        return segment.region_id
    if segment.connector_id is not None:
        # return f"connector:{segment.connector_id}"
        return segment.connector_id
    if segment.kind == "coverage":
        return "coverage"
    return "connector"


def _expanded_txt_route_points(
    route: Route, spacing_m: float
) -> Tuple[Tuple[Point, str], ...]:
    """Expand route segments while retaining corners and removing joins."""
    expanded: list[Tuple[Point, str]] = []
    for segment_index, segment in enumerate(route.segments):
        if not segment.points:
            raise PlanningError(f"segment {segment_index} has no coordinates")
        label = _txt_segment_label(segment)
        segment_points: list[Point] = []
        if len(segment.points) == 1:
            segment_points.append(segment.points[0])
        else:
            for first, second in zip(segment.points, segment.points[1:]):
                interpolated = _interpolate_geo_segment(first, second, spacing_m)
                if segment_points and interpolated:
                    if (
                        abs(segment_points[-1][0] - interpolated[0][0]) <= 1e-12
                        and abs(segment_points[-1][1] - interpolated[0][1]) <= 1e-12
                    ):
                        interpolated = interpolated[1:]
                segment_points.extend(interpolated)

        if expanded and segment_points:
            previous = expanded[-1][0]
            first = segment_points[0]
            if (
                abs(previous[0] - first[0]) <= 1e-12
                and abs(previous[1] - first[1]) <= 1e-12
            ):
                segment_points = segment_points[1:]
        expanded.extend((point, label) for point in segment_points)
    if not expanded:
        raise PlanningError("route has no coordinates")
    return tuple(expanded)


def _bearing_degrees(first: Point, second: Point) -> float:
    """Return the initial bearing from ``first`` to ``second`` in degrees."""
    latitude_first = math.radians(first[1])
    latitude_second = math.radians(second[1])
    delta_lon = math.radians(second[0] - first[0])
    y_value = math.sin(delta_lon) * math.cos(latitude_second)
    x_value = (
        math.cos(latitude_first) * math.sin(latitude_second)
        - math.sin(latitude_first)
        * math.cos(latitude_second)
        * math.cos(delta_lon)
    )
    return (math.degrees(math.atan2(y_value, x_value)) + 360.0) % 360.0


def route_to_txt(
    route: Route, point_spacing: float = DEFAULT_TXT_SPACING_M
) -> str:
    """Serialize a route in the TXT format consumed by ``rtk_nav``.

    Every route corner is retained. Long straight segments are interpolated
    with the same ``ceil(distance / dense_spacing)`` rule as the dense
    planner, whose default ``dense_spacing`` is 15 metres.
    """
    if not math.isfinite(point_spacing) or point_spacing <= 0.0:
        raise PlanningError("point_spacing must be a positive finite number")

    expanded = _expanded_txt_route_points(route, point_spacing)
    points = tuple(point for point, _ in expanded)
    headings = [0.0]
    if len(points) > 1:
        headings = [
            _bearing_degrees(first, second)
            for first, second in zip(points, points[1:])
        ]
        headings.append(headings[-1])

    lines = ["序号,经度,纬度,航向角(度)"]
    current_label = None
    for index, ((longitude, latitude), label) in enumerate(expanded, 1):
        if label != current_label:
            lines.append(f"#{label}")
            current_label = label
        lines.append(
            f"{index},{longitude:.8f},{latitude:.8f},{headings[index - 1]:.2f}"
        )
    return "\n".join(lines)


def route_to_geojson(route: Route, map_data: Optional[AutoMap] = None) -> str:
    """Serialize route and optional map geometry as a GeoJSON FeatureCollection."""
    features = []
    if map_data is not None:
        for region in map_data.regions:
            for polygon_index, polygon in enumerate(region.polygons):
                coordinates = [
                    [[point[0], point[1]] for point in polygon.boundary]
                ]
                coordinates.extend(
                    [[point[0], point[1]] for point in hole]
                    for hole in polygon.holes
                )
                features.append(
                    {
                        "type": "Feature",
                        "properties": {
                            "kind": "boundary",
                            "region_id": region.id,
                            "polygon_index": polygon_index,
                            "edge_distance_lon": list(polygon.edge_distance_lon),
                            "edge_distance_lat": list(polygon.edge_distance_lat),
                        },
                        "geometry": {
                            "type": "Polygon",
                            "coordinates": coordinates,
                        },
                    }
                )
                if region.guide is not None:
                    features[-1]["properties"]["guide"] = region.guide
        for connector in map_data.connectors:
            properties = {
                "kind": "bridge",
                "connector_id": connector.id,
            }
            if connector.edge_distance_lon is not None:
                properties["edge_distance_lon"] = list(
                    connector.edge_distance_lon
                )
            if connector.edge_distance_lat is not None:
                properties["edge_distance_lat"] = list(
                    connector.edge_distance_lat
                )
            if connector.from_region is not None:
                properties["from_region"] = connector.from_region
                properties["to_region"] = connector.to_region
            features.append(
                {
                    "type": "Feature",
                    "properties": properties,
                    "geometry": {
                        "type": "LineString",
                        "coordinates": [
                            [point[0], point[1]] for point in connector.path
                        ],
                    },
                }
            )
    for index, segment in enumerate(route.segments):
        segment_coordinates = [list(point) for point in segment.points]
        if not segment_coordinates:
            raise PlanningError(f"segment {index} has no coordinates")
        if len(segment_coordinates) == 1:
            geometry = {
                "type": "Point",
                "coordinates": segment_coordinates[0],
            }
        else:
            geometry = {
                "type": "LineString",
                "coordinates": segment_coordinates,
            }
        features.append(
            {
                "type": "Feature",
                "properties": {
                    "index": index,
                    "sequence": index,
                    "kind": segment.kind,
                    "length_m": round(segment.length_m, 6),
                },
                "geometry": geometry,
            }
        )
        if segment.region_id is not None:
            features[-1]["properties"]["region_id"] = segment.region_id
        if segment.connector_id is not None:
            features[-1]["properties"]["connector_id"] = segment.connector_id
        if segment.from_region is not None:
            features[-1]["properties"]["from_region"] = segment.from_region
        if segment.to_region is not None:
            features[-1]["properties"]["to_region"] = segment.to_region
    return json.dumps(
        {"type": "FeatureCollection", "features": features},
        ensure_ascii=False,
        indent=2,
    )


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input",
        required=True,
        help="auto map JSON file or legacy area YAML file",
    )
    parser.add_argument("--output", default=None, help="route JSON output file")
    parser.add_argument(
        "--geojson-output",
        default=None,
        help="route GeoJSON output file (default: --output with .geojson suffix)",
    )
    parser.add_argument(
        "--txt-output",
        default=None,
        help="legacy-compatible cleaning path TXT output file",
    )
    parser.add_argument(
        "--txt-spacing",
        type=float,
        default=DEFAULT_TXT_SPACING_M,
        help="maximum TXT waypoint spacing in metres (default: 15.0)",
    )
    parser.add_argument(
        "--map-output",
        default=None,
        help="write a converted v2 map when --input is a legacy YAML file",
    )
    parser.add_argument(
        "--sweep-spacing",
        type=float,
        default=None,
        help="coverage line spacing in metres (default: map interval or 1.0)",
    )
    parser.add_argument(
        "--edge-clearance", type=float, default=0.3, help="coverage edge clearance in metres"
    )
    parser.add_argument(
        "--max-connector", type=float, default=8.0, help="maximum connector length in metres"
    )
    parser.add_argument(
        "--turn-penalty-m",
        type=float,
        default=DEFAULT_TURN_PENALTY_M,
        help="equivalent metres charged per route turn",
    )
    parser.add_argument(
        "--max-connector-penalty",
        type=float,
        default=DEFAULT_MAX_CONNECTOR_PENALTY,
        help="weight applied to the longest connector in the objective",
    )
    parser.add_argument(
        "--keep-polygon-contiguous",
        action="store_true",
        help="finish each disconnected polygon before entering the next one",
    )
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    """Run the auto planner command line interface."""
    args = _build_parser().parse_args(argv)
    try:
        input_path = Path(args.input)
        is_yaml = input_path.suffix.lower() in {".yaml", ".yml"}
        if is_yaml:
            if args.map_output is None:
                raise PlanningError("YAML input requires --map-output")
            map_payload = convert_legacy_yaml_to_map(args.input)
            map_output_path = Path(args.map_output)
            map_output_path.parent.mkdir(parents=True, exist_ok=True)
            map_output_path.write_text(
                json.dumps(map_payload, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
            map_data = load_map(map_payload)
        else:
            if args.output is None:
                raise PlanningError("JSON input requires --output")
            map_data = load_map(args.input)

        if args.output is None:
            print(f"generated map {args.map_output}")
            return 0

        output_path = Path(args.output)
        geojson_path = Path(args.geojson_output) if args.geojson_output else output_path.with_suffix(
            ".geojson"
        )
        txt_path = Path(args.txt_output) if args.txt_output else None
        input_resolved = input_path.resolve()
        output_resolved = output_path.resolve()
        geojson_resolved = geojson_path.resolve()
        output_resolved_paths = {output_resolved, geojson_resolved}
        if txt_path is not None:
            output_resolved_paths.add(txt_path.resolve())
        if input_resolved in output_resolved_paths:
            raise PlanningError(
                "input map and route outputs must use different paths; "
                "keep auto_map_*.json separate from route JSON/GeoJSON/TXT outputs"
            )
        if len(output_resolved_paths) != (3 if txt_path is not None else 2):
            raise PlanningError(
                "route JSON, GeoJSON, and TXT outputs must use different paths"
            )
        sweep_spacing = (
            map_data.defaults.interval
            if args.sweep_spacing is None
            else args.sweep_spacing
        )
        route = plan_route(
            map_data,
            sweep_spacing=sweep_spacing,
            edge_clearance=args.edge_clearance,
            max_connector=args.max_connector,
            turn_penalty_m=args.turn_penalty_m,
            max_connector_penalty=args.max_connector_penalty,
            preserve_polygon_order=args.keep_polygon_contiguous,
        )
        txt_document = (
            route_to_txt(route, point_spacing=args.txt_spacing)
            if txt_path is not None
            else None
        )
        output_path.parent.mkdir(parents=True, exist_ok=True)
        geojson_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(route_to_json(route) + "\n", encoding="utf-8")
        geojson_path.write_text(
            route_to_geojson(route, map_data) + "\n", encoding="utf-8"
        )
        if txt_path is not None and txt_document is not None:
            txt_path.parent.mkdir(parents=True, exist_ok=True)
            txt_path.write_text(txt_document + "\n", encoding="utf-8")
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
