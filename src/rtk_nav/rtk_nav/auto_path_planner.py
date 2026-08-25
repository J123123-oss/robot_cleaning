#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Generate a safe serpentine route from a small set of RTK traces."""

import argparse
import heapq
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


@dataclass(frozen=True)
class Connector:
    """An explicit traversable path between two named regions."""

    id: str
    from_region: str
    to_region: str
    path: Tuple[Point, ...]


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
        from_region = _parse_id(
            value.get("from", value.get("from_region")),
            f"connectors[{index}].from",
        )
        to_region = _parse_id(
            value.get("to", value.get("to_region")),
            f"connectors[{index}].to",
        )
        if from_region not in region_ids or to_region not in region_ids:
            raise PlanningError(f"connector {connector_id} references an unknown region")
        raw_path = value.get("path")
        if not isinstance(raw_path, Sequence) or isinstance(raw_path, (str, bytes)):
            raise PlanningError(f"connectors[{index}].path must be a list of points")
        path = tuple(
            _coerce_point(point, f"connectors[{index}].path") for point in raw_path
        )
        if len(path) < 2 or _distance(path[0], path[-1]) <= EPSILON:
            raise PlanningError(f"connectors[{index}].path needs two distinct endpoints")
        connectors.append(
            Connector(
                id=connector_id,
                from_region=from_region,
                to_region=to_region,
                path=path,
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
        if len(connectors) == len(regions) - 1:
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

    if seen_regions != region_ids:
        missing = sorted(region_ids - seen_regions)
        raise PlanningError(f"order does not include regions: {', '.join(missing)}")

    for previous, current in zip(order, order[1:]):
        if previous in region_ids and current in region_ids:
            raise PlanningError(
                f"regions {previous} and {current} need an explicit connector"
            )

    for index, item in enumerate(order):
        if item not in connector_by_id:
            continue
        if index == 0 or index == len(order) - 1:
            raise PlanningError(f"connector must be between two regions: {item}")
        connector = connector_by_id[item]
        if order[index - 1] != connector.from_region or order[index + 1] != connector.to_region:
            raise PlanningError(
                f"connector {item} does not connect {order[index - 1]} to {order[index + 1]}"
            )
    return order


def load_map(source: JsonSource) -> AutoMap:
    """Load and validate a JSON map object or JSON file path."""
    payload = _read_json_source(source)
    if payload.get("format") == "rtk_auto_map_v2" or "regions" in payload:
        raw_defaults = payload.get("defaults", payload.get("default"))
        defaults = _parse_defaults(raw_defaults)
        regions = _parse_regions(payload.get("regions"), defaults)
        raw_guides = payload.get("guides")
        if raw_guides is None:
            longest_edge = max(
                (
                    (first, second)
                    for region in regions
                    for polygon in region.polygons
                    for first, second in _ring_edges(polygon.boundary)
                ),
                key=lambda edge: _distance(edge[0], edge[1]),
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


RotatedPolygon = Tuple[
    Tuple[LocalPoint, ...], Tuple[Tuple[LocalPoint, ...], ...]
]
RotatedRegion = Tuple[RotatedPolygon, ...]


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

    min_x = min(point[0] for point in ring[:-1])
    max_x = max(point[0] for point in ring[:-1])
    min_y = min(point[1] for point in ring[:-1])
    max_y = max(point[1] for point in ring[:-1])

    # Preserve the legacy YAML order: longitude is [right, left], latitude
    # is [bottom, top]. Positive values inset; negative values extend.
    adjusted_min_x = min_x + edge_distance_lon[1]
    adjusted_max_x = max_x - edge_distance_lon[0]
    adjusted_min_y = min_y + edge_distance_lat[0]
    adjusted_max_y = max_y - edge_distance_lat[1]
    if adjusted_max_x - adjusted_min_x <= EPSILON:
        raise PlanningError("edge_distance_lon leaves no usable polygon width")
    if adjusted_max_y - adjusted_min_y <= EPSILON:
        raise PlanningError("edge_distance_lat leaves no usable polygon height")

    return (
        (adjusted_min_x, adjusted_min_y),
        (adjusted_max_x, adjusted_min_y),
        (adjusted_max_x, adjusted_max_y),
        (adjusted_min_x, adjusted_max_y),
        (adjusted_min_x, adjusted_min_y),
    )


def _rotated_region_geometry(
    map_data: AutoMap,
    region: Region,
    axis_angle: float,
    apply_edge_distance: bool = True,
) -> Tuple[_LocalFrame, RotatedRegion]:
    frame = _LocalFrame.from_map(map_data)

    def rotate_ring(ring: Sequence[Point]) -> Tuple[LocalPoint, ...]:
        return tuple(_rotate(frame.to_xy(point), axis_angle) for point in ring)

    geometry = tuple(
        (
            (
                _edge_adjusted_ring(
                    rotate_ring(polygon.boundary),
                    polygon.edge_distance_lon,
                    polygon.edge_distance_lat,
                )
                if apply_edge_distance
                else rotate_ring(polygon.boundary)
            ),
            tuple(rotate_ring(hole) for hole in polygon.holes),
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


def _region_scanline_intervals(
    geometry: RotatedRegion,
    sweep_value: float,
    edge_clearance: float,
) -> list[Tuple[float, float]]:
    intervals = []
    for boundary, holes in geometry:
        allowed = _ring_intervals(boundary, sweep_value)
        hole_intervals = [
            interval
            for hole in holes
            for interval in _ring_intervals(hole, sweep_value)
        ]
        intervals.extend(_subtract_intervals(allowed, hole_intervals))

    clipped = []
    for left, right in _merge_intervals(intervals):
        left += edge_clearance
        right -= edge_clearance
        if right - left > EPSILON:
            clipped.append((left, right))
    return clipped


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


def _safe_connector_length(
    start: LocalPoint,
    end: LocalPoint,
    geometry: RotatedRegion,
    max_connector: float,
    connection_tolerance_m: float = 0.0,
) -> Optional[float]:
    """Return a safe orthogonal connector length with a fast common path."""
    if _local_distance(start, end) <= EPSILON:
        return 0.0

    direct_candidates = []
    for candidate in _connector_candidates(start, end)[1:]:
        if all(
            _line_is_allowed_region(
                first, second, geometry, connection_tolerance_m
            )
            for first, second in zip(candidate, candidate[1:])
        ):
            length = _path_length(candidate)
            if length <= max_connector + EPSILON:
                direct_candidates.append(length)
    if direct_candidates:
        return min(direct_candidates)

    try:
        return _path_length(
            _find_region_connector(
                start,
                end,
                geometry,
                max_connector,
                connection_tolerance_m,
            )
        )
    except PlanningError:
        return None


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


def _path_length(points: Sequence[LocalPoint]) -> float:
    return sum(_local_distance(first, second) for first, second in zip(points, points[1:]))


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
    _, geometry = _rotated_region_geometry(map_data, region, axis_angle)
    segments = []
    for row_index, sweep_value in enumerate(
        _region_scan_values(geometry, sweep_spacing, edge_clearance)
    ):
        intervals = _region_scanline_intervals(geometry, sweep_value, edge_clearance)
        if row_index % 2:
            intervals = list(reversed(intervals))
        for left, right in intervals:
            if row_index % 2:
                segments.append(((right, sweep_value), (left, sweep_value)))
            else:
                segments.append(((left, sweep_value), (right, sweep_value)))
    if not segments:
        raise PlanningError(f"region {region.id} has no usable coverage segment")
    return geometry, segments


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
    connector_cache: dict[Tuple[LocalPoint, LocalPoint], Optional[float]] = {}

    def connector_length(
        start: LocalPoint, end: LocalPoint
    ) -> Optional[float]:
        key = (start, end)
        if key not in connector_cache:
            connector_cache[key] = _safe_connector_length(
                start,
                end,
                geometry,
                max_connector,
                connection_tolerance_m,
            )
        return connector_cache[key]

    def segment_start(index: int, orientation: int) -> LocalPoint:
        start, end = local_segments[index]
        return start if orientation == 0 else end

    def segment_end(index: int, orientation: int) -> LocalPoint:
        start, end = local_segments[index]
        return end if orientation == 0 else start

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

    def ranking(state):
        _, last, orientation, longest, total, _ = state
        tail = (
            0.0
            if local_exit is None
            else connector_length(segment_end(last, orientation), local_exit)
        )
        # A missing tail is kept as a valid partial state, but ranked after
        # states that can already reach the next bridge.
        tail_rank = math.inf if tail is None else tail
        return (longest, total, tail_rank)

    beam.sort(key=ranking)
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
                    if previous is None or (
                        next_longest,
                        next_total,
                    ) < (previous[3], previous[4]):
                        next_states[next_state_key] = candidate

        if not next_states:
            raise PlanningError(
                "no safe connector ordering between multi-polygon coverage segments"
            )
        beam = sorted(next_states.values(), key=ranking)[:beam_width]

    best = None
    best_metric = (math.inf, math.inf, math.inf)
    for state in beam:
        _, last, orientation, longest, total, selected = state
        tail = (
            0.0
            if local_exit is None
            else connector_length(segment_end(last, orientation), local_exit)
        )
        if tail is None:
            continue
        metric = (max(longest, tail), total + tail, tail)
        if metric < best_metric:
            best_metric = metric
            best = selected

    if best is None:
        raise PlanningError(
            "no safe connector ordering between multi-polygon coverage segments"
        )
    return tuple(best)


def _optimized_multi_polygon_segments(
    map_data: AutoMap,
    axis_angle: float,
    local_segments: Sequence[Tuple[LocalPoint, LocalPoint]],
    geometry: RotatedRegion,
    max_connector: float,
    connection_tolerance_m: float,
    start_point: Optional[Point],
    exit_point: Optional[Point],
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

    # Each metric is (longest connector, total connector length). This keeps
    # one unavoidable branch from becoming the dominant turn in the route.
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
    regions = {region.id: region for region in map_data.regions}
    connectors = {connector.id: connector for connector in map_data.connectors}
    if not regions:
        legacy_region = Region(
            "legacy", (Polygon(map_data.boundary, map_data.no_go),), map_data.start
        )
        regions = {legacy_region.id: legacy_region}
    order = map_data.order or tuple(regions)

    output = []
    max_connector_length = 0.0
    previous_region: Optional[Region] = None
    pending_entry: Optional[Point] = None

    for index, item in enumerate(order):
        if item in regions:
            region = regions[item]
            start_point = pending_entry
            prefer_diagonal_attachment = pending_entry is not None
            if start_point is None:
                start_point = region.start
            if index == 0 and map_data.start is not None:
                start_point = map_data.start
            next_item = order[index + 1] if index + 1 < len(order) else None
            exit_point = (
                connectors[next_item].path[0]
                if next_item in connectors
                else None
            )
            geometry, local_segments = _local_region_coverage_segments(
                map_data, region, axis_angle, sweep_spacing, edge_clearance
            )
            routing_geometry = geometry
            if len(region.polygons) > 1:
                # Edge offsets shape coverage lines, but travel between
                # adjoining polygons must still be allowed on their original
                # shared boundary.
                _, routing_geometry = _rotated_region_geometry(
                    map_data, region, axis_angle, apply_edge_distance=False
                )
            _, attachment_geometry = _rotated_region_geometry(
                map_data, region, axis_angle, apply_edge_distance=False
            )
            preserve_segment_order = False
            if len(region.polygons) > 1:
                local_segments = _optimized_multi_polygon_segments(
                    map_data,
                    axis_angle,
                    local_segments,
                    routing_geometry,
                    max_connector,
                    region.connection_tolerance_m,
                    start_point,
                    exit_point,
                )
                preserve_segment_order = True
            region_route = _route_from_local_segments(
                map_data,
                axis_angle,
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
            continue

        connector = connectors.get(item)
        if connector is None:
            raise PlanningError(f"order references unknown connector: {item}")
        if previous_region is None or previous_region.id != connector.from_region:
            raise PlanningError(f"connector {item} has no matching source region")
        if index == len(order) - 1 or order[index + 1] != connector.to_region:
            raise PlanningError(f"connector {item} has no matching destination region")

        frame, source_geometry = _rotated_region_geometry(
            map_data,
            previous_region,
            axis_angle,
            apply_edge_distance=False,
        )
        connector_start_local = _rotate(frame.to_xy(connector.path[0]), axis_angle)
        if not _point_is_allowed_region(
            connector_start_local, source_geometry, allow_hole_boundary=False
        ):
            raise PlanningError(f"connector {item} starts outside region {connector.from_region}")
        previous_end = output[-1].points[-1]
        previous_end_local = _rotate(frame.to_xy(previous_end), axis_angle)
        if _local_distance(previous_end_local, connector_start_local) > EPSILON:
            attach = _bridge_attachment_connector(
                previous_end_local,
                connector_start_local,
                source_geometry,
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

        destination_region = regions[connector.to_region]
        destination_frame, destination_geometry = _rotated_region_geometry(
            map_data,
            destination_region,
            axis_angle,
            apply_edge_distance=False,
        )
        destination_end_local = _rotate(
            destination_frame.to_xy(connector.path[-1]), axis_angle
        )
        if not _point_is_allowed_region(
            destination_end_local, destination_geometry, allow_hole_boundary=False
        ):
            raise PlanningError(f"connector {item} ends outside region {connector.to_region}")

        connector_local_path = tuple(
            _rotate(frame.to_xy(point), axis_angle) for point in connector.path
        )
        all_region_geometries = [
            _rotated_region_geometry(
                map_data, region, axis_angle, apply_edge_distance=False
            )[1]
            for region in regions.values()
        ]
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
                connector.path,
                connector_length,
                connector_id=connector.id,
                from_region=connector.from_region,
                to_region=connector.to_region,
            )
        )
        max_connector_length = max(max_connector_length, connector_length)
        pending_entry = connector.path[-1]

    total_length = sum(segment.length_m for segment in output)
    return Route(
        axis_angle_rad=axis_angle,
        segments=tuple(output),
        total_length_m=total_length,
        max_connector_length_m=max_connector_length,
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
            "order": list(route.order),
        },
    }
    return json.dumps(payload, ensure_ascii=False, indent=2)


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
        for connector in map_data.connectors:
            features.append(
                {
                    "type": "Feature",
                    "properties": {
                        "kind": "bridge",
                        "connector_id": connector.id,
                        "from_region": connector.from_region,
                        "to_region": connector.to_region,
                    },
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
    parser.add_argument("--input", required=True, help="auto map JSON file")
    parser.add_argument("--output", required=True, help="route JSON output file")
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
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    """Run the auto planner command line interface."""
    args = _build_parser().parse_args(argv)
    output_path = Path(args.output)
    try:
        map_data = load_map(args.input)
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
        )
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(route_to_json(route) + "\n", encoding="utf-8")
        output_path.with_suffix(".geojson").write_text(
            route_to_geojson(route, map_data) + "\n", encoding="utf-8"
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
