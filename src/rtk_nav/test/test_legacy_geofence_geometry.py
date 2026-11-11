from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).parents[1] / "rtk_nav"))

from geofence_geometry import (
    build_polyline_corridor,
    build_geofence_payload,
    expand_convex_polygon,
    point_allowed_by_payload,
    point_in_convex_polygon,
    point_to_polyline_distance,
)


def test_expanded_rectangle_accepts_twenty_centimeter_boundary():
    rectangle = [(0.0, 0.0), (10.0, 0.0), (10.0, 5.0), (0.0, 5.0)]
    expanded = expand_convex_polygon(rectangle, 0.2)

    assert point_in_convex_polygon((10.19, 2.5), expanded)
    assert not point_in_convex_polygon((10.21, 2.5), expanded)


def test_bridge_corridor_uses_polyline_distance():
    centerline = [(0.0, 0.0), (5.0, 0.0), (5.0, 5.0)]
    corridor = build_polyline_corridor(centerline, 0.2)

    assert point_in_convex_polygon((4.9, 0.1), corridor[0])
    assert point_in_convex_polygon((5.1, 4.9), corridor[1])
    assert point_to_polyline_distance((2.5, 0.21), centerline) > 0.2


def test_payload_allows_area_and_bridge_union():
    payload = build_geofence_payload(
        ["E1", "E2"],
        [[(0.0, 0.0), (10.0, 0.0), (10.0, 5.0), (0.0, 5.0)],
         [(20.0, 0.0), (30.0, 0.0), (30.0, 5.0), (20.0, 5.0)]],
        [{
            "name": "E1_to_E2",
            "from_area": "E1",
            "to_area": "E2",
            "centerline_utm": [(10.0, 2.5), (20.0, 2.5)],
        }],
        (49, "S"),
    )

    assert point_allowed_by_payload((5.0, 2.5), payload)
    assert point_allowed_by_payload((15.0, 2.5), payload)
    assert not point_allowed_by_payload((15.0, 3.0), payload)


def test_payload_bridge_corner_has_no_segment_join_gap():
    payload = build_geofence_payload(
        ["E1"],
        [[(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]],
        [{
            "name": "turning_bridge",
            "from_area": "E1",
            "to_area": "E1",
            "centerline_utm": [(2.0, 0.0), (2.0, 1.0), (3.0, 1.0)],
        }],
        (49, "S"),
    )

    assert point_allowed_by_payload((2.1, 0.9), payload)


def test_payload_skips_zero_area_bridge_placeholder_regions():
    payload = build_geofence_payload(
        ["bridge_E1_E2", "E1"],
        [[(0.0, 0.0), (0.0, 1.0), (0.0, 1.0), (0.0, 0.0)],
         [(10.0, 0.0), (20.0, 0.0), (20.0, 5.0), (10.0, 5.0)]],
        [],
        (49, "S"),
    )

    assert [area["name"] for area in payload["areas"]] == ["E1"]
