# Copyright 2026
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

import json
import math
import os
import sys
import unittest
from pathlib import Path


PACKAGE_ROOT = Path(__file__).parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from rtk_nav.auto_path_planner import (  # noqa: E402
    PlanningError,
    extract_scanline_intervals,
    estimate_axis_angle,
    load_map,
    main,
    plan_route,
    route_to_geojson,
    route_to_json,
)
from rtk_nav.auto_path_planner import (  # noqa: E402
    Route,
    Segment,
    _rotated_region_geometry,
)


def _rectangle_map(width_m=10.0, height_m=5.0, no_go=False):
    origin_lon = 110.0
    origin_lat = 35.0
    lon_step = width_m / (111320.0 * math.cos(math.radians(origin_lat)))
    lat_step = height_m / 110540.0
    x0 = origin_lon
    x1 = origin_lon + lon_step
    y0 = origin_lat
    y1 = origin_lat + lat_step
    payload = {
        "boundary": [[x0, y0], [x1, y0], [x1, y1], [x0, y1]],
        "guides": [[[x0, y0 + lat_step / 2.0], [x1, y0 + lat_step / 2.0]]],
    }
    if no_go:
        payload["no_go"] = [
            [
                [x0 + lon_step * 0.4, y0 + lat_step * 0.2],
                [x0 + lon_step * 0.6, y0 + lat_step * 0.2],
                [x0 + lon_step * 0.6, y0 + lat_step * 0.8],
                [x0 + lon_step * 0.4, y0 + lat_step * 0.8],
            ]
        ]
    return payload


def _metric_point(x_m, y_m):
    origin_lon = 110.0
    origin_lat = 35.0
    lon_step = 1.0 / (111320.0 * math.cos(math.radians(origin_lat)))
    lat_step = 1.0 / 110540.0
    return [origin_lon + x_m * lon_step, origin_lat + y_m * lat_step]


def _metric_xy(point):
    origin_lat = math.radians(35.0)
    return (
        (point[0] - 110.0) * 111320.0 * math.cos(origin_lat),
        (point[1] - 35.0) * 110540.0,
    )


def _ring(points):
    return [_metric_point(x, y) for x, y in points]


def _complex_multi_region_map():
    return {
        "format": "rtk_auto_map_v2",
        "guides": [[_metric_point(0, 0), _metric_point(10, 0)]],
        "regions": [
            {
                "id": "E9",
                "polygons": [
                    {
                        "boundary": _ring(
                            [(0, 0), (4, 0), (4, 2), (2, 2), (2, 5), (0, 5)]
                        ),
                        "holes": [],
                    }
                ],
            },
            {
                "id": "E10",
                "polygons": [
                    {
                        "boundary": _ring([(6, 0), (10, 0), (10, 5), (6, 5)]),
                        "holes": [_ring([(7, 1), (8, 1), (8, 4), (7, 4)])],
                    }
                ],
            },
        ],
        "connectors": [
            {
                "id": "bridge_9-10B",
                "from": "E9",
                "to": "E10",
                "path": [_metric_point(1.8, 4.8), _metric_point(6.2, 4.8)],
            }
        ],
        "order": ["E9", "bridge_9-10B", "E10"],
    }


def _bridge_entry_map():
    return {
        "format": "rtk_auto_map_v2",
        "guides": [[_metric_point(0, 0), _metric_point(10, 0)]],
        "regions": [
            {
                "id": "E9",
                "polygons": [
                    {"boundary": _ring([(0, 10), (10, 10), (10, 18), (0, 18)])}
                ],
            },
            {
                "id": "E10",
                "polygons": [
                    {"boundary": _ring([(0, 0), (10, 0), (10, 10), (0, 10)])}
                ],
            },
        ],
        "connectors": [
            {
                "id": "bridge_9-10B",
                "from": "E9",
                "to": "E10",
                "path": [_metric_point(9.8, 10.2), _metric_point(9.8, 9.8)],
            }
        ],
        "order": ["E9", "bridge_9-10B", "E10"],
    }


class AutoPathPlannerTests(unittest.TestCase):
    def test_e9_e11_map_models_long_blocks_and_bridge_endpoints(self):
        map_path = REPOSITORY_ROOT / "auto_map_e9_e11.json"
        if not map_path.exists():
            self.skipTest("repository E9/E11 map fixture is not present")

        payload = json.loads(map_path.read_text(encoding="utf-8"))
        model = load_map(payload)
        regions = {region.id: region for region in model.regions}
        self.assertLess(max(point[1] for point in regions["E9"].polygons[0].boundary), 35.6051)
        self.assertEqual(len(regions["E10"].polygons), 3)
        self.assertTrue(all(not polygon.holes for polygon in regions["E10"].polygons))
        self.assertEqual(len(regions["E11"].polygons), 3)

        route = plan_route(model, sweep_spacing=2.0, edge_clearance=1.0, max_connector=50.0)
        for connector in model.connectors:
            bridge_index = next(
                index
                for index, segment in enumerate(route.segments)
                if segment.connector_id == connector.id
            )
            self.assertEqual(route.segments[bridge_index].points[0], connector.path[0])
            self.assertEqual(route.segments[bridge_index].points[-1], connector.path[-1])
            self.assertEqual(route.segments[bridge_index - 1].points[-1], connector.path[0])
            self.assertEqual(route.segments[bridge_index + 1].points[0], connector.path[-1])

        e10_bridge_index = next(
            index
            for index, segment in enumerate(route.segments)
            if segment.connector_id == "bridge_10A-11B"
        )
        e10_internal_attach = route.segments[e10_bridge_index - 1]
        self.assertEqual(e10_internal_attach.region_id, "E10")
        self.assertLess(e10_internal_attach.length_m, 15.0)
        self.assertLess(route.max_connector_length_m, 35.0)
        self.assertGreater(route.segments[-1].points[-1][0], 110.6477)

    def test_e12_e14_map_uses_two_polygon_e13_with_gap_tolerance(self):
        map_path = REPOSITORY_ROOT / "auto_map_e12_e14.json"
        if not map_path.exists():
            self.skipTest("repository E12/E14 map fixture is not present")

        model = load_map(json.loads(map_path.read_text(encoding="utf-8")))
        regions = {region.id: region for region in model.regions}
        self.assertEqual(len(regions["E13"].polygons), 2)
        self.assertEqual(regions["E13"].connection_tolerance_m, 3.0)
        self.assertEqual(
            model.order,
            ("E12", "bridge_12-13B", "E13", "bridge_13-14B", "E14"),
        )

        route = plan_route(
            model,
            sweep_spacing=2.0,
            edge_clearance=1.0,
            max_connector=50.0,
        )
        self.assertTrue(any(segment.kind == "coverage" for segment in route.segments))
        self.assertLess(route.max_connector_length_m, 50.0)
        geojson = json.loads(route_to_geojson(route, model))
        e13_boundary = next(
            feature
            for feature in geojson["features"]
            if feature["properties"].get("region_id") == "E13"
            and feature["properties"].get("polygon_index") == 0
        )
        self.assertEqual(e13_boundary["properties"]["edge_distance_lon"], [0.15, 0.5])
        self.assertEqual(e13_boundary["properties"]["edge_distance_lat"], [0.3, 0.3])

    def test_v2_accepts_named_four_corner_boundaries_and_point_objects(self):
        payload = {
            "format": "rtk_auto_map_v2",
            "regions": [
                {
                    "id": "E9",
                    "boundary": {
                        "top_left": {"lon": 110.0, "lat": 35.001},
                        "top_right": {"lon": 110.001, "lat": 35.001},
                        "bottom_right": {"lon": 110.001, "lat": 35.0},
                        "bottom_left": {"lon": 110.0, "lat": 35.0},
                    },
                }
            ],
        }
        model = load_map(payload)
        self.assertEqual(model.regions[0].id, "E9")
        self.assertEqual(len(model.regions[0].polygons[0].boundary), 5)
        self.assertTrue(model.guides)

    def test_edge_distances_adjust_four_edges_with_legacy_order(self):
        payload = {
            "format": "rtk_auto_map_v2",
            "guides": [[_metric_point(0, 0), _metric_point(10, 0)]],
            "regions": [
                {
                    "id": "adjusted",
                    "edge_distance_lon": [1.0, 2.0],
                    "edge_distance_lat": [0.5, 1.5],
                    "polygons": [{"boundary": _ring([(0, 0), (10, 0), (10, 4), (0, 4)])}],
                },
            ],
            "order": ["adjusted"],
        }
        model = load_map(payload)
        self.assertEqual(model.regions[0].polygons[0].edge_distance_lon, (1.0, 2.0))
        self.assertEqual(model.regions[0].polygons[0].edge_distance_lat, (0.5, 1.5))

        _, adjusted = _rotated_region_geometry(model, model.regions[0], 0.0)
        adjusted_boundary = adjusted[0][0]
        self.assertTrue(
            all(
                math.isclose(actual, expected, abs_tol=1e-6)
                for actual, expected in zip(
                    (
                        min(point[0] for point in adjusted_boundary[:-1]),
                        max(point[0] for point in adjusted_boundary[:-1]),
                        min(point[1] for point in adjusted_boundary[:-1]),
                        max(point[1] for point in adjusted_boundary[:-1]),
                    ),
                    (2.0, 9.0, 0.5, 2.5),
                )
            )
        )
        expanded_payload = {
            "format": "rtk_auto_map_v2",
            "guides": [[_metric_point(0, 0), _metric_point(10, 0)]],
            "regions": [
                {
                    "id": "expanded",
                    "polygons": [
                        {
                            "boundary": _ring([(0, 0), (10, 0), (10, 4), (0, 4)]),
                            "edge_distance_lon": [-1.0, -2.0],
                            "edge_distance_lat": [-0.5, -1.5],
                        }
                    ],
                }
            ],
            "order": ["expanded"],
        }
        expanded_model = load_map(expanded_payload)
        _, expanded = _rotated_region_geometry(
            expanded_model, expanded_model.regions[0], 0.0
        )
        expanded_boundary = expanded[0][0]
        self.assertTrue(
            all(
                math.isclose(actual, expected, abs_tol=1e-6)
                for actual, expected in zip(
                    (
                        min(point[0] for point in expanded_boundary[:-1]),
                        max(point[0] for point in expanded_boundary[:-1]),
                        min(point[1] for point in expanded_boundary[:-1]),
                        max(point[1] for point in expanded_boundary[:-1]),
                    ),
                    (-2.0, 11.0, -0.5, 5.5),
                )
            )
        )

    def test_geojson_uses_point_for_degenerate_segment(self):
        route = Route(
            axis_angle_rad=0.0,
            segments=(
                Segment(
                    kind="connector",
                    points=((110.0, 35.0),),
                    length_m=0.0,
                ),
            ),
            total_length_m=0.0,
            max_connector_length_m=0.0,
        )
        document = json.loads(route_to_geojson(route))
        geometry = document["features"][0]["geometry"]
        self.assertEqual(geometry["type"], "Point")
        self.assertEqual(geometry["coordinates"], [110.0, 35.0])

    def test_load_map_closes_boundary_and_requires_guide(self):
        model = load_map(
            {
                "boundary": [[0, 0], [10, 0], [10, 10]],
                "guides": [[[1, 1], [1, 9]]],
            }
        )
        self.assertEqual(model.boundary[0], model.boundary[-1])
        with self.assertRaises(PlanningError):
            load_map(
                {
                    "boundary": [[0, 0], [10, 0], [10, 10]],
                    "guides": [],
                }
            )

    def test_opposite_guide_directions_produce_same_axis(self):
        forward = estimate_axis_angle([[(0, 0), (0, 10)]])
        reverse = estimate_axis_angle([[(0, 10), (0, 0)]])
        self.assertTrue(math.isclose(abs(forward - reverse), 0.0, abs_tol=1e-9))

    def test_rectangle_generates_non_overlapping_parallel_coverage_segments(self):
        model = load_map(_rectangle_map())
        route = plan_route(model, sweep_spacing=1.0, edge_clearance=0.2)
        coverage = [segment for segment in route.segments if segment.kind == "coverage"]
        self.assertEqual(len(coverage), 5)
        self.assertTrue(all(segment.length_m > 9.0 for segment in coverage))
        self.assertEqual(len(route.segments), 9)

    def test_no_go_splits_scanline_and_rejects_unsafe_connector(self):
        model = load_map(_rectangle_map(no_go=True))
        intervals = extract_scanline_intervals(
            model, axis_angle=0.0, sweep_value=2.0, edge_clearance=0.2
        )
        self.assertEqual(len(intervals), 2)
        with self.assertRaises(PlanningError):
            plan_route(
                model,
                sweep_spacing=1.0,
                edge_clearance=0.2,
                max_connector=0.5,
            )

    def test_route_serializers_include_segment_kinds_and_metrics(self):
        model = load_map(_rectangle_map(width_m=4.0, height_m=2.0))
        route = plan_route(model, sweep_spacing=1.0, edge_clearance=0.1)
        route_json = json.loads(route_to_json(route))
        route_geojson = json.loads(route_to_geojson(route, model))
        self.assertEqual(route_json["format"], "rtk_auto_route_v1")
        self.assertIn("coverage_segments", route_json["metrics"])
        self.assertEqual(route_geojson["type"], "FeatureCollection")
        self.assertEqual(
            {feature["properties"]["kind"] for feature in route_geojson["features"]},
            {"boundary", "coverage", "connector"},
        )
        boundary = next(
            feature
            for feature in route_geojson["features"]
            if feature["properties"]["kind"] == "boundary"
        )
        self.assertEqual(boundary["geometry"]["type"], "Polygon")

    def test_start_must_be_inside_usable_area(self):
        model_data = _rectangle_map(width_m=4.0, height_m=2.0)
        model_data["start"] = [111.0, 36.0]
        with self.assertRaises(PlanningError):
            plan_route(load_map(model_data), sweep_spacing=1.0, edge_clearance=0.1)

    def test_v2_map_parses_regions_connectors_and_order(self):
        model = load_map(_complex_multi_region_map())
        self.assertEqual([region.id for region in model.regions], ["E9", "E10"])
        self.assertEqual([connector.id for connector in model.connectors], ["bridge_9-10B"])
        self.assertEqual(model.order, ("E9", "bridge_9-10B", "E10"))
        self.assertEqual(len(model.regions[1].polygons[0].holes), 1)

    def test_concave_and_hole_scanlines_remain_split_into_safe_intervals(self):
        model = load_map(_complex_multi_region_map())
        e9 = model.regions[0]
        e10 = model.regions[1]
        e9_intervals = extract_scanline_intervals(
            model, axis_angle=0.0, sweep_value=3.0, edge_clearance=0.1, region_id="E9"
        )
        e10_intervals = extract_scanline_intervals(
            model, axis_angle=0.0, sweep_value=2.0, edge_clearance=0.1, region_id="E10"
        )
        self.assertEqual(len(e9_intervals), 1)
        self.assertGreater(e9_intervals[0][1] - e9_intervals[0][0], 1.0)
        self.assertEqual(len(e10_intervals), 2)
        self.assertIsNotNone(e9)

    def test_v2_route_follows_order_and_marks_region_and_bridge_segments(self):
        route = plan_route(
            load_map(_complex_multi_region_map()),
            sweep_spacing=1.0,
            edge_clearance=0.2,
            max_connector=12.0,
        )
        coverage = [segment for segment in route.segments if segment.kind == "coverage"]
        explicit_bridges = [
            segment
            for segment in route.segments
            if segment.connector_id == "bridge_9-10B"
        ]
        self.assertTrue(coverage)
        self.assertEqual({segment.region_id for segment in coverage}, {"E9", "E10"})
        self.assertEqual(len(explicit_bridges), 1)
        self.assertEqual(explicit_bridges[0].kind, "connector")
        self.assertEqual(route.order, ("E9", "bridge_9-10B", "E10"))
        route_json = json.loads(route_to_json(route))
        bridge_json = next(
            item for item in route_json["segments"] if item.get("connector_id")
        )
        self.assertEqual(bridge_json["connector_id"], "bridge_9-10B")
        route_geojson = json.loads(route_to_geojson(route))
        bridge_feature = next(
            feature
            for feature in route_geojson["features"]
            if feature["properties"].get("connector_id")
        )
        self.assertEqual(bridge_feature["properties"]["to_region"], "E10")
        for segment in coverage:
            for first, second in zip(segment.points, segment.points[1:]):
                midpoint = (
                    (first[0] + second[0]) / 2.0,
                    (first[1] + second[1]) / 2.0,
                )
                x_m, y_m = _metric_xy(midpoint)
                self.assertFalse(7.0 < x_m < 8.0 and 1.0 < y_m < 4.0)

    def test_bridge_entry_keeps_every_turn_short(self):
        route = plan_route(
            load_map(_bridge_entry_map()),
            sweep_spacing=2.0,
            edge_clearance=0.2,
            max_connector=3.0,
        )
        coverage = [segment for segment in route.segments if segment.kind == "coverage"]
        self.assertTrue(coverage)
        self.assertEqual({segment.region_id for segment in coverage}, {"E9", "E10"})

    def test_hole_connectors_are_orthogonal(self):
        route = plan_route(
            load_map(_rectangle_map(no_go=True)),
            sweep_spacing=1.0,
            edge_clearance=0.2,
            max_connector=100.0,
        )
        connectors = [segment for segment in route.segments if segment.kind == "connector"]
        self.assertTrue(connectors)
        for segment in connectors:
            for first, second in zip(segment.points, segment.points[1:]):
                first_x, first_y = _metric_xy(first)
                second_x, second_y = _metric_xy(second)
                self.assertTrue(
                    abs(first_x - second_x) < 1e-5
                    or abs(first_y - second_y) < 1e-5
                )

    def test_v2_region_can_combine_touching_polygons_without_duplicate_sweeps(self):
        payload = {
            "format": "rtk_auto_map_v2",
            "guides": [[_metric_point(0, 0), _metric_point(8, 0)]],
            "regions": [
                {
                    "id": "combined",
                    "polygons": [
                        {"boundary": _ring([(0, 0), (4, 0), (4, 2), (0, 2)])},
                        {"boundary": _ring([(4, 0), (8, 0), (8, 2), (4, 2)])},
                    ],
                }
            ],
            "order": ["combined"],
        }
        route = plan_route(load_map(payload), sweep_spacing=1.0, edge_clearance=0.2)
        coverage = [segment for segment in route.segments if segment.kind == "coverage"]
        self.assertEqual(len(coverage), 2)
        self.assertTrue(all(segment.length_m > 7.0 for segment in coverage))
        self.assertEqual({segment.region_id for segment in coverage}, {"combined"})

    def test_v2_rejects_order_with_unknown_or_mismatched_connector(self):
        unknown_order = _complex_multi_region_map()
        unknown_order["order"] = ["E9", "missing", "E10"]
        with self.assertRaises(PlanningError):
            load_map(unknown_order)

        mismatched = _complex_multi_region_map()
        mismatched["connectors"][0]["from"] = "E10"
        with self.assertRaises(PlanningError):
            load_map(mismatched)

    def test_v2_rejects_region_without_a_safe_internal_connector(self):
        payload = {
            "format": "rtk_auto_map_v2",
            "guides": [[_metric_point(0, 0), _metric_point(10, 0)]],
            "regions": [
                {
                    "id": "broken",
                    "polygons": [
                        {"boundary": _ring([(0, 0), (2, 0), (2, 2), (0, 2)]), "holes": []},
                        {"boundary": _ring([(5, 0), (7, 0), (7, 2), (5, 2)]), "holes": []},
                    ],
                }
            ],
            "connectors": [],
            "order": ["broken"],
        }
        with self.assertRaises(PlanningError):
            plan_route(load_map(payload), sweep_spacing=1.0, edge_clearance=0.1)

    def test_v2_region_connection_tolerance_bridges_small_polygon_gap(self):
        payload = {
            "format": "rtk_auto_map_v2",
            "guides": [[_metric_point(0, 0), _metric_point(6, 0)]],
            "regions": [
                {
                    "id": "combined",
                    "connection_tolerance_m": 2.0,
                    "polygons": [
                        {"boundary": _ring([(0, 0), (2, 0), (2, 2), (0, 2)])},
                        {"boundary": _ring([(3, 0), (5, 0), (5, 2), (3, 2)])},
                    ],
                }
            ],
            "order": ["combined"],
        }

        model = load_map(payload)
        self.assertEqual(model.regions[0].connection_tolerance_m, 2.0)
        route = plan_route(
            model,
            sweep_spacing=1.0,
            edge_clearance=0.1,
            max_connector=10.0,
        )
        internal_connectors = [
            segment
            for segment in route.segments
            if segment.kind == "connector"
        ]
        self.assertTrue(internal_connectors)
        self.assertTrue(any(segment.length_m > 1.0 for segment in internal_connectors))

    def test_v2_rejects_explicit_connector_path_through_a_hole(self):
        payload = _complex_multi_region_map()
        payload["connectors"][0]["path"] = [
            _metric_point(1.8, 2.0),
            _metric_point(7.5, 2.0),
            _metric_point(6.2, 2.0),
        ]
        with self.assertRaises(PlanningError):
            plan_route(load_map(payload), sweep_spacing=1.0, edge_clearance=0.2)

    def test_v2_rejects_connector_with_source_endpoint_outside_region(self):
        payload = _complex_multi_region_map()
        payload["connectors"][0]["path"][0] = _metric_point(5.0, 5.0)
        with self.assertRaises(PlanningError):
            plan_route(load_map(payload), sweep_spacing=1.0, edge_clearance=0.2)

    def test_v2_does_not_accept_a_shortcut_through_a_narrow_hole(self):
        payload = {
            "format": "rtk_auto_map_v2",
            "guides": [[_metric_point(0, 0), _metric_point(10, 0)]],
            "regions": [
                {
                    "id": "narrow-hole",
                    "polygons": [
                        {
                            "boundary": _ring([(0, 0), (10, 0), (10, 1), (0, 1)]),
                            "holes": [_ring([(4, 0.001), (4.1, 0.001), (4.1, 0.999), (4, 0.999)])],
                        }
                    ],
                }
            ],
            "order": ["narrow-hole"],
        }
        with self.assertRaises(PlanningError):
            plan_route(
                load_map(payload),
                sweep_spacing=2.0,
                edge_clearance=0.01,
                max_connector=0.2,
            )

    def test_v2_rejects_self_intersecting_boundary_and_outside_hole(self):
        self_intersecting = {
            "format": "rtk_auto_map_v2",
            "guides": [[_metric_point(0, 0), _metric_point(2, 0)]],
            "regions": [
                {
                    "id": "bad-boundary",
                    "polygons": [{"boundary": _ring([(0, 0), (2, 2), (0, 2), (2, 0)])}],
                }
            ],
            "order": ["bad-boundary"],
        }
        with self.assertRaises(PlanningError):
            load_map(self_intersecting)

        outside_hole = {
            "format": "rtk_auto_map_v2",
            "guides": [[_metric_point(0, 0), _metric_point(2, 0)]],
            "regions": [
                {
                    "id": "bad-hole",
                    "polygons": [
                        {
                            "boundary": _ring([(0, 0), (2, 0), (2, 2), (0, 2)]),
                            "holes": [_ring([(1.5, 0.5), (2.5, 0.5), (2.5, 1.5), (1.5, 1.5)])],
                        }
                    ],
                }
            ],
            "order": ["bad-hole"],
        }
        with self.assertRaises(PlanningError):
            load_map(outside_hole)

    def test_cli_writes_json_and_geojson(self):
        model_data = _rectangle_map(width_m=4.0, height_m=2.0)
        temp_path = PACKAGE_ROOT.parent.parent / "tmp"
        temp_path.mkdir(parents=True, exist_ok=True)
        suffix = str(os.getpid())
        input_path = temp_path / f"test_auto_map_{suffix}.json"
        output_path = temp_path / f"test_auto_route_{suffix}.json"
        try:
            input_path.write_text(json.dumps(model_data), encoding="utf-8")
            result = main(
                [
                    "--input",
                    str(input_path),
                    "--output",
                    str(output_path),
                    "--sweep-spacing",
                    "1.0",
                    "--edge-clearance",
                    "0.1",
                ]
            )
            self.assertEqual(result, 0)
            self.assertTrue(output_path.exists())
            self.assertTrue(output_path.with_suffix(".geojson").exists())
            route_document = json.loads(output_path.read_text(encoding="utf-8"))
            geojson_document = json.loads(
                output_path.with_suffix(".geojson").read_text(encoding="utf-8")
            )
            self.assertEqual(route_document["format"], "rtk_auto_route_v1")
            self.assertEqual(geojson_document["type"], "FeatureCollection")
            self.assertTrue(all(item["type"] == "Feature" for item in geojson_document["features"]))
        finally:
            for generated_path in (
                input_path,
                output_path,
                output_path.with_suffix(".geojson"),
            ):
                generated_path.unlink(missing_ok=True)


if __name__ == "__main__":
    unittest.main()
