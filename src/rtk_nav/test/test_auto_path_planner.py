# Copyright 2026
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

import json
import math
import sys
import tempfile
import unittest
from pathlib import Path


PACKAGE_ROOT = Path(__file__).parents[1]
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
        route_geojson = json.loads(route_to_geojson(route))
        self.assertEqual(route_json["format"], "rtk_auto_route_v1")
        self.assertIn("coverage_segments", route_json["metrics"])
        self.assertEqual(route_geojson["type"], "FeatureCollection")
        self.assertEqual(
            {feature["properties"]["kind"] for feature in route_geojson["features"]},
            {"coverage", "connector"},
        )

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
        with tempfile.TemporaryDirectory(dir=PACKAGE_ROOT) as temp_dir:
            temp_path = Path(temp_dir)
            input_path = temp_path / "auto_map.json"
            output_path = temp_path / "auto_route.json"
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


if __name__ == "__main__":
    unittest.main()
