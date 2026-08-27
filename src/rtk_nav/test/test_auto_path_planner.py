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
    convert_legacy_yaml_to_map,
    extract_scanline_intervals,
    estimate_axis_angle,
    load_map,
    main,
    plan_route,
    route_to_geojson,
    route_to_json,
    route_to_txt,
    yaml as planner_yaml,
)
from rtk_nav.auto_path_planner import (  # noqa: E402
    Route,
    Segment,
    _bridge_attachment_connector,
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


def _connector_boundary_offset_map():
    return {
        "format": "rtk_auto_map_v2",
        "guides": [[_metric_point(0, 0), _metric_point(10, 0)]],
        "regions": [
            {
                "id": "source",
                "polygons": [
                    {"boundary": _ring([(0, 0), (10, 0), (10, 4), (0, 4)])}
                ],
            },
            {
                "id": "destination",
                "polygons": [
                    {"boundary": _ring([(0, 6), (10, 6), (10, 10), (0, 10)])}
                ],
            },
        ],
        "connectors": [
            {
                "id": "bridge-source-destination",
                "from": "source",
                "to": "destination",
                "path": [
                    _metric_point(10, 4),
                    _metric_point(10, 5),
                    _metric_point(9, 6),
                ],
            }
        ],
        "order": ["source", "bridge-source-destination", "destination"],
    }


class AutoPathPlannerTests(unittest.TestCase):
    def test_route_to_txt_keeps_corners_and_interpolates_long_segments(self):
        route = Route(
            axis_angle_rad=0.0,
            segments=(
                Segment(
                    kind="coverage",
                    points=(_metric_point(0.0, 0.0), _metric_point(40.0, 0.0), _metric_point(40.0, 20.0)),
                    length_m=60.0,
                    region_id="E14",
                ),
            ),
            total_length_m=60.0,
            max_connector_length_m=0.0,
        )

        document = route_to_txt(route)
        lines = document.splitlines()
        self.assertEqual(lines[0], "序号,经度,纬度,航向角(度)")
        rows = [line.split(",") for line in lines[1:] if not line.startswith("#")]
        points = [(float(row[1]), float(row[2])) for row in rows]
        headings = [float(row[3]) for row in rows]

        self.assertEqual(len(points), 6)
        self.assertTrue(
            math.isclose(_metric_xy(points[0])[0], 0.0, abs_tol=0.001)
        )
        self.assertTrue(
            any(
                math.isclose(_metric_xy(point)[0], 40.0, abs_tol=0.001)
                and math.isclose(_metric_xy(point)[1], 0.0, abs_tol=0.001)
                for point in points
            )
        )
        self.assertTrue(
            math.isclose(_metric_xy(points[-1])[0], 40.0, abs_tol=0.001)
        )
        self.assertTrue(
            math.isclose(_metric_xy(points[-1])[1], 20.0, abs_tol=0.001)
        )
        self.assertAlmostEqual(headings[0], 90.0, places=3)
        self.assertAlmostEqual(headings[3], 0.0, places=3)
        self.assertAlmostEqual(headings[-1], headings[-2], places=3)

        distances = [
            math.hypot(
                _metric_xy(first)[0] - _metric_xy(second)[0],
                _metric_xy(first)[1] - _metric_xy(second)[1],
            )
            for first, second in zip(points, points[1:])
        ]
        self.assertLessEqual(max(distances), 15.0 + 1e-6)

    def test_route_to_txt_groups_internal_connectors_under_region_label(self):
        route = Route(
            axis_angle_rad=0.0,
            segments=(
                Segment(
                    kind="coverage",
                    points=(_metric_point(0.0, 0.0), _metric_point(1.0, 0.0)),
                    length_m=1.0,
                    region_id="E12",
                ),
                Segment(
                    kind="connector",
                    points=(_metric_point(1.0, 0.0), _metric_point(1.0, 1.0)),
                    length_m=1.0,
                    region_id="E12",
                ),
                Segment(
                    kind="coverage",
                    points=(_metric_point(1.0, 1.0), _metric_point(2.0, 1.0)),
                    length_m=1.0,
                    region_id="E12",
                ),
                Segment(
                    kind="connector",
                    points=(_metric_point(2.0, 1.0), _metric_point(3.0, 1.0)),
                    length_m=1.0,
                    connector_id="bridge_12-13B",
                ),
            ),
            total_length_m=4.0,
            max_connector_length_m=1.0,
        )

        labels = [
            line
            for line in route_to_txt(route).splitlines()
            if line.startswith("#")
        ]

        self.assertEqual(labels, ["#E12", "#bridge_12-13B"])

    def test_route_to_txt_rejects_non_positive_spacing(self):
        route = Route(
            axis_angle_rad=0.0,
            segments=(
                Segment(
                    kind="coverage",
                    points=(_metric_point(0.0, 0.0), _metric_point(1.0, 0.0)),
                    length_m=1.0,
                ),
            ),
            total_length_m=1.0,
            max_connector_length_m=0.0,
        )

        with self.assertRaises(PlanningError):
            route_to_txt(route, point_spacing=0.0)

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
            bridge_segment = route.segments[bridge_index]
            self.assertEqual(
                route.segments[bridge_index - 1].points[-1],
                bridge_segment.points[0],
            )
            self.assertEqual(
                route.segments[bridge_index + 1].points[0],
                bridge_segment.points[-1],
            )
            self.assertIsNone(bridge_segment.region_id)

        first_bridge = next(
            segment
            for segment in route.segments
            if segment.connector_id == "bridge_9-10B"
        )
        self.assertNotEqual(first_bridge.points[0], model.connectors[0].path[0])

        e10_bridge_index = next(
            index
            for index, segment in enumerate(route.segments)
            if segment.connector_id == "bridge_10A-11B"
        )
        e10_bridge = route.segments[e10_bridge_index]
        self.assertIsNone(e10_bridge.region_id)
        self.assertEqual(route.segments[e10_bridge_index - 1].kind, "coverage")
        self.assertEqual(route.segments[e10_bridge_index + 1].kind, "coverage")
        self.assertLess(route.max_connector_length_m, 35.0)
        self.assertGreater(route.segments[-1].points[-1][0], 110.6477)

    def test_e13_ends_near_next_bridge_at_one_meter_spacing(self):
        map_path = REPOSITORY_ROOT / "auto_map_e12_e14.json"
        if not map_path.exists():
            self.skipTest("repository E12/E14 map fixture is not present")

        model = load_map(json.loads(map_path.read_text(encoding="utf-8")))
        route = plan_route(
            model,
            sweep_spacing=1.0,
            edge_clearance=1.0,
            max_connector=400.0,
        )
        bridge_index = next(
            index
            for index, segment in enumerate(route.segments)
            if segment.connector_id == "bridge_13-14B"
        )
        e13_coverage = [
            segment
            for segment in route.segments[:bridge_index]
            if segment.kind == "coverage" and segment.region_id == "E13"
        ]
        self.assertTrue(e13_coverage)

        origin = model.boundary[0]
        lon_scale = 111320.0 * math.cos(math.radians(origin[1]))
        lat_scale = 110540.0
        bridge_start = next(
            connector.path[0]
            for connector in model.connectors
            if connector.id == "bridge_13-14B"
        )
        end_point = e13_coverage[-1].points[-1]
        end_distance = math.hypot(
            (end_point[0] - bridge_start[0]) * lon_scale,
            (end_point[1] - bridge_start[1]) * lat_scale,
        )

        self.assertLess(end_distance, 8.0)
        self.assertEqual(
            route.segments[bridge_index - 1].points[-1],
            route.segments[bridge_index].points[0],
        )

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
        bridge_feature = next(
            feature
            for feature in geojson["features"]
            if feature["properties"].get("connector_id") == "bridge_12-13B"
            and feature["properties"].get("kind") == "bridge"
        )
        self.assertEqual(bridge_feature["properties"]["edge_distance_lon"], [0.1, 0.1])
        self.assertEqual(bridge_feature["properties"]["edge_distance_lat"], [0.1, -1.0])
        e13_boundary = next(
            feature
            for feature in geojson["features"]
            if feature["properties"].get("region_id") == "E13"
            and feature["properties"].get("polygon_index") == 0
        )
        self.assertEqual(e13_boundary["properties"]["edge_distance_lon"], [0.15, 0.5])
        self.assertEqual(
            e13_boundary["properties"]["edge_distance_lat"],
            list(regions["E13"].polygons[0].edge_distance_lat),
        )

    def test_multi_polygon_coverage_completes_each_polygon_once(self):
        map_path = REPOSITORY_ROOT / "auto_map_e12_e14.json"
        if not map_path.exists():
            self.skipTest("repository E12/E14 map fixture is not present")

        model = load_map(json.loads(map_path.read_text(encoding="utf-8")))
        route = plan_route(
            model,
            sweep_spacing=2.0,
            edge_clearance=1.0,
            max_connector=50.0,
        )
        e13 = next(region for region in model.regions if region.id == "E13")
        bounds = [
            (
                min(point[0] for point in polygon.boundary),
                max(point[0] for point in polygon.boundary),
            )
            for polygon in e13.polygons
        ]

        labels = []
        coverage_segments = []
        for segment in route.segments:
            if segment.kind != "coverage" or segment.region_id != "E13":
                continue
            coverage_segments.append(segment)
            midpoint = (
                sum(point[0] for point in segment.points) / len(segment.points),
                sum(point[1] for point in segment.points) / len(segment.points),
            )
            polygon_index = next(
                index
                for index, (left, right) in enumerate(bounds)
                if left - 1e-10 <= midpoint[0] <= right + 1e-10
            )
            labels.append(polygon_index)

        self.assertTrue(labels)
        self.assertEqual(set(labels), set(range(len(e13.polygons))))
        unique_segments = {
            tuple(
                sorted(
                    tuple(round(value, 12) for value in point)
                    for point in segment.points
                )
            )
            for segment in coverage_segments
        }
        self.assertEqual(len(unique_segments), len(coverage_segments))
        transitions = sum(
            first != second for first, second in zip(labels, labels[1:])
        )
        self.assertEqual(transitions, len(e13.polygons) - 1)
        for polygon_index in range(len(e13.polygons)):
            first = labels.index(polygon_index)
            last = len(labels) - 1 - labels[::-1].index(polygon_index)
            self.assertEqual(
                labels[first : last + 1],
                [polygon_index] * (last - first + 1),
            )

    @unittest.skipIf(planner_yaml is None, "PyYAML is not installed")
    def test_legacy_yaml_converter_recognizes_all_cardinal_region_prefixes(self):
        def area(name, x):
            return {
                "name": name,
                "calib_point_a": {"lon": x + 0.001, "lat": 35.001},
                "calib_point_b": {"lon": x + 0.001, "lat": 35.002},
                "calib_point_c": {"lon": x, "lat": 35.002},
            }

        source = {
            "default": {
                "interval": 1.0,
                "edge_distance_lon": 0.0,
                "edge_distance_lat": 0.0,
            },
            "areas": [
                area("E1", 110.000),
                {
                    "name": "bridge_E1A-W2B",
                    "calib_point_a": {"lon": 110.001, "lat": 35.001},
                    "calib_point_b": {"lon": 110.003, "lat": 35.002},
                    "calib_point_c": {"lon": 110.001, "lat": 35.001},
                },
                area("W2", 110.002),
                {
                    "name": "bridge_W2A-N3B",
                    "calib_point_a": {"lon": 110.003, "lat": 35.001},
                    "calib_point_b": {"lon": 110.005, "lat": 35.002},
                    "calib_point_c": {"lon": 110.003, "lat": 35.001},
                },
                area("N3", 110.004),
                {
                    "name": "bridge_N3A-S4B",
                    "calib_point_a": {"lon": 110.005, "lat": 35.001},
                    "calib_point_b": {"lon": 110.007, "lat": 35.002},
                    "calib_point_c": {"lon": 110.005, "lat": 35.001},
                },
                area("S4", 110.006),
            ],
        }

        payload = convert_legacy_yaml_to_map(source)

        self.assertEqual(
            [region["id"] for region in payload["regions"]],
            ["E1", "W2", "N3", "S4"],
        )
        connector_by_id = {
            connector["id"]: connector for connector in payload["connectors"]
        }
        self.assertEqual(
            (connector_by_id["bridge_E1A-W2B"]["from"],
             connector_by_id["bridge_E1A-W2B"]["to"]),
            ("E1", "W2"),
        )
        self.assertEqual(
            (connector_by_id["bridge_W2A-N3B"]["from"],
             connector_by_id["bridge_W2A-N3B"]["to"]),
            ("W2", "N3"),
        )
        self.assertEqual(
            (connector_by_id["bridge_N3A-S4B"]["from"],
             connector_by_id["bridge_N3A-S4B"]["to"]),
            ("N3", "S4"),
        )
        load_map(payload)

    @unittest.skipIf(planner_yaml is None, "PyYAML is not installed")
    def test_006_e22_w24_retraces_a_multi_polygon_tail_to_reach_exit_bridge(self):
        yaml_path = (
            PACKAGE_ROOT / "rtk_nav" / "config" / "006-E22-W24.yaml"
        )
        model = load_map(convert_legacy_yaml_to_map(yaml_path))

        route = plan_route(
            model,
            sweep_spacing=1.0,
            edge_clearance=1.0,
            max_connector=40.0,
        )

        e23_segments = [
            segment
            for segment in route.segments
            if segment.region_id == "E23"
        ]
        self.assertTrue(any(segment.kind == "coverage" for segment in e23_segments))
        self.assertTrue(
            any(
                segment.kind == "connector"
                and segment.connector_id is None
                and segment.region_id == "E23"
                for segment in e23_segments
            )
        )

    @unittest.skipIf(planner_yaml is None, "PyYAML is not installed")
    def test_003_e12_e14_yaml_converts_rectangles_and_full_connector_order(self):
        yaml_path = (
            REPOSITORY_ROOT
            / "src"
            / "rtk_nav"
            / "rtk_nav"
            / "config"
            / "003-E12-E14.yaml"
        )

        payload = convert_legacy_yaml_to_map(yaml_path)
        self.assertEqual(payload["format"], "rtk_auto_map_v2")
        self.assertIn("guides", payload)
        self.assertEqual(len(payload["guides"]), 1)
        self.assertEqual(len(payload["guides"][0]), 2)
        self.assertEqual(
            [region["id"] for region in payload["regions"]],
            ["E12", "E13", "E14"],
        )
        self.assertEqual(
            [region.get("guide") for region in payload["regions"]],
            ["horizontal", "horizontal", "horizontal"],
        )
        self.assertEqual(len(payload["regions"][1]["polygons"]), 2)
        self.assertNotIn("travel_segments", payload)
        self.assertEqual(
            [item["id"] for item in payload["connectors"]],
            [
                "bridge_5B-6B",
                "bridge_6A-7bB",
                "bridge_7bA-7A",
                "bridge_8B-9A",
                "bridge_10A-11B",
                "bridge_11A-12B",
                "bridge_12-13B",
                "bridge_13-14B",
                "back_14B-13A_stop",
                "back_13B-12A",
                "back_12B-11A",
                "back_11B-10A",
                "back_9A-8B",
                "back_7A-7bA",
                "back_7bB-6A",
                "back_6B-5A",
                "back_5B-4A",
            ],
        )
        self.assertEqual(
            payload["order"],
            [
                "bridge_5B-6B",
                "bridge_6A-7bB",
                "bridge_7bA-7A",
                "bridge_8B-9A",
                "bridge_10A-11B",
                "bridge_11A-12B",
                "E12",
                "bridge_12-13B",
                "E13",
                "bridge_13-14B",
                "E14",
                "back_14B-13A_stop",
                "back_13B-12A",
                "back_12B-11A",
                "back_11B-10A",
                "back_9A-8B",
                "back_7A-7bA",
                "back_7bB-6A",
                "back_6B-5A",
                "back_5B-4A",
            ],
        )
        self.assertTrue(
            all(
                "from" not in connector and "to" not in connector
                for connector in payload["connectors"][:6]
                + payload["connectors"][8:]
            )
        )
        self.assertEqual(
            [(connector["from"], connector["to"]) for connector in payload["connectors"][6:8]],
            [("E12", "E13"), ("E13", "E14")],
        )
        bridge_offsets = {
            connector["id"]: (
                connector["edge_distance_lon"],
                connector["edge_distance_lat"],
            )
            for connector in payload["connectors"][6:8]
        }
        self.assertEqual(
            bridge_offsets["bridge_12-13B"],
            ([0.1, 0.1], [0.1, -0.3]),
        )
        self.assertEqual(
            bridge_offsets["bridge_13-14B"],
            ([0.1, 0.1], [0.1, -0.3]),
        )

        e12 = payload["regions"][0]["polygons"][0]["boundary"]
        a = [110.64776463744738, 35.60429273226822]
        b = [110.64776416327175, 35.60442191035046]
        c = [110.64727006172468, 35.60442127126381]
        expected_d = [a[0] + c[0] - b[0], a[1] + c[1] - b[1]]
        self.assertEqual(e12[0], expected_d)
        self.assertEqual(e12[1:4], [a, b, c])
        self.assertEqual(e12[4], expected_d)

        model = load_map(payload)
        route = plan_route(
            model,
            sweep_spacing=1.0,
            edge_clearance=1.0,
            max_connector=50.0,
        )
        self.assertEqual(
            [segment.connector_id for segment in route.segments if segment.connector_id],
            [connector["id"] for connector in payload["connectors"]],
        )

        source = planner_yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
        explicit_guides = [[_metric_point(1.0, 1.0), _metric_point(2.0, 1.0)]]
        source["guides"] = explicit_guides
        explicit_payload = convert_legacy_yaml_to_map(source)
        self.assertEqual(explicit_payload["guides"], explicit_guides)

        conflicting_source = planner_yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
        next(
            area for area in conflicting_source["areas"] if area["name"] == "E13"
        )["guide"] = "vertical"
        with self.assertRaises(PlanningError):
            convert_legacy_yaml_to_map(conflicting_source)

    @unittest.skipIf(planner_yaml is None, "PyYAML is not installed")
    def test_e14_edge_offsets_keep_the_measured_slanted_edge(self):
        yaml_path = (
            REPOSITORY_ROOT
            / "src"
            / "rtk_nav"
            / "rtk_nav"
            / "config"
            / "003-E12-E14.yaml"
        )
        payload = convert_legacy_yaml_to_map(yaml_path)
        e14_payload = next(
            region for region in payload["regions"] if region["id"] == "E14"
        )
        raw_boundary = e14_payload["polygons"][0]["boundary"]
        raw_b = raw_boundary[2]
        raw_c = raw_boundary[3]
        self.assertGreater(abs(raw_c[1] - raw_b[1]), 1e-9)

        model = load_map(payload)
        e14 = next(region for region in model.regions if region.id == "E14")
        _, original_geometry = _rotated_region_geometry(
            model, e14, 0.0, apply_edge_distance=False
        )
        _, adjusted_geometry = _rotated_region_geometry(model, e14, 0.0)
        original = original_geometry[0][0]
        adjusted = adjusted_geometry[0][0]

        original_top = (
            original[3][0] - original[2][0],
            original[3][1] - original[2][1],
        )
        adjusted_top = (
            adjusted[3][0] - adjusted[2][0],
            adjusted[3][1] - adjusted[2][1],
        )
        self.assertGreater(abs(adjusted_top[1]), 1e-6)
        self.assertTrue(
            math.isclose(
                original_top[0] * adjusted_top[1]
                - original_top[1] * adjusted_top[0],
                0.0,
                abs_tol=1e-6,
            )
        )

        old_rectangular_d = (
            min(point[0] for point in original[:-1]) + 0.5,
            min(point[1] for point in original[:-1]) + 0.3,
        )
        self.assertGreater(
            math.hypot(
                adjusted[0][0] - old_rectangular_d[0],
                adjusted[0][1] - old_rectangular_d[1],
            ),
            0.05,
        )

    def test_route_serializers_preserve_unbound_connector_ids(self):
        payload = {
            "format": "rtk_auto_map_v2",
            "regions": [
                {
                    "id": "only",
                    "boundary": [
                        [110.0, 35.0],
                        [110.001, 35.0],
                        [110.001, 35.001],
                        [110.0, 35.001],
                    ],
                }
            ],
            "connectors": [
                {"id": "pre", "path": [[110.0, 35.0], [109.999, 35.0]]},
                {"id": "post", "path": [[110.001, 35.001], [110.002, 35.001]]},
            ],
            "order": ["pre", "only", "post"],
        }
        route = plan_route(load_map(payload), sweep_spacing=1.0, edge_clearance=0.1)
        document = json.loads(route_to_json(route))
        self.assertEqual(
            [
                segment.get("connector_id")
                for segment in document["segments"]
                if segment.get("connector_id") in {"pre", "post"}
            ],
            ["pre", "post"],
        )
        self.assertNotIn("travel_segments", document["metrics"])

    def test_v2_inherits_legacy_defaults_for_unconfigured_polygon_edges(self):
        payload = {
            "format": "rtk_auto_map_v2",
            "guides": [[_metric_point(0, 0), _metric_point(10, 0)]],
            "regions": [
                {
                    "id": "defaulted",
                    "polygons": [
                        {"boundary": _ring([(0, 0), (10, 0), (10, 4), (0, 4)])}
                    ],
                }
            ],
            "order": ["defaulted"],
        }

        model = load_map(payload)

        self.assertEqual(model.defaults.interval, 1.0)
        self.assertEqual(model.defaults.start_corner, "top_left")
        self.assertFalse(model.defaults.swap_wh_select)
        self.assertEqual(model.defaults.edge_distance_lon, (0.1, 0.1))
        self.assertEqual(model.defaults.edge_distance_lat, (0.1, 0.1))
        self.assertEqual(model.regions[0].polygons[0].edge_distance_lon, (0.1, 0.1))
        self.assertEqual(model.regions[0].polygons[0].edge_distance_lat, (0.1, 0.1))

    def test_v2_defaults_can_override_legacy_values(self):
        payload = {
            "format": "rtk_auto_map_v2",
            "defaults": {
                "interval": 1.5,
                "start_corner": "bottom_right",
                "swap_wh_select": True,
                "edge_distance_lon": 0.2,
                "edge_distance_lat": [0.3, 0.4],
            },
            "guides": [[_metric_point(0, 0), _metric_point(10, 0)]],
            "regions": [
                {
                    "id": "defaulted",
                    "boundary": _ring([(0, 0), (10, 0), (10, 4), (0, 4)]),
                }
            ],
            "order": ["defaulted"],
        }

        model = load_map(payload)

        self.assertEqual(model.defaults.interval, 1.5)
        self.assertEqual(model.defaults.start_corner, "bottom_right")
        self.assertTrue(model.defaults.swap_wh_select)
        self.assertEqual(model.regions[0].polygons[0].edge_distance_lon, (0.2, 0.2))
        self.assertEqual(model.regions[0].polygons[0].edge_distance_lat, (0.3, 0.4))

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

    def test_v2_region_guide_accepts_horizontal_and_vertical_only(self):
        payload = {
            "format": "rtk_auto_map_v2",
            "guides": [[_metric_point(0, 0), _metric_point(10, 0)]],
            "regions": [
                {
                    "id": "guided",
                    "guide": "vertical",
                    "boundary": _ring([(0, 0), (2, 0), (2, 10), (0, 10)]),
                }
            ],
            "order": ["guided"],
        }

        model = load_map(payload)
        self.assertEqual(model.regions[0].guide, "vertical")

        payload["regions"][0]["guide"] = "diagonal"
        with self.assertRaises(PlanningError):
            load_map(payload)

    def test_region_without_guide_uses_own_longest_edge_and_override_is_cardinal(self):
        base_payload = {
            "format": "rtk_auto_map_v2",
            "guides": [[_metric_point(0, 0), _metric_point(10, 0)]],
            "regions": [
                {
                    "id": "tall",
                    "boundary": _ring([(0, 0), (2, 0), (2, 10), (0, 10)]),
                }
            ],
            "order": ["tall"],
        }

        fallback_route = plan_route(
            load_map(base_payload),
            sweep_spacing=2.0,
            edge_clearance=0.2,
        )
        fallback_coverage = [
            segment for segment in fallback_route.segments if segment.kind == "coverage"
        ]
        self.assertTrue(fallback_coverage)
        for segment in fallback_coverage:
            first_x, first_y = _metric_xy(segment.points[0])
            last_x, last_y = _metric_xy(segment.points[-1])
            self.assertGreater(abs(last_y - first_y), abs(last_x - first_x))

        base_payload["regions"][0]["guide"] = "horizontal"
        override_route = plan_route(
            load_map(base_payload),
            sweep_spacing=1.0,
            edge_clearance=0.2,
        )
        override_coverage = [
            segment for segment in override_route.segments if segment.kind == "coverage"
        ]
        self.assertTrue(override_coverage)
        for segment in override_coverage:
            first_x, first_y = _metric_xy(segment.points[0])
            last_x, last_y = _metric_xy(segment.points[-1])
            self.assertGreater(abs(last_x - first_x), abs(last_y - first_y))

        override_geojson = json.loads(
            route_to_geojson(override_route, load_map(base_payload))
        )
        boundary = next(
            feature
            for feature in override_geojson["features"]
            if feature["properties"].get("kind") == "boundary"
        )
        self.assertEqual(boundary["properties"]["guide"], "horizontal")

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

    def test_bridge_attachments_prefer_one_safe_diagonal_segment(self):
        payload = _bridge_entry_map()
        payload["connectors"][0]["path"] = [
            _metric_point(8.0, 10.7),
            _metric_point(8.0, 9.3),
        ]
        route = plan_route(
            load_map(payload),
            sweep_spacing=2.0,
            edge_clearance=0.2,
            max_connector=20.0,
        )
        bridge_index = next(
            index
            for index, segment in enumerate(route.segments)
            if segment.connector_id == "bridge_9-10B"
        )

        bridge = route.segments[bridge_index]
        self.assertEqual(route.segments[bridge_index - 1].kind, "coverage")
        self.assertEqual(route.segments[bridge_index + 1].kind, "coverage")
        for first, second in (
            (bridge.points[0], bridge.points[1]),
            (bridge.points[-2], bridge.points[-1]),
        ):
            first_x, first_y = _metric_xy(first)
            second_x, second_y = _metric_xy(second)
            self.assertGreater(abs(second_x - first_x), 1e-6)
            self.assertGreater(abs(second_y - first_y), 1e-6)

    def test_connector_endpoints_inherit_boundary_defaults(self):
        model = load_map(_connector_boundary_offset_map())
        route = plan_route(
            model,
            sweep_spacing=2.0,
            edge_clearance=0.2,
            max_connector=20.0,
        )
        bridge = next(
            segment
            for segment in route.segments
            if segment.connector_id == "bridge-source-destination"
        )

        self.assertEqual(len(bridge.points), 5)
        first_x, first_y = _metric_xy(bridge.points[1])
        middle_x, middle_y = _metric_xy(bridge.points[2])
        last_x, last_y = _metric_xy(bridge.points[3])
        middle_ratio = 1.0 / (1.0 + math.sqrt(2.0))
        self.assertTrue(math.isclose(first_x, 9.9, abs_tol=1e-6))
        self.assertTrue(math.isclose(first_y, 3.9, abs_tol=1e-6))
        self.assertTrue(
            math.isclose(
                middle_x,
                10.0 - 0.1 * (1.0 - middle_ratio),
                abs_tol=1e-6,
            )
        )
        self.assertTrue(
            math.isclose(
                middle_y,
                5.0 - 0.1 * (1.0 - middle_ratio) + 0.1 * middle_ratio,
                abs_tol=1e-6,
            )
        )
        self.assertTrue(math.isclose(last_x, 9.0, abs_tol=1e-6))
        self.assertTrue(math.isclose(last_y, 6.1, abs_tol=1e-6))
        self.assertNotEqual(bridge.points[1], tuple(_metric_point(10, 5)))

    def test_connector_endpoint_uses_explicit_polygon_edges(self):
        payload = _connector_boundary_offset_map()
        payload["regions"][0]["polygons"][0].update(
            {
                "edge_distance_lon": [1.0, 2.0],
                "edge_distance_lat": [0.5, 1.5],
            }
        )
        payload["connectors"][0]["path"] = [
            _metric_point(10, 4),
            _metric_point(10, 5),
            _metric_point(10, 6),
        ]
        model = load_map(payload)
        route = plan_route(
            model,
            sweep_spacing=2.0,
            edge_clearance=0.2,
            max_connector=20.0,
        )
        bridge = next(
            segment
            for segment in route.segments
            if segment.connector_id == "bridge-source-destination"
        )

        first_x, first_y = _metric_xy(bridge.points[1])
        middle_x, middle_y = _metric_xy(bridge.points[2])
        last_x, last_y = _metric_xy(bridge.points[3])
        self.assertTrue(math.isclose(first_x, 9.0, abs_tol=1e-6))
        self.assertTrue(math.isclose(first_y, 2.5, abs_tol=1e-6))
        self.assertTrue(math.isclose(middle_x, 9.45, abs_tol=1e-6))
        self.assertTrue(math.isclose(middle_y, 4.3, abs_tol=1e-6))
        self.assertTrue(math.isclose(last_x, 9.9, abs_tol=1e-6))
        self.assertTrue(math.isclose(last_y, 6.1, abs_tol=1e-6))

    def test_connector_edges_override_region_edges(self):
        payload = _connector_boundary_offset_map()
        for region in payload["regions"]:
            region["polygons"][0].update(
                {
                    "edge_distance_lon": [1.0, 1.0],
                    "edge_distance_lat": [1.0, 1.0],
                }
            )
        payload["connectors"][0].update(
            {
                "edge_distance_lon": [0.0, 0.0],
                "edge_distance_lat": [0.5, -0.5],
            }
        )

        model = load_map(payload)
        connector = model.connectors[0]
        self.assertEqual(connector.edge_distance_lon, (0.0, 0.0))
        self.assertEqual(connector.edge_distance_lat, (0.5, -0.5))
        route = plan_route(
            model,
            sweep_spacing=2.0,
            edge_clearance=0.2,
            max_connector=20.0,
        )
        bridge = next(
            segment
            for segment in route.segments
            if segment.connector_id == "bridge-source-destination"
        )

        # The raw-boundary attachment and offset bridge points share one
        # ordered segment; the explicit bridge starts at points[2].
        first_x, first_y = _metric_xy(bridge.points[2])
        shifted_x, shifted_y = _metric_xy(bridge.points[3])
        last_x, last_y = _metric_xy(bridge.points[4])
        self.assertTrue(math.isclose(first_x, 10.0, abs_tol=1e-6))
        self.assertTrue(math.isclose(first_y, 4.5, abs_tol=1e-6))
        self.assertTrue(math.isclose(shifted_x, 9.8535534, abs_tol=1e-6))
        self.assertTrue(math.isclose(shifted_y, 5.4393398, abs_tol=1e-6))
        self.assertTrue(math.isclose(last_x, 8.6464466, abs_tol=1e-6))
        self.assertTrue(math.isclose(last_y, 6.3535534, abs_tol=1e-6))

    def test_ordered_connector_merge_joins_region_tails_and_keeps_coverage(self):
        route = plan_route(
            load_map(_connector_boundary_offset_map()),
            sweep_spacing=2.0,
            edge_clearance=0.2,
            max_connector=20.0,
        )

        bridge_indexes = [
            index
            for index, segment in enumerate(route.segments)
            if segment.connector_id == "bridge-source-destination"
        ]
        self.assertEqual(len(bridge_indexes), 1)
        bridge_index = bridge_indexes[0]
        bridge = route.segments[bridge_index]
        self.assertEqual(bridge.kind, "connector")
        self.assertEqual(bridge.from_region, "source")
        self.assertEqual(bridge.to_region, "destination")
        self.assertIsNone(bridge.region_id)

        before = route.segments[bridge_index - 1]
        after = route.segments[bridge_index + 1]
        self.assertEqual(before.kind, "coverage")
        self.assertEqual(after.kind, "coverage")
        self.assertEqual(bridge.points[0], before.points[-1])
        self.assertEqual(bridge.points[-1], after.points[0])
        self.assertEqual(len(bridge.points), 5)
        expected_length = sum(
            math.hypot(
                _metric_xy(first)[0] - _metric_xy(second)[0],
                _metric_xy(first)[1] - _metric_xy(second)[1],
            )
            for first, second in zip(bridge.points, bridge.points[1:])
        )
        self.assertAlmostEqual(
            bridge.length_m,
            expected_length,
            delta=1e-6,
        )

        internal_connectors = [
            segment
            for segment in route.segments
            if segment.kind == "connector" and segment.connector_id is None
        ]
        self.assertTrue(internal_connectors)
        self.assertTrue(
            any(segment.region_id == "source" for segment in internal_connectors)
        )
        self.assertTrue(
            any(segment.region_id == "destination" for segment in internal_connectors)
        )

    def test_unbound_connector_keeps_raw_travel_path(self):
        base = _rectangle_map(width_m=4.0, height_m=2.0)
        payload = {
            "format": "rtk_auto_map_v2",
            "guides": base["guides"],
            "regions": [{"id": "only", "boundary": base["boundary"]}],
            "connectors": [
                {
                    "id": "access",
                    "path": [_metric_point(-1, 0), _metric_point(0, 0)],
                }
            ],
            "order": ["access", "only"],
        }

        route = plan_route(load_map(payload), sweep_spacing=1.0, edge_clearance=0.1)
        access = next(segment for segment in route.segments if segment.connector_id == "access")
        self.assertEqual(
            access.points,
            tuple(tuple(point) for point in payload["connectors"][0]["path"]),
        )

    def test_unbound_entry_and_exit_use_nearest_region_corners(self):
        payload = {
            "format": "rtk_auto_map_v2",
            "defaults": {
                "edge_distance_lon": 0.0,
                "edge_distance_lat": 0.0,
                "start_corner": "top_left",
            },
            "guides": [[_metric_point(0, 0), _metric_point(10, 0)]],
            "regions": [
                {
                    "id": "only",
                    "boundary": _ring([(0, 0), (10, 0), (10, 6), (0, 6)]),
                }
            ],
            "connectors": [
                {
                    "id": "access",
                    "path": [_metric_point(11, 5.9), _metric_point(10, 5.9)],
                },
                {
                    "id": "back_only",
                    "path": [_metric_point(0, 0.1), _metric_point(-1, 0.1)],
                },
            ],
            "order": ["access", "only", "back_only"],
        }

        route = plan_route(
            load_map(payload),
            sweep_spacing=2.0,
            edge_clearance=0.2,
            max_connector=20.0,
        )
        coverage = [segment for segment in route.segments if segment.kind == "coverage"]
        self.assertTrue(coverage)

        first_x, first_y = _metric_xy(coverage[0].points[0])
        last_x, last_y = _metric_xy(coverage[-1].points[-1])
        self.assertGreater(first_x, 9.0)
        self.assertGreater(first_y, 3.0)
        self.assertLess(last_x, 1.0)
        self.assertLess(last_y, 2.0)

        access_index = next(
            index for index, segment in enumerate(route.segments)
            if segment.connector_id == "access"
        )
        back_index = next(
            index for index, segment in enumerate(route.segments)
            if segment.connector_id == "back_only"
        )
        self.assertEqual(
            route.segments[access_index].points[-1],
            route.segments[access_index + 1].points[0],
        )
        self.assertEqual(
            route.segments[back_index - 1].points[-1],
            route.segments[back_index].points[0],
        )

    def test_unbound_bridge_corner_selection_ignores_start_corner_default(self):
        payload = {
            "format": "rtk_auto_map_v2",
            "defaults": {
                "edge_distance_lon": 0.0,
                "edge_distance_lat": 0.0,
                "start_corner": "top_left",
            },
            "guides": [[_metric_point(0, 0), _metric_point(10, 0)]],
            "regions": [
                {
                    "id": "only",
                    "boundary": _ring([(0, 0), (10, 0), (10, 8), (0, 8)]),
                }
            ],
            "connectors": [
                {
                    "id": "access",
                    "path": [_metric_point(11, 7.9), _metric_point(10, 7.9)],
                }
            ],
            "order": ["access", "only"],
        }

        top_left_route = plan_route(
            load_map(payload), sweep_spacing=2.0, edge_clearance=0.2, max_connector=20.0
        )
        payload["defaults"]["start_corner"] = "bottom_right"
        bottom_right_route = plan_route(
            load_map(payload), sweep_spacing=2.0, edge_clearance=0.2, max_connector=20.0
        )

        top_left_coverage = [
            segment.points for segment in top_left_route.segments if segment.kind == "coverage"
        ]
        bottom_right_coverage = [
            segment.points
            for segment in bottom_right_route.segments
            if segment.kind == "coverage"
        ]
        self.assertEqual(top_left_coverage, bottom_right_coverage)

    def test_exit_bridge_uses_reverse_end_when_entry_and_exit_share_corner(self):
        payload = {
            "format": "rtk_auto_map_v2",
            "defaults": {
                "edge_distance_lon": 0.0,
                "edge_distance_lat": 0.0,
            },
            "guides": [[_metric_point(0, 0), _metric_point(10, 0)]],
            "regions": [
                {
                    "id": "only",
                    "boundary": _ring([(0, 0), (10, 0), (10, 6), (0, 6)]),
                }
            ],
            "connectors": [
                {
                    "id": "access",
                    "path": [_metric_point(11, 5.9), _metric_point(10, 5.9)],
                },
                {
                    "id": "back",
                    "path": [_metric_point(11, 5.9), _metric_point(10, 5.9)],
                },
            ],
            "order": ["access", "only", "back"],
        }

        route = plan_route(
            load_map(payload),
            sweep_spacing=2.0,
            edge_clearance=0.2,
            max_connector=20.0,
        )
        coverage = [segment for segment in route.segments if segment.kind == "coverage"]
        back_index = next(
            index
            for index, segment in enumerate(route.segments)
            if segment.connector_id == "back"
        )
        reverse_end = route.segments[back_index - 2]
        exit_attachment = route.segments[back_index - 1]

        self.assertEqual(reverse_end.kind, "connector")
        self.assertEqual(reverse_end.region_id, "only")
        self.assertIsNone(reverse_end.connector_id)
        self.assertEqual(reverse_end.points[0], coverage[-1].points[-1])
        self.assertEqual(reverse_end.points[-1], coverage[-1].points[0])

        attachment_start_x, attachment_start_y = _metric_xy(exit_attachment.points[0])
        attachment_end_x, attachment_end_y = _metric_xy(exit_attachment.points[-1])
        self.assertTrue(math.isclose(attachment_start_x, 9.8, abs_tol=0.01))
        self.assertTrue(math.isclose(attachment_end_x, 10.0, abs_tol=0.01))
        self.assertGreater(attachment_end_y, attachment_start_y)

        for first, second in zip(exit_attachment.points, exit_attachment.points[1:]):
            first_x, first_y = _metric_xy(first)
            second_x, second_y = _metric_xy(second)
            self.assertTrue(
                math.isclose(first_x, second_x, abs_tol=1e-6)
                or math.isclose(first_y, second_y, abs_tol=1e-6)
            )

    def test_bound_exit_bridge_merges_reverse_end_before_bridge_path(self):
        payload = {
            "format": "rtk_auto_map_v2",
            "start": _metric_point(10, 5.9),
            "defaults": {
                "edge_distance_lon": 0.0,
                "edge_distance_lat": 0.0,
            },
            "guides": [[_metric_point(0, 0), _metric_point(10, 0)]],
            "regions": [
                {
                    "id": "source",
                    "boundary": _ring([(0, 0), (10, 0), (10, 6), (0, 6)]),
                },
                {
                    "id": "destination",
                    "boundary": _ring([(0, 8), (10, 8), (10, 14), (0, 14)]),
                },
            ],
            "connectors": [
                {
                    "id": "bridge",
                    "from": "source",
                    "to": "destination",
                    "path": [_metric_point(10, 6), _metric_point(10, 8)],
                }
            ],
            "order": ["source", "bridge", "destination"],
        }

        route = plan_route(
            load_map(payload),
            sweep_spacing=2.0,
            edge_clearance=0.2,
            max_connector=20.0,
        )
        source_coverage = [
            segment
            for segment in route.segments
            if segment.kind == "coverage" and segment.region_id == "source"
        ]
        bridge = next(
            segment
            for segment in route.segments
            if segment.connector_id == "bridge"
        )

        self.assertEqual(bridge.points[0], source_coverage[-1].points[-1])
        first_x, first_y = _metric_xy(bridge.points[0])
        reverse_x, reverse_y = _metric_xy(bridge.points[1])
        attached_x, attached_y = _metric_xy(bridge.points[2])
        self.assertGreater(reverse_x, first_x)
        self.assertTrue(math.isclose(reverse_y, first_y, abs_tol=0.01))
        self.assertTrue(math.isclose(attached_y, reverse_y, abs_tol=0.01))
        self.assertTrue(math.isclose(attached_x, 10.0, abs_tol=0.01))
        next_x, next_y = _metric_xy(bridge.points[3])
        self.assertTrue(math.isclose(next_x, attached_x, abs_tol=0.01))
        self.assertGreater(next_y, attached_y)

    def test_unbound_entry_path_is_oriented_by_order_when_recorded_backwards(self):
        payload = {
            "format": "rtk_auto_map_v2",
            "defaults": {
                "edge_distance_lon": 0.0,
                "edge_distance_lat": 0.0,
            },
            "guides": [[_metric_point(0, 0), _metric_point(10, 0)]],
            "regions": [
                {
                    "id": "only",
                    "boundary": _ring([(0, 0), (10, 0), (10, 6), (0, 6)]),
                }
            ],
            "connectors": [
                {
                    "id": "access",
                    "path": [_metric_point(10, 5.9), _metric_point(11, 5.9)],
                }
            ],
            "order": ["access", "only"],
        }

        route = plan_route(
            load_map(payload), sweep_spacing=2.0, edge_clearance=0.2, max_connector=20.0
        )
        access = next(segment for segment in route.segments if segment.connector_id == "access")
        self.assertEqual(access.points[0], tuple(_metric_point(11, 5.9)))
        self.assertEqual(access.points[-1], tuple(_metric_point(10, 5.9)))

    def test_bridge_attachment_falls_back_when_diagonal_crosses_hole(self):
        boundary = (
            (0.0, 0.0),
            (10.0, 0.0),
            (10.0, 10.0),
            (0.0, 10.0),
            (0.0, 0.0),
        )
        hole = (
            (4.0, 4.0),
            (6.0, 4.0),
            (6.0, 6.0),
            (4.0, 6.0),
            (4.0, 4.0),
        )
        geometry = ((boundary, (hole,)),)

        path = _bridge_attachment_connector(
            (2.0, 2.0),
            (8.0, 8.0),
            geometry,
            max_connector=20.0,
        )

        self.assertGreater(len(path), 2)
        for first, second in zip(path, path[1:]):
            self.assertTrue(
                math.isclose(first[0], second[0], abs_tol=1e-9)
                or math.isclose(first[1], second[1], abs_tol=1e-9)
            )

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

        partial = _complex_multi_region_map()
        partial["connectors"][0].pop("to")
        with self.assertRaises(PlanningError):
            load_map(partial)

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
        txt_path = temp_path / f"test_auto_route_{suffix}.txt"
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
                    "--txt-output",
                    str(txt_path),
                ]
            )
            self.assertEqual(result, 0)
            self.assertTrue(output_path.exists())
            self.assertTrue(output_path.with_suffix(".geojson").exists())
            self.assertTrue(txt_path.exists())
            route_document = json.loads(output_path.read_text(encoding="utf-8"))
            geojson_document = json.loads(
                output_path.with_suffix(".geojson").read_text(encoding="utf-8")
            )
            self.assertEqual(route_document["format"], "rtk_auto_route_v1")
            self.assertIn("turn_count", route_document["metrics"])
            self.assertGreaterEqual(route_document["metrics"]["turn_count"], 0)
            self.assertEqual(geojson_document["type"], "FeatureCollection")
            self.assertTrue(all(item["type"] == "Feature" for item in geojson_document["features"]))
            txt_rows = [
                line
                for line in txt_path.read_text(encoding="utf-8").splitlines()
                if line and not line.startswith("#")
            ]
            self.assertEqual(txt_rows[0], "序号,经度,纬度,航向角(度)")
            self.assertTrue(all(len(row.split(",")) == 4 for row in txt_rows[1:]))
        finally:
            for generated_path in (
                input_path,
                output_path,
                output_path.with_suffix(".geojson"),
                txt_path,
            ):
                generated_path.unlink(missing_ok=True)

    def test_cli_rejects_overwriting_input_map(self):
        model_data = _rectangle_map(width_m=4.0, height_m=2.0)
        temp_path = PACKAGE_ROOT.parent.parent / "tmp"
        temp_path.mkdir(parents=True, exist_ok=True)
        suffix = f"{os.getpid()}_same_path"
        input_path = temp_path / f"test_auto_map_{suffix}.json"
        original = json.dumps(model_data)
        try:
            input_path.write_text(original, encoding="utf-8")
            result = main(
                [
                    "--input",
                    str(input_path),
                    "--output",
                    str(input_path),
                ]
            )
            self.assertEqual(result, 2)
            self.assertEqual(input_path.read_text(encoding="utf-8"), original)
        finally:
            input_path.unlink(missing_ok=True)

    def test_load_map_rejects_geojson_route_output(self):
        with self.assertRaises(PlanningError):
            load_map({"type": "FeatureCollection", "features": []})


if __name__ == "__main__":
    unittest.main()
