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

    def test_cli_writes_json_and_geojson(self):
        model_data = _rectangle_map(width_m=4.0, height_m=2.0)
        with tempfile.TemporaryDirectory() as temp_dir:
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
