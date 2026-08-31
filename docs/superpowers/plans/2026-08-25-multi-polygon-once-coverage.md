# Multi-Polygon Once-Only Coverage Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make each polygon in a logical multi-polygon cleaning region fully covered in one contiguous visit before the route moves to the next polygon.

**Architecture:** Generate coverage groups from the existing global scan rows, merging polygons whose original boundaries touch and keeping disconnected components separate. Keep the existing serpentine candidates inside each group, then run a bounded group-level search whose visited mask prevents re-entering a completed disconnected component. Feed the selected flattened order into the existing safe connector route builder.

**Tech Stack:** Python 3, `unittest`, existing local-frame geometry and connector search in `rtk_nav.auto_path_planner`.

---

### Task 1: Add the E13 regression test

**Files:**
- Modify: `src/rtk_nav/test/test_auto_path_planner.py` after `test_e12_e14_map_uses_two_polygon_e13_with_gap_tolerance`

- [ ] **Step 1: Add a test that classifies E13 coverage lines by polygon longitude span.**

```python
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
        for segment in route.segments:
            if segment.kind != "coverage" or segment.region_id != "E13":
                continue
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
        transitions = sum(
            first != second for first, second in zip(labels, labels[1:])
        )
        self.assertEqual(transitions, len(e13.polygons) - 1)
        for polygon_index in range(len(e13.polygons)):
            first = labels.index(polygon_index)
            last = len(labels) - 1 - labels[::-1].index(polygon_index)
            self.assertEqual(labels[first : last + 1], [polygon_index] * (last - first + 1))
```

- [ ] **Step 2: Run the new test before changing the planner.**

Run:

```powershell
python -m unittest src/rtk_nav/test/test_auto_path_planner.py -k multi_polygon_coverage_completes_each_polygon_once -v
```

Expected: FAIL because the current flattened segment optimizer produces more than one transition between E13 polygons.

### Task 2: Generate connected coverage groups

**Files:**
- Modify: `src/rtk_nav/rtk_nav/auto_path_planner.py` near `_region_scanline_intervals` and `_local_region_coverage_segments`

- [ ] **Step 1: Add a helper that returns one polygon's raw scanline intervals using the existing hole subtraction.**

The helper must use the existing `_ring_intervals`, `_subtract_intervals`, and `_merge_intervals` behavior, but receive one `RotatedPolygon` rather than the full region.

- [ ] **Step 2: Add `_local_region_coverage_groups`.**

Use region-wide scan values to preserve narrow polygon behavior. Build connected components by checking whether original polygon boundary vertices lie on another polygon boundary. Generate merged intervals for each component, append its serpentine scan segments to one group, and skip components that have no usable scan row. Return the rotated region and a tuple of non-empty groups.

- [ ] **Step 3: Keep `_local_region_coverage_segments` as a compatibility wrapper.**

Call `_local_region_coverage_groups`, flatten the groups in polygon order, and return the existing `(geometry, list[segment])` shape so current private callers and tests retain their behavior.

### Task 3: Search polygon groups without re-entry

**Files:**
- Modify: `src/rtk_nav/rtk_nav/auto_path_planner.py` near `_optimized_multi_polygon_segments`

- [ ] **Step 1: Add `_optimized_multi_polygon_groups`.**

Build the four `_serpentine_candidates` for each group. Convert `start_point` and `exit_point` into the existing rotated local frame. Cache `_safe_connector_length` by local point pair, using the un-inset routing geometry and `connection_tolerance_m`.

- [ ] **Step 2: Use a visited-group dynamic-programming state.**

Use state `(visited_mask, last_group, candidate_index)` with metric `(longest_connector, total_connector_length)`. Initialize each group candidate from `local_start`; transition only to groups whose mask bit is not set. At completion, include the connector to `local_exit` and compare `(max_longest, total_plus_tail, tail)` so bridge-adjacent endings remain preferred.

- [ ] **Step 3: Add a bounded beam fallback for more than 12 polygon groups.**

Keep the same state data and ranking, but retain at most `max(256, min(2048, group_count * 64))` partial states per depth. The selected state still contains every group exactly once because transitions always set a new mask bit.

- [ ] **Step 4: Return the selected group candidates flattened in route order.**

Raise `PlanningError("no safe connector ordering between multi-polygon coverage groups")` when no complete state can reach the requested entry and exit points.

### Task 4: Use group planning for multi-polygon regions

**Files:**
- Modify: `src/rtk_nav/rtk_nav/auto_path_planner.py` in `plan_route`

- [ ] **Step 1: Replace the multi-polygon call site.**

Call `_local_region_coverage_groups` once. For one polygon, flatten its only group and preserve the existing ordering path. For multiple polygons, pass the groups to `_optimized_multi_polygon_groups`; if all polygons form one touching component, retain the old segment-level exit optimization, otherwise search disconnected groups. Then call `_route_from_local_segments` with `preserve_segment_order=True` and the existing original-geometry routing/attachment behavior.

- [ ] **Step 2: Retain the existing edge-distance and bridge endpoint behavior.**

Do not alter `routing_geometry`, `attachment_geometry`, connector validation, `pending_entry`, or `exit_point` selection. The only changed behavior is the order of coverage segments inside a multi-polygon logical region.

### Task 5: Verify behavior and outputs

**Files:**
- Test: `src/rtk_nav/test/test_auto_path_planner.py`
- Output: `tmp/auto_route_e12_e14.json`, `tmp/auto_route_e12_e14.geojson`

- [ ] **Step 1: Run the focused regression and planner test module.**

```powershell
python -m unittest src/rtk_nav/test/test_auto_path_planner.py -v
```

Expected: all tests pass, including the new contiguous-coverage assertion.

- [ ] **Step 2: Generate the route from the repository map.**

```powershell
python -m rtk_nav.auto_path_planner --input .\auto_map_e12_e14.json --output .\tmp\auto_route_e12_e14.json --sweep-spacing 2.0 --edge-clearance 1.0 --max-connector 50.0
```

Expected: the command reports coverage and connector counts without a planning error and writes the route JSON plus sibling GeoJSON.

- [ ] **Step 3: Validate output structure and E13 transitions.**

Load the JSON and GeoJSON with Python's `json` module, assert the GeoJSON root is `FeatureCollection`, count E13 polygon labels from coverage midpoints, and assert exactly one transition for the two E13 polygons.

- [ ] **Step 4: Review the final diff without staging unrelated work.**

```powershell
git diff -- src/rtk_nav/rtk_nav/auto_path_planner.py src/rtk_nav/test/test_auto_path_planner.py
git status --short
```

Only the planner, its focused tests, and the newly added design/plan documents should be part of this task's change set; existing unrelated worktree changes must remain untouched.
