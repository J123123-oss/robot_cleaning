# RTK Auto 扫描式路径规划 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an offline `auto` planner that turns one boundary RTK trace plus a few guide traces into a validated serpentine cleaning route without per-rectangle calibration or manual bridge areas.

**Architecture:** Keep `full_path_planner_dense.py` and the current YAML/manual flow unchanged. Add a dependency-light `auto_path_planner.py` with pure geometry helpers, a JSON input contract, scanline generation, safe connector validation, JSON/GeoJSON writers, and a console entry point named `full_path_planner_auto`.

**Tech Stack:** Python standard library, JSON, dataclasses, local equirectangular meter projection, ROS2-independent CLI, pytest-compatible contract tests.

---

### Task 1: Add failing auto-planner contract tests

**Files:**
- Create: `src/rtk_nav/test/test_auto_path_planner.py`
- Test: `src/rtk_nav/test/test_auto_path_planner.py`

- [x] **Step 1: Add input and geometry test cases.**

Create a `unittest.TestCase` test class for these exact behaviors (the repository's ROS2 CI can also collect these tests through pytest):

```python
def test_load_map_closes_boundary_and_requires_guide(self):
    model = load_map({"boundary": [[0, 0], [10, 0], [10, 10]], "guides": [[[1, 1], [1, 9]]]})
    assert model.boundary[0] == model.boundary[-1]
    with self.assertRaises(PlanningError):
        load_map({"boundary": [[0, 0], [10, 0], [10, 10]], "guides": []})


def test_opposite_guide_directions_produce_same_axis(self):
    forward = estimate_axis_angle([[(0, 0), (0, 10)]])
    reverse = estimate_axis_angle([[(0, 10), (0, 0)]])
    assert math.isclose(abs(forward - reverse), 0.0, abs_tol=1e-9)


def test_rectangle_generates_non_overlapping_parallel_coverage_segments(self):
    model = load_map({
        "boundary": [[0, 0], [10, 0], [10, 5], [0, 5]],
        "guides": [[[0, 1], [10, 1]]],
    })
    route = plan_route(model, sweep_spacing=1.0, edge_clearance=0.2)
    coverage = [segment for segment in route.segments if segment.kind == "coverage"]
    assert len(coverage) == 5
    assert all(segment.length_m > 9.0 for segment in coverage)


def test_no_go_splits_scanline_and_rejects_unsafe_connector(self):
    model = load_map({
        "boundary": [[0, 0], [10, 0], [10, 5], [0, 5]],
        "guides": [[[0, 1], [10, 1]]],
        "no_go": [[[4, 1], [6, 1], [6, 4], [4, 4]]],
    })
    intervals = extract_scanline_intervals(model, axis_angle=0.0, sweep_value=2.0, edge_clearance=0.2)
    assert len(intervals) == 2
    with self.assertRaises(PlanningError):
        plan_route(model, sweep_spacing=1.0, edge_clearance=0.2, max_connector=0.5)
```

Use the module-level names `load_map`, `estimate_axis_angle`, `plan_route`, and `PlanningError`; the implementation will define these exact interfaces. Add a small `if __name__ == "__main__"` runner only if it helps execute the tests in environments without pytest.

- [x] **Step 2: Run the new test file and confirm RED.**

Run:

```powershell
python -m unittest src/rtk_nav/test/test_auto_path_planner.py -v
```

Expected: test import fails because `rtk_nav.auto_path_planner` does not exist yet.

### Task 2: Implement the pure auto planner

**Files:**
- Create: `src/rtk_nav/rtk_nav/auto_path_planner.py`

- [x] **Step 1: Define the public data model and input validation.**

Implement `PlanningError`, `Point`, `Segment`, `AutoMap`, `Route`, `load_map`, and `validate_ring`. `load_map` must parse `[lon, lat]`, require at least three boundary points and one non-degenerate guide, close open rings, and preserve optional `no_go` rings and `start`.

- [x] **Step 2: Implement local meter projection and axis fitting.**

Use the first boundary point as origin and the formulas:

```python
x = (lon - origin_lon) * 111320.0 * cos(origin_lat_rad)
y = (lat - origin_lat) * 110540.0
```

Fit the undirected guide axis using the doubled-angle average:

```python
theta = 0.5 * atan2(sum(sin(2 * angle)), sum(cos(2 * angle)))
```

Expose `estimate_axis_angle(guides)` in radians. Rotate projected points into `(u, v)` coordinates where `u` follows the cleaning direction and `v` is the sweep normal.

- [x] **Step 3: Implement scanline interval extraction.**

Implement a standard even-odd edge intersection routine. For each scanline `v`, collect all X intersections from the outer ring and `no_go` rings, sort them, pair adjacent intersections into allowed intervals, discard intervals shorter than `2 * edge_clearance`, and trim both endpoints by `edge_clearance`. Generate scanlines from the usable min/max V range at `sweep_spacing`.

- [x] **Step 4: Implement serpentine ordering and safe connectors.**

Represent every coverage interval as a two-point segment. Alternate interval direction on consecutive scanlines. For each transition, try a direct segment and then the two L-shaped candidates. Validate candidate lines by sampling at a step no larger than `0.25m`; every sample must be inside the boundary and outside all `no_go` rings. Raise `PlanningError` when no candidate is safe or the chosen connector exceeds `max_connector`.

- [x] **Step 5: Implement route metrics and serializers.**

Serialize a route to `rtk_auto_route_v1` JSON with ordered `coverage` and `connector` segments, axis angle, and total/max connector lengths. Add GeoJSON `FeatureCollection` serialization with one `LineString` feature per segment and `kind`/`index` properties.

- [x] **Step 6: Run the focused tests and confirm GREEN.**

Run:

```powershell
python -m unittest src/rtk_nav/test/test_auto_path_planner.py -v
```

Expected: all focused auto-planner tests pass.

### Task 3: Add the auto console entry point

**Files:**
- Modify: `src/rtk_nav/setup.py:22-35`
- Modify: `src/rtk_nav/rtk_nav/auto_path_planner.py`

- [x] **Step 1: Add the CLI parser.**

Implement `main(argv=None)` with required `--input` and `--output`, plus defaults for `--sweep-spacing`, `--edge-clearance`, and `--max-connector`. Load the map, call `plan_route`, write the requested JSON, and write a sibling `.geojson` file. Catch `PlanningError`, print `auto planner error: ...` to stderr, and return `2`.

- [x] **Step 2: Register the script.**

Add this exact setup entry point:

```python
"full_path_planner_auto = rtk_nav.auto_path_planner:main",
```

- [x] **Step 3: Verify the CLI on a temporary fixture.**

Run:

```powershell
python -m rtk_nav.auto_path_planner --input tmp/auto_map.json --output tmp/auto_route.json --sweep-spacing 1.0 --edge-clearance 0.2
```

Expected: both `tmp/auto_route.json` and `tmp/auto_route.geojson` exist, JSON has `format == "rtk_auto_route_v1"`, and `segments` contains both `coverage` and `connector` entries.

### Task 4: Add package documentation and verify scope

**Files:**
- Create: `src/rtk_nav/README_AUTO_PLANNER.md`
- Verify: `src/rtk_nav/rtk_nav/auto_path_planner.py`
- Verify: `src/rtk_nav/setup.py`
- Verify: `src/rtk_nav/test/test_auto_path_planner.py`

- [x] **Step 1: Document recording and execution.**

Document the JSON format, field meanings, command example, required mapping traces, and the safety rule that failed connectors stop generation.

- [x] **Step 2: Run syntax, focused tests, and whitespace checks.**

Run:

```powershell
python -m py_compile src/rtk_nav/rtk_nav/auto_path_planner.py src/rtk_nav/test/test_auto_path_planner.py src/rtk_nav/setup.py
python -m unittest src/rtk_nav/test/test_auto_path_planner.py -v
git diff --check -- src/rtk_nav/rtk_nav/auto_path_planner.py src/rtk_nav/setup.py src/rtk_nav/test/test_auto_path_planner.py src/rtk_nav/README_AUTO_PLANNER.md
```

Expected: compilation succeeds, focused tests pass, and `git diff --check` has no output.

- [x] **Step 3: Run the existing RTK contract tests that do not require hardware.**

Run:

```powershell
python -m pytest src/rtk_nav/test -q
```

Expected: existing tests remain unchanged; any failure unrelated to the new module must be reported rather than fixed as part of this task.

Observed in this desktop environment: 30 contract tests passed; the three ROS2 quality-plugin modules (`ament_copyright`, `ament_flake8`, and `ament_pep257`) were unavailable during collection.

- [x] **Step 4: Inspect the final diff.**

Run:

```powershell
git diff -- src/rtk_nav/rtk_nav/auto_path_planner.py src/rtk_nav/setup.py src/rtk_nav/test/test_auto_path_planner.py src/rtk_nav/README_AUTO_PLANNER.md
```

Confirm no existing manual planner, navigation node, YAML, or unrelated dirty file was reverted or modified.
