# Auto Planner Connector Boundary Offset Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans (recommended) to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make explicit connector endpoints inherit and apply the same four-side boundary offsets used by dense polygon coverage planning.

**Architecture:** Keep `Polygon` as the owner of effective edge distances. Add a pure local-coordinate endpoint projection helper that maps raw connector endpoints to the corresponding adjusted polygon boundary. Precompute adjusted connector paths once per route, then distribute the two endpoint displacement vectors over every connector point by cumulative path length before using the result for exit ordering, attachments, validation, length limits, and route serialization.

**Tech Stack:** Python 3, dataclasses, existing local metre frame and geometry predicates, `unittest`.

---

### Task 1: Add regression tests for endpoint projection

**Files:**
- Modify: `src/rtk_nav/test/test_auto_path_planner.py`

- [ ] **Step 1: Add a two-region fixture with a multi-point bridge.**

Use the existing `_metric_point()` and `_ring()` helpers. The source rectangle is
`(0,0)-(10,4)`, the destination rectangle is `(0,6)-(10,10)`, and the bridge
path is `[(10,4), (10,5), (9,6)]`. Omit edge settings so the built-in defaults
are inherited.

- [ ] **Step 2: Add the failing assertion.**

Plan the fixture with `sweep_spacing=2.0`, `edge_clearance=0.2`, and
`max_connector=20.0`. Assert the explicit bridge route segment has three
points and its middle point has the cumulative-length interpolation of the
source and destination displacement vectors. Assert the endpoint coordinates
differ from the raw bridge endpoints by approximately `0.1 m` on the affected
rectangle edges.

- [ ] **Step 3: Add explicit override coverage.**

Set source polygon `edge_distance_lon=[1.0, 2.0]` and
`edge_distance_lat=[0.5, 1.5]`, leaving the destination unconfigured. Assert
the source connector endpoint uses those values while the destination uses the
root default values.

- [ ] **Step 4: Run the focused tests before implementation.**

Run:

```powershell
$env:PYTHONPATH = "$PWD\src\rtk_nav;$env:PYTHONPATH"
python -m unittest src.rtk_nav.test.test_auto_path_planner.AutoPathPlannerTests.test_connector_endpoints_inherit_boundary_defaults -v
```

Expected result: FAIL because the current route preserves raw connector
endpoints.

### Task 2: Implement pure endpoint projection

**Files:**
- Modify: `src/rtk_nav/rtk_nav/auto_path_planner.py:1098-1130` and the connector planning helpers near `plan_route()`

- [ ] **Step 1: Add `_adjusted_connector_endpoint()`.**

The helper accepts one local point, one raw `RotatedPolygon`, and one adjusted
`RotatedPolygon`. Return the original point when it is already allowed by the
adjusted polygon or when all four edge offsets are zero. For a rectangular
boundary, read raw and adjusted x/y bounds and replace coordinates near a raw
minimum/maximum with the corresponding adjusted minimum/maximum. Preserve
coordinates along the edge. Return the candidate only when
`_point_is_allowed_region()` accepts it; otherwise return `None` so the caller
can raise `PlanningError`.

- [ ] **Step 2: Add `_project_connector_endpoint()`.**

Try each raw/adjusted polygon pair in the same region. Select the pair where
the raw endpoint is allowed and return the first valid projected endpoint.
Raise `PlanningError` with the region and connector endpoint name when no pair
can accept the point.

- [ ] **Step 3: Add `_effective_connector_path()`.**

Convert a connector path to local coordinates, project its first point against
the source region and last point against the destination region, then apply the
two endpoint displacement vectors to every point using cumulative raw path
length. Return both local and geographic coordinates as needed by the caller.

### Task 3: Interpolate and use effective paths throughout route planning

**Files:**
- Modify: `src/rtk_nav/rtk_nav/auto_path_planner.py:2634-2830`

- [ ] **Step 1: Precompute effective connector paths after `axis_angle`.**

For every explicit connector, load raw and adjusted source/destination
geometries, validate each raw endpoint against the raw region geometry, project
both endpoints, and apply the endpoint displacement vectors to all path points
using cumulative raw path length. Store the effective geographic and local
paths by connector id.

- [ ] **Step 2: Use projected source endpoints for region exit ordering.**

When a region is followed by a connector, pass the effective path first point
as `exit_point` to `_route_from_local_segments()`.

- [ ] **Step 3: Use projected endpoints and adjusted geometries for attachments.**

Validate the projected source endpoint in the adjusted source geometry. Attach
the last coverage endpoint to it using the existing diagonal-first helper.
Pass the projected destination endpoint as `pending_entry`, so the next region
starts from its adjusted boundary.

- [ ] **Step 4: Validate all effective points and enforce limits.**

Use the effective path for hole checks, length calculation, output segment
points, and `max_connector`; preserve original coordinates only for points
whose interpolated displacement is zero.

### Task 4: Verify serialization and real maps

**Files:**
- Modify: `src/rtk_nav/test/test_auto_path_planner.py` only if existing endpoint assertions need the new contract.

- [ ] **Step 1: Run all planner tests.**

```powershell
$env:PYTHONPATH = "$PWD\src\rtk_nav;$env:PYTHONPATH"
python -m unittest discover -s src/rtk_nav/test -p 'test_auto_path_planner.py' -v
```

Expected result: all planner tests pass, including the new endpoint tests.

- [ ] **Step 2: Run compile and whitespace checks.**

```powershell
python -m py_compile src/rtk_nav/rtk_nav/auto_path_planner.py src/rtk_nav/test/test_auto_path_planner.py
git diff --check
```

Expected result: both commands exit with code 0.

- [ ] **Step 3: Generate the E12/E14 route and validate GeoJSON.**

```powershell
$env:PYTHONPATH = "$PWD\src\rtk_nav;$env:PYTHONPATH"
python -m rtk_nav.auto_path_planner --input .\auto_map_e12_e14.json --output .\tmp\auto_route_e12_e14.json --sweep-spacing 2.0 --edge-clearance 1.0 --max-connector 50.0
python -c "import json; p=json.load(open('tmp/auto_route_e12_e14.geojson', encoding='utf-8')); assert p['type']=='FeatureCollection'; print(len(p['features']))"
```

Expected result: route generation succeeds and the GeoJSON root is a standard
`FeatureCollection`.

- [ ] **Step 4: Review only task-owned changes.**

Run `git diff --stat -- src/rtk_nav/rtk_nav/auto_path_planner.py src/rtk_nav/test/test_auto_path_planner.py docs/superpowers` and leave unrelated user files untouched.
