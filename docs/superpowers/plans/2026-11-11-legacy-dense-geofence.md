# Legacy Dense Planner Geofence Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Reuse the legacy `full_path_planner_dense.py` geometry to generate cleaning-area and bridge geofences, export them beside the generated TXT route, and enforce the combined allowed region in `rtk_nav.py`.

**Architecture:** The legacy planner remains the single source of truth for adjusted cleaning rectangles and bridge centerlines. It exports a sidecar JSON containing UTM geometries; `rtk_nav.py` loads that file and performs real-time point-in-geometry checks before publishing motor speeds. Each adjusted area boundary and bridge centerline is expanded by `0.20 m`; the final allowed region is the logical union of all expanded geometries.

**Tech Stack:** Python, ROS2, existing UTM helpers in `full_path_planner_dense.py`, JSON sidecar files, pytest.

**Spec:** Existing confirmed design in conversation; legacy planner only, no `auto_path_planner`.

## Global Constraints

- Keep `full_path_planner_dense.py` as the only producer of adjusted area and bridge geometry.
- Reuse `inner_corners_utm` produced after `edge_distance_lon/lat`; do not recalculate offsets in `rtk_nav.py`.
- Use `0.20 m` as the configured expansion radius for both adjusted rectangles and bridge centerlines.
- Keep the existing TXT route format compatible.
- Do not require `shapely`; evaluate expanded rectangles and bridge corridors with project-local geometry helpers.
- A fence stop must override navigation speed output and require explicit reset.

## Review Focus

- Adjusted area corners and route strips must use the same `edge_distance_*` result.
- Bridge corridors must contain the dense bridge points and overlap both neighboring area fences.
- A single RTK outlier must not stop the robot; consecutive out-of-region fixes must.
- RTK loss or stale fixes must stop automatic motion.
- Missing or malformed sidecar data must fail closed for automatic cleaning.

### Task 1: Add reusable legacy geometry helpers

**Files:**
- Modify: `src/rtk_nav/rtk_nav/full_path_planner_dense.py`
- Test: `src/rtk_nav/test/test_legacy_geofence_geometry.py`

**Interfaces:**
- Produce `expand_rotated_rectangle(corners_utm, radius_m) -> list[tuple[float, float]]`.
- Produce `point_in_convex_polygon(point_utm, polygon_utm) -> bool`.
- Produce `point_to_polyline_distance(point_utm, polyline_utm) -> float`.
- Produce `build_polyline_corridor(polyline_utm, radius_m) -> list[tuple[float, float]]` or an equivalent segment-based representation consumed by the planner and runtime.

- [ ] **Step 1: Write failing geometry tests**
  - Assert an adjusted rectangle expands by `0.20 m` on all four sides.
  - Assert points on an expanded rectangle boundary are accepted.
  - Assert a point `0.21 m` outside the bridge centerline is rejected for a `0.20 m` corridor.
  - Assert a point at each bridge endpoint is accepted.
- [ ] **Step 2: Run the focused tests and confirm failure**
  - Run `pytest -q src/rtk_nav/test/test_legacy_geofence_geometry.py`.
- [ ] **Step 3: Implement helpers**
  - Use the existing UTM coordinate system and convex rectangle math.
  - Represent bridge corridors as polylines plus radius if polygon construction would add unnecessary complexity.
- [ ] **Step 4: Run the focused tests and confirm pass**
  - Run the same pytest command.

### Task 2: Export fences from the legacy dense planner

**Files:**
- Modify: `src/rtk_nav/rtk_nav/full_path_planner_dense.py`
- Test: `src/rtk_nav/test/test_legacy_dense_fence_export.py`

**Interfaces:**
- Extend the existing multi-area planning flow to write `<route-stem>.fence.json`.
- Preserve existing `path_latlon`, `path_utm`, `original_corners_utm`, and `inner_corners_utm` behavior.

- [ ] **Step 1: Write failing export tests**
  - Build a two-area fixture and assert the sidecar contains each adjusted `inner_corners_utm` boundary.
  - Assert the bridge centerline equals the dense points generated between the previous area end and next area start.
  - Assert `expansion_radius_m` is `0.20`.
  - Assert bridge metadata contains both source and target area names.
- [ ] **Step 2: Run the focused tests and confirm failure**
  - Run `pytest -q src/rtk_nav/test/test_legacy_dense_fence_export.py`.
- [ ] **Step 3: Implement sidecar export**
  - Reuse the existing `all_inner_corners` list.
  - Capture `bridge_dense_utm` at the same point where bridge TXT points are appended.
  - Write UTM coordinates, zone, area names, bridge names, and expansion radius as JSON.
  - Validate each bridge corridor intersects both adjacent expanded area fences; log an error and fail generation when it does not.
- [ ] **Step 4: Run the focused tests and confirm pass**
  - Run the same pytest command.

### Task 3: Load and enforce the sidecar fence in `rtk_nav.py`

**Files:**
- Modify: `src/rtk_nav/rtk_nav/rtk_nav.py`
- Test: `src/rtk_nav/test/test_rtk_geofence_contract.py`

**Interfaces:**
- Add `load_geofence_sidecar(path: str) -> bool`.
- Add `check_geofence_position(position_utm: tuple[float, float]) -> tuple[bool, str]`.
- Add a lat/lon-to-UTM conversion path using the sidecar UTM zone.
- Add a fence-stop state and explicit reset path without changing normal waypoint state transitions.

- [ ] **Step 1: Write failing runtime tests**
  - Assert a point inside an adjusted area fence is allowed.
  - Assert a point inside a bridge corridor is allowed.
  - Assert a point outside all geometries is rejected.
  - Assert one rejected fix does not stop, while `confirm_frames=3` rejected fixes stop.
  - Assert stale or invalid RTK data stops automatic motion.
  - Assert missing sidecar prevents automatic cleaning from arming.
- [ ] **Step 2: Run the focused tests and confirm failure**
  - Run `pytest -q src/rtk_nav/test/test_rtk_geofence_contract.py`.
- [ ] **Step 3: Implement runtime enforcement**
  - Derive the sidecar path from `rtk_path_file`.
  - Check fences on every valid `/fix` before navigation speed publication.
  - Evaluate all expanded area and bridge geometries as a logical union.
  - Track consecutive out-of-region fixes and clear the counter only after a valid in-region fix.
  - Publish zero motor speed and lock `FENCE_STOP` on confirmation.
  - Keep the fence stop latched until the existing control path performs an explicit reset.
- [ ] **Step 4: Run the focused tests and confirm pass**
  - Run the same pytest command.

### Task 4: Validate integration with existing navigation tests

**Files:**
- Modify: `src/rtk_nav/test/test_waypoint_tracking_recovery.py` only if existing fixtures need the new fence defaults.
- Test: existing `src/rtk_nav/test/` suite.

- [ ] **Step 1: Run focused legacy and navigation tests**
  - Run `pytest -q src/rtk_nav/test/test_legacy_geofence_geometry.py src/rtk_nav/test/test_legacy_dense_fence_export.py src/rtk_nav/test/test_rtk_geofence_contract.py`.
- [ ] **Step 2: Run the complete RTK navigation test suite**
  - Run `pytest -q src/rtk_nav/test`.
- [ ] **Step 3: Review generated sidecar output**
  - Confirm existing TXT output remains unchanged except for the new adjacent `.fence.json`.
  - Confirm bridge points and area boundaries use the same UTM zone and adjusted geometry as the route.

