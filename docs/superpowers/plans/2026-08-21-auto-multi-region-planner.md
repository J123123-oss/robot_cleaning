# Auto 多区域复杂轮廓规划 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Extend the dependency-free auto planner to generate ordered cleaning and bridge routes for concave, offset, holed, and multi-polygon regions while preserving the legacy single-boundary JSON format.

**Architecture:** Normalize both input formats into `Region` and `Connector` objects. Generate scanline intervals per polygon and hole, connect segments only inside the owning region with a visibility graph, then assemble the route from an explicit `order` containing region and connector IDs. Serialize segment ownership metadata to JSON and GeoJSON.

**Tech Stack:** Python standard library, `dataclasses`, JSON, local equirectangular meter projection, `unittest`.

---

### Task 1: Add failing contracts for multi-region geometry

**Files:**
- Modify: `src/rtk_nav/test/test_auto_path_planner.py`

- [x] **Step 1: Add reusable local-coordinate fixtures and the v2 map fixture.**

Add helpers that convert metre coordinates around `(110.0, 35.0)` to `[lon, lat]`, build a concave region with a rectangular hole, and build two regions joined by an explicit connector. Keep the existing legacy rectangle helpers unchanged.

- [x] **Step 2: Add tests for normalization and ownership metadata.**

Assert that `load_map` parses `regions`, `connectors`, and `order`, and that `plan_route` emits coverage segments with `region_id`, bridge segments with `connector_id`, and the exact requested sequence.

- [x] **Step 3: Add tests for concave, multi-polygon, and hole scanlines.**

Use a concave L-shaped boundary and a region with two offset rectangles. Assert scanlines return multiple safe intervals and route generation does not add coverage outside the polygons or holes.

- [x] **Step 4: Add tests for invalid order and impossible region connection.**

Assert that duplicate IDs, an order entry that does not exist, a connector with mismatched `from/to`, and a region whose consecutive intervals cannot be safely connected raise `PlanningError`.

- [x] **Step 5: Run the focused tests and confirm RED.**

Run:

```powershell
python -m unittest src/rtk_nav/test/test_auto_path_planner.py -v
```

Expected: the legacy tests pass and the new v2 tests fail because the current `AutoMap` has no `regions/connectors/order` model or multi-region planner.

### Task 2: Normalize input into regions and connectors

**Files:**
- Modify: `src/rtk_nav/rtk_nav/auto_path_planner.py`

- [x] **Step 1: Add data structures.**

Define immutable `Polygon`, `Region`, and `Connector` dataclasses. Extend `AutoMap` with `regions`, `connectors`, and `order`, while keeping legacy `boundary`, `no_go`, and `start` properties available for existing tests and callers.

- [x] **Step 2: Parse both formats.**

Keep the current legacy parser behavior. For `rtk_auto_map_v2`, validate unique region/connector IDs, close all rings, parse `holes`, validate connector paths, require `order` to contain every region exactly once and each connector at most once, and validate connector `from/to` adjacency in the order.

- [x] **Step 3: Run parser tests.**

Run the focused unittest file and verify the new parser contracts pass without changing legacy behavior.

### Task 3: Implement polygon-aware scanline generation

**Files:**
- Modify: `src/rtk_nav/rtk_nav/auto_path_planner.py`

- [x] **Step 1: Generalize rotated geometry.**

Project and rotate every region polygon and hole. Use the first available region point as the frame origin and the global guide axis for all regions.

- [x] **Step 2: Generate intervals from multiple polygons and holes.**

For each scanline, collect outer-ring intersections as allowed spans and hole intersections as removed spans. Merge overlapping allowed spans from multiple polygons, subtract holes, and trim by `edge_clearance`.

- [x] **Step 3: Keep legacy extraction behavior.**

Make `extract_scanline_intervals` continue to operate on a legacy map and add a region-aware internal helper used by v2 planning.

- [x] **Step 4: Run geometry tests.**

Run the focused unittest file and verify concave, hole, and offset multi-polygon cases pass.

### Task 4: Connect coverage safely and assemble explicit bridge order

**Files:**
- Modify: `src/rtk_nav/rtk_nav/auto_path_planner.py`

- [x] **Step 1: Add polygon visibility checks.**

Replace the single-ring connector check with a region predicate that accepts points in any polygon and rejects holes. Build visibility-graph candidates from interval endpoints and polygon vertices; use the shortest safe path, subject to `max_connector`.

- [x] **Step 2: Plan one region at a time.**

Return coverage segments tagged with `region_id`. Do not connect the end of one region directly to another region.

- [x] **Step 3: Materialize connectors from recorded paths.**

Validate the source endpoint is in `from`, the destination endpoint is in `to`, and no connector line segment crosses any region hole. A bridge may lie outside cleaning polygons because it is an explicitly recorded traversable corridor. Preserve its points and emit a `connector` segment tagged with `connector_id`, `from_region`, and `to_region`.

- [x] **Step 4: Assemble according to `order`.**

Require an order of the form `region, connector, region`; verify the connector's `from/to` matches neighboring region IDs and combine all segments into one route with metrics.

- [x] **Step 5: Run connector/order tests.**

Run the focused unittest file and confirm invalid order and impossible connections fail with `PlanningError`.

### Task 5: Update serializers and documentation

**Files:**
- Modify: `src/rtk_nav/rtk_nav/auto_path_planner.py`
- Modify: `src/rtk_nav/README_AUTO_PLANNER.md`

- [x] **Step 1: Add segment metadata to JSON and GeoJSON.**

Preserve `rtk_auto_route_v1`, add ownership fields only when present, and include the ordered sequence in route metrics. Ensure the GeoJSON properties allow coverage and connectors to be styled separately.

- [x] **Step 2: Document v2 input and visualization.**

Explain E9/E10/E11 mapping, how to record outer boundaries and bridges, how concave boundaries and holes are represented, how to run the CLI, and how to open `.geojson` in QGIS/geojson.io or use a plotting script.

- [x] **Step 3: Verify docs and syntax.**

Run:

```powershell
python -m py_compile src/rtk_nav/rtk_nav/auto_path_planner.py src/rtk_nav/test/test_auto_path_planner.py src/rtk_nav/setup.py
git diff --check -- src/rtk_nav/rtk_nav/auto_path_planner.py src/rtk_nav/test/test_auto_path_planner.py src/rtk_nav/README_AUTO_PLANNER.md docs/superpowers/specs/2026-08-21-auto-multi-region-planner-design.md docs/superpowers/plans/2026-08-21-auto-multi-region-planner.md
```

### Task 6: Run end-to-end verification

**Files:**
- Verify: `src/rtk_nav/rtk_nav/auto_path_planner.py`
- Verify: `src/rtk_nav/test/test_auto_path_planner.py`

- [x] **Step 1: Run focused tests.**

```powershell
python -m unittest src/rtk_nav/test/test_auto_path_planner.py -v
```

- [x] **Step 2: Run the full available RTK test collection.**

```powershell
python -m pytest src/rtk_nav/test -q
```

Report unrelated ROS2 plugin or hardware failures without changing those modules. In this environment, 43 tests passed and 3 quality-plugin tests could not import because `ament_copyright`, `ament_flake8`, and `ament_pep257` are unavailable; `pytest` is also unavailable.

- [x] **Step 3: Inspect scoped diff.**

```powershell
git diff -- src/rtk_nav/rtk_nav/auto_path_planner.py src/rtk_nav/test/test_auto_path_planner.py src/rtk_nav/README_AUTO_PLANNER.md docs/superpowers/specs/2026-08-21-auto-multi-region-planner-design.md docs/superpowers/plans/2026-08-21-auto-multi-region-planner.md
```

Confirm no manual planner, navigation node, YAML, or unrelated dirty file was changed.
