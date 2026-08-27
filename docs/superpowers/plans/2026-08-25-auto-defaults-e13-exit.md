# Auto Planner Defaults And E13 Exit Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Match the legacy YAML defaults for unconfigured auto-map polygons and keep large multi-polygon regions, including E13 at 1 m spacing, ending near the next bridge.

**Architecture:** Add a validated `PlannerDefaults` value to `AutoMap`, accepting root `defaults` and legacy `default` objects. Regions and polygons inherit the default edge distances, while explicit values still win. Replace the arbitrary multi-polygon size cutoff with a bounded beam search that uses both the incoming start and outgoing bridge endpoint in its route cost; the exact bitmask optimizer remains for small regions.

**Tech Stack:** Python 3, dataclasses, existing local-coordinate geometry, `unittest`.

---

### Task 1: Add failing regression tests

**Files:** `src/rtk_nav/test/test_auto_path_planner.py`

- [x] Test a v2 map without edge fields and assert built-in legacy defaults are `(0.1, 0.1)` for both longitude and latitude edges.
- [x] Test `auto_map_e12_e14.json` at `sweep_spacing=1.0` and assert the final E13 coverage point is within 8 m of `bridge_13-14B.path[0]`.
- [x] Run the focused tests and record the expected failures: missing `AutoMap.defaults` and E13 ending about 31 m from the bridge.

### Task 2: Implement legacy defaults

**Files:** `src/rtk_nav/rtk_nav/auto_path_planner.py`, `auto_map_e12_e14.json`

- [x] Add immutable `PlannerDefaults` with interval `1.0`, `start_corner="top_left"`, `swap_wh_select=False`, and scalar-compatible edge distances `0.1`.
- [x] Parse root `defaults` or legacy `default`; validate interval, corner, boolean, and finite distance values.
- [x] Pass parsed edge defaults through `_parse_regions`; explicit region or polygon values override them.
- [x] Store defaults in both legacy and v2 `AutoMap` values.
- [x] Let CLI omission of `--sweep-spacing` use `map_data.defaults.interval`, while explicit CLI values continue to win.
- [x] Add the defaults object to `auto_map_e12_e14.json` for visible, reproducible configuration.

### Task 3: Add bounded exit-aware ordering for large regions

**Files:** `src/rtk_nav/rtk_nav/auto_path_planner.py`

- [x] Add a beam-search variant of `_optimized_multi_polygon_segments` for more than 16 coverage segments. Keep safe connector caching and the existing exact optimizer for small regions.
- [x] Keep both start and exit endpoints in the objective, prioritizing the largest connector, then total connector length, then final tail distance.
- [x] Invoke the beam variant for every multi-polygon region instead of falling back to fixed scanline order.
- [x] Preserve orthogonal internal connectors and the existing diagonal bridge attachment behavior.

### Task 4: Document and verify

**Files:** `src/rtk_nav/README_AUTO_PLANNER.md`, tests and generated temporary outputs

- [x] Document root `defaults` inheritance and large-region exit-aware ordering.
- [x] Run all planner unit tests, compile checks, and `git diff --check`.
- [x] Generate E12/E14 at 1 m and 2 m sweep spacing and confirm the E13 final coverage point and connector are near `bridge_13-14B`.
- [ ] Stage only implementation, test, map, and README files if the user asks for a commit; leave logs and generated `tmp/` files untouched.
