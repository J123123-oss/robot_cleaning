# Auto Planner Region Guides Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make each cleaning region use its own longest boundary edge by default, with optional `horizontal` or `vertical` overrides, while preserving root-level guides during YAML-to-v2 conversion.

**Architecture:** Add an optional validated `guide` to `Region`. Keep `AutoMap.guides` as coordinate metadata and generate a root guide from the global longest edge when YAML does not provide one. Resolve one axis per region during `plan_route`; connectors continue using the existing common map frame so their measured paths and offsets are unchanged.

**Tech Stack:** Python 3, dataclasses, PyYAML, unittest/pytest, JSON/YAML route fixtures.

---

### Task 1: Add regression tests for guide resolution

**Files:**
- Modify: `src/rtk_nav/test/test_auto_path_planner.py`

- [ ] **Step 1: Add v2 guide parsing and validation tests**

Add tests that construct a v2 map with a tall rectangle and a horizontal root guide, then assert:

```python
def test_region_guide_accepts_horizontal_and_vertical_only(self):
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
```

- [ ] **Step 2: Add per-region axis behavior tests**

Add one test with a tall region and no `guide`; its coverage segments must be predominantly vertical even though the root guide is horizontal. Add a second region with `guide: horizontal`; its coverage segments must be predominantly horizontal. Compare each segment's metric endpoint delta rather than serialized route metadata.

- [ ] **Step 3: Extend the existing E12-E14 YAML conversion test**

Assert that conversion emits `guides`, that the guide has two endpoints, and that the configured cleaning regions expose `guide: horizontal` in the generated v2 regions. Assert that `E13_long_block` and `E13` resolve to one logical E13 guide. Add a source mapping with an explicit `guides` list and assert the converted payload preserves that list.

- [ ] **Step 4: Run the focused tests and verify the new assertions fail before implementation**

Run:

```powershell
python -m pytest .\src\rtk_nav\test\test_auto_path_planner.py -q
```

Expected: the new guide attribute and per-region direction assertions fail because `Region` has no guide field and `plan_route` still uses one global axis.

### Task 2: Implement validated region guides and longest-edge fallback

**Files:**
- Modify: `src/rtk_nav/rtk_nav/auto_path_planner.py:55-75, 470-575, 750-815, 1068-1110, 897-1060`

- [ ] **Step 1: Add the `Region.guide` field and a guide parser**

Keep the new dataclass field after existing positional fields for compatibility:

```python
VALID_GUIDES = {"horizontal", "vertical"}

def _parse_guide(raw: Any, field: str) -> Optional[str]:
    if raw is None:
        return None
    if not isinstance(raw, str) or raw.strip() not in VALID_GUIDES:
        raise PlanningError(f"{field} must be horizontal or vertical")
    return raw.strip()
```

Parse `regions[index].guide` in `_parse_regions` and store it in `Region.guide`.

- [ ] **Step 2: Add metric longest-edge helpers**

Use local metre scaling at the edge midpoint and select the longest edge from closed rings:

```python
def _geo_distance_m(first: Point, second: Point) -> float:
    latitude = math.radians((first[1] + second[1]) / 2.0)
    delta_lon = (second[0] - first[0]) * METERS_PER_DEGREE_LON * math.cos(latitude)
    delta_lat = (second[1] - first[1]) * METERS_PER_DEGREE_LAT
    return math.hypot(delta_lon, delta_lat)

def _longest_edge(rings: Sequence[Sequence[Point]]) -> Tuple[Point, Point]:
    edges = [edge for ring in rings for edge in _ring_edges(ring)]
    if not edges:
        raise PlanningError("regions do not contain a usable boundary edge")
    return max(edges, key=lambda edge: _geo_distance_m(edge[0], edge[1]))
```

Use `_longest_edge` for v2 root-guide fallback and for the per-region axis fallback. Keep legacy maps' explicit root-guide behavior unchanged.

- [ ] **Step 3: Preserve or generate root guides in the YAML converter**

After all `region_polygons` are built, use `_parse_guides(payload["guides"])` when the YAML contains `guides`, serialize the validated coordinates back to nested lists, and otherwise create one guide from `_longest_edge` over all generated region boundaries. Add no direction strings to root `guides`.

- [ ] **Step 4: Carry the guide from YAML areas into logical regions**

For each recognized cleaning area, parse `areas[index].guide`. Store it by logical region id. When multiple YAML areas map to the same region, accept repeated identical values and raise `PlanningError` on conflicting non-null values. Add the resolved value to `regions[].guide` only when present.

- [ ] **Step 5: Run the focused tests**

Run:

```powershell
python -m pytest .\src\rtk_nav\test\test_auto_path_planner.py -q
```

Expected: all new guide parsing, fallback, conversion, and validation tests pass; any unrelated pre-existing assertion is reported separately.

### Task 3: Use each region's guide for coverage planning

**Files:**
- Modify: `src/rtk_nav/rtk_nav/auto_path_planner.py:3325-3690`

- [ ] **Step 1: Add region axis resolution**

Resolve explicit guides without using coordinate direction input:

```python
def _region_axis_angle(
    region: Region,
    default_axis: float,
    use_region_fallback: bool,
) -> float:
    if region.guide == "horizontal":
        return 0.0
    if region.guide == "vertical":
        return math.pi / 2.0
    if not use_region_fallback:
        return default_axis
    longest = _longest_edge(
        polygon.boundary for polygon in region.polygons
    )
    return _geo_axis_angle(longest[0], longest[1])
```

Call it with `use_region_fallback=not map_data.legacy`; legacy maps therefore retain
their root-guide axis, while v2 regions without a guide use their own longest edge.

- [ ] **Step 2: Resolve axes once in `plan_route` and pass the region axis to coverage**

Build `region_axes` after `regions` is normalized. Use `region_axes[region.id]` for `_local_region_coverage_groups`, `_rotated_region_geometry`, `_optimized_multi_polygon_groups`, and `_route_from_local_segments` for the current region. Keep `axis_angle` for route metadata and existing connector processing.

- [ ] **Step 3: Keep connector geometry independent of coverage guide**

Do not rotate connector paths or change `_effective_connector_path` semantics. Connector attachments continue using the common `axis_angle` frame; endpoint distances and hole checks are invariant under the rigid frame rotation. Convert the region route's endpoints back to geographic coordinates before connector handling, as already done by `_route_from_local_segments`.

- [ ] **Step 4: Expose region guide in GeoJSON boundary properties**

When serializing map boundaries in `route_to_geojson`, add `guide` only when `region.guide` is not `None`. This makes generated full-map visualization auditable without changing route JSON compatibility.

- [ ] **Step 5: Run focused planner tests**

Run:

```powershell
python -m pytest .\src\rtk_nav\test\test_auto_path_planner.py -q
```

Expected: the per-region horizontal/vertical and longest-edge coverage tests pass, with existing connector and E14 slanted-edge tests retaining their prior results.

### Task 4: Configure E12-E14 and verify full-map generation

**Files:**
- Modify: `src/rtk_nav/rtk_nav/config/003-E12-E14.yaml:42-90`

- [ ] **Step 1: Set explicit cleaning-region guides**

Add `guide: horizontal` to `E12_start`, `E13_long_block`, `E13`, and `E14`. Leave bridge/back areas without a guide because they are connectors, not coverage regions.

- [ ] **Step 2: Generate a full map and route from YAML**

Run:

```powershell
python .\src\rtk_nav\rtk_nav\auto_path_planner.py `
  --input .\src\rtk_nav\rtk_nav\config\003-E12-E14.yaml `
  --map-output .\tmp\auto_map_e12_e14_full_from_yaml.json `
  --output .\tmp\auto_route_e12_e14_full_from_yaml.json `
  --sweep-spacing 1.0 `
  --edge-clearance 1.0 `
  --max-connector 40.0
```

Expected: the command reports generated coverage and connector segments; the map contains root `guides` plus `regions[].guide`; the route's E12/E13/E14 coverage segments are horizontal while connector geometry remains unchanged.

- [ ] **Step 3: Verify generated map is independently reloadable**

Run:

```powershell
python -c "import json; from pathlib import Path; import sys; sys.path.insert(0, 'src/rtk_nav'); from rtk_nav.auto_path_planner import load_map, plan_route; p=json.loads(Path('tmp/auto_map_e12_e14_full_from_yaml.json').read_text(encoding='utf-8')); m=load_map(p); r=plan_route(m, sweep_spacing=1.0, edge_clearance=1.0, max_connector=40.0); print(len(m.guides), [(x.id, x.guide) for x in m.regions], len(r.segments), r.axis_angle_rad)"
```

Expected: one or more root guides, `[('E12', 'horizontal'), ('E13', 'horizontal'), ('E14', 'horizontal')]`, and a route with coverage plus connector segments.

### Task 5: Full verification and final diff review

**Files:**
- Test: `src/rtk_nav/test/test_auto_path_planner.py`
- Source: `src/rtk_nav/rtk_nav/auto_path_planner.py`
- Config: `src/rtk_nav/rtk_nav/config/003-E12-E14.yaml`

- [ ] **Step 1: Run the complete rtk_nav pytest suite**

Run:

```powershell
python -m pytest .\src\rtk_nav\test -q
```

Expected: no new failures caused by guide support. Record any pre-existing failure separately rather than weakening its assertion.

- [ ] **Step 2: Run syntax and source checks**

Run:

```powershell
python -m py_compile .\src\rtk_nav\rtk_nav\auto_path_planner.py
git diff --check
```

Expected: both commands complete without errors.

- [ ] **Step 3: Review the scoped diff**

Run:

```powershell
git diff -- src/rtk_nav/rtk_nav/auto_path_planner.py src/rtk_nav/rtk_nav/config/003-E12-E14.yaml src/rtk_nav/test/test_auto_path_planner.py
```

Confirm that no route-output JSON or unrelated dirty files are included in the implementation diff.
