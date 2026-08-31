# Full E12-E14 Auto Map Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Convert the legacy E12-E14 YAML calibration file into a validated v2 auto map and plan one route containing pre-E12 bridges, E12/E13/E14 coverage, inter-region bridges, and post-E14 back paths.

**Architecture:** Extend `auto_path_planner.py` with an optional `TravelSegment` model for ordered non-coverage paths before and after the existing region/connector route. Add a pure legacy-YAML converter that computes rectangle corner D with `D = A + C - B`, groups E13's two polygons, and emits the existing v2 map format with a `travel_segments` extension. Preserve the current JSON CLI and planner behavior for maps without travel segments.

**Tech Stack:** Python 3, dataclasses, JSON, PyYAML, unittest/pytest, existing RTK local-coordinate geometry.

---

## File Map

- Modify: `src/rtk_nav/rtk_nav/auto_path_planner.py`
  - Add travel-segment data and parsing.
  - Add legacy YAML-to-map conversion.
  - Add optional YAML/map-output CLI flow.
  - Include travel paths in route and GeoJSON serialization.
- Modify: `src/rtk_nav/test/test_auto_path_planner.py`
  - Add conversion, corner, ordering, and route-output tests.
- Create: `auto_map_e12_e14_full.json`
  - Generated v2 map source containing three logical regions, two inter-region connectors, six pre-route bridges, and nine post-route backs.
- Create during verification: `tmp/auto_route_e12_e14_full.json` and its `.geojson`
  - Generated route artifacts; do not hand-edit them.

### Task 1: Add Failing Contract Tests

**Files:**
- Modify: `src/rtk_nav/test/test_auto_path_planner.py` imports and `AutoPathPlannerTests`

- [ ] **Step 1: Import the converter symbol and add the full YAML contract test.**

Add `convert_legacy_yaml_to_map` to the existing import list, then add this test:

```python
    def test_003_e12_e14_yaml_converts_rectangles_and_travel_segments(self):
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
        self.assertEqual(
            [region["id"] for region in payload["regions"]],
            ["E12", "E13", "E14"],
        )
        self.assertEqual(len(payload["regions"][1]["polygons"]), 2)
        self.assertEqual(
            [item["id"] for item in payload["travel_segments"]["before"]],
            [
                "bridge_5B-6B",
                "bridge_6A-7bB",
                "bridge_7bA-7A",
                "bridge_8B-9A",
                "bridge_10A-11B",
                "bridge_11A-12B",
            ],
        )
        self.assertEqual(
            [item["id"] for item in payload["travel_segments"]["after"]],
            [
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
            [connector["id"] for connector in payload["connectors"]],
            ["bridge_12-13B", "bridge_13-14B"],
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
        route = plan_route(model, sweep_spacing=1.0, edge_clearance=1.0, max_connector=50.0)
        self.assertEqual(
            [segment.travel_id for segment in route.segments[:6]],
            [item["id"] for item in payload["travel_segments"]["before"]],
        )
        self.assertEqual(
            [segment.travel_id for segment in route.segments[-9:]],
            [item["id"] for item in payload["travel_segments"]["after"]],
        )
        self.assertEqual(
            sum(segment.kind == "travel" for segment in route.segments),
            15,
        )
        self.assertEqual(
            [segment.connector_id for segment in route.segments if segment.connector_id],
            ["bridge_12-13B", "bridge_13-14B"],
        )
```

- [ ] **Step 2: Add parser/serializer assertions for travel IDs.**

Add a focused test beside the existing serializer tests:

```python
    def test_route_serializers_preserve_travel_segment_ids(self):
        payload = {
            "format": "rtk_auto_map_v2",
            "regions": [
                {
                    "id": "only",
                    "boundary": [[110.0, 35.0], [110.001, 35.0], [110.001, 35.001], [110.0, 35.001]],
                }
            ],
            "travel_segments": {
                "before": [{"id": "pre", "path": [[110.0, 35.0], [109.999, 35.0]]}],
                "after": [{"id": "post", "path": [[110.001, 35.001], [110.002, 35.001]]}],
            },
            "order": ["only"],
        }
        route = plan_route(load_map(payload), sweep_spacing=1.0, edge_clearance=0.1)
        document = json.loads(route_to_json(route))
        self.assertEqual(document["metrics"]["travel_segments"], 2)
        self.assertEqual(
            [segment.get("travel_id") for segment in document["segments"] if segment["kind"] == "travel"],
            ["pre", "post"],
        )
```

- [ ] **Step 3: Run the new tests and verify they fail for missing symbols/behavior.**

Run:

```text
python -m pytest src/rtk_nav/test/test_auto_path_planner.py -q
```

Expected: FAIL because `convert_legacy_yaml_to_map` and travel-segment fields do not exist yet. Existing unrelated tests must remain collected.

### Task 2: Implement the Travel Segment Data Model and Map Parsing

**Files:**
- Modify: `src/rtk_nav/rtk_nav/auto_path_planner.py` near `Connector`, `AutoMap`, `Segment`, `load_map`, `_segment_dict`, `route_to_json`, and `route_to_geojson`

- [ ] **Step 1: Add the immutable travel model and optional IDs.**

Add:

```python
@dataclass(frozen=True)
class TravelSegment:
    """An ordered non-coverage path before or after the cleaning route."""

    id: str
    path: Tuple[Point, ...]
```

Add `travel_before` and `travel_after` tuple fields to `AutoMap`, and add
`travel_id: Optional[str] = None` to `Segment`.

- [ ] **Step 2: Add strict travel parsing.**

Implement `_parse_travel_segments(raw, field)` with the same accepted mapping/list style as `_parse_connectors`. Require a non-empty ID, a path list with at least two points, finite points through `_coerce_point`, and distinct endpoints. Reject duplicate IDs within one phase and IDs colliding with regions/connectors in `load_map`.

- [ ] **Step 3: Load optional `travel_segments` without changing existing maps.**

In the v2 branch of `load_map`, parse:

```python
raw_travel = payload.get("travel_segments", {})
if raw_travel is None:
    raw_travel = {}
if not isinstance(raw_travel, Mapping):
    raise PlanningError("travel_segments must be an object")
travel_before = _parse_travel_segments(raw_travel.get("before", []), "travel_segments.before")
travel_after = _parse_travel_segments(raw_travel.get("after", []), "travel_segments.after")
```

Pass both tuples into `AutoMap`. Leave legacy maps and v2 maps without the
field as empty travel sequences.

- [ ] **Step 4: Serialize the travel identity and metric.**

In `_segment_dict`, emit `travel_id` when present. In `route_to_json`, add:

```python
"travel_segments": sum(segment.kind == "travel" for segment in route.segments),
```

In `route_to_geojson`, add map-level `travel` LineString features for both
travel phases and copy `travel_id` onto route features.

- [ ] **Step 5: Run the serializer and existing map tests.**

Run:

```text
python -m pytest src/rtk_nav/test/test_auto_path_planner.py -q
```

Expected: the serializer travel test passes; the YAML conversion test remains
the only expected failure.

### Task 3: Implement YAML-to-v2 Map Conversion

**Files:**
- Modify: `src/rtk_nav/rtk_nav/auto_path_planner.py` after map-loading helpers and near CLI construction

- [ ] **Step 1: Add YAML reading and point helpers.**

Import `yaml` with the package's existing ROS/Python dependency convention. Add
helpers that read a path or mapping, require `areas` to be a non-empty list,
and convert each `calib_point_a/b/c` object to `(lon, lat)` with finite numeric
validation. Raise `PlanningError` with the area name and field on failure.

- [ ] **Step 2: Add the requested fourth-corner calculation.**

Implement:

```python
def _approximate_fourth_corner(a: Point, b: Point, c: Point) -> Point:
    return (a[0] + c[0] - b[0], a[1] + c[1] - b[1])
```

Build a closed boundary as `[D, A, B, C, D]`. Reject a degenerate cleaning
rectangle by passing it through `_close_ring`/`_validate_polygon_geometry`.

- [ ] **Step 3: Classify the exact E12-E14 source areas.**

Use the source names as follows:

```python
cleaning_region_names = {
    "E12_start": "E12",
    "E13_long_block": "E13",
    "E13": "E13",
    "E14": "E14",
}
inter_region_bridge_names = {"bridge_12-13B", "bridge_13-14B"}
```

Put every other `bridge_*` in `travel_segments.before` and every `back_*` in
`travel_segments.after`, preserving YAML order. Any other name must raise
`PlanningError` so a renamed or silently omitted area cannot produce an
incomplete map.

- [ ] **Step 4: Build regions, connectors, and defaults.**

For each cleaning area, emit one polygon object with the source edge-distance
overrides. Group E13's two polygon objects in source order and set
`connection_tolerance_m` to `3.0`. Build connectors with source/destination
IDs `E12`/`E13` and `E13`/`E14`, using the bridge A-to-B path. If the endpoint
is not on the destination region boundary, append the nearest destination
calibration corner; this preserves the existing `bridge_13-14B` three-point
path and avoids an invalid connector endpoint.

Copy the YAML `default` values into the v2 `defaults` object, retaining
`interval`, `start_corner`, `swap_wh_select`, `edge_distance_lon`, and
`edge_distance_lat`. Return a mapping with `format`, `defaults`, `regions`,
`connectors`, `travel_segments`, and the physical region `order`.

- [ ] **Step 5: Run the YAML contract test.**

Run:

```text
python -m pytest src/rtk_nav/test/test_auto_path_planner.py::AutoPathPlannerTests::test_003_e12_e14_yaml_converts_rectangles_and_travel_segments -q
```

Expected: PASS, including six pre-route bridge IDs, two inter-region bridge
connectors, nine post-route back IDs, and the D calculation.

### Task 4: Insert Travel Paths into Planning and CLI

**Files:**
- Modify: `src/rtk_nav/rtk_nav/auto_path_planner.py` in `plan_route`, CLI parser, and `main`

- [ ] **Step 1: Emit pre-route travel segments before the existing order loop.**

At the start of `plan_route`, append:

```python
for travel in map_data.travel_before:
    output.append(
        Segment(
            kind="travel",
            points=travel.path,
            length_m=_path_length(travel.path),
            travel_id=travel.id,
        )
    )
```

Append the analogous `travel_after` segments after the existing region/
connector order loop. Include these lengths in `total_length_m`, but do not
raise `max_connector_length_m` for them.

- [ ] **Step 2: Add YAML map-output CLI options while preserving JSON mode.**

Change `--output` from unconditionally required to optional and add:

```python
parser.add_argument(
    "--map-output",
    default=None,
    help="write a converted v2 map when --input is a legacy YAML file",
)
```

In `main`, detect `.yaml`/`.yml`, call `convert_legacy_yaml_to_map`, require
`--map-output`, write the map JSON, load the converted mapping, and plan a route
only when `--output` is also supplied. For JSON input, require `--output` and
retain the current route plus GeoJSON behavior. Report a clear `PlanningError`
if YAML is used without `--map-output`.

- [ ] **Step 3: Run focused and full planner tests.**

Run:

```text
python -m pytest src/rtk_nav/test/test_auto_path_planner.py -q
```

Expected: all auto planner tests pass, including the new 15-travel-segment
route assertions.

### Task 5: Generate and Verify the Requested Artifacts

**Files:**
- Create: `auto_map_e12_e14_full.json`
- Create: `tmp/auto_route_e12_e14_full.json`
- Create: `tmp/auto_route_e12_e14_full.geojson`

- [ ] **Step 1: Convert the YAML and plan the full route.**

Run:

```text
python -m rtk_nav.auto_path_planner \
  --input src/rtk_nav/rtk_nav/config/003-E12-E14.yaml \
  --map-output auto_map_e12_e14_full.json \
  --output tmp/auto_route_e12_e14_full.json \
  --sweep-spacing 1.0 \
  --edge-clearance 1.0 \
  --max-connector 50.0
```

Expected stdout reports coverage and connector counts; both JSON and GeoJSON
route files are written. The map file must load through `load_map` without
raising `PlanningError`.

- [ ] **Step 2: Verify generated map counts and route boundaries.**

Run:

```text
python -c "import json; from pathlib import Path; p=json.loads(Path('auto_map_e12_e14_full.json').read_text(encoding='utf-8')); r=json.loads(Path('tmp/auto_route_e12_e14_full.json').read_text(encoding='utf-8')); assert [x['id'] for x in p['travel_segments']['before']]==['bridge_5B-6B','bridge_6A-7bB','bridge_7bA-7A','bridge_8B-9A','bridge_10A-11B','bridge_11A-12B']; assert len(p['travel_segments']['after'])==9; assert sum(x['kind']=='travel' for x in r['segments'])==15; print('full E12-E14 artifact checks passed')"
```

Expected: `full E12-E14 artifact checks passed`.

- [ ] **Step 3: Run the package-level focused checks and inspect the diff.**

Run:

```text
python -m pytest src/rtk_nav/test/test_auto_path_planner.py -q
git diff --check
git status --short
```

Expected: all focused tests pass, no whitespace errors, and only the planner,
focused tests, generated map, generated route artifacts, and plan/design files
appear as task-related changes. Preserve all pre-existing unrelated worktree
changes.

