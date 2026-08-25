# E12-E14 Full Auto Map Design

## Goal

Convert `src/rtk_nav/rtk_nav/config/003-E12-E14.yaml` into
`auto_map_e12_e14_full.json`, then use `auto_path_planner.py` to generate and
validate a complete route containing:

- the six `bridge_*` travel paths before E12;
- coverage of E12, E13, and E14;
- the two bridges between the cleaning regions;
- all nine `back_*` travel paths after E14.

The existing JSON map and route formats must remain compatible with current
tests and callers.

## Geometry

The YAML calibration points describe three consecutive corners of a rectangle
in the order A, B, C. The missing corner is computed with the requested local
approximation:

```text
D.lon = A.lon + C.lon - B.lon
D.lat = A.lat + C.lat - B.lat
```

Because the map covers a small area, direct longitude/latitude arithmetic is
appropriate for this approximation. Polygon boundaries are serialized in the
order `[D, A, B, C, D]`, which matches the existing E12/E13/E14 map fixture.

`bridge_*` and `back_*` entries have `A == C`. Applying the formula to them
would create a collinear, zero-area ring, so they are represented as travel
paths `A -> B`, not as coverage polygons.

The two E13 source rectangles, `E13_long_block` and `E13`, become one logical
region with two polygons. Its existing 3 m connection tolerance is retained.

## Map Model

Keep the `rtk_auto_map_v2` format and add an optional field:

```json
"travel_segments": {
  "before": [
    {"id": "bridge_5B-6B", "path": [[lon, lat], [lon, lat]]}
  ],
  "after": [
    {"id": "back_14B-13A_stop", "path": [[lon, lat], [lon, lat]]}
  ]
}
```

Travel segments are ordered line paths and are emitted as `travel` route
segments. They are not coverage regions and are not limited by the 8 m
coverage connector limit; several source bridge paths are longer than that.
The existing `connectors` field remains responsible for transitions between
logical cleaning regions:

```text
E12 -> bridge_12-13B -> E13 -> bridge_13-14B -> E14
```

`bridge_13-14B` retains the existing map fixture's final entry point on the
E14 boundary. The generated connector path therefore follows its source
calibration points and appends the nearest E14 boundary calibration point when
the recorded bridge endpoint stops just outside that boundary.

The generated full route is ordered as:

```text
before travel -> E12 -> bridge_12-13B -> E13
              -> bridge_13-14B -> E14 -> after travel
```

The placement is based on physical connectivity. The two inter-region bridge
paths are not moved before E12 merely because their names also start with
`bridge_`.

## YAML Conversion

`auto_path_planner.py` will expose a pure conversion helper and a CLI mode that
accepts the legacy YAML area configuration. The converter will:

1. validate the `areas` list and each A/B/C point;
2. classify names into cleaning regions, inter-region bridges, pre-route
   bridges, and post-route backs;
3. compute D for the four cleaning rectangles;
4. preserve the configured edge distances and the existing E13 tolerance;
5. write the v2 map JSON to the requested map output path.

The CLI remains backward compatible for JSON input. A YAML invocation will be
able to write both the map and, when requested, a route output, for example:

```text
python -m rtk_nav.auto_path_planner \
  --input src/rtk_nav/rtk_nav/config/003-E12-E14.yaml \
  --map-output auto_map_e12_e14_full.json \
  --output tmp/auto_route_e12_e14_full.json \
  --sweep-spacing 1.0 --edge-clearance 1.0 --max-connector 50.0
```

## Validation and Errors

- Missing or malformed YAML points fail with a `PlanningError` naming the
  area and field.
- A cleaning rectangle with a degenerate A/B/C geometry fails before output.
- A travel segment with fewer than two distinct points fails before output.
- Existing safe connector, polygon, hole, and edge-clearance validation is
  unchanged.
- YAML conversion writes the map only after validation succeeds; route output
  is written only after the converted map can be loaded and planned.

## Tests

Add focused tests that:

- verify D for a rectangle and the `[D, A, B, C, D]` ring order;
- verify the YAML conversion produces three logical regions and two E13
  polygons;
- verify six pre-route bridges, two inter-region connectors, and nine
  post-route backs are all present in the expected order;
- verify a planned route starts with `travel` segments, includes both
  inter-region connectors, ends with `travel` segments, and reports the new
  travel metric;
- continue running the existing auto planner test suite to prove JSON v2 and
  legacy map behavior remains unchanged.

