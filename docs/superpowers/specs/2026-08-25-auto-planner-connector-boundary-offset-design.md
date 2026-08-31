# Auto Planner Connector Boundary Offset Design

## Goal

Apply the same per-side boundary inset semantics used by
`full_path_planner_dense.py` to `auto_path_planner.py` connector transitions.
The explicit bridge path remains the source of truth for its shape and point
count, while its endpoints are moved into the effective source and destination
cleaning boundaries. The endpoint displacement is smoothly interpolated over
the bridge path so every sampled point uses the same boundary-offset
correction.

## Existing behavior and root cause

`Polygon.edge_distance_lon/lat` is already parsed and applied by
`_edge_adjusted_ring()` when coverage scanlines are generated. Explicit
connector paths, however, are validated and emitted with
`apply_edge_distance=False`; their endpoint attachments therefore terminate
on the surveyed outer boundary. Root-level `defaults` are inherited by
four-corner cleaning polygons but never participate in connector endpoint
selection.

## Decision

1. Keep connector point count and path shape; move every point using a smooth
   displacement interpolation.
2. For each connector source endpoint, find the matching source polygon and
   project the endpoint from the raw rotated boundary into that polygon's
   edge-adjusted rectangle.
3. Apply the same mapping as `_edge_adjusted_ring()`:
   `lon=[right,left]` moves the local x minimum/maximum inward and
   `lat=[bottom,top]` moves the local y minimum/maximum inward.
4. If an endpoint already lies in the adjusted polygon, preserve it. If no
   valid adjusted endpoint can be found, fail with `PlanningError` instead of
   silently falling back to the raw boundary.
5. Interpolate each intermediate point by cumulative raw path length between
   the source and destination endpoint displacements. Use the projected source
   endpoint for the region exit optimization and source attachment. Use the
   projected destination endpoint as the pending entry point for the next
   region.
6. Recompute connector length and enforce `max_connector` after endpoint
   projection. Explicit bridge paths with no effective edge offset retain
   their original coordinates.
7. `interval`, `start_corner`, and `swap_wh_select` remain cleaning scan
   parameters; they do not create a new sweep over an explicit bridge line.

## Data flow

```text
polygon boundary + inherited defaults
        -> rotated raw/adjusted geometry
connector raw first/last point
        -> endpoint projection against matching polygon
        -> adjusted region attachment and bridge segment
        -> route JSON / GeoJSON
```

The map GeoJSON continues to show the surveyed polygon and original bridge
feature. The route features show the actual endpoint-adjusted route that the
robot will follow.

## Validation and compatibility

- Concave polygons and holes continue to use the existing safe connector
  predicates; endpoint projection is only used for four-corner polygons with
  non-zero edge distances.
- Connector middle segments are not re-routed or re-scanned; their sampled
  points receive the interpolated endpoint displacement.
- Existing maps that explicitly set zero edge distances retain their bridge
  coordinates.
- Legacy YAML conversion continues to preserve per-area edge settings; its
  converted connector paths gain the same endpoint projection at planning
  time.

## Tests

- A two-region map with omitted edge settings inherits `(0.1, 0.1)` and moves
  both bridge endpoints inward while also moving intermediate points by the
  interpolated displacement.
- Explicit polygon edge settings override root defaults at both connector
  endpoints.
- A connector endpoint already inside the effective polygon is unchanged.
- The existing diagonal attachment and hole fallback tests remain valid.
- E9/E10/E11 and E12/E13/E14 route fixtures still produce safe routes and
  standard FeatureCollection GeoJSON.
