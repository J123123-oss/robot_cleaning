# Multi-Polygon Once-Only Coverage Design

## Goal

Ensure a logical cleaning region made from multiple polygons completes each polygon's coverage once, then leaves it permanently, instead of interleaving scan segments between polygons.

## Root Cause

`auto_path_planner.py` currently flattens every polygon's scanline interval into one list. The multi-polygon optimizer chooses the next individual coverage segment by connector cost. This minimizes travel locally, but it has no state that records whether a polygon has already been completed. For E13, this produces an order such as main polygon, `E13_long_block`, main polygon, `E13_long_block`, and so on. The horizontal lines are generally distinct; the repeated visual pattern is caused by re-entering a polygon multiple times.

## Design

### Coverage groups

Generate scanline coverage segments from the existing global scan rows, but partition them into connected polygon components. A component contains polygons whose original boundaries touch; their intervals remain merged so a shared edge is not swept twice. Disconnected polygons become separate groups. Each group includes multiple intervals caused by concave boundaries or holes and keeps the existing serpentine ordering and its four valid reverse/orientation candidates.

Using the region-wide scan rows preserves the existing behavior for narrow polygons that only intersect scan rows established by a larger neighboring polygon. Existing edge clearance and polygon edge-distance handling remain unchanged.

### Group ordering

Replace the segment-level multi-polygon search at the planning call site with a bounded dynamic-programming search over polygon groups. A state contains the visited polygon mask, last group, and selected serpentine candidate. Transition cost is the existing safe connector cost through the un-inset routing geometry and the region's connection tolerance. The objective remains:

1. minimize the longest connector;
2. minimize total connector length;
3. prefer the shortest final connector to the next bridge.

The start point and explicit next-region connector endpoint remain part of the cost. Once a disconnected group is visited, no later state can select it again, which guarantees one contiguous visit per disconnected component. If all polygons form one touching component, the existing segment-level exit optimization is retained for bridge-adjacent endings.

For a single polygon, keep the existing route generation unchanged. For multi-polygon regions, flatten the selected group candidates into one ordered segment list and pass it to the existing route builder with `preserve_segment_order=True`.

### Safety and compatibility

The route builder continues to validate each connector against the original polygon geometry and connection tolerance. This preserves safe movement across small gaps between polygons while retaining holes and no-go geometry. The public JSON and GeoJSON formats do not change.

## Testing

Add a regression test using `auto_map_e12_e14.json` that:

- collects E13 coverage segments;
- classifies each segment by its polygon boundary;
- asserts each polygon's coverage entries are contiguous;
- asserts the number of polygon transitions is exactly one;
- keeps the existing assertion that the E13 final coverage endpoint remains close to `bridge_13-14B`.

Run the complete `src/rtk_nav/test/test_auto_path_planner.py` suite and generate the E12-E14 route to verify the CLI still emits valid JSON and GeoJSON.
