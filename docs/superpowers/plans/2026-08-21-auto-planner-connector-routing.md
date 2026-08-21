# Auto Planner Connector Routing Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Generate bridge-aware, non-diagonal internal route connectors for multi-region RTK sweep maps.

**Architecture:** Keep coverage generation unchanged, but order its complete serpentine candidates using entry and exit attachment costs.  Build orthogonal local routing graph nodes from allowed coordinate intersections and use the existing Dijkstra search to route safely around holes.

**Tech Stack:** Python 3 standard library, `unittest`, ROS 2 Python package layout.

---

### Task 1: Add regression tests

**Files:**

- Modify: `src/rtk_nav/test/test_auto_path_planner.py`

- [ ] **Step 1: Write the failing bridge-entry test**

```python
def test_bridge_entry_keeps_every_turn_short():
    route = plan_route(load_map(_bridge_entry_map()), sweep_spacing=2.0,
                       edge_clearance=0.2, max_connector=8.0)
    self.assertTrue(route.segments)
```

- [ ] **Step 2: Run it and observe failure from a full-width diagonal turn**

Run: `python -m unittest src.rtk_nav.test.test_auto_path_planner.AutoPathPlannerTests.test_bridge_entry_keeps_every_turn_short`

- [ ] **Step 3: Write the failing hole-routing assertion**

```python
for segment in connectors:
    for first, second in zip(segment.points, segment.points[1:]):
        dx, dy = _metric_xy(first)[0] - _metric_xy(second)[0], _metric_xy(first)[1] - _metric_xy(second)[1]
        self.assertTrue(abs(dx) < 1e-6 or abs(dy) < 1e-6)
```

- [ ] **Step 4: Run it and observe a diagonal visibility-graph edge**

Run: `python -m unittest src.rtk_nav.test.test_auto_path_planner.AutoPathPlannerTests.test_hole_connectors_are_orthogonal`

### Task 2: Make coverage traversal bridge-aware

**Files:**

- Modify: `src/rtk_nav/rtk_nav/auto_path_planner.py`

- [ ] **Step 1: Produce the four complete serpentine order candidates**

```python
def _serpentine_candidates(segments):
    # forward/reverse sweep order, each with left-first or right-first direction
    ...
```

- [ ] **Step 2: Score candidates by entry and outgoing bridge attachment paths**

```python
def _segments_from_start(..., exit_point=None):
    # choose a full candidate; never rotate or flip only its first segment
    ...
```

- [ ] **Step 3: Pass the following explicit connector start as `exit_point`**

```python
next_item = order[index + 1] if index + 1 < len(order) else None
exit_point = connectors[next_item].path[0] if next_item in connectors else None
```

### Task 3: Route internal travel orthogonally

**Files:**

- Modify: `src/rtk_nav/rtk_nav/auto_path_planner.py`

- [ ] **Step 1: Add allowed local coordinate-intersection nodes**

```python
def _orthogonal_connector_nodes(start, end, geometry):
    # retain only permitted combinations of boundary, hole, start, and end coordinates
    ...
```

- [ ] **Step 2: Connect graph nodes only when they share U or V**

```python
if abs(first[0] - second[0]) <= EPSILON or abs(first[1] - second[1]) <= EPSILON:
    if _line_is_allowed_region(first, second, geometry):
        ...
```

### Task 4: Verify

**Files:**

- Test: `src/rtk_nav/test/test_auto_path_planner.py`

- [ ] **Step 1: Run all automatic planner tests**

Run: `python -m unittest src.rtk_nav.test.test_auto_path_planner -v`

- [ ] **Step 2: Compile planner code**

Run: `python -m py_compile src/rtk_nav/rtk_nav/auto_path_planner.py`

- [ ] **Step 3: Inspect the E9/E10/E11 route in GeoJSON**

Run: `python -m rtk_nav.auto_path_planner --input auto_map_e9_e11.json --output tmp/auto_route_e9_e11.json --sweep-spacing 2.0 --edge-clearance 1.0 --max-connector 40.0`
