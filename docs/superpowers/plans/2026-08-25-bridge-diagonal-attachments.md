# Bridge Diagonal Attachments Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace safe L-shaped bridge-to-coverage attachment paths with one direct diagonal segment, while retaining the existing orthogonal fallback for unsafe lines.

**Architecture:** Add a focused attachment selector in `auto_path_planner.py`. It will test the direct segment with the existing region safety predicate and `max_connector`, then delegate to `_find_region_connector` if the direct segment is unsafe or too long. Explicit bridge exit attachment will opt into this selector; all coverage-to-coverage connections and initial map-start attachment will keep the current orthogonal behavior.

**Tech Stack:** Python 3, `unittest`, existing local-coordinate geometry and route serializers.

---

### Task 1: Add regression tests for diagonal preference and unsafe fallback

**Files:**
- Modify: `src/rtk_nav/test/test_auto_path_planner.py:18-32,456-466`

- [ ] **Step 1: Import the new private helper used by the focused geometry test**

Extend the second `auto_path_planner` import group:

```python
    _bridge_attachment_connector,
```

- [ ] **Step 2: Add a failing route-level test for both sides of a bridge**

Add this test after `test_bridge_entry_keeps_every_turn_short`:

```python
    def test_bridge_attachments_prefer_one_safe_diagonal_segment(self):
        route = plan_route(
            load_map(_bridge_entry_map()),
            sweep_spacing=2.0,
            edge_clearance=0.2,
            max_connector=20.0,
        )
        bridge_index = next(
            index
            for index, segment in enumerate(route.segments)
            if segment.connector_id == "bridge_9-10B"
        )

        for attachment in (
            route.segments[bridge_index - 1],
            route.segments[bridge_index + 1],
        ):
            self.assertEqual(attachment.kind, "connector")
            self.assertEqual(len(attachment.points), 2)
            first_x, first_y = _metric_xy(attachment.points[0])
            second_x, second_y = _metric_xy(attachment.points[1])
            self.assertGreater(abs(second_x - first_x), 1e-6)
            self.assertGreater(abs(second_y - first_y), 1e-6)
```

- [ ] **Step 3: Add a failing geometry test for the orthogonal fallback**

Add this test directly after the route-level test:

```python
    def test_bridge_attachment_falls_back_when_diagonal_crosses_hole(self):
        boundary = ((0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0), (0.0, 0.0))
        hole = ((4.0, 4.0), (6.0, 4.0), (6.0, 6.0), (4.0, 6.0), (4.0, 4.0))
        geometry = ((boundary, (hole,)),)

        path = _bridge_attachment_connector(
            (2.0, 2.0),
            (8.0, 8.0),
            geometry,
            max_connector=20.0,
        )

        self.assertGreater(len(path), 2)
        for first, second in zip(path, path[1:]):
            self.assertTrue(
                math.isclose(first[0], second[0], abs_tol=1e-9)
                or math.isclose(first[1], second[1], abs_tol=1e-9)
            )
```

- [ ] **Step 4: Run only the new tests and verify the pre-change failure**

Run:

```bash
python -m unittest src/rtk_nav/test/test_auto_path_planner.py -k bridge_attachment -v
```

Expected before implementation: the route-level test fails because the current attachment has more than two points, and the helper test fails with an import error because `_bridge_attachment_connector` does not exist yet.

### Task 2: Implement the focused bridge attachment selector

**Files:**
- Modify: `src/rtk_nav/rtk_nav/auto_path_planner.py:1184-1230`

- [ ] **Step 1: Add the minimal selector after `_safe_connector_length`**

Insert this function before `_path_length`:

```python
def _bridge_attachment_connector(
    start: LocalPoint,
    end: LocalPoint,
    geometry: RotatedRegion,
    max_connector: float,
    connection_tolerance_m: float = 0.0,
) -> Tuple[LocalPoint, ...]:
    """Prefer one safe direct bridge attachment, then use the grid fallback."""
    if _local_distance(start, end) <= EPSILON:
        return (start,)

    direct_length = _local_distance(start, end)
    if direct_length <= max_connector + EPSILON and _line_is_allowed_region(
        start, end, geometry, connection_tolerance_m
    ):
        return (start, end)

    return _find_region_connector(
        start,
        end,
        geometry,
        max_connector,
        connection_tolerance_m,
    )
```

- [ ] **Step 2: Run the two focused tests and verify they pass**

Run:

```bash
python -m unittest src/rtk_nav/test/test_auto_path_planner.py -k bridge_attachment -v
```

Expected: both tests pass. The direct route attachments have exactly two points; the hole case has at least one orthogonal turn.

### Task 3: Use diagonal selection only at explicit bridge attachments

**Files:**
- Modify: `src/rtk_nav/rtk_nav/auto_path_planner.py:1622-1680,1725-1755`

- [ ] **Step 1: Add an opt-in flag to `_route_from_local_segments`**

Extend the signature after `attachment_geometry`:

```python
    prefer_diagonal_attachment: bool = False,
```

Replace only the initial `start_local` attachment call with:

```python
            connector = (
                _bridge_attachment_connector(
                    start_local,
                    first_start,
                    attachment_geometry,
                    max_connector,
                    connection_tolerance_m,
                )
                if prefer_diagonal_attachment
                else _find_region_connector(
                    start_local,
                    first_start,
                    attachment_geometry,
                    max_connector,
                    connection_tolerance_m,
                )
            )
```

Leave the `previous_end` to `start` loop unchanged so internal cleaning transitions remain orthogonal.

- [ ] **Step 2: Pass the flag only when entering a region through an explicit connector**

In `plan_route`, before calling `_route_from_local_segments`, set:

```python
            prefer_diagonal_attachment = pending_entry is not None
```

Pass it as a keyword argument:

```python
                prefer_diagonal_attachment=prefer_diagonal_attachment,
```

The value is captured before `pending_entry = None`, and the first region or a region with only a normal `start` point continues to use the existing orthogonal path.

- [ ] **Step 3: Replace the source-side bridge attachment call**

In the explicit connector branch, replace the `_find_region_connector` call that connects `previous_end_local` to `connector_start_local` with:

```python
            attach = _bridge_attachment_connector(
                previous_end_local,
                connector_start_local,
                source_geometry,
                max_connector,
                previous_region.connection_tolerance_m,
            )
```

Do not change the explicit bridge path validation or the `connector_id` segment construction.

- [ ] **Step 4: Run the focused bridge and existing orthogonality tests**

Run:

```bash
python -m unittest src/rtk_nav/test/test_auto_path_planner.py -k "bridge or hole_connectors" -v
```

Expected: all selected tests pass. The bridge attachments are diagonal where safe, while hole and internal connections remain orthogonal.

### Task 4: Document the output behavior

**Files:**
- Modify: `src/rtk_nav/README_AUTO_PLANNER.md:133-140`

- [ ] **Step 1: Add the bridge attachment rule beside the connection-order explanation**

Add this bullet:

```markdown
- 桥架轨迹与清扫线之间的区域内补接会优先使用一条经过安全校验的斜向直线；如果斜线穿过边界、孔洞或超过 `--max-connector`，自动回退到原有正交连接。清扫线之间的内部连接仍保持正交绕行。
```

- [ ] **Step 2: Check documentation formatting**

Run:

```bash
git diff --check -- src/rtk_nav/README_AUTO_PLANNER.md
```

Expected: no output and exit code 0.

### Task 5: Full verification and real-map smoke test

**Files:**
- Verify: `src/rtk_nav/rtk_nav/auto_path_planner.py`
- Verify: `src/rtk_nav/test/test_auto_path_planner.py`
- Verify: `auto_map_e9_e11.json`, `auto_map_e12_e14.json`

- [ ] **Step 1: Run the complete planner test suite and compile check**

Run:

```bash
python -m unittest discover -s src/rtk_nav/test -p test_auto_path_planner.py -v
python -m py_compile src/rtk_nav/rtk_nav/auto_path_planner.py src/rtk_nav/test/test_auto_path_planner.py
git diff --check -- src/rtk_nav/rtk_nav/auto_path_planner.py src/rtk_nav/test/test_auto_path_planner.py src/rtk_nav/README_AUTO_PLANNER.md
```

Expected: all tests pass, compilation exits 0, and `git diff --check` prints nothing.

- [ ] **Step 2: Generate both real route outputs and inspect bridge attachments**

Run:

```bash
python -m rtk_nav.auto_path_planner --input ./auto_map_e9_e11.json --output ./tmp/auto_route_e9_e11_diagonal.json --sweep-spacing 2.0 --edge-clearance 1.0 --max-connector 50.0
python -m rtk_nav.auto_path_planner --input ./auto_map_e12_e14.json --output ./tmp/auto_route_e12_e14_diagonal.json --sweep-spacing 2.0 --edge-clearance 1.0 --max-connector 400.0
```

Expected: both commands succeed and write the paired JSON/GeoJSON files. The connector immediately before and after each bridge has two points when the direct segment is safe; bridge path endpoints remain identical to the input map.

- [ ] **Step 3: Review the final diff and commit implementation files**

Run:

```bash
git diff --stat
git diff --check
git status --short
```

Stage only the implementation files and commit:

```bash
git add src/rtk_nav/rtk_nav/auto_path_planner.py src/rtk_nav/test/test_auto_path_planner.py src/rtk_nav/README_AUTO_PLANNER.md
git commit -m "feat(rtk_nav): prefer diagonal bridge attachments"
```

The commit must not include existing logs, caches, generated `tmp/` output, or unrelated worktree changes.

