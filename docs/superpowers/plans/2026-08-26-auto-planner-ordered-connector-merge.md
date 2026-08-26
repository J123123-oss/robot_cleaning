# Auto Planner Ordered Connector Merge Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 按 `order` 将跨区域桥架两侧的收尾 connector 与显式 bridge 合并为一条连续 Segment。

**Architecture:** 保留现有规划和安全校验流程，在 `plan_route()` 生成原子 Segment 后增加一个纯后处理函数。它只识别有 `from_region/to_region` 的显式 connector，向两侧吸收同一源区/目标区的连续无 ID connector；coverage 和无绑定 connector 不参与合并。合并后统一重算 route metrics，现有 JSON、GeoJSON、TXT 序列化无需新增格式。

**Tech Stack:** Python 3, dataclasses, unittest/pytest, JSON/YAML fixtures.

---

### Task 1: Add the failing ordered-merge regression test

**Files:**
- Modify: `src/rtk_nav/test/test_auto_path_planner.py`

- [ ] **Step 1: Add a focused test for bridge span merging**

Use `_connector_boundary_offset_map()` and assert that the explicit bridge is the only connector segment in its local source/bridge/destination span, its points equal the concatenation of adjacent connector points without duplicate joins, its `region_id` is absent, and its length equals the sum of the original adjacent lengths.

- [ ] **Step 2: Add a guard for internal cleaning connectors**

Use the same route and assert that coverage remains present on both sides of the bridge and that the merged span stops at the first coverage segment; ordinary coverage-to-coverage connector segments elsewhere remain represented as connectors.

- [ ] **Step 3: Run the focused test before implementation**

Run `python -m pytest src/rtk_nav/test/test_auto_path_planner.py -k ordered_connector_merge -q`.

Expected result: the new assertions fail because the current route still emits separate attachment, bridge, and destination connector segments.

### Task 2: Implement order-aware connector span merging

**Files:**
- Modify: `src/rtk_nav/rtk_nav/auto_path_planner.py:3412-3778`

- [ ] **Step 1: Add `_merge_ordered_inter_region_connector_spans()`**

Implement a helper accepting the generated `Sequence[Segment]`, `order`, and connector mapping. Iterate explicit inter-region connector IDs in reverse `order`, locate their unique explicit route segment, scan backward/forward only across connector segments with no `connector_id` and the matching source/destination `region_id`, concatenate points while dropping equal adjacent endpoints, and replace the span with one connector Segment retaining the explicit connector metadata.

- [ ] **Step 2: Call the helper before route metrics are computed**

Apply it to `output` after the existing planning loop. Recalculate `total_length_m`, `max_connector_length_m`, and `turn_count` from the merged output. Do not rerun safety checks or alter `max_connector` acceptance because every atomic segment has already passed the existing safety checks.

- [ ] **Step 3: Run the focused regression test**

Run `python -m pytest src/rtk_nav/test/test_auto_path_planner.py -k ordered_connector_merge -q`.

Expected result: all ordered-merge tests pass.

### Task 3: Verify full E12-E14 output and repository hygiene

**Files:**
- Regenerate: `tmp/auto_route_e12_e14_full_from_yaml.json`
- Regenerate: `tmp/auto_route_e12_e14_full_from_yaml.geojson`

- [ ] **Step 1: Run planner focused tests and compile check**

Run `python -m pytest src/rtk_nav/test/test_auto_path_planner.py -q` and `python -m py_compile src/rtk_nav/rtk_nav/auto_path_planner.py`.

- [ ] **Step 2: Generate and inspect the full route**

Run `python -m rtk_nav.auto_path_planner --input src/rtk_nav/rtk_nav/config/003-E12-E14.yaml --map-output tmp/auto_map_e12_e14_full_from_yaml.json --output tmp/auto_route_e12_e14_full_from_yaml.json --geojson-output tmp/auto_route_e12_e14_full_from_yaml.geojson --sweep-spacing 1.0 --edge-clearance 1.0 --max-connector 40.0`.

Verify that coverage count is unchanged, `bridge_12-13B` and `bridge_13-14B` each occur once as route segments, each merged segment has continuous consecutive points, and total route length matches the pre-merge result within floating-point tolerance.

- [ ] **Step 3: Run diff checks and commit only task files**

Run `git diff --check` and stage only the planner, its focused tests, the design/plan documents, and the regenerated route artifacts that belong to this change. Commit with `fix(rtk_nav): merge ordered bridge attachments`.
