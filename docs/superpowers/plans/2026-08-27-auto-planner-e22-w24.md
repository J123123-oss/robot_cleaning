# E22-W24 Auto Planner Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Document the complete legacy-YAML-to-map workflow, generate the E22-W24 map and route artifacts, and verify that `--max-connector` can be adjusted for long bridge paths.

**Architecture:** Keep `auto_path_planner.py` as the single converter and planner. Convert `006-E22-W24.yaml` to `rtk_auto_map_v2`, then plan from the generated map into separate JSON, GeoJSON, and legacy TXT outputs. Use an explicit `--max-connector 100.0` for this data set while leaving the planner's default behavior unchanged.

**Tech Stack:** Python 3, PyYAML, JSON, GeoJSON, unittest/pytest, PowerShell.

---

### Task 1: Verify the planner contract and existing changes

**Files:**
- Read: `src/rtk_nav/rtk_nav/auto_path_planner.py`
- Read: `src/rtk_nav/rtk_nav/config/006-E22-W24.yaml`
- Read: `src/rtk_nav/test/test_auto_path_planner.py`

- [x] **Step 1: Check the CLI threshold and YAML conversion entry point**

Run:

```powershell
rg -n "max-connector|map-output|convert_legacy_yaml_to_map|E22|W24|N|S" src/rtk_nav/rtk_nav/auto_path_planner.py src/rtk_nav/test/test_auto_path_planner.py
```

Expected: `--max-connector` accepts a float, YAML input requires `--map-output`, and the four cardinal region prefixes are covered by code/tests.

### Task 2: Complete the operator documentation

**Files:**
- Modify: `src/rtk_nav/README_AUTO_PLANNER.md`

- [x] **Step 1: Document point collection and map authoring**

Add the exact A/B/C manual collection rules, automatic D/closing-point rules, direct JSON input when YAML is unavailable, and the `bridge_*`/`back_*` ordering behavior.

- [x] **Step 2: Document threshold selection and commands**

Explain that `--max-connector` is a per-connector safety limit in metres, that it can be changed per invocation, that it is not a substitute for `connection_tolerance_m`, and include the complete E22-W24 command using `--max-connector 100.0`.

### Task 3: Generate E22-W24 artifacts

**Files:**
- Read: `src/rtk_nav/rtk_nav/config/006-E22-W24.yaml`
- Create: `auto_map_e22_w24.json`
- Create: `auto_route_e22_w24.json`
- Create: `auto_route_e22_w24.geojson`
- Create: `auto_route_e22_w24.txt`

- [x] **Step 1: Convert YAML and plan the route**

Run:

```powershell
python src/rtk_nav/rtk_nav/auto_path_planner.py `
  --input src/rtk_nav/rtk_nav/config/006-E22-W24.yaml `
  --map-output auto_map_e22_w24.json `
  --output auto_route_e22_w24.json `
  --geojson-output auto_route_e22_w24.geojson `
  --txt-output auto_route_e22_w24.txt `
  --sweep-spacing 1.0 `
  --edge-clearance 1.0 `
  --max-connector 100.0 `
  --turn-penalty-m 1.0 `
  --max-connector-penalty 1.0
```

Expected: the command exits with code 0 and writes all four files without using a route output as the map input.

### Task 4: Verify generated data and regression behavior

**Files:**
- Read: `auto_map_e22_w24.json`
- Read: `auto_route_e22_w24.json`
- Read: `auto_route_e22_w24.geojson`
- Read: `auto_route_e22_w24.txt`
- Test: `src/rtk_nav/test/test_auto_path_planner.py`

- [x] **Step 1: Validate JSON structure and region names**

Run a read-only JSON check confirming `format == "rtk_auto_map_v2"`, all expected `E22`, `E23`, `E24`, and `W24` regions exist, and connector paths are present in `order`.

- [x] **Step 2: Run focused planner tests and compile check**

Run:

```powershell
python -m pytest src/rtk_nav/test/test_auto_path_planner.py -q
python -m py_compile src/rtk_nav/rtk_nav/auto_path_planner.py src/rtk_nav/test/test_auto_path_planner.py
```

Expected: the focused planner tests and compile command exit with code 0. The full planner file currently has 49 passing tests and 4 existing E12/E14 fixture failures caused by user-modified legacy data; those failures are reported separately.

- [x] **Step 3: Review the scoped diff**

Run:

```powershell
git diff -- src/rtk_nav/rtk_nav/auto_path_planner.py src/rtk_nav/test/test_auto_path_planner.py src/rtk_nav/README_AUTO_PLANNER.md auto_map_e22_w24.json auto_route_e22_w24.json auto_route_e22_w24.geojson auto_route_e22_w24.txt
```

Expected: only the requested planner documentation, generated E22-W24 artifacts, and related implementation/test changes are included in the scoped review.
