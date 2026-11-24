# RTK Point Affine Calibration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the viewer's fragile similarity calibration with validated affine calibration for three or more control points and regenerate the 1112 preview.

**Architecture:** Keep the existing generated single-file HTML architecture and coordinate conversion flow. Add a shared affine model shape to the Python generator and the embedded JavaScript, use a two-point model only for provisional placement, and gate trusted rendering on a three-point affine fit with acceptable RMSE.

**Tech Stack:** Python 3, embedded JavaScript, Leaflet CRS.Simple, pytest/unittest-compatible tests, PowerShell validation commands.

**Spec:** `docs/superpowers/specs/2026-11-24-rtk-point-affine-calibration-design.md`

## Global Constraints

- Preserve WGS84, GCJ02, and BD09 input handling.
- Preserve exported control point fields and saved view fields.
- Do not modify RTK point recordings.
- Keep the generated page self-contained with the existing embedded SVG.
- Update `FIX_SUMMARY.md` with behavior and validation results.

## Review Focus

- Two nearly vertical controls must not be reported as a trustworthy calibration.
- Three non-collinear controls must fit the clicked pixels without the old direction reversal.
- Collinear controls must fail clearly without producing a usable model.
- Four or more controls must use residual RMSE to reject inconsistent points.
- Re-generated HTML must contain the new model code and current point dataset.

### Task 1: Add affine model regression tests

**Files:**
- Create: `tools/test_plot_rtk_points.py`
- Test: `tools/test_plot_rtk_points.py`

**Interfaces:**
- Consumes: `fit_affine` and `apply_affine` from `tools.plot_rtk_points`.
- Produces: failing tests that define exact affine fit, degeneracy rejection, and two-point provisional behavior.

- [ ] **Step 1: Write the failing tests**

  Cover exact recovery of the 25/26/27 pixel coordinates, rejection of collinear controls, and the existing similarity function's two-point zero-residual behavior being excluded from trusted calibration by the page policy.

- [ ] **Step 2: Run the focused tests**

  Run: `python -m pytest tools/test_plot_rtk_points.py -q`

  Expected: FAIL because the affine helpers do not exist.

### Task 2: Implement affine calibration in generator and page

**Files:**
- Modify: `tools/plot_rtk_points.py`
- Test: `tools/test_plot_rtk_points.py`

**Interfaces:**
- Consumes: the failing tests from Task 1.
- Produces: `fit_affine(control_points)`, `apply_affine(point, model)`, and embedded JavaScript with matching behavior.

- [ ] **Step 1: Implement the Python affine helpers**

  Fit six affine coefficients by least squares, reject rank-deficient control sets, calculate inverse pixel residuals and RMSE, and retain the existing similarity helper only where compatibility requires it.

- [ ] **Step 2: Implement the embedded JavaScript model**

  Use affine mapping for three or more controls, use a clearly provisional two-point similarity model for placement, and update point rendering, coordinate reverse lookup, reference points, status text, residual display, and export metadata.

- [ ] **Step 3: Run the focused tests**

  Run: `python -m pytest tools/test_plot_rtk_points.py -q`

  Expected: PASS.

### Task 3: Update instructions and regenerate preview

**Files:**
- Modify: `docs/rtk_points_viewer_tutorial.md`
- Modify: `FIX_SUMMARY.md`
- Regenerate: `tools/rtk_preview/rtk_points_1112.html`

**Interfaces:**
- Consumes: the new generator behavior from Task 2.
- Produces: user-facing instructions that require three controls plus an independent fourth-point check, and a generated page containing the current 20-point dataset.

- [ ] **Step 1: Update the tutorial**

  Document the two-point provisional state, three-point affine calibration, non-collinear point selection, independent fourth-point validation, transformed-background picking limitation, and RMSE fallback behavior.

- [ ] **Step 2: Regenerate the HTML**

  Run:

  ```powershell
  python tools/plot_rtk_points.py --input src/rtk_nav/rtk_nav/config/rtk_points.md --output tools/rtk_preview/rtk_points_1112.html --background-image "布置图1112.svg"
  ```

- [ ] **Step 3: Update the change summary**

  Record the viewer behavior change and offline validation results with the date `2026-11-24`.

### Task 4: Verify generated output and repository behavior

**Files:**
- Verify: `tools/plot_rtk_points.py`
- Verify: `tools/rtk_preview/rtk_points_1112.html`
- Verify: `docs/rtk_points_viewer_tutorial.md`

- [ ] **Step 1: Run focused tests and syntax checks**

  Run:

  ```powershell
  python -m pytest tools/test_plot_rtk_points.py -q
  python -m py_compile tools/plot_rtk_points.py tools/test_plot_rtk_points.py
  ```

- [ ] **Step 2: Validate generated HTML content**

  Confirm the HTML contains the affine model, the current `rtk_points.md` records for sequences 24 through 43, and the embedded SVG.

- [ ] **Step 3: Inspect the diff**

  Run: `git diff --check` and review the changed files for unrelated edits.
