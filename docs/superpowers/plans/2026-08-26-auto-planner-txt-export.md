# Auto Planner TXT Export Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a legacy-compatible TXT serializer for auto-planner routes, keeping every route corner and inserting points at no more than 15 metres on long straight segments.

**Architecture:** Keep route planning and existing JSON/GeoJSON serializers unchanged. Add pure geographic interpolation and heading helpers near the serializers, expose `route_to_txt(route, point_spacing=15.0)`, and optionally write that text through a `--txt-output` CLI argument.

**Tech Stack:** Python standard library, existing `Route`/`Segment` dataclasses, `unittest`.

---

### Task 1: Add TXT serializer behavior

**Files:**
- Modify: `src/rtk_nav/rtk_nav/auto_path_planner.py`
- Test: `src/rtk_nav/test/test_auto_path_planner.py`

- [ ] Add tests covering corner preservation, 15 m interpolation, duplicate junction removal, heading calculation, and invalid spacing.
- [ ] Implement `route_to_txt()` with the old header and four CSV fields, preserving `#region`/`#connector` comments.
- [ ] Use geodesic bearing in degrees `[0, 360)` and reuse the previous heading for the final point.

### Task 2: Expose TXT output from CLI

**Files:**
- Modify: `src/rtk_nav/rtk_nav/auto_path_planner.py`
- Test: `src/rtk_nav/test/test_auto_path_planner.py`

- [ ] Add `--txt-output` and `--txt-spacing`, defaulting to 15.0 m.
- [ ] Reject identical TXT and JSON/GeoJSON output paths.
- [ ] Write TXT alongside the existing route JSON and GeoJSON when requested.

### Task 3: Verify generated output

**Files:**
- No source changes.

- [ ] Run the focused auto-planner tests.
- [ ] Run the package test file if dependencies permit.
- [ ] Generate a sample E12/E14 route with TXT output and validate its header, CSV fields, corner endpoints, and maximum adjacent distance.
