# Auto Planner Connector Routing Design

## Problem

The automatic RTK route can attach a completed region to its outgoing bridge with a long diagonal.  It can also draw diagonal crossings around a hole because each scan segment is connected independently through an unrestricted visibility graph.

## Decision

Coverage is ordered as a complete serpentine traversal.  The planner evaluates the four valid combinations of sweep order and initial direction, using the region entry point and outgoing bridge endpoint as costs.  It never flips only one coverage segment.

Internal connector edges are constrained to the sweep axes.  The routing graph includes valid horizontal and vertical bend points, so it routes around holes as orthogonal travel rather than a visibility-graph diagonal.  A segment remains rejected when no safe connector within `max_connector` exists.

## Scope

The change is limited to `auto_path_planner.py` and its unit tests.  Cross-region travel remains an explicit `connectors[]` path.  No runtime ROS node or legacy YAML path format changes.

## Verification

Tests cover a bridge entering a multi-row region at one edge and a rectangular hole.  They require all normal sweep turns to remain short and every internal connector edge to be horizontal or vertical in the guide-aligned local frame.
