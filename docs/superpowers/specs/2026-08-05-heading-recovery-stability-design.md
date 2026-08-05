# RTK Heading Recovery Stability Design

## Goal

Prevent AUTO navigation from resuming into Stanley tracking while a recovered
dual-Fixed INS heading is still converging slowly after a long non-Fixed
period. Preserve the existing `rtk_status` publication format.

## Evidence

The current five-second circular heading span accepts a slow one-direction
drift whenever its movement during any individual five-second window is at or
below one degree. The recovered AUTO path then restores `WAYPOINT_MOVE`
directly; it only starts heading recalibration when the path-heading error is
greater than 15 degrees. Smaller, persistent errors can therefore reach
Stanley control and produce alternating steering corrections.

## State Flow

`AUTO entry or dual-Fixed recovery` -> `dual-Fixed continuous sampling` ->
`heading settle` -> `stopped path alignment` -> `WAYPOINT_MOVE`.

1. A sample is admissible only when `position_status == 4`, `fix_status == 4`,
   and `position_data_valid` is true.
2. Loss of any condition immediately clears all heading samples, stable state,
   and the dual-Fixed start time.
3. The gate requires both a five-second circular span no greater than one
   degree and a 30-second circular span no greater than one degree. The latter
   rejects the slow monotonic drift that can pass the short window.
4. The gate waits for up to 180 seconds. Until it passes or times out it
   continuously publishes zero speed. Timeout retains the existing recoverable
   `auto_heading_gate_timeout` PAUSE behavior.
5. When the gate passes for a saved `WAYPOINT_MOVE` state, navigation remains
   stopped and starts the existing heading recalibration toward the current
   path direction. Only after that calibration finishes may it resume Stanley
   tracking. A saved `WAYPOINT_CALIB` state remains in its existing calibration
   path.

## Boundaries

- No change to `motor_control.py` or its `rtk_status` string.
- The sample history is updated only while the AUTO gate is pending, avoiding
  normal-driving stable/unstable log churn.
- Circular heading calculations preserve correct behavior around 0/360 degrees.
- Existing RTK timeout, non-Fixed PAUSE, boundary safety, and manual-intervention
  contracts remain unchanged.

## Tests

Add deterministic unit coverage for:

- short-window pass combined with long-window failure for a slow monotonic
  drift;
- a stationary noisy heading that passes both windows;
- quality loss resetting both windows and the dual-Fixed epoch;
- gate release from `WAYPOINT_MOVE` entering stopped path alignment before
  Stanley wheel-speed output.
