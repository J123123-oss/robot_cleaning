# Output-Shaft RPM Unit Migration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove the implicit legacy `0~10` wheel-speed unit from the two upper ROS 2 control nodes so every wheel and brush command is expressed directly in reducer output-shaft `r/min`.

**Architecture:** Keep the AIMotor driver as the only layer that converts output-shaft `r/min` to motor-side pulses per second. Convert the already-calibrated legacy constants once to direct `r/min` values in `motor_control.py` and `rtk_nav.py`, keep existing ROS parameter names for launch compatibility, and clamp wheel commands at the 0.35 m/s physical limit.

**Tech Stack:** Python 3, ROS 2 `rclpy`, `pytest`, `Vector3` speed messages.

**Spec:** Current task request and the confirmed migration values: wheel radius `0.05 m`, maximum linear speed `0.35 m/s`, and direct output-shaft speed commands.

## Global Constraints

- `WHEEL_RPM_TO_MPS = 2*pi*0.05/60` remains the only wheel output-shaft `r/min` to linear-speed conversion.
- `MAX_SPEED` and all final wheel limits equal `0.35 / WHEEL_RPM_TO_MPS = 66.845076... r/min`.
- Do not keep or reintroduce `LEGACY_SPEED_UNIT_TO_RPM` in either upper node.
- Preserve existing ROS parameter names and topic/function interfaces.
- AIMotor receives output-shaft `r/min` and remains responsible for the per-motor reduction-ratio and pulse conversion.

---

### Task 1: Migrate `motor_control.py` constants and correction gains

**Files:**
- Modify: `src/motor_control/motor_control/motor_control.py`
- Test: `src/motor_control/test/test_motor_driver_aimotor.py` (only if the existing contract needs an updated unit assertion)

**Interfaces:**
- Consumes: output-shaft `r/min` from `/rtk/motor_speed`, keyboard/MQTT values, and internal navigation corrections.
- Produces: `/motor_speed` `Vector3` values whose `x`/`y` fields are wheel output-shaft `r/min`.

- [x] **Step 1: Replace legacy-derived constants with direct output-shaft values**

Use these values, preserving the prior physical response:

```python
MAX_SPEED = MAX_LINEAR_SPEED_MPS / WHEEL_RPM_TO_MPS
NAV_SPEED_BASE = 33.422538
NAV_DRIVE_KP = 0.534761  # r/min per degree
NAV_DRIVE_MAX_CORR = 13.369015
MAX_CORRECTION = 5.347606
```

Remove `LEGACY_SPEED_UNIT_TO_RPM` and update nearby comments/docstrings to state the output-shaft `r/min` contract.

- [x] **Step 2: Convert `get_speed_correction()` gains to direct `r/min` gains**

Use `0.334225`, `0.133690`, `0.033423`, and `0.334225` for the existing large-, medium-, small-error proportional gains and derivative gain respectively. Keep the error thresholds and sign convention unchanged.

- [x] **Step 3: Enforce the wheel output limit at `set_motors_speed()`**

Convert inputs to floats, clamp both wheel values to `[MIN_SPEED, MAX_SPEED]`, then store and publish the clamped values. Keep brush speed independent because the configured brush motor has its own output-speed command.

- [x] **Step 4: Run the focused motor-control syntax and driver tests**

Run: `python -m py_compile src/motor_control/motor_control/motor_control.py src/motor_control/motor_control/motor_driver(AIMotor).py`

Run: `python -m pytest -p no:cacheprovider src/motor_control/test/test_motor_driver_aimotor.py -q`

Expected: syntax compilation succeeds and all existing AIMotor driver tests pass.

### Task 2: Migrate `rtk_nav.py` navigation, Stanley, retreat, and visual correction values

**Files:**
- Modify: `src/rtk_nav/rtk_nav/rtk_nav.py`
- Verify: `src/rtk_nav/launch/run.launch.py` keeps the existing parameter names and direct `r/min` defaults.
- Test: `src/rtk_nav/test/test_stanley_visual_speed_scaling.py` and relevant navigation contract tests.

**Interfaces:**
- Consumes: navigation state, Stanley geometry, visual correction parameters, and existing launch arguments.
- Produces: `/rtk/motor_speed` wheel and brush output-shaft `r/min` commands, limited to `RTK_OUTPUT_SPEED_LIMIT`.

- [x] **Step 1: Remove the legacy conversion constant and convert navigation defaults**

Replace legacy-derived values with these direct output-shaft values:

```python
CALIB_RETRY_BACKUP_SPEED = 6.684508
TURN_SPEED_FAST = 10.026761
TURN_SPEED_MID = 6.684508
TURN_SPEED_SLOW = 2.673803
MAX_CORRECTION = 13.369015
RTK_MAX_CORRECTION_SPEED = 10.026761
```

Keep `MAX_WHEEL_SPEED_RPM`, `LINEAR_SPEED_BASE`, `RTK_OUTPUT_SPEED_LIMIT`, and `STANLEY_MIN_SPEED_RPM` derived from `WHEEL_RPM_TO_MPS`, because those expressions already use the direct `r/min` contract.

- [x] **Step 2: Convert visual correction and low-speed thresholds**

Use direct defaults `0.334225 r/min/degree`, `16.711269 r/min/m`, and `3.342254 r/min` for the visual parameters. Convert all boundary-turn minimums to `3.342254`, the stopped-motion threshold to `0.668451`, the retreat default to `13.369015`, the retreat correction limit to `6.684508`, and retreat minimum drive speed to `2.005352`.

- [x] **Step 3: Convert Stanley and heading PID gains**

Replace the legacy multiplications with direct gains while preserving thresholds, integrator behavior, signs, and saturation. Use these concrete branch values:

```python
# straight_get_speed_correction
kp = (1.002676, 1.002676, 0.802141)
kd = 0.267380
ki = 0.053476

# get_speed_correction
kp = (1.002676, 0.668451, 0.334225)
kd = 0.534761
```

- [x] **Step 4: Convert stuck-recovery and remaining turn thresholds**

Set `STUCK_ESCALATE_SPEED = 4.679155`, the remaining minimum turn speed to `3.342254`, and remove every remaining reference to `LEGACY_SPEED_UNIT_TO_RPM`. Keep logs and docstrings explicit about `r/min`.

- [x] **Step 5: Run navigation contract tests and inspect the unit boundary**

Run: `python -m py_compile src/rtk_nav/rtk_nav/rtk_nav.py src/rtk_nav/launch/run.launch.py`

Run: `python -m pytest -p no:cacheprovider src/rtk_nav/test/test_stanley_visual_speed_scaling.py src/rtk_nav/test/test_visual_correction_switch_contract.py -q`

Run: `rg -n "LEGACY_SPEED_UNIT_TO_RPM" src/motor_control/motor_control/motor_control.py src/rtk_nav/rtk_nav/rtk_nav.py`

Expected: compilation and targeted tests pass; the final search returns no matches; the launch parameter names remain unchanged. The package-wide run has unrelated fixture and missing-dependency failures documented in the handoff.

### Task 3: Final cross-node verification

**Files:**
- Verify: `src/motor_control/motor_control/motor_control.py`
- Verify: `src/rtk_nav/rtk_nav/rtk_nav.py`
- Verify: `src/rtk_nav/launch/run.launch.py`

- [x] **Step 1: Check the physical conversion numerically**

Confirm `MAX_SPEED * WHEEL_RPM_TO_MPS` is `0.35 m/s` within floating-point tolerance and that a `20 r/min` output command is converted by the AIMotor driver to `20 * reduction_ratio / 60 * 1000` motor pulses per second.

- [x] **Step 2: Run repository diff checks**

Run: `git diff --check`

Run: `git status --short`

Expected: no whitespace errors and only the intended migration files (plus the plan document) are changed after the first commit.
