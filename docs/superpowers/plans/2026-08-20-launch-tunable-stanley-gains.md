# Launch Tunable Stanley Gains Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Expose independent RTK Stanley and visual correction tuning parameters through `run.launch.py` while preserving the existing adaptive RTK selection and visual safety gates.

**Architecture:** `run.launch.py` declares two RTK gain arguments and five visual tuning arguments, converting each to a typed ROS parameter for `rtk_nav`. `RTKNavControlNode` stores the two Stanley gains and selects the near-target or normal value in `get_adaptive_stanley_k()`; visual correction remains an independently bounded additive term controlled by `enable_visual_correction`.

**Tech Stack:** ROS2 Humble launch, `rclpy`, Python AST contract tests, `py_compile`.

---

### Task 1: Add failing parameter contract tests

**Files:**
- Modify: `src/rtk_nav/test/test_visual_correction_switch_contract.py`
- Test: `src/rtk_nav/test/test_visual_correction_switch_contract.py`

- [x] **Step 1: Add launch contract assertions.**

Add a test that requires these launch declarations and typed parameter forwarding:

```python
def test_launch_exposes_independent_rtk_and_visual_tuning_parameters():
    source = LAUNCH_SOURCE_PATH.read_text(encoding="utf-8")

    for name, default in (
        ("stanley_k_path", "0.45"),
        ("stanley_k_near_target", "0.42"),
        ("visual_heading_gain", "0.2"),
        ("visual_lateral_gain", "10.0"),
        ("visual_max_steering_deg", "3.0"),
        ("visual_confidence_threshold", "0.5"),
        ("visual_timeout_sec", "0.5"),
    ):
        assert f'"{name}"' in source
        assert f'default_value=TextSubstitution(text="{default}")' in source
        assert (
            f'ParameterValue(LaunchConfiguration("{name}"), value_type=float)'
            in source
        )
```

- [x] **Step 2: Add RTK parameter and selection assertions.**

Add an AST test that requires the node initializer to declare/read both RTK parameters and executes the helper for both distance branches:

```python
def test_rtk_stanley_gain_parameters_select_normal_and_near_target_values():
    source = RTK_SOURCE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    initializer = ast.unparse(_function(tree, "__init__"))
    helper = _function(tree, "get_adaptive_stanley_k")

    assert 'self.declare_parameter("stanley_k_path", 0.45)' in initializer
    assert 'self.declare_parameter("stanley_k_near_target", 0.42)' in initializer
    assert "self.stanley_k_path" in initializer
    assert "self.stanley_k_near_target" in initializer

    namespace = {}
    exec(
        compile(ast.Module(body=[helper], type_ignores=[]), str(RTK_SOURCE_PATH), "exec"),
        namespace,
    )

    class State:
        stanley_k_path = 0.31
        stanley_k_near_target = 0.27

    state = State()
    select_gain = namespace["get_adaptive_stanley_k"]
    assert select_gain(state, 0.0, 0.4) == 0.27
    assert select_gain(state, 0.0, 1.3) == 0.31
```

- [x] **Step 3: Run the focused tests and confirm the expected RED state.**

Run:

```powershell
python -m pytest src/rtk_nav/test/test_visual_correction_switch_contract.py -q
```

Expected: the two new tests fail because the launch arguments and RTK parameter-backed gain selection do not yet exist. Existing visual switch tests may pass.

### Task 2: Implement typed launch parameters

**Files:**
- Modify: `src/rtk_nav/launch/run.launch.py:21-100`

- [x] **Step 1: Declare the seven tunable arguments.**

Add `DeclareLaunchArgument` entries with defaults `0.45`, `0.42`, `0.2`, `10.0`, `3.0`, `0.5`, and `0.5` for the names in Task 1.

- [x] **Step 2: Forward each argument as a float parameter.**

Inside the existing `rtk_navigator` parameter dictionary, use this exact shape for each parameter:

```python
"stanley_k_path": ParameterValue(
    LaunchConfiguration("stanley_k_path"), value_type=float
),
```

Repeat it for the other six names. Keep the existing boolean conversion for `enable_visual_correction` and keep the line detector/camera conditions unchanged.

### Task 3: Implement parameter-backed RTK gain selection

**Files:**
- Modify: `src/rtk_nav/rtk_nav/rtk_nav.py:301-330,2637-2640`

- [x] **Step 1: Declare and read the two RTK gain parameters.**

Immediately before the existing visual parameter declarations, add:

```python
self.declare_parameter("stanley_k_path", 0.45)
self.declare_parameter("stanley_k_near_target", 0.42)
self.stanley_k_path = max(
    0.0, float(self.get_parameter("stanley_k_path").value)
)
self.stanley_k_near_target = max(
    0.0, float(self.get_parameter("stanley_k_near_target").value)
)
```

- [x] **Step 2: Replace hardcoded adaptive returns.**

Use the stored values while retaining the existing threshold and branch semantics:

```python
def get_adaptive_stanley_k(self, velocity, distance_to_target):
    if distance_to_target < 1.3:
        return self.stanley_k_near_target
    return self.stanley_k_path
```

### Task 4: Verify behavior and scope

**Files:**
- Verify: `src/rtk_nav/launch/run.launch.py`
- Verify: `src/rtk_nav/rtk_nav/rtk_nav.py`
- Verify: `src/rtk_nav/test/test_visual_correction_switch_contract.py`

- [x] **Step 1: Run the focused contract tests and confirm GREEN.**

Run:

```powershell
python -m pytest src/rtk_nav/test/test_visual_correction_switch_contract.py -q
```

Expected: all tests in the file pass.

The environment does not provide `pytest`, so the same 9 test functions were
executed directly with `runpy`; all 9 passed.

- [x] **Step 2: Run syntax and whitespace checks.**

Run:

```powershell
python -c "import ast, py_compile, tempfile; from pathlib import Path; files=['src/rtk_nav/rtk_nav/rtk_nav.py','src/rtk_nav/launch/run.launch.py','src/rtk_nav/test/test_visual_correction_switch_contract.py']; [ast.parse(Path(f).read_text(encoding='utf-8')) for f in files]; d=Path(tempfile.mkdtemp(prefix='stanley_gain_compile_')); [py_compile.compile(f,cfile=str(d/(Path(f).stem+'.pyc')),doraise=True) for f in files]; print('AST and py_compile: PASS')"
git diff --check -- src/rtk_nav/rtk_nav/rtk_nav.py src/rtk_nav/launch/run.launch.py src/rtk_nav/test/test_visual_correction_switch_contract.py
```

Expected: the Python command prints `AST and py_compile: PASS` and `git diff --check` exits successfully.

- [x] **Step 3: Inspect the final diff.**

Run:

```powershell
git diff -- src/rtk_nav/rtk_nav/rtk_nav.py src/rtk_nav/launch/run.launch.py src/rtk_nav/test/test_visual_correction_switch_contract.py
```

Confirm that only independent gain parameters, their tests, and the required documentation changed; no unrelated dirty files are staged or reverted.
