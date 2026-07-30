# Small-Angle Ball Control Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace direct ball-error-to-frequency control with a bounded two-stage controller whose rod target never exceeds +/-2.0 degrees during edge recovery.

**Architecture:** The existing vision measurement remains unchanged. A pure outer PD calculation produces `target_angle_deg`; a pure inner position tracker compares that target with the estimated rod angle and produces bounded STEP frequency and direction. The existing `D36AStepper`, touch-zero flow, LCD/UART publishing, and reusable software watchdog execute the resulting command unchanged.

**Tech Stack:** CanMV MicroPython, K230 `machine.PWM`/`Timer`, D36A STEP/DIR/EN, Python AST-based pytest tests.

## Global Constraints

- Rod angle limit is exactly +/-2.0 degrees.
- Ball-position deadband is exactly +/-0.15 cm.
- STEP frequency is limited to 150-400 Hz.
- Direction reversal must decelerate to zero before switching.
- Invalid or stale vision data immediately stops STEP and disables EN.
- Pulse conversion remains 8.8889 pulses per rod degree.
- Do not change camera, KPU, pipe tracking, LCD refresh, UART refresh, touch calibration, Wi-Fi, or RTSP behavior.

---

### Task 1: Specify the two-stage control behavior with failing tests

**Files:**
- Modify: `tests/test_stepper_control.py`
- Test: `tests/test_stepper_control.py`

**Interfaces:**
- Consumes: existing `compute_stepper_command(...)` pure function and state keys `frequency_hz`, `motion_sign`, `estimated_angle_deg`, `last_update_ms`.
- Produces: executable requirements for returned keys `target_angle_deg`, `enabled`, `frequency_hz`, `direction`, `estimated_angle_deg`, and `fault`.

- [ ] **Step 1: Update the test helper to describe angle control**

Replace frequency-domain outer gains with these arguments:

```python
"kp_angle_deg_per_cm": 0.18,
"kd_angle_deg_per_cm_s": 0.04,
"angle_track_hz_per_deg": 800.0,
"angle_tolerance_deg": 0.03,
"deadband_cm": 0.15,
"min_frequency_hz": 40.0,
"max_frequency_hz": 500.0,
"frequency_ramp_hz_s": 1500.0,
"pulses_per_degree": 8.8889,
"angle_limit_deg": 1.0,
```

- [ ] **Step 2: Add tests for bounded target angle and inner tracking**

```python
def test_large_ball_error_clamps_target_angle_to_one_degree():
    result = command(error_cm=20.0, frequency_ramp_hz_s=1000000.0)
    assert result["target_angle_deg"] == 1.0
    assert result["frequency_hz"] <= 500.0


def test_inner_loop_tracks_target_from_estimated_angle():
    state = base_state()
    state["estimated_angle_deg"] = 0.8
    result = command(state=state, error_cm=1.0,
                     frequency_ramp_hz_s=1000000.0)
    assert result["target_angle_deg"] == 0.18
    assert result["direction"] == -1
```

- [ ] **Step 3: Add tests for deadband leveling and safety**

```python
def test_ball_deadband_returns_nonlevel_rod_toward_zero():
    state = base_state()
    state["estimated_angle_deg"] = 0.5
    result = command(state=state, error_cm=0.1,
                     frequency_ramp_hz_s=1000000.0)
    assert result["target_angle_deg"] == 0.0
    assert result["enabled"] is True
    assert result["direction"] == -1


def test_level_rod_inside_ball_deadband_stops():
    result = command(error_cm=0.1)
    assert result["target_angle_deg"] == 0.0
    assert result["enabled"] is False
    assert result["fault"] == "angle_deadband"
```

- [ ] **Step 4: Run the focused tests and verify RED**

Run:

```powershell
$py='C:\Users\Sayo\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe'
& $py -m pytest tests/test_stepper_control.py -q
```

Expected: failures because the current function has no `target_angle_deg` and still accepts frequency-domain outer gains.

- [ ] **Step 5: Commit the failing specification tests**

```powershell
git add tests/test_stepper_control.py
git commit -m "test: specify small-angle rod control"
```

---

### Task 2: Implement the outer PD angle target and inner angle tracker

**Files:**
- Modify: `main_two_touch_calibration.py:156-170`
- Modify: `main_two_touch_calibration.py:231-330`
- Modify: `main_two_touch_calibration.py:1796-1814`
- Test: `tests/test_stepper_control.py`

**Interfaces:**
- Consumes: ball `error_cm`, ball `velocity_cm_s`, measurement validity, zero state, and previous stepper state.
- Produces: `compute_stepper_command(...) -> dict` including `target_angle_deg: float`, plus all existing command fields consumed by `D36AStepper.apply()`.

- [ ] **Step 1: Replace controller constants**

Use these exact constants:

```python
STEPPER_KP_ANGLE_DEG_PER_CM = 0.18
STEPPER_KD_ANGLE_DEG_PER_CM_S = 0.04
STEPPER_ANGLE_TRACK_HZ_PER_DEG = 800.0
STEPPER_ANGLE_TOLERANCE_DEG = 0.03
STEPPER_DEADBAND_CM = 0.15
STEPPER_MIN_FREQUENCY_HZ = 40.0
STEPPER_MAX_FREQUENCY_HZ = 500.0
STEPPER_FREQUENCY_RAMP_HZ_S = 1500.0
STEPPER_ANGLE_LIMIT_DEG = 1.0
```

- [ ] **Step 2: Add target angle to initial and returned state**

Add `"target_angle_deg": 0.0` to `new_stepper_control_state()` and every result from `compute_stepper_command()`.

- [ ] **Step 3: Replace the outer frequency equation with a target-angle equation**

Implement:

```python
if abs(error_cm) <= deadband_cm:
    target_angle_deg = 0.0
else:
    target_angle_deg = (
        kp_angle_deg_per_cm * error_cm -
        kd_angle_deg_per_cm_s * velocity_cm_s)
target_angle_deg = max(
    -angle_limit_deg, min(angle_limit_deg, target_angle_deg))
angle_error_deg = target_angle_deg - estimated_angle
```

- [ ] **Step 4: Convert angle error to bounded STEP frequency**

Implement:

```python
if abs(angle_error_deg) <= angle_tolerance_deg:
    result["fault"] = "angle_deadband"
    return result
logical_direction = 1 if angle_error_deg > 0.0 else -1
target_frequency = max(
    min(abs(angle_error_deg) * angle_track_hz_per_deg,
        max_frequency_hz),
    min_frequency_hz)
```

Retain the existing ramp limiter and reverse-before-switch logic. Angle-limit handling must use the current `estimated_angle` and must always permit movement back toward `target_angle_deg`.

- [ ] **Step 5: Pass new constants from `update_stepper_control()`**

Update the call to `compute_stepper_command()` to use the new angle-domain gain constants and angle tolerance without changing vision freshness handling.

- [ ] **Step 6: Run focused tests and verify GREEN**

Run:

```powershell
& $py -m pytest tests/test_stepper_control.py -q
```

Expected: every control test passes.

- [ ] **Step 7: Commit the control implementation**

```powershell
git add main_two_touch_calibration.py tests/test_stepper_control.py
git commit -m "feat: control rod with bounded target angle"
```

---

### Task 3: Expose target angle and verify integration

**Files:**
- Modify: `main_two_touch_calibration.py:1463-1470`
- Modify: `main_two_touch_calibration.py:1938-1960`
- Modify: `tests/test_stepper_integration.py`
- Modify: `tests/test_stepper_structure.py`

**Interfaces:**
- Consumes: command fields `target_angle_deg` and `estimated_angle_deg`.
- Produces: LCD diagnostic `A:<actual>/<target>` and UART command frame containing both actual and target rod angles.

- [ ] **Step 1: Write failing display and UART tests**

Update the expected UART frame to include target angle:

```python
message = fn({
    "zeroed": True, "enabled": True, "frequency_hz": 345.4,
    "direction": -1, "estimated_angle_deg": 0.25,
    "target_angle_deg": -0.40, "fault": "none",
})
assert message == b"M:1,R:1,F:0345,D:-1,A:+0.25,T:-0.40,E:none\n"
```

Add an AST assertion that LCD formatting contains:

```python
"A:{:+.2f}/{:+.2f}"
```

- [ ] **Step 2: Run integration tests and verify RED**

Run:

```powershell
& $py -m pytest tests/test_stepper_integration.py tests/test_stepper_structure.py -q
```

Expected: UART and LCD target-angle assertions fail.

- [ ] **Step 3: Update UART and LCD diagnostics**

Change the UART frame to:

```text
M:<zeroed>,R:<running>,F:<hz>,D:<dir>,A:<estimated>,T:<target>,E:<fault>
```

Change the LCD angle line to display estimated and target angles together:

```text
A:+0.25/-0.40
```

- [ ] **Step 4: Run syntax and full regression verification**

Run:

```powershell
& $py -m py_compile main_two_touch_calibration.py
& $py -m pytest -q
git diff --check
```

Expected: syntax succeeds, all tests pass, and `git diff --check` reports no whitespace errors.

- [ ] **Step 5: Commit and push**

```powershell
git add main_two_touch_calibration.py tests/test_stepper_integration.py tests/test_stepper_structure.py
git commit -m "feat: expose rod target angle diagnostics"
git push
```

---

### Task 4: Perform on-board acceptance

**Files:**
- Modify only if a measured parameter needs tuning: `main_two_touch_calibration.py:160-169`

**Interfaces:**
- Consumes: LCD `A:<estimated>/<target>`, UART control frames, physical rod motion, and ball response.
- Produces: confirmed initial tuning values safe for the competition hardware.

- [ ] **Step 1: Run without motor power**

Verify the camera, LCD, touch-zero button, ball position, target angle, and UART continue updating without exceptions.

- [ ] **Step 2: Enable the D36A with the rod level**

Tap zero and verify the estimated and target angle remain within +/-2.0 degrees.

- [ ] **Step 3: Test small ball offsets**

Move the ball approximately 1 cm, then 3 cm, then 5 cm from target. Verify the rod motion remains small, reverses without impact, and approaches level when the ball returns to the deadband.

- [ ] **Step 4: Test vision loss**

Temporarily cover the ball and verify UART reports `vision_invalid`, STEP output stops, and EN becomes low.

- [ ] **Step 5: Tune only gains, one at a time**

The field-tuned values are `KP=0.45`, `KD=0.06`, tracking gain `1200 Hz/deg`, `150-400 Hz`, and an `8000 Hz/s` ramp. Do not increase `STEPPER_ANGLE_LIMIT_DEG` above `2.0` without a new physical safety review.

- [ ] **Step 6: Commit measured tuning values**

```powershell
git add main_two_touch_calibration.py
git commit -m "tune: calibrate small-angle ball response"
git push
```
