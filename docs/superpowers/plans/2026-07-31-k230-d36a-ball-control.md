# K230 + D36A Ball Control Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add safe, low-latency automatic stepper adjustment to the existing K230 ball tracker using D36A STEP/DIR/EN control.

**Architecture:** Keep pure control math testable on the host and place K230 PWM/GPIO access behind a small driver class in the existing single-file CanMV application. Update the driver immediately after each valid axis measurement; LCD and diagnostic output remain rate-limited observers. Startup requires manual leveling plus a touch confirmation because no homing switch exists.

**Tech Stack:** CanMV MicroPython, K230 `machine.PWM`/`machine.Pin`/`machine.FPIOA`, pytest host-side structural and control tests, D36A/ATD5984 stepper driver.

## Global Constraints

- STEP is physical pin 13 (`PWM0/IO42`), DIR is pin 11 (`IO5`), EN is pin 12 (`IO6`), and control ground is pin 14.
- Existing UART1 on IO3/IO4 remains unchanged.
- EN is low for sleep and high for enable; STEP acts on rising edges.
- Startup and every restart require manual leveling plus touch confirmation.
- Invalid/stale vision, lost ball, missing pipe geometry, angle limit, or exception must stop PWM and pull EN low.
- Existing LCD, green-pipe geometry, KPU/blob tracking, UART, and RTSP behavior must remain available.

---

### Task 1: Pure stepper control law

**Files:**
- Modify: `main_two_touch_calibration.py`
- Test: `tests/test_stepper_control.py`

**Interfaces:**
- Consumes: target/ball positions in centimeters, ball velocity in cm/s, elapsed seconds, zeroed flag, and vision validity.
- Produces: `compute_stepper_command(...) -> dict` with `enabled`, `direction`, `frequency_hz`, `estimated_angle_deg`, and `fault`.

- [x] Write failing tests for deadband, direction, derivative damping, frequency clamp/ramp, angle limits, and invalid-vision shutdown.
- [x] Run `python -m pytest tests/test_stepper_control.py -q` and confirm failure because the API does not exist.
- [x] Add configuration constants and pure control functions without importing K230-only modules.
- [x] Run the focused tests and confirm all pass.

### Task 2: D36A hardware adapter and safe shutdown

**Files:**
- Modify: `main_two_touch_calibration.py`
- Test: `tests/test_stepper_structure.py`

**Interfaces:**
- Consumes: command dictionary from Task 1.
- Produces: `D36AStepper.apply(command)`, `D36AStepper.stop(disable=True)`, and `D36AStepper.deinit()`.

- [x] Write failing structural tests requiring IO42/PWM0 STEP, IO5 DIR, IO6 EN, active-high EN, PWM frequency update, and `finally` cleanup.
- [x] Run the focused test and confirm it fails on the missing adapter.
- [x] Implement the FPIOA/PWM/GPIO adapter with startup disabled and idempotent shutdown.
- [x] Run the focused test and confirm it passes.

### Task 3: Manual-zero touch state and real-time integration

**Files:**
- Modify: `main_two_touch_calibration.py`
- Test: `tests/test_stepper_integration.py`

**Interfaces:**
- Consumes: existing touch points and `axis_measurement(...)` output.
- Produces: an armed/zeroed state, immediate driver updates after valid measurements, LCD status, and extended UART telemetry.

- [x] Write failing tests that require motor-disabled startup, a dedicated touch-to-zero action, update-before-LCD ordering, stale/lost-ball shutdown, and control telemetry fields.
- [x] Run the focused test and confirm expected failures.
- [x] Add the zero/arm UI state, control update call, low-rate LCD fields, and compact UART fields.
- [x] Ensure exceptions and normal exit call `deinit()` before other teardown.
- [x] Run the focused integration tests and confirm they pass.

### Task 4: Regression verification and delivery

**Files:**
- Modify: `docs/superpowers/plans/2026-07-31-k230-d36a-ball-control.md`

- [x] Run `python -m pytest -q` and confirm zero failures.
- [x] Run `python -m py_compile main_two_touch_calibration.py` and confirm syntax success.
- [x] Run `git diff --check` and inspect the complete diff for unrelated changes.
- [x] Update this plan's checkboxes, commit only tracked implementation/test/plan files, and leave `main_green_pipe_tracking.py` untouched.
- [x] Push branch `agent/h264-circle-stream` and report the exact wiring, DIP settings, startup procedure, and hardware-only validation still required.
