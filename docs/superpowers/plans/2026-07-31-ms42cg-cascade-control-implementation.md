# MS42CG Cascade Control Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace pulse-count-estimated rod angle with MS42CG ABZ/PWM feedback and run a visual outer loop plus a 200 Hz encoder angle inner loop on K230.

**Architecture:** The existing visual controller continues to produce a bounded target rod angle. A new MS42CG adapter captures ABZ and startup/calibration PWM data, while a pure inner PID converts real angle error into signed D36A STEP frequency. The sole `Timer(-1)` becomes the periodic inner-loop scheduler and also enforces the existing vision watchdog.

**Tech Stack:** CanMV K230 MicroPython v1.8, `machine.Pin`, `machine.Timer`, `machine.PWM`, JSON on `/sdcard`, pytest host-side AST/pure-function tests.

## Global Constraints

- Encoder inputs are 3.3 V single-ended.
- A=IO19, B=IO20, Z=IO32, PWM absolute=IO33.
- D36A remains STEP=IO42/PWM0, DIR=IO5, EN=IO6.
- MS42CG is 1024 lines; quadrature resolution is 4096 counts/rev and 0.087890625°/count.
- Rod limit remains ±16°.
- Inner loop target is 200 Hz; actual rate below 80 Hz for 200 ms is a fault.
- No fallback to STEP pulse-count angle estimation is allowed.
- Existing vision, LCD, UART, H.264/RTSP, green-pipe and touch behavior must remain operational.
- Do not add or modify the unrelated untracked `main_green_pipe_tracking.py`.

---

### Task 1: Pure encoder math and calibration data

**Files:**
- Modify: `main_two_touch_calibration.py`
- Create: `tests/test_ms42cg_encoder.py`

**Interfaces:**
- Produces: `quadrature_delta(previous_state: int, current_state: int) -> int`
- Produces: `wrapped_encoder_delta(current: int, zero: int, counts_per_rev: int) -> int`
- Produces: `encoder_count_to_angle(delta: int, counts_per_rev: int) -> float`
- Produces: `pwm_duty_to_count(high_us: int, period_us: int, counts_per_rev: int, duty_min: float, duty_max: float, invert: bool) -> int | None`
- Produces: `validate_encoder_calibration(data: dict) -> dict | None`

- [ ] **Step 1: Write failing tests for all 16 Gray-code transitions**

Test valid forward sequence `00→01→11→10→00`, reverse sequence, repeated state, and invalid two-bit jumps.

- [ ] **Step 2: Run the focused tests and verify missing-function failures**

Run: `python -m pytest -p no:cacheprovider tests/test_ms42cg_encoder.py -q`

- [ ] **Step 3: Implement table-driven quadrature decoding**

Use a 16-entry tuple indexed by `(previous_state << 2) | current_state`. Invalid two-bit jumps return a separate invalid indication without changing count.

- [ ] **Step 4: Add failing wraparound and PWM conversion tests**

Cover `4095→0`, `0→4095`, 25%, 50%, 75% duty, invalid period, and configured duty endpoints.

- [ ] **Step 5: Implement circular angle and PWM conversion**

Use half-revolution wrapping and clamp normalized PWM into `[0, counts_per_rev - 1]` only when duty is inside configured endpoints.

- [ ] **Step 6: Add failing calibration schema tests**

Accept only version 1, `encoder_model == "MS42CG"`, 4096 counts/rev, integer zero count in range, boolean inversion and optional integer Z index.

- [ ] **Step 7: Implement calibration validation and run focused tests**

Run: `python -m pytest -p no:cacheprovider tests/test_ms42cg_encoder.py -q`

- [ ] **Step 8: Commit Task 1**

Commit message: `feat: add MS42CG encoder math`

---

### Task 2: MS42CG GPIO/IRQ adapter and PWM sampling

**Files:**
- Modify: `main_two_touch_calibration.py`
- Create: `tests/test_ms42cg_adapter.py`
- Modify: `tests/test_stepper_structure.py`

**Interfaces:**
- Produces class: `MS42CGEncoder`
- Constructor: `MS42CGEncoder(fpioa, ticks_us_fn=time.ticks_us)`
- Methods: `start_abz()`, `start_pwm_capture()`, `stop_pwm_capture()`, `snapshot(now_us)`, `set_zero_from_absolute(count)`, `deinit()`
- Snapshot keys: `count`, `angle_deg`, `velocity_deg_s`, `absolute_count`, `pwm_valid`, `z_seen`, `invalid_transitions`, `last_edge_us`.

- [ ] **Step 1: Write failing structural tests for IO19/20/32/33 mapping**

Assert FPIOA input mappings, `Pin.IN`, `Pin.IRQ_BOTH`, and no output drive on encoder pins.

- [ ] **Step 2: Write failing adapter tests with fake pins and timestamps**

Feed forward/reverse AB states, Z edge, PWM rising/falling edges, and verify preallocated state changes.

- [ ] **Step 3: Verify the tests fail because the adapter is absent**

Run: `python -m pytest -p no:cacheprovider tests/test_ms42cg_adapter.py tests/test_stepper_structure.py -q`

- [ ] **Step 4: Implement AB IRQ callbacks**

Callbacks only read A/B, update integer count, update `last_edge_us`, and increment invalid-transition counters. Do not print, allocate dictionaries, write files or touch D36A PWM in callbacks.

- [ ] **Step 5: Implement Z and temporary PWM capture callbacks**

PWM capture accumulates a fixed-size preallocated sample buffer and disables its IRQ once enough stable samples exist. It is enabled only during startup or explicit calibration.

- [ ] **Step 6: Implement snapshot velocity calculation outside IRQ context**

Use count/time differences with bounded `dt`; reject timestamp wrap and implausible jumps.

- [ ] **Step 7: Run focused tests and commit Task 2**

Commit message: `feat: capture MS42CG ABZ and PWM feedback`

---

### Task 3: Replace estimated-angle control with real-angle PID

**Files:**
- Modify: `main_two_touch_calibration.py`
- Modify: `tests/test_stepper_control.py`
- Modify: `tests/test_stepper_integration.py`

**Interfaces:**
- Replaces: estimated-angle integration inside `compute_stepper_command`
- Produces: `compute_angle_pid(target_angle_deg, actual_angle_deg, actual_velocity_deg_s, dt_s, state, kp_hz_per_deg, ki_hz_per_deg_s, kd_hz_per_deg_s, max_frequency_hz, ramp_hz_s, angle_limit_deg) -> dict`
- State keys: `integral_hz`, `frequency_hz`, `direction`, `last_update_ms`, `fault`.

- [ ] **Step 1: Write failing PID tests**

Cover positive/negative error, one-count deadband, derivative braking, integral anti-windup, ±16° directional limit, frequency saturation, ramping, and decelerate-before-reverse.

- [ ] **Step 2: Run focused tests and confirm expected failures**

Run: `python -m pytest -p no:cacheprovider tests/test_stepper_control.py -q`

- [ ] **Step 3: Implement the pure real-angle PID**

Output signed desired frequency but keep D36A application separate. Clamp integral whenever output saturates or an angle limit blocks motion.

- [ ] **Step 4: Refactor `D36AStepper.apply()` to consume the PID command**

Retain active-high enable, 5 µs DIR setup, PWM0 frequency updates and 50% STEP duty. Remove pulse integration from all control state.

- [ ] **Step 5: Add a failing test proving pulse estimation cannot reappear**

AST assertion: production control must not update angle using `frequency * elapsed / pulses_per_degree`.

- [ ] **Step 6: Run focused tests and commit Task 3**

Commit message: `feat: close rod angle loop with encoder PID`

---

### Task 4: 200 Hz scheduler, watchdog and encoder safety

**Files:**
- Modify: `main_two_touch_calibration.py`
- Create: `tests/test_encoder_safety.py`
- Modify: `tests/test_stepper_integration.py`

**Interfaces:**
- Produces class: `RodCascadeController`
- Methods: `set_visual_target(angle_deg, timestamp_ms, valid)`, `tick(timer)`, `status(now_ms)`, `deinit()`.
- Owns the sole `Timer(-1)` at 5 ms periodic mode.

- [ ] **Step 1: Write failing safety tests**

Cover visual age >150 ms, commanded movement with no encoder edges for 80 ms, AB/PWM mismatch >2°, invalid AB transitions, control rate <80 Hz for 200 ms, and valid recovery paths.

- [ ] **Step 2: Verify failure before implementation**

Run: `python -m pytest -p no:cacheprovider tests/test_encoder_safety.py -q`

- [ ] **Step 3: Implement periodic cascade controller**

At every tick: snapshot encoder, validate safety, calculate PID, apply D36A command, and update preallocated status fields. Use `hard=False` because the callback calls MicroPython objects and may allocate.

- [ ] **Step 4: Merge the old one-shot vision watchdog**

Delete the separate `D36AStepper.watchdog` and enforce vision timeout inside `RodCascadeController.tick()` so only one `Timer(-1)` exists.

- [ ] **Step 5: Add actual callback-rate monitoring**

Count ticks per 200 ms window. A sustained rate below 80 Hz produces `CTRL SLOW`, stops STEP and leaves EN holding only when the measured angle is within ±16°.

- [ ] **Step 6: Run safety/integration tests and commit Task 4**

Commit message: `feat: schedule safe encoder cascade loop`

---

### Task 5: Persistent absolute zero and startup restoration

**Files:**
- Modify: `main_two_touch_calibration.py`
- Create: `tests/test_encoder_calibration_io.py`
- Modify: `tests/test_stepper_integration.py`

**Interfaces:**
- Produces constants: `ENCODER_CALIBRATION_PATH`, `ENCODER_CALIBRATION_TEMP_PATH`
- Produces: `load_encoder_calibration(path=...)`
- Produces: `save_encoder_calibration(path, calibration, open_fn=open, rename_fn=os.rename)`
- Produces: `calibrate_encoder_zero(encoder_snapshot, z_index_count=None)`

- [ ] **Step 1: Write failing tests for load/save and atomic replacement**

Cover valid file, missing file, malformed JSON, wrong version, out-of-range count, temporary write failure, and successful rename.

- [ ] **Step 2: Implement validated atomic persistence**

Write the temporary JSON, close it, then rename. Never overwrite a good calibration before the temporary file is complete.

- [ ] **Step 3: Write failing startup restoration tests**

Verify PWM/zero circular difference seeds the AB count, including 0/4095 wrap and inversion.

- [ ] **Step 4: Integrate startup PWM sampling**

If calibration and PWM are valid, initialize true angle and arm the controller. Otherwise keep D36A disabled and expose `ENC ZERO REQUIRED` or `PWM INVALID`.

- [ ] **Step 5: Integrate the existing touch ZERO action**

Stop STEP, acquire stable PWM samples, save zero, reset AB relative count, then arm the controller. Do not reuse the visual target calibration file for encoder zero.

- [ ] **Step 6: Run focused tests and commit Task 5**

Commit message: `feat: persist MS42CG absolute zero`

---

### Task 6: Main-loop, LCD and UART integration

**Files:**
- Modify: `main_two_touch_calibration.py`
- Modify: `tests/test_main_structure.py`
- Modify: `tests/test_stepper_integration.py`

**Interfaces:**
- Visual outer loop calls `cascade_controller.set_visual_target(...)` once per valid/invalid measurement.
- LCD fields: actual angle `A`, target angle `T`, angular velocity `V`, inner error `E`, STEP frequency `F`, encoder state `ENC`, actual control rate `Hz`.

- [ ] **Step 1: Write failing AST/integration tests for ownership boundaries**

Assert the visual loop only updates target data; only the cascade controller applies D36A commands; RTSP worker paths cannot call control functions.

- [ ] **Step 2: Replace `update_stepper_control()` main-loop actuation**

Convert ball error/velocity to target angle using the existing outer-loop formula, then publish target and timestamp to the cascade controller without directly applying STEP output.

- [ ] **Step 3: Add LCD and UART encoder telemetry**

Keep current T display and append real A/V/E/F/ENC/Hz values. Rate-limit diagnostic text without reducing ball-coordinate UART frequency.

- [ ] **Step 4: Ensure cleanup order is safe**

Stop the cascade timer, disable encoder IRQs, stop STEP, release D36A PWM, then tear down RTSP/media.

- [ ] **Step 5: Run integration tests and commit Task 6**

Commit message: `feat: integrate MS42CG cascade telemetry`

---

### Task 7: Full verification and hardware handoff

**Files:**
- Modify: `README.md` or existing hardware handoff document if present
- Modify: `docs/superpowers/specs/2026-07-31-ms42cg-cascade-control-design.md` only if implementation reveals an explicitly approved correction

- [ ] **Step 1: Run syntax and full host tests**

Run:

```powershell
python -m py_compile main_two_touch_calibration.py
python -m pytest -p no:cacheprovider -q
git diff --check
```

Expected: syntax success, all tests pass, no whitespace errors.

- [ ] **Step 2: Produce exact wiring/startup checklist**

Document 3.3V, common ground, A/B/Z/PWM pins, D36A pins, first horizontal calibration, normal boot messages and every encoder fault string.

- [ ] **Step 3: Perform staged on-board validation**

1. D36A disabled: inspect PWM absolute count while hand rotating.
2. Inspect AB count and direction while hand rotating.
3. Confirm Z once per revolution.
4. Calibrate horizontal and power-cycle.
5. Run ±1° low-speed angle commands.
6. Run ±5° commands and verify no count mismatch.
7. Enable visual outer loop with stationary car.
8. Enable low-speed straight driving.
9. Enable full line following and record edge excursions.

- [ ] **Step 4: Commit final handoff documentation**

Commit message: `docs: add MS42CG wiring and validation guide`

- [ ] **Step 5: Push the completed branch**

Push `agent/h264-circle-stream` only after all host tests and staged checks available in the current environment pass.

