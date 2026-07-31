# K230 Full Ball Control Optimization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Convert the current CanMV program into a stable 30 Hz vision outer loop plus 200 Hz encoder angle inner loop that returns the ball to the geometric rail centre within 1 cm.

**Architecture:** The vision loop only publishes filtered ball position, velocity, validity, and a target rod angle. A single 5 ms `RodCascadeController` timer owns all D36A commands, encoder safety checks, visual timeout recovery, and control-rate telemetry; LCD, UART, KPU, Blob, and H.264 remain outside that callback.

**Tech Stack:** CanMV K230 MicroPython, GC2093, KPU/AI2D, RGB565 Blob tracking, `machine.Pin`, `machine.Timer`, D36A PWM, MS42CG ABZ/PWM encoder, pytest AST-isolated desktop tests.

## Global Constraints

- Source baseline is the synchronized WXWork `main_two_touch_calibration.py` at commit `8367299`.
- GC2093 and H.264/VLC remain 1920×1080@30; no false 60 FPS claim.
- AI and traditional vision coordinates use 320×320.
- Only one software timer exists, with ID `-1`, period 5 ms, and no `hard` keyword.
- Timer and GPIO IRQ callbacks allocate no lists, dictionaries, or strings.
- The geometric rail midpoint is target O; task-3 trajectory remains disabled.
- Existing unrelated untracked files are never added.

---

### Task 1: CanMV-safe MS42CG adapter and calibration

**Files:**
- Modify: `main_two_touch_calibration.py`
- Test: `tests/test_ms42cg_encoder.py`
- Test: `tests/test_ms42cg_adapter.py`
- Test: `tests/test_encoder_calibration_io.py`

**Interfaces:**
- Produces `wrapped_encoder_delta()`, parameterized `encoder_count_to_angle()`, `pwm_duty_to_count()`, `validate_encoder_calibration()`, `calibrate_encoder_zero()`, `restore_encoder_from_absolute()`, and `MS42CGEncoder(fpioa, ticks_us_fn=None, ticks_diff_fn=None)`.

- [ ] Run the existing focused tests and retain their current failures as the RED evidence.
- [ ] Replace PWM IRQ append operations with preallocated sample arrays and an integer index.
- [ ] Add injectable tick functions, active/released flags, and `Pin.__del__()` one-time release.
- [ ] Keep AB/Z/PWM IRQ callbacks limited to primitive numeric updates.
- [ ] Implement strict calibration schema and circular absolute-angle restore.
- [ ] Run the three focused test modules until all pass.
- [ ] Commit as `fix: make MS42CG feedback CanMV safe`.

### Task 2: Real 200 Hz encoder cascade controller

**Files:**
- Modify: `main_two_touch_calibration.py`
- Test: `tests/test_stepper_control.py`
- Test: `tests/test_encoder_safety.py`
- Test: `tests/test_stepper_structure.py`
- Test: `tests/test_stepper_integration.py`

**Interfaces:**
- Produces `compute_angle_pid(...)` with explicit gains and limits plus `RodCascadeController.set_visual_target(angle_deg, timestamp_ms, valid)`, `.tick(timer)`, `.status(now_ms)`, `.arm(zero_count)`, `.disarm(fault)`, and `.deinit()`.

- [ ] Run the focused tests and preserve failures for missing cascade behavior.
- [ ] Remove the private D36A watchdog timer; keep D36A as a hardware-only adapter.
- [ ] Implement one `Timer(-1)` initialized with `mode`, `period=5`, and `callback` only.
- [ ] Move the only production `stepper.apply()` call into `RodCascadeController.tick()`.
- [ ] Integrate stall, invalid transition, absolute mismatch, low-rate, angle-limit, and visual-timeout faults into the same tick.
- [ ] For visual age below 80 ms hold target, from 80 to 150 ms ramp target toward zero, and over 150 ms command zero and stop after level.
- [ ] Wire startup calibration restore and cleanup in safe order.
- [ ] Run focused tests until all pass.
- [ ] Commit as `feat: add 200hz encoder cascade control`.

### Task 3: Stable centre-return outer controller

**Files:**
- Modify: `main_two_touch_calibration.py`
- Create: `tests/test_outer_balance_optimization.py`
- Test: `tests/test_green_pipe_geometry.py`

**Interfaces:**
- Produces `alpha_beta_update(state, position_cm, timestamp_ms, ...)` and `compute_outer_balance_target(error_cm, velocity_cm_s, dt_s, state, ...)` returning target angle plus updated integral state.

- [ ] Add failing tests for real-time velocity estimation, reversal damping, bounded 0.3 degree bias integral, centre settling, smooth segmented angle limits, and edge recovery.
- [ ] Run the tests to verify failures are due to missing interfaces.
- [ ] Implement alpha-beta position/velocity filtering with jump, velocity, and acceleration clamps.
- [ ] Implement conditional anti-windup integral only inside 2 cm and below 5 cm/s.
- [ ] Implement smooth gain scheduling: 0.3 degree near centre, 1 to 2 degrees mid-rail, and at most 5 degrees at the edge.
- [ ] Apply target-angle slew limiting and reset integral on invalid vision or sign-inconsistent jumps.
- [ ] Run focused geometry and outer-controller tests until all pass.
- [ ] Commit as `feat: stabilize centre return outer loop`.

### Task 4: 320×320 hybrid vision and low-rate rail validation

**Files:**
- Modify: `main_two_touch_calibration.py`
- Test: `tests/test_fast_vision_telemetry.py`
- Create: `tests/test_vision_scheduler.py`

**Interfaces:**
- Produces 320×320 AI/Blob coordinates, `should_validate_pipe(frame, locked)`, `ball_tracking_roi(x, y)`, and the existing KPU/Blob hybrid state machine with KPU validation every three frames.

- [ ] Run the existing fast-vision tests as RED for 640×360 constants.
- [ ] Add failing scheduler tests for five-frame startup rail lock, one-in-ten locked rail validation, bounded local ball ROI, and KPU cadence three.
- [ ] Scale ROI, size, pixel-area, distance, prediction, and pipe geometry constants to 320×320.
- [ ] Lock the rail only after five stable observations and validate it every ten frames thereafter.
- [ ] Use a 96×96 local ROI when tracking is active and a 128×128 recovery ROI before returning to global KPU search.
- [ ] Keep LCD/VLC display dimensions and H.264 path unchanged.
- [ ] Run focused vision tests until all pass.
- [ ] Commit as `perf: accelerate hybrid vision pipeline`.

### Task 5: Nonblocking telemetry and state cleanup

**Files:**
- Modify: `main_two_touch_calibration.py`
- Test: `tests/test_fast_vision_telemetry.py`
- Test: `tests/test_stepper_integration.py`
- Create: `tests/test_balance_state_cleanup.py`

**Interfaces:**
- Produces `runtime_telemetry`, `compute_window_fps()`, `format_ball_telemetry()`, and a single ACTIVE startup path gated by valid PWM zero restore.

- [ ] Add failing tests for P/BV/AI/CTRL display strings, 15-frame FPS window, removal of unreachable WAIT_CENTER prompts, and refusal to arm on session-only zero.
- [ ] Implement cached telemetry with OSD cadence 2 or 3 while UART/control remain per valid measurement.
- [ ] Remove unreachable WAIT_CENTER transitions and misleading CENTER CONFIRM text.
- [ ] Block ACTIVE until encoder PWM absolute zero is valid or a fresh valid LEVEL CONFIRM succeeds.
- [ ] Preserve periodic performance metrics without printing in IRQ/timer callbacks.
- [ ] Run telemetry, integration, and state tests until all pass.
- [ ] Commit as `feat: add safe runtime telemetry and startup state`.

### Task 6: Full verification and deployment handoff

**Files:**
- Verify: `main_two_touch_calibration.py`
- Verify: `tests/`
- Update: `docs/MS42CG_WIRING_AND_VALIDATION.md`

**Interfaces:**
- Produces the final tested CanMV script and board validation instructions.

- [ ] Run `python -m py_compile main_two_touch_calibration.py`.
- [ ] Run all pytest tests with a worktree-local `--basetemp` and require zero failures.
- [ ] Run `git diff --check`, inspect the complete diff, and ensure unrelated files are absent.
- [ ] Update wiring/validation documentation with hard-reset, zero-restore, push-return, edge-return, occlusion, soft-rerun, LCD, and VLC checks.
- [ ] Commit as `docs: update full control validation procedure`.
- [ ] Push `agent/full-ball-optimization` to the private origin and preserve the worktree for board feedback.
