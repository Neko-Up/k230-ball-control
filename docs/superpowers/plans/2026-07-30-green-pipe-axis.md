# Green Pipe Dynamic Axis Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace fixed two-touch axis calibration with per-frame green-pipe geometry, an official midpoint O, and a one-touch target whose signed centimeter error follows pipe motion.

**Architecture:** Reuse the existing RGB565 fast channel for both ball Blob tracking and green-pipe Blob detection. Convert the green Blob's rotated minimum-area rectangle into a dynamic axis, store the touched target as a normalized axis parameter, project the ball onto the current axis, and derive position/target/error using the known 25 cm pipe length.

**Tech Stack:** CanMV K230 MicroPython, OpenMV-style `image.find_blobs`, existing KPU/Blob tracker, LCD OSD, UART, H.264/RTSP.

## Global Constraints

- Preserve the existing ball detector, control cadence, LCD, UART, Wi-Fi, and VLC/H.264 pipeline.
- Detect the green pipe on the existing RGB565 channel without adding another camera channel.
- The official coordinate origin O is always the geometric midpoint of the detected 25 cm pipe.
- One touch selects the target position; save it as a normalized pipe parameter so it follows camera motion.
- All geometry and control outputs use the dynamic pipe axis; never store a fixed screen-space line.

---

### Task 1: Pure pipe geometry

**Files:**
- Modify: `main_two_touch_calibration.py`
- Test: `tests/test_green_pipe_geometry.py`

**Interfaces:**
- Produces: `pipe_geometry_from_corners`, `project_point_to_pipe`, `measure_pipe_position`.

- [ ] Write failing tests for horizontal, tilted, reversed-corner, and clamped-touch geometry.
- [ ] Run tests and confirm missing-function failure.
- [ ] Implement the minimum pure geometry helpers.
- [ ] Run tests and confirm they pass.

### Task 2: Green pipe detection and dynamic state

**Files:**
- Modify: `main_two_touch_calibration.py`
- Test: `tests/test_green_pipe_geometry.py`

**Interfaces:**
- Consumes: pure geometry helpers from Task 1.
- Produces: `detect_green_pipe`, `update_pipe_state`, and a pipe state containing corners, centerline, midpoint, and length.

- [ ] Add selection/state tests for valid, missing, and short pipe observations.
- [ ] Confirm tests fail before implementation.
- [ ] Detect the largest elongated green Blob and derive its rotated frame.
- [ ] Reuse the last valid geometry only for a short miss window.
- [ ] Run tests.

### Task 3: One-touch target and centimeter control

**Files:**
- Modify: `main_two_touch_calibration.py`
- Test: `tests/test_green_pipe_geometry.py`

**Interfaces:**
- Consumes: current dynamic pipe state and current ball center.
- Produces: `ball_position_cm`, `target_position_cm`, `error_cm`, projected ball point, target point, and lateral error.

- [ ] Add failing tests proving O remains the midpoint and the target follows a moved/rotated pipe.
- [ ] Replace the two-touch calibration file format with normalized target parameter version 2.
- [ ] Make one release-touch select the target; retain long-hold recalibration.
- [ ] Update control measurement without adding filtering delay.
- [ ] Run tests.

### Task 4: Runtime and OSD integration

**Files:**
- Modify: `main_two_touch_calibration.py`

**Interfaces:**
- Consumes: RGB565 frame, dynamic pipe state, target state, ball state.
- Produces: rotated green frame, green centerline, O marker, target marker, projected ball marker, target-to-ball line, and centimeter labels.

- [ ] Detect pipe and ball from the same RGB565 snapshot.
- [ ] Keep KPU validation and H.264/RTSP behavior unchanged.
- [ ] Draw all dynamic geometry in display coordinates.
- [ ] Preserve per-frame UART/control publication.
- [ ] Compile-check the resulting Python source.

### Task 5: Verification and delivery

**Files:**
- Verify: `main_two_touch_calibration.py`
- Verify: `tests/test_green_pipe_geometry.py`

- [ ] Run all geometry tests.
- [ ] Run Python syntax compilation.
- [ ] Inspect the diff for accidental KPU/RTSP regressions.
- [ ] Copy/retain the verified file at the requested Desktop WorkSpace path.
