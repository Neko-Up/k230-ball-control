# K230 Fast Ball Telemetry Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Run the K230 ball/pipe vision path at 320×320 while preserving HD LCD/VLC output and continuously showing ball position, ball velocity, and measured vision FPS.

**Architecture:** Keep the GC2093 sensor and display/RTSP channels unchanged, but reduce the RGB888P AI channel to 320×320. Centralize coordinate scaling and runtime telemetry so the control/UART path uses centimeter measurements immediately while the LCD reads cached metrics without delaying detection.

**Tech Stack:** CanMV K230 MicroPython, `nn.ai2d`, KPU, traditional blob tracking, pytest AST-isolated unit tests.

## Global Constraints

- GC2093 remains 1920×1080@30.
- LCD and H.264/VLC retain the existing high-resolution channel.
- AI and traditional vision use a 320×320 RGB888P channel.
- UART control publication remains per valid control frame.
- Existing D36A, MS42CG, touch calibration, hybrid tracker, and RTSP behavior remain unchanged.

---

### Task 1: 320×320 Vision Geometry

**Files:**
- Modify: `main_two_touch_calibration.py`
- Test: `tests/test_main_structure.py`
- Test: `tests/test_tracking_state_machine.py`

**Interfaces:**
- Consumes: `OUT_RGB888P_WIDTH`, `OUT_RGB888P_HEIGH`, ROI constants, `ai_to_disp(x, y)`.
- Produces: a 320×320 AI channel and correctly scaled ROI/overlay coordinates.

- [ ] **Step 1: Write failing structural and geometry tests**

Assert that both AI dimensions are 320, the camera AI channel uses those constants, and every configured ROI remains inside the AI frame.

- [ ] **Step 2: Run the focused tests and confirm failure**

Run: `python -m pytest -p no:cacheprovider --basetemp C:\\Users\\Sayo\\Documents\\Codex\\2026-07-30\\chatgpt-conversation-6a69426a-cd94-83ee-872e\\work\\pytest-fast-vision tests/test_main_structure.py tests/test_tracking_state_machine.py -q`

Expected: failure because the AI width is currently 640 and the current ROIs are expressed in the old coordinate space.

- [ ] **Step 3: Implement the minimal resolution and ROI scaling change**

Set:

```python
OUT_RGB888P_WIDTH = 320
OUT_RGB888P_HEIGH = 320
```

Convert old 640×480 ROI constants with centralized scale helpers so bounds are valid and LCD mapping continues through `ai_to_disp()`.

- [ ] **Step 4: Run focused tests and confirm pass**

Run the command from Step 2 and require zero failures.

- [ ] **Step 5: Commit**

Commit message: `perf: run ball vision at 320 square`

### Task 2: Runtime FPS and Ball Telemetry

**Files:**
- Modify: `main_two_touch_calibration.py`
- Test: `tests/test_main_structure.py`
- Test: `tests/test_ball_control.py`

**Interfaces:**
- Consumes: measurement keys `ball_position_cm`, `velocity_cm_s`, and actual frame timestamps.
- Produces: `runtime_telemetry["vision_fps"]` and LCD labels `P`, `BV`, and `AI`.

- [ ] **Step 1: Write failing telemetry tests**

Add tests that require a cached real-time FPS value, require LCD strings for signed position/velocity/FPS, and verify speed remains computed in cm/s from real elapsed time.

- [ ] **Step 2: Run focused tests and confirm failure**

Run: `python -m pytest -p no:cacheprovider --basetemp C:\\Users\\Sayo\\Documents\\Codex\\2026-07-30\\chatgpt-conversation-6a69426a-cd94-83ee-872e\\work\\pytest-fast-vision tests/test_main_structure.py tests/test_ball_control.py -q`

Expected: failure because current FPS is only printed to serial and LCD does not show ball velocity or FPS.

- [ ] **Step 3: Implement cached FPS and LCD rendering**

Add a module-level dictionary:

```python
runtime_telemetry = {"vision_fps": 0.0}
```

Update it at each performance window and render:

```text
P:+0.00cm
BV:+0.0cm/s
AI:00.0FPS
```

Use invalid markers when control measurement is invalid; do not alter UART publication cadence.

- [ ] **Step 4: Run focused tests and confirm pass**

Run the command from Step 2 and require zero failures.

- [ ] **Step 5: Commit**

Commit message: `feat: show live ball telemetry on LCD`

### Task 3: Full Regression and Deployment Safety

**Files:**
- Inspect: `main_two_touch_calibration.py`
- Test: `tests/`

**Interfaces:**
- Consumes: completed 320×320 vision and telemetry changes.
- Produces: a syntactically valid deployable script with all tests green.

- [ ] **Step 1: Run syntax validation**

Run: `python -m py_compile main_two_touch_calibration.py`

- [ ] **Step 2: Run the complete test suite**

Run: `python -m pytest -p no:cacheprovider --basetemp C:\\Users\\Sayo\\Documents\\Codex\\2026-07-30\\chatgpt-conversation-6a69426a-cd94-83ee-872e\\work\\pytest-fast-vision -q`

- [ ] **Step 3: Check the patch**

Run: `git diff --check` and inspect `git diff --stat` plus `git status --short`. Do not add the unrelated untracked `main_green_pipe_tracking.py`.

- [ ] **Step 4: Commit and push**

Commit any direct regression fix separately, then push `agent/h264-circle-stream` to the private origin.

- [ ] **Step 5: Board validation handoff**

Instruct the user to hard-reset once to clear IRQ state left by the old crashing build, deploy the new script, and verify LCD/VLC, FPS, signed position, signed velocity, UART cadence, and soft rerun.
