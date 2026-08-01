# K230 Hybrid Fast Ball Tracking Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace per-frame KPU-only ball tracking with KPU reacquisition plus per-frame ROI Blob tracking, so LCD, UART, and K230-local control receive one shared low-latency predicted position while RTC output is removed.

**Architecture:** Camera channel 0 remains the LCD/WBC/H.264 source, channel 1 becomes a 640×360 RGB565 Blob source, and channel 2 remains the 640×360 RGBP888 KPU source. SEARCH and RECOVER run KPU every frame; TRACK publishes Blob control first on every frame and runs KPU only every sixth frame for validation. A three-sample velocity estimate and bounded 20–40 ms prediction feed one shared control state consumed by OSD, UART, and the future K230 servo hook.

**Tech Stack:** CanMV K230 MicroPython v1.8, `media.sensor`, `image.find_blobs`, KPU/nncase, WBC H.264/RTSP, UART, host-side Python AST tests.

## Global Constraints

- Preserve `/sdcard/mp_deployment_source/` and `/sdcard/mp_deployment_source/deploy_config.json`.
- Preserve UART at 115200 baud and exact valid format `X:{:+04d},Y:{:+04d}\n`; invalid format remains `X:----,Y:----\n`.
- Keep `K230_BALL`, its existing password, automatic startup channel selection, RTSP port 8554, and session `ball`.
- H.264/RTSP must remain outside the control path and must not stop detection, LCD, UART, or servo control on failure.
- Blob loss may be predicted for one frame only; continued loss must publish invalid state.
- Do not invent servo GPIO, PWM frequency, neutral pulse, or motion limits. Expose the shared control state to the existing/future K230 servo hook.
- Keep `main.py.autosave`, generated caches, and unrelated user files out of Git.

---

## File Map

- Modify `main.py`: camera channels, hybrid tracker, shared control state, OSD/UART integration, metrics, RTC removal.
- Modify `tests/test_main_structure.py`: pure algorithm tests and structural compatibility checks.
- Modify `docs/superpowers/specs/2026-07-30-k230-hybrid-fast-ball-tracking-design.md` only if board evidence requires a design correction.

### Task 1: Remove RTC and restore full-rate display/control cadence

**Files:**
- Modify: `main.py:68-99, 918-930, 1055-1070`
- Test: `tests/test_main_structure.py`

**Interfaces:**
- Consumes: existing `draw_osd(...)`, `format_deviation_msg(...)`.
- Produces: `OSD_EVERY_N_FRAMES = 1`; no `format_iso_time` or RTC startup warning.

- [ ] **Step 1: Write the failing structural test**

```python
def test_control_ui_is_full_rate_and_rtc_is_removed():
    function_names = {
        item.name for item in TREE.body if isinstance(item, ast.FunctionDef)
    }
    assert "format_iso_time" not in function_names
    text = SOURCE.read_text(encoding="utf-8")
    assert "System time:" not in text
    assert "RTC time is not calibrated" not in text
    assert "OSD_EVERY_N_FRAMES       = 1" in text
```

- [ ] **Step 2: Run the test and observe the expected failure**

Run: `python tests/test_main_structure.py`

Expected: FAIL because `format_iso_time` exists and OSD cadence is 2.

- [ ] **Step 3: Implement the minimal change**

Delete `format_iso_time`, the two startup time prints, and set:

```python
OSD_EVERY_N_FRAMES = 1
```

- [ ] **Step 4: Run tests and syntax validation**

Run:

```powershell
python tests/test_main_structure.py
python -m py_compile main.py
git diff --check
```

Expected: all commands exit 0.

- [ ] **Step 5: Commit**

```powershell
git add -- main.py tests/test_main_structure.py
git commit -m "Remove RTC and restore full-rate control display"
```

### Task 2: Add bounded three-sample motion prediction

**Files:**
- Modify: `main.py:130-150, 640-910`
- Test: `tests/test_main_structure.py`

**Interfaces:**
- Consumes: measured `(x, y, timestamp_ms)` tuples.
- Produces: `estimate_velocity(samples) -> (vx_px_ms, vy_px_ms)` and `predict_position(x, y, vx, vy, horizon_ms, max_shift_px) -> (pred_x, pred_y, clamped)`.

- [ ] **Step 1: Write failing pure-function tests**

```python
def test_three_sample_velocity_and_bounded_prediction():
    estimate_velocity = load_pure_function("estimate_velocity")
    predict_position = load_pure_function("predict_position")
    samples = [(100, 50, 0), (104, 50, 20), (110, 52, 40)]
    vx, vy = estimate_velocity(samples)
    assert round(vx, 3) == 0.25
    assert round(vy, 3) == 0.05
    assert predict_position(110, 52, vx, vy, 40, 16) == (120, 54, False)
    assert predict_position(110, 52, 1.0, 0.0, 40, 16) == (126, 52, True)
```

- [ ] **Step 2: Run and observe missing functions**

Run: `python tests/test_main_structure.py`

Expected: FAIL with `estimate_velocity is missing`.

- [ ] **Step 3: Implement pure motion helpers**

```python
PREDICTION_HORIZON_MS = 35
PREDICTION_MIN_MS = 20
PREDICTION_MAX_MS = 40
PREDICTION_MAX_SHIFT_PX = 16

def estimate_velocity(samples):
    if len(samples) < 2:
        return 0.0, 0.0
    velocities = []
    for index in range(1, len(samples)):
        x0, y0, t0 = samples[index - 1]
        x1, y1, t1 = samples[index]
        dt = t1 - t0
        if dt > 0:
            velocities.append(((x1 - x0) / dt, (y1 - y0) / dt))
    if not velocities:
        return 0.0, 0.0
    return (
        sum(item[0] for item in velocities) / len(velocities),
        sum(item[1] for item in velocities) / len(velocities),
    )

def predict_position(x, y, vx, vy, horizon_ms, max_shift_px):
    shift_x = vx * horizon_ms
    shift_y = vy * horizon_ms
    clamped = abs(shift_x) > max_shift_px or abs(shift_y) > max_shift_px
    shift_x = max(-max_shift_px, min(max_shift_px, shift_x))
    shift_y = max(-max_shift_px, min(max_shift_px, shift_y))
    return int(round(x + shift_x)), int(round(y + shift_y)), clamped
```

- [ ] **Step 4: Add the single shared control state**

```python
control_state = {
    "x": 0, "y": 0, "vx": 0.0, "vy": 0.0,
    "valid": False, "source": "none", "timestamp_ms": 0,
}
motion_samples = []
```

Add `publish_measurement(x, y, source, now_ms)` to keep the newest three samples, estimate velocity, apply bounded prediction, and update `control_state` atomically. Add `invalidate_control_state()` for loss protection.

- [ ] **Step 5: Run all checks and commit**

Run the Task 1 check commands. Expected: all exit 0.

```powershell
git add -- main.py tests/test_main_structure.py
git commit -m "Add bounded low-latency ball prediction"
```

### Task 3: Replace multi-target control selection with one KPU capture

**Files:**
- Modify: `main.py:180-280, 580-650, 700-750, 1030-1065`
- Test: `tests/test_main_structure.py`

**Interfaces:**
- Consumes: KPU `det_boxes` entries and the existing size/aspect constraints.
- Produces: `select_best_ai_ball(det_boxes) -> dict | None` with `box`, `cx`, `cy`, and `score`.

- [ ] **Step 1: Write the failing selector test**

```python
def test_single_ai_capture_selects_highest_valid_confidence():
    select_best_ai_ball = load_pure_function("select_best_ai_ball")
    detections = [
        [0, 0.40, 10, 10, 30, 30],
        [0, 0.90, 100, 100, 130, 130],
        [0, 0.99, 0, 0, 300, 10],
    ]
    result = select_best_ai_ball(detections)
    assert result["cx"] == 115
    assert result["cy"] == 115
    assert result["score"] == 0.90
```

- [ ] **Step 2: Run and observe the missing selector**

Run: `python tests/test_main_structure.py`

Expected: FAIL with `select_best_ai_ball is missing`.

- [ ] **Step 3: Implement a standalone single-target selector**

The helper must reject boxes outside `MIN_BOX_SIZE`, `MAX_BOX_SIZE`, and `MAX_ASPECT_RATIO`, then return only the highest-confidence valid ball. Keep the existing multi-track functions temporarily but remove them from the control call path.

- [ ] **Step 4: Route SEARCH measurements through the shared state**

After KPU post-processing:

```python
capture = select_best_ai_ball(det_boxes)
if capture is not None:
    publish_measurement(
        capture["cx"], capture["cy"], "kpu", time.ticks_ms())
```

LCD and UART must use `control_state`, not a separate smoothed track box.

- [ ] **Step 5: Verify and commit**

Run all checks, then:

```powershell
git add -- main.py tests/test_main_structure.py
git commit -m "Use single-target KPU ball capture"
```

### Task 4: Add channel 1 ROI Blob tracking

**Files:**
- Modify: `main.py:30-55, 120-150, 950-990`
- Test: `tests/test_main_structure.py`

**Interfaces:**
- Consumes: channel 1 RGB565 image, prior predicted center, fixed rod ROI.
- Produces: `select_blob_candidate(candidates, expected_x, expected_y) -> dict | None` and `detect_blob_measurement(img, dynamic_roi) -> dict | None`.

- [ ] **Step 1: Write failing Blob candidate tests**

```python
def test_blob_candidate_prefers_nearest_valid_ball():
    select_blob_candidate = load_pure_function("select_blob_candidate")
    candidates = [
        {"x": 90, "y": 180, "w": 20, "h": 18, "pixels": 240},
        {"x": 200, "y": 180, "w": 22, "h": 20, "pixels": 300},
        {"x": 105, "y": 180, "w": 80, "h": 5, "pixels": 300},
    ]
    result = select_blob_candidate(candidates, 100, 190)
    assert result["x"] == 90
```

- [ ] **Step 2: Run and observe the missing selector**

Run: `python tests/test_main_structure.py`

Expected: FAIL with `select_blob_candidate is missing`.

- [ ] **Step 3: Add explicit Blob configuration**

```python
BLOB_THRESHOLDS = [(0, 70, -20, 20, -20, 20)]
BLOB_GLOBAL_ROI = (0, 110, 640, 140)
BLOB_ROI_HALF_WIDTH = 96
BLOB_MIN_PIXELS = 40
BLOB_MAX_PIXELS = 1600
BLOB_MAX_ASPECT_RATIO = 1.8
BLOB_MAX_CENTER_DISTANCE = 80
```

- [ ] **Step 4: Configure the dedicated channel before `MediaManager.init()`**

```python
sensor.set_framesize(
    width=OUT_RGB888P_WIDTH, height=OUT_RGB888P_HEIGH,
    chn=CAM_CHN_ID_1)
sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_1)
```

If channel 1 initialization fails on the target firmware, catch the error, print `Blob channel unavailable; KPU fallback active`, and keep pure-KPU operation.

- [ ] **Step 5: Implement ROI detection using CanMV `find_blobs`**

```python
blobs = img.find_blobs(
    BLOB_THRESHOLDS,
    roi=dynamic_roi,
    pixels_threshold=BLOB_MIN_PIXELS,
    area_threshold=BLOB_MIN_PIXELS,
    merge=False)
```

Convert each blob to a plain dictionary before calling `select_blob_candidate`. Do not retain image or blob objects after the frame.

- [ ] **Step 6: Verify and commit**

Run all checks, then:

```powershell
git add -- main.py tests/test_main_structure.py
git commit -m "Add dedicated ROI Blob tracking channel"
```

### Task 5: Implement SEARCH/TRACK/RECOVER hybrid state machine

**Files:**
- Modify: `main.py:80-150, 990-1090`
- Test: `tests/test_main_structure.py`

**Interfaces:**
- Consumes: KPU captures, Blob captures, current tracker counters.
- Produces: `hybrid_transition(state, blob_valid, ai_valid, blob_misses, ai_failures) -> str` with `SEARCH`, `TRACK`, or `RECOVER`.

- [ ] **Step 1: Write failing transition tests**

```python
def test_hybrid_tracking_transitions():
    transition = load_pure_function("hybrid_transition")
    assert transition("SEARCH", False, True, 0, 0) == "TRACK"
    assert transition("TRACK", False, False, 1, 0) == "TRACK"
    assert transition("TRACK", False, False, 2, 0) == "RECOVER"
    assert transition("TRACK", True, False, 0, 2) == "RECOVER"
    assert transition("RECOVER", False, True, 0, 0) == "TRACK"
```

- [ ] **Step 2: Run and observe the missing transition function**

Run: `python tests/test_main_structure.py`

Expected: FAIL with `hybrid_transition is missing`.

- [ ] **Step 3: Implement state constants and transition helper**

```python
TRACK_SEARCH = "SEARCH"
TRACK_ACTIVE = "TRACK"
TRACK_RECOVER = "RECOVER"
AI_VALIDATE_INTERVAL = 6
BLOB_LOST_TO_RECOVER = 2
AI_FAILURES_TO_RECOVER = 2
PREDICT_ONLY_MAX_FRAMES = 1
```

The pure transition helper must implement the exact truth table from the test.

- [ ] **Step 4: Integrate control-first scheduling**

For each loop:

```text
1. Snapshot channel 1 when Blob is available.
2. Detect Blob and immediately publish state, update servo hook, UART, and OSD.
3. In TRACK, run KPU only when frame_counter % 6 == 0.
4. In SEARCH/RECOVER, run KPU every frame.
5. Apply state transition counters.
6. Permit one predicted-only frame, then invalidate.
```

Do not place Blob publication after KPU inference on validation frames.

- [ ] **Step 5: Add source-colored circular OSD**

Use yellow for SEARCH/KPU capture, green for TRACK/Blob, and red for RECOVER. Draw no ball circle when `control_state["valid"]` is false.

- [ ] **Step 6: Verify and commit**

Run all checks, then:

```powershell
git add -- main.py tests/test_main_structure.py
git commit -m "Run hybrid ball tracking state machine"
```

### Task 6: Add low-rate latency metrics and preserve failure isolation

**Files:**
- Modify: `main.py:95-110, 1000-1090`
- Test: `tests/test_main_structure.py`

**Interfaces:**
- Consumes: Blob/KPU processing durations and tracker events.
- Produces: one metrics line every 60 control frames.

- [ ] **Step 1: Write a structural failure-isolation test**

Assert that RTSP startup remains inside its existing `try/except`, and that neither `publish_measurement`, UART writes, nor the K230 servo hook is called from the RTSP thread.

- [ ] **Step 2: Add counters without per-frame allocation**

Maintain scalar counters:

```python
blob_frame_count = 0
blob_total_ms = 0
kpu_validation_count = 0
kpu_total_ms = 0
blob_loss_count = 0
kpu_reacquire_count = 0
prediction_clamp_count = 0
```

Print every 60 control frames:

```text
TRACK:<state> CTRL:<fps> Blob:<avg_ms> KPU:<avg_ms> Lost:<n> Reacq:<n> Clamp:<n>
```

- [ ] **Step 3: Run compatibility checks**

Verify UART format, model paths, Wi-Fi constants, RTSP constants, full-rate OSD, and absence of RTC text with `tests/test_main_structure.py`.

- [ ] **Step 4: Commit**

```powershell
git add -- main.py tests/test_main_structure.py
git commit -m "Measure hybrid tracking latency"
```

### Task 7: K230 board validation and GitHub publication

**Files:**
- Modify: `main.py` only for evidence-driven threshold or compatibility corrections.
- Modify: `tests/test_main_structure.py` for every correction.

**Interfaces:**
- Consumes: K230 serial logs, LCD observation, UART capture, VLC stream.
- Produces: verified board configuration and updated draft PR.

- [ ] **Step 1: Run fresh desktop verification**

```powershell
python tests/test_main_structure.py
python -m py_compile main.py
git diff --check
git status -sb
```

Expected: tests print `tests: OK`, syntax exits 0, no diff errors, and only intended branch state appears.

- [ ] **Step 2: Deploy to K230 and collect evidence**

Confirm logs contain Blob control FPS, KPU validation timing, TRACK state, Wi-Fi IP, and the RTSP URL. Confirm RTC output is absent.

- [ ] **Step 3: Tune only measured Blob constants**

Use the real board image to adjust `BLOB_THRESHOLDS`, area limits, global ROI, and dynamic ROI width one variable at a time. For each change, record false positives, misses, and Blob average processing time.

- [ ] **Step 4: Run functional tests**

Verify:

1. Slow ball movement: no coordinate jitter large enough to move the servo unnecessarily.
2. Fast movement: LCD circle and UART stay within one camera frame.
3. One-frame obstruction: prediction bridges the gap without a servo spike.
4. Sustained obstruction: state becomes invalid rather than driving stale position.
5. Lighting change: KPU reacquires after Blob loss.
6. VLC recording: H.264 stream continues without affecting control FPS.

- [ ] **Step 5: Commit evidence-driven corrections**

```powershell
git add -- main.py tests/test_main_structure.py
git commit -m "Tune hybrid tracker on K230"
```

Skip this commit if no correction is required.

- [ ] **Step 6: Push and update the existing draft PR**

```powershell
git push
gh pr view 1 --json url,isDraft,state,headRefName,baseRefName
```

Expected: branch `agent/h264-circle-stream` is pushed and draft PR #1 remains open against `main`.

---

## Plan Self-Review

- Spec coverage: camera channels, AI/Blob roles, state machine, velocity/prediction, shared state, OSD/UART, RTSP isolation, RTC removal, metrics, loss behavior, board tuning, and publication are covered.
- Placeholder scan: no TBD/TODO or unspecified implementation steps remain. Hardware-only Blob thresholds have explicit safe initial values and an evidence-driven tuning procedure.
- Interface consistency: all later tasks use the same `control_state`, `estimate_velocity`, `predict_position`, `select_best_ai_ball`, `select_blob_candidate`, and `hybrid_transition` names defined by earlier tasks.
