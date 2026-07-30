# -*- coding: utf-8 -*-
#
# CanMV K230 钢珠定位 — 新模型 + 动态绿色管轴 + 单触摸目标 + UART
# 模型文件放在: /sdcard/mp_deployment_source/
#
# 工作流程：
#   1. 每帧识别 25cm 绿色管壁，旋转框中点自动定义为官方中心 O
#   2. 首次启动触摸一次目标点，保存其沿管长的比例；长按 2 秒可重选
#   3. 将钢球投影到动态管中线，输出球坐标、目标坐标与控制误差

import gc
import math
import time
import uctypes
import network
import _thread
import multimedia as mm

import aicube
import image
import nncase_runtime as nn
import ujson
import uos
import ulab.numpy as np
from libs.PipeLine import ScopedTiming
from libs.Utils import *
from machine import FPIOA, UART, TOUCH, PWM, Pin, Timer
from media.display import *
from media.media import *
from media.sensor import *
from media.vencoder import *
from _media import Display as WBCDisplay


# ============================================================
# 显示 / 摄像头
# ============================================================
display_mode = "lcd"
CAMERA_CSI_ID = 2             # Hiwonder 固件中的传感器名为 gc2093_csi2
CAMERA_BOOT_DELAY_MS = 3000   # 脱机上电时等待摄像头电源/时钟稳定
CAMERA_PROBE_RETRIES = 3
if display_mode == "lcd":
    DISPLAY_WIDTH  = ALIGN_UP(800, 16)
    DISPLAY_HEIGHT = 480
else:
    DISPLAY_WIDTH  = ALIGN_UP(1920, 16)
    DISPLAY_HEIGHT = 1080

OUT_RGB888P_WIDTH  = ALIGN_UP(640, 16)
OUT_RGB888P_HEIGH  = 360

GEOM_CENTER_X = OUT_RGB888P_WIDTH // 2
GEOM_CENTER_Y = OUT_RGB888P_HEIGH // 2

root_path   = "/sdcard/mp_deployment_source/"
config_path = "/sdcard/mp_deployment_source/deploy_config.json"
debug_mode  = 0

# ============================================================
# Wi-Fi + VLC H.264/RTSP 图传
# ============================================================
WIFI_MODE         = "ap"       # "ap": K230 开热点；"sta": 连接现有 Wi-Fi
WIFI_AP_SSID      = "K230_BALL"
WIFI_AP_PASSWORD  = "12345678" # 至少 8 位
WIFI_AP_CHANNEL   = 0   # 0=启动时自动选择，选定后固定
WIFI_STA_SSID     = "YOUR_WIFI"
WIFI_STA_PASSWORD = "YOUR_PASSWORD"
WIFI_CONNECT_MS   = 15000

RTSP_PORT         = 8554
RTSP_SESSION      = "ball"
H264_BITRATE      = 1000       # Kbit/s
H264_FPS          = 30         # 匹配摄像头/WBC，避免消费过慢形成帧积压
H264_GOP          = 15

# ============================================================
CAL_WAIT_TARGET = "WAIT_TARGET"
CAL_READY = "READY"
TOUCH_RELEASE_EVENT = 2
TOUCH_RECALIBRATE_HOLD_MS = 2000
TOUCH_POLL_EVERY_N_FRAMES = 3

# ============================================================
# 检测 & 跟踪参数
# ============================================================
DETECT_CONF_THRESHOLD    = 0.28
HIGH_CONF_THRESHOLD      = 0.58
CONFIRM_FRAMES           = 1
HOLD_FRAMES              = 1
SMOOTH_ALPHA             = 0.95
MAX_TRACKS               = 12
MAX_DETECTIONS_PER_FRAME = 25
PRINT_EVERY_N_FRAMES     = 30
GC_EVERY_N_FRAMES        = 60      # 降低强制 GC 频率，减少周期性停顿
PERF_EVERY_N_FRAMES      = 60      # 低频统计实际 AI 主循环性能
METRICS_EVERY_N_CONTROL_FRAMES = 60
OSD_EVERY_N_FRAMES       = 1       # 控制/UART 全帧运行，叠加层每帧刷新
DISPLAY_LABEL            = "gz"
MERGED_CLASS_ID          = 0       # 新模型只有 gangqiu 一个类
MIN_BOX_SIZE             = 4
MAX_BOX_SIZE             = 170
MAX_ASPECT_RATIO         = 1.8
AI_ROD_ROI               = (0, 110, 640, 140)
DEDUP_IOU_THRESHOLD      = 0.35
DEDUP_CENTER_RATIO       = 0.55
TRACK_MERGE_IOU_THRESHOLD = 0.25
TRACK_MERGE_CENTER_RATIO  = 0.75
PREDICTION_HORIZON_MS     = 35
PREDICTION_MIN_MS         = 20
PREDICTION_MAX_MS         = 40
PREDICTION_MAX_SHIFT_PX   = 16
MOTION_RESET_JUMP_PX      = 48
VELOCITY_REVERSAL_MIN_SPEED = 0.01
BLOB_THRESHOLDS           = [(0, 70, -20, 20, -20, 20)]
BLOB_GLOBAL_ROI           = (0, 110, 640, 140)
BLOB_ROI_HALF_WIDTH       = 96
BLOB_MIN_PIXELS           = 40
BLOB_MAX_PIXELS           = 1600
BLOB_MAX_ASPECT_RATIO     = 1.8
BLOB_MAX_CENTER_DISTANCE  = 80
PIPE_GREEN_THRESHOLDS     = [(30, 85, -70, -8, -25, 45)]
PIPE_GLOBAL_ROI           = (0, 70, 640, 220)
PIPE_MIN_PIXELS           = 1200
PIPE_MIN_LENGTH_PX        = 180.0
PIPE_MIN_ASPECT_RATIO     = 3.0
PIPE_LENGTH_CM            = 25.0
PIPE_HOLD_MISSES          = 3
PIPE_SMOOTH_ALPHA         = 0.85
PIPE_LOCK_FIRST           = True
TRACK_SEARCH              = "SEARCH"
TRACK_ACTIVE              = "TRACK"
TRACK_RECOVER             = "RECOVER"
AI_VALIDATE_INTERVAL      = 6
BLOB_LOST_TO_RECOVER      = 2
AI_FAILURES_TO_RECOVER    = 2
PREDICT_ONLY_MAX_FRAMES   = 1
AI_BLOB_IDENTITY_MAX_DISTANCE = 80

# ============================================================
# 中值滤波
# ============================================================
MEDIAN_WINDOW = 1
pos_history_x = [0] * MEDIAN_WINDOW
pos_history_y = [0] * MEDIAN_WINDOW
pos_hist_idx  = 0
pos_hist_full = False

# ============================================================
# UART
# ============================================================
UART_BAUDRATE       = 115200
SEND_EVERY_N_FRAMES = 1
DEVIATION_DEADZONE  = 3

# D36A stepper: physical header pins 13/11/12 -> IO42/IO5/IO6.
STEPPER_STEP_IO = 42
STEPPER_PWM_CHANNEL = 0
STEPPER_DIR_IO = 5
STEPPER_EN_IO = 6
STEPPER_KP_ANGLE_DEG_PER_CM = 0.45
STEPPER_KD_ANGLE_DEG_PER_CM_S = 0.06
STEPPER_ANGLE_TRACK_HZ_PER_DEG = 1200.0
STEPPER_ANGLE_TOLERANCE_DEG = 0.03
STEPPER_DEADBAND_CM = 0.15
STEPPER_MIN_FREQUENCY_HZ = 150.0
STEPPER_MAX_FREQUENCY_HZ = 400.0
STEPPER_FREQUENCY_RAMP_HZ_S = 8000.0
STEPPER_PULSES_PER_ROD_DEG = 8.8889  # 1.8 deg motor, 1/16, direct drive
STEPPER_ANGLE_LIMIT_DEG = 2.0
STEPPER_DIRECTION_INVERT = True
STEPPER_VISION_TIMEOUT_MS = 150
STEPPER_WATCHDOG_TIMER_ID = -1  # software timer; media pipeline owns hard timers
STEPPER_ZERO_TOUCH_RECT = (210, 398, 380, 58)
STEPPER_ZERO_TOUCH_EVENT = TOUCH_RELEASE_EVENT

# One-touch target calibration. The official O is always pipe midpoint.
CALIBRATION_VERSION = 2
CALIBRATION_PATH = "/sdcard/ball_axis_calibration.json"
CALIBRATION_TEMP_PATH = "/sdcard/ball_axis_calibration.tmp"

tracks            = []
frame_counter     = 0
tracker_state     = TRACK_SEARCH
current_deviation = {
    "dx": 0, "dy": 0, "valid": False,
    "position_cm": 0.0, "velocity_cm_s": 0.0,
}
control_state = {
    "x": 0, "y": 0, "vx": 0.0, "vy": 0.0,
    "valid": False, "source": "none", "timestamp_ms": 0,
}
pipe_state = {
    "valid": False,
    "geometry": None,
    "misses": 0,
    "locked": False,
}
motion_samples = []
blob_frame_count = 0
blob_total_ms = 0
kpu_validation_count = 0
kpu_total_ms = 0
blob_loss_count = 0
kpu_reacquire_count = 0
prediction_clamp_count = 0


# ============================================================
# 工具函数
# ============================================================

def estimate_velocity(samples, ticks_diff_fn=None):
    if len(samples) < 2:
        return 0.0, 0.0
    velocities = []
    for index in range(1, len(samples)):
        x0, y0, t0 = samples[index - 1]
        x1, y1, t1 = samples[index]
        if ticks_diff_fn is None:
            dt = t1 - t0
        else:
            dt = ticks_diff_fn(t1, t0)
        if dt > 0:
            velocities.append(((x1 - x0) / dt, (y1 - y0) / dt))
    if not velocities:
        return 0.0, 0.0
    return (
        sum(item[0] for item in velocities) / len(velocities),
        sum(item[1] for item in velocities) / len(velocities),
    )


def new_stepper_control_state(now_ms=0):
    return {
        "zeroed": False,
        "frequency_hz": 0.0,
        "direction": 0,
        "motion_sign": 0,
        "estimated_angle_deg": 0.0,
        "target_angle_deg": 0.0,
        "last_update_ms": now_ms,
        "fault": "not_zeroed",
    }


def compute_stepper_command(
        error_cm, velocity_cm_s, measurement_valid, zeroed,
        now_ms, state, kp_angle_deg_per_cm, kd_angle_deg_per_cm_s,
        angle_track_hz_per_deg, angle_tolerance_deg, deadband_cm,
        min_frequency_hz, max_frequency_hz,
        frequency_ramp_hz_s, pulses_per_degree, angle_limit_deg,
        max_motion_ms,
        direction_invert=False, ticks_diff_fn=None):
    """Return the next safe D36A command without touching hardware."""
    if ticks_diff_fn is None:
        elapsed_ms = time.ticks_diff(now_ms, state["last_update_ms"])
    else:
        elapsed_ms = ticks_diff_fn(now_ms, state["last_update_ms"])
    elapsed_s = min(max(elapsed_ms, 0), max_motion_ms) / 1000.0

    previous_frequency = max(float(state.get("frequency_hz", 0.0)), 0.0)
    previous_sign = state.get("motion_sign", state.get("direction", 0))
    estimated_angle = float(state.get("estimated_angle_deg", 0.0))
    if pulses_per_degree > 0.0:
        estimated_angle += (
            previous_sign * previous_frequency * elapsed_s /
            pulses_per_degree)
    estimated_angle = max(
        -angle_limit_deg, min(angle_limit_deg, estimated_angle))

    result = {
        "zeroed": bool(zeroed),
        "enabled": False,
        "frequency_hz": 0.0,
        "direction": 0,
        "motion_sign": 0,
        "estimated_angle_deg": estimated_angle,
        "target_angle_deg": 0.0,
        "last_update_ms": now_ms,
        "fault": "none",
    }
    if not zeroed:
        result["fault"] = "not_zeroed"
        return result
    if not measurement_valid:
        result["fault"] = "vision_invalid"
        return result

    if abs(error_cm) <= deadband_cm:
        target_angle = 0.0
    else:
        target_angle = (
            kp_angle_deg_per_cm * error_cm -
            kd_angle_deg_per_cm_s * velocity_cm_s)
    target_angle = max(
        -angle_limit_deg, min(angle_limit_deg, target_angle))
    result["target_angle_deg"] = target_angle
    angle_error = target_angle - estimated_angle
    if abs(angle_error) <= angle_tolerance_deg:
        result["fault"] = "angle_deadband"
        return result

    logical_direction = 1 if angle_error > 0.0 else -1
    if ((estimated_angle >= angle_limit_deg and logical_direction > 0) or
            (estimated_angle <= -angle_limit_deg and logical_direction < 0)):
        result["fault"] = "angle_limit"
        return result

    target_frequency = max(
        min(abs(angle_error) * angle_track_hz_per_deg,
            max_frequency_hz),
        min_frequency_hz)
    max_delta = frequency_ramp_hz_s * elapsed_s
    if previous_sign != 0 and previous_sign != logical_direction:
        next_frequency = max(0.0, previous_frequency - max_delta)
        if next_frequency > 0.0:
            logical_direction = previous_sign
            result["fault"] = "reversing"
        else:
            result["fault"] = "reversing"
            return result
    elif target_frequency >= previous_frequency:
        next_frequency = min(
            target_frequency, previous_frequency + max_delta)
        if previous_frequency <= 0.0 and next_frequency > 0.0:
            next_frequency = max(next_frequency, min_frequency_hz)
    else:
        next_frequency = max(
            target_frequency, previous_frequency - max_delta)
    physical_direction = (-logical_direction if direction_invert
                          else logical_direction)
    result.update({
        "enabled": next_frequency > 0.0,
        "frequency_hz": next_frequency,
        "direction": physical_direction,
        "motion_sign": logical_direction,
    })
    return result


class D36AStepper:
    """Hardware-only D36A adapter; control decisions stay in pure code."""

    def __init__(self, fpioa):
        fpioa.set_function(STEPPER_STEP_IO, fpioa.PWM0, ie=0, oe=1)
        fpioa.set_function(STEPPER_DIR_IO, fpioa.GPIO5, ie=0, oe=1)
        fpioa.set_function(STEPPER_EN_IO, fpioa.GPIO6, ie=0, oe=1)
        self.dir_pin = Pin(
            STEPPER_DIR_IO, Pin.OUT, pull=Pin.PULL_NONE, drive=7)
        self.en_pin = Pin(
            STEPPER_EN_IO, Pin.OUT, pull=Pin.PULL_NONE, drive=7)
        self.en_pin.value(0)
        self.dir_pin.value(0)
        self.pwm = PWM(STEPPER_PWM_CHANNEL)
        self.pwm.freq(int(STEPPER_MIN_FREQUENCY_HZ))
        self.pwm.duty(0)
        self.running = False
        self.last_frequency_hz = 0
        self.last_direction = 0
        self.watchdog = Timer(STEPPER_WATCHDOG_TIMER_ID)
        self.watchdog_armed = False

    def _watchdog_expired(self, timer):
        self.watchdog_armed = False
        self.stop(disable=True, cancel_watchdog=False)

    def apply(self, command):
        if not command.get("enabled", False):
            disable = command.get("fault") in (
                "not_zeroed", "vision_invalid")
            self.stop(disable=disable)
            return
        frequency_hz = max(
            int(round(command["frequency_hz"])),
            int(STEPPER_MIN_FREQUENCY_HZ))
        direction = 1 if command["direction"] > 0 else -1
        if self.running and direction != self.last_direction:
            self.pwm.duty(0)
            self.running = False
        self.dir_pin.value(1 if direction > 0 else 0)
        if direction != self.last_direction:
            time.sleep_us(5)
            self.last_direction = direction
        if frequency_hz != self.last_frequency_hz:
            self.pwm.freq(frequency_hz)
            self.last_frequency_hz = frequency_hz
        self.en_pin.value(1)
        if not self.running:
            self.pwm.duty(50)
            self.running = True
        try:
            self.watchdog.init(
                mode=Timer.ONE_SHOT, period=STEPPER_VISION_TIMEOUT_MS,
                callback=self._watchdog_expired)
            self.watchdog_armed = True
        except Exception:
            self.watchdog_armed = False
            self.stop(disable=True, cancel_watchdog=False)
            raise

    def stop(self, disable=True, cancel_watchdog=True):
        if cancel_watchdog and self.watchdog_armed:
            try:
                self.watchdog.deinit()
            finally:
                # CanMV Timer objects cannot be initialized again after
                # deinit(); create a fresh software timer for the next move.
                self.watchdog = Timer(STEPPER_WATCHDOG_TIMER_ID)
                self.watchdog_armed = False
        try:
            if self.running:
                self.pwm.duty(0)
        finally:
            self.running = False
            self.last_frequency_hz = 0
            if disable:
                self.en_pin.value(0)
            else:
                self.en_pin.value(1)

    def deinit(self):
        self.stop(disable=True)
        self.pwm.deinit()


def is_velocity_reversal(previous_vx, previous_vy, next_vx, next_vy):
    min_speed_sq = (
        VELOCITY_REVERSAL_MIN_SPEED * VELOCITY_REVERSAL_MIN_SPEED)
    previous_speed_sq = previous_vx * previous_vx + previous_vy * previous_vy
    next_speed_sq = next_vx * next_vx + next_vy * next_vy
    if previous_speed_sq < min_speed_sq or next_speed_sq < min_speed_sq:
        return False
    return previous_vx * next_vx + previous_vy * next_vy < 0


def update_motion_history(samples, x, y, now_ms,
                          ticks_diff_fn=None, jump_threshold=None):
    if jump_threshold is None:
        jump_threshold = MOTION_RESET_JUMP_PX
    history = list(samples)
    reset_velocity = False
    if history:
        previous_x, previous_y, previous_ms = history[-1]
        delta_x = x - previous_x
        delta_y = y - previous_y
        if (delta_x * delta_x + delta_y * delta_y >
                jump_threshold * jump_threshold):
            reset_velocity = True
        else:
            if ticks_diff_fn is None:
                elapsed_ms = now_ms - previous_ms
            else:
                elapsed_ms = ticks_diff_fn(now_ms, previous_ms)
            if elapsed_ms > 0 and len(history) >= 2:
                previous_vx, previous_vy = estimate_velocity(
                    history, ticks_diff_fn)
                next_vx = delta_x / elapsed_ms
                next_vy = delta_y / elapsed_ms
                reset_velocity = is_velocity_reversal(
                    previous_vx, previous_vy, next_vx, next_vy)
    current_sample = (x, y, now_ms)
    if reset_velocity:
        return [current_sample], 0.0, 0.0, True
    history = (history + [current_sample])[-3:]
    vx, vy = estimate_velocity(history, ticks_diff_fn)
    return history, vx, vy, False


def predict_position(x, y, vx, vy, horizon_ms, max_shift_px):
    shift_x = vx * horizon_ms
    shift_y = vy * horizon_ms
    clamped = abs(shift_x) > max_shift_px or abs(shift_y) > max_shift_px
    shift_x = max(-max_shift_px, min(max_shift_px, shift_x))
    shift_y = max(-max_shift_px, min(max_shift_px, shift_y))
    return int(round(x + shift_x)), int(round(y + shift_y)), clamped


def hybrid_transition(state, blob_valid, ai_valid,
                      blob_misses, ai_failures):
    if state == TRACK_SEARCH or state == TRACK_RECOVER:
        if ai_valid:
            return TRACK_ACTIVE
        return state
    if (blob_misses >= BLOB_LOST_TO_RECOVER or
            ai_failures >= AI_FAILURES_TO_RECOVER):
        return TRACK_RECOVER
    return TRACK_ACTIVE


def ai_capture_counters(state, blob_valid, blob_misses):
    if state != TRACK_ACTIVE or not blob_valid:
        blob_misses = 0
    return blob_misses, 0, 0


def kpu_blob_identity_match(blob_x, blob_y, capture):
    if capture is None:
        return False
    delta_x = capture["cx"] - blob_x
    delta_y = capture["cy"] - blob_y
    return (delta_x * delta_x + delta_y * delta_y <=
            AI_BLOB_IDENTITY_MAX_DISTANCE *
            AI_BLOB_IDENTITY_MAX_DISTANCE)


def kpu_validation_outcome(published_control, blob_x, blob_y,
                           capture, ai_failures):
    valid = kpu_blob_identity_match(blob_x, blob_y, capture)
    if valid:
        return published_control, True, 0
    return published_control, False, ai_failures + 1


def apply_kpu_validation_anchor(published_control, blob_x, blob_y,
                                capture, motion_history,
                                jump_threshold=None):
    if not kpu_blob_identity_match(blob_x, blob_y, capture):
        return published_control, None, motion_history
    if jump_threshold is None:
        jump_threshold = MOTION_RESET_JUMP_PX
    delta_x = capture["cx"] - blob_x
    delta_y = capture["cy"] - blob_y
    if (delta_x * delta_x + delta_y * delta_y >
            jump_threshold * jump_threshold):
        motion_history = []
    return (
        published_control,
        {"x": capture["cx"], "y": capture["cy"]},
        motion_history,
    )


def hybrid_frame_actions(state, frame_number, blob_available, blob_valid=True):
    if not blob_available:
        return ("kpu",)
    if state != TRACK_ACTIVE:
        return ("kpu",)
    if not blob_valid:
        return ("kpu",)
    if frame_number % AI_VALIDATE_INTERVAL == 0:
        return ("blob_control", "kpu")
    return ("blob_control",)


def prediction_for_missed_frame(current_state, predicted_frames, now_ms):
    global prediction_clamp_count
    if (not current_state["valid"] or
            predicted_frames >= PREDICT_ONLY_MAX_FRAMES):
        return None, predicted_frames
    pred_x, pred_y, clamped = predict_position(
        current_state["x"], current_state["y"],
        current_state["vx"], current_state["vy"],
        PREDICTION_HORIZON_MS, PREDICTION_MAX_SHIFT_PX)
    if clamped:
        prediction_clamp_count += 1
    return ({
        "x": pred_x, "y": pred_y,
        "vx": current_state["vx"], "vy": current_state["vy"],
        "valid": True, "source": "predict", "timestamp_ms": now_ms,
    }, predicted_frames + 1)


def publish_measurement(x, y, source, now_ms):
    global control_state, motion_samples, prediction_clamp_count
    motion_samples, vx, vy, _ = update_motion_history(
        motion_samples, x, y, now_ms,
        time.ticks_diff, MOTION_RESET_JUMP_PX)
    pred_x, pred_y, clamped = predict_position(
        x, y, vx, vy, PREDICTION_HORIZON_MS, PREDICTION_MAX_SHIFT_PX)
    if clamped:
        prediction_clamp_count += 1
    control_state = {
        "x": pred_x, "y": pred_y, "vx": vx, "vy": vy,
        "valid": True, "source": source, "timestamp_ms": now_ms,
    }


def tracking_metrics_report(
        frame_count, tracking_state, now_ms, start_ms,
        blob_frames, blob_ms, kpu_validations, kpu_ms,
        blob_losses, kpu_reacquires, prediction_clamps):
    if (frame_count <= 0 or
            frame_count % METRICS_EVERY_N_CONTROL_FRAMES != 0):
        return None
    elapsed_ms = time.ticks_diff(now_ms, start_ms)
    control_fps = 0.0
    if elapsed_ms > 0:
        control_fps = (
            METRICS_EVERY_N_CONTROL_FRAMES * 1000.0 / elapsed_ms)
    blob_avg_ms = 0.0
    if blob_frames > 0:
        blob_avg_ms = blob_ms * 1.0 / blob_frames
    kpu_avg_ms = 0.0
    if kpu_validations > 0:
        kpu_avg_ms = kpu_ms * 1.0 / kpu_validations
    return (
        "TRACK:{} CTRL:{:.1f} Blob:{:.1f} KPU:{:.1f} "
        "Lost:{} Reacq:{} Clamp:{}".format(
            tracking_state, control_fps, blob_avg_ms, kpu_avg_ms,
            blob_losses, kpu_reacquires, prediction_clamps),
        now_ms, 0, 0, 0, 0, 0, 0, 0,
    )


def reset_tracking_metrics_window(now_ms):
    return now_ms, 0, 0, 0, 0, 0, 0, 0


def invalidate_control_state():
    global control_state, motion_samples
    motion_samples = []
    control_state = {
        "x": 0, "y": 0, "vx": 0.0, "vy": 0.0,
        "valid": False, "source": "none", "timestamp_ms": 0,
    }


def select_blob_candidate(candidates, expected_x, expected_y):
    best_candidate = None
    best_distance = None
    for candidate in candidates:
        width = candidate["w"]
        height = candidate["h"]
        pixels = candidate["pixels"]
        if width <= 0 or height <= 0:
            continue
        if pixels < BLOB_MIN_PIXELS or pixels > BLOB_MAX_PIXELS:
            continue
        if max(width, height) / min(width, height) > BLOB_MAX_ASPECT_RATIO:
            continue
        center_x = candidate["x"] + width / 2
        center_y = candidate["y"] + height / 2
        distance = ((center_x - expected_x) ** 2 +
                    (center_y - expected_y) ** 2) ** 0.5
        if distance > BLOB_MAX_CENTER_DISTANCE:
            continue
        if best_distance is None or distance < best_distance:
            best_candidate = candidate
            best_distance = distance
    return best_candidate


def detect_blob_measurement(img, dynamic_roi,
                            expected_x=None, expected_y=None):
    blobs = img.find_blobs(
        BLOB_THRESHOLDS,
        roi=dynamic_roi,
        pixels_threshold=BLOB_MIN_PIXELS,
        area_threshold=BLOB_MIN_PIXELS,
        merge=False)
    candidates = []
    for blob in blobs:
        x, y, width, height = blob.rect()
        candidates.append({
            "x": x, "y": y, "w": width, "h": height,
            "pixels": blob.pixels(),
        })
    if expected_x is None:
        expected_x = dynamic_roi[0] + dynamic_roi[2] / 2
    if expected_y is None:
        expected_y = dynamic_roi[1] + dynamic_roi[3] / 2
    return select_blob_candidate(candidates, expected_x, expected_y)


def update_pipe_geometry_state(previous, observation,
                               hold_misses=3, smooth_alpha=0.85,
                               lock_first=False):
    if (lock_first and previous.get("locked") and
            previous.get("geometry") is not None):
        return {
            "valid": True,
            "geometry": previous["geometry"],
            "misses": 0,
            "locked": True,
        }
    if observation is None:
        misses = int(previous.get("misses", 0)) + 1
        return {
            "valid": previous.get("geometry") is not None and
                     misses <= hold_misses,
            "geometry": previous.get("geometry"),
            "misses": misses,
            "locked": bool(previous.get("locked", False)),
        }
    geometry = observation
    previous_geometry = previous.get("geometry")
    if (previous.get("valid") and previous_geometry is not None and
            0.0 < smooth_alpha < 1.0):
        blended_corners = []
        for old_point, new_point in zip(
                previous_geometry["corners"], observation["corners"]):
            blended_corners.append((
                old_point[0] * (1.0 - smooth_alpha) +
                new_point[0] * smooth_alpha,
                old_point[1] * (1.0 - smooth_alpha) +
                new_point[1] * smooth_alpha))
        blended = pipe_geometry_from_corners(blended_corners)
        if blended is not None:
            geometry = blended
    return {
        "valid": True,
        "geometry": geometry,
        "misses": 0,
        "locked": bool(lock_first),
    }


def pipe_corners_from_pose(center_x, center_y, ux, uy,
                           length_px, width_px):
    nx = -uy
    ny = ux
    half_length = length_px * 0.5
    half_width = width_px * 0.5
    return [
        (center_x - ux * half_length - nx * half_width,
         center_y - uy * half_length - ny * half_width),
        (center_x + ux * half_length - nx * half_width,
         center_y + uy * half_length - ny * half_width),
        (center_x + ux * half_length + nx * half_width,
         center_y + uy * half_length + ny * half_width),
        (center_x - ux * half_length + nx * half_width,
         center_y - uy * half_length + ny * half_width),
    ]


def green_pipe_geometry_from_blob(blob):
    try:
        corners = blob.min_corners()
    except Exception:
        x, y, width, height = blob.rect()
        center_x = x + width * 0.5
        center_y = y + height * 0.5
        try:
            rotation = float(blob.rotation())
        except Exception:
            rotation = 0.0 if width >= height else math.pi * 0.5
        ux = math.cos(rotation)
        uy = math.sin(rotation)
        corners = pipe_corners_from_pose(
            center_x, center_y, ux, uy,
            max(width, height), min(width, height))
    return pipe_geometry_from_corners(corners)


def detect_green_pipe(img):
    blobs = img.find_blobs(
        PIPE_GREEN_THRESHOLDS,
        roi=PIPE_GLOBAL_ROI,
        x_stride=4,
        y_stride=3,
        pixels_threshold=PIPE_MIN_PIXELS,
        area_threshold=PIPE_MIN_PIXELS,
        merge=True,
        margin=12)
    best_geometry = None
    best_score = None
    for blob in blobs:
        geometry = green_pipe_geometry_from_blob(blob)
        if geometry is None:
            continue
        if geometry["length_px"] < PIPE_MIN_LENGTH_PX:
            continue
        aspect = geometry["length_px"] / max(1.0, geometry["width_px"])
        if aspect < PIPE_MIN_ASPECT_RATIO:
            continue
        score = float(blob.pixels()) * aspect
        if best_score is None or score > best_score:
            best_geometry = geometry
            best_score = score
    return best_geometry


def snapshot_blob_channel(sensor, should_detect, dynamic_roi,
                          expected_x, expected_y, detect_pipe=True):
    if not should_detect:
        return None, None, True
    blob_img = None
    try:
        blob_img = sensor.snapshot(chn=CAM_CHN_ID_1, timeout=2000)
        ball_capture = detect_blob_measurement(
            blob_img, dynamic_roi, expected_x, expected_y)
        pipe_geometry = detect_green_pipe(blob_img) if detect_pipe else None
        return ball_capture, pipe_geometry, True
    except Exception as e:
        print("Blob channel unavailable; KPU fallback active:", e)
        return None, None, False
    finally:
        del blob_img


def should_snapshot_blob_channel(state, blob_available):
    # The same RGB565 frame feeds both fast ball tracking and the dynamic
    # green-pipe coordinate system, including before the first ball lock.
    return blob_available


def blob_tracking_roi(expected_x):
    global_x, global_y, global_width, global_height = BLOB_GLOBAL_ROI
    roi_width = min(global_width, BLOB_ROI_HALF_WIDTH * 2)
    roi_x = int(expected_x) - BLOB_ROI_HALF_WIDTH
    roi_x = max(global_x, min(global_x + global_width - roi_width, roi_x))
    return roi_x, global_y, roi_width, global_height


def configure_camera_sensor(sensor, enable_blob_channel):
    sensor.reset()
    sensor.set_hmirror(False)
    sensor.set_vflip(False)
    sensor.set_framesize(width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT)
    sensor.set_pixformat(PIXEL_FORMAT_YUV_SEMIPLANAR_420)
    sensor.set_framesize(
        width=OUT_RGB888P_WIDTH, height=OUT_RGB888P_HEIGH,
        chn=CAM_CHN_ID_2)
    sensor.set_pixformat(
        PIXEL_FORMAT_RGB_888_PLANAR, chn=CAM_CHN_ID_2)
    if enable_blob_channel:
        sensor.set_framesize(
            width=OUT_RGB888P_WIDTH, height=OUT_RGB888P_HEIGH,
            chn=CAM_CHN_ID_1)
        sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_1)
    return sensor


def create_camera_sensor(enable_blob_channel):
    sensor = None
    last_sensor_error = None
    for probe_attempt in range(1, CAMERA_PROBE_RETRIES + 1):
        try:
            sensor = Sensor(id=CAMERA_CSI_ID, fps=30)
            break
        except RuntimeError as e:
            last_sensor_error = e
            print("Camera probe {}/{} failed: {}".format(
                probe_attempt, CAMERA_PROBE_RETRIES, e))
            gc.collect()
            time.sleep_ms(1500)
    if sensor is None:
        raise last_sensor_error
    try:
        return configure_camera_sensor(sensor, enable_blob_channel)
    except BaseException:
        cleanup_camera_start(sensor, False, False)
        raise


def cleanup_camera_start(sensor, display_started, media_attempted):
    if sensor is not None:
        try:
            sensor.stop(is_del=True)
        except TypeError:
            try:
                sensor.stop()
            except BaseException:
                pass
        except BaseException:
            pass
    if display_started:
        try:
            Display.deinit()
        except BaseException:
            pass
    if media_attempted:
        try:
            MediaManager.deinit()
        except BaseException:
            pass


def cleanup_runtime_resources(rtsp_server, sensor, tensor_holder):
    try:
        if rtsp_server is not None and rtsp_server.running:
            rtsp_server.stop()
    except BaseException:
        pass
    try:
        if sensor is not None:
            sensor.stop()
    except BaseException:
        pass
    try:
        Display.deinit()
    except BaseException:
        pass
    try:
        MediaManager.deinit()
    except BaseException:
        pass
    try:
        tensor_holder[0] = None
    except BaseException:
        pass
    try:
        gc.collect()
    except BaseException:
        pass
    try:
        nn.shrink_memory_pool()
    except BaseException:
        pass


def start_camera_pipeline(enable_blob_channel):
    sensor = None
    display_started = False
    media_attempted = False
    try:
        sensor = create_camera_sensor(enable_blob_channel)
        sensor_bind_info = sensor.bind_info(x=0, y=0, chn=CAM_CHN_ID_0)
        Display.bind_layer(**sensor_bind_info, layer=Display.LAYER_VIDEO1)
        display_started = True
        if display_mode == "lcd":
            Display.init(Display.ST7701, to_ide=False)
        else:
            Display.init(Display.LT9611, to_ide=False)
        osd_img = image.Image(
            DISPLAY_WIDTH, DISPLAY_HEIGHT, image.ARGB8888)
        rtsp_server = LowLatencyRtspH264Server(
            Display.width(), Display.height(), RTSP_PORT, RTSP_SESSION)
        media_attempted = True
        MediaManager.init()
        sensor.run()
        return sensor, osd_img, rtsp_server
    except BaseException:
        cleanup_camera_start(sensor, display_started, media_attempted)
        raise


def start_camera_with_blob_fallback(start_attempt):
    try:
        sensor, osd_img, rtsp_server = start_attempt(True)
        return sensor, osd_img, rtsp_server, True
    except BaseException:
        print("Blob channel unavailable; KPU fallback active")
        sensor, osd_img, rtsp_server = start_attempt(False)
        return sensor, osd_img, rtsp_server, False


def two_side_pad_param(input_size, output_size):
    ratio_w = output_size[0] / input_size[0]
    ratio_h = output_size[1] / input_size[1]
    ratio   = min(ratio_w, ratio_h)
    new_w   = int(ratio * input_size[0])
    new_h   = int(ratio * input_size[1])
    dw      = (output_size[0] - new_w) / 2
    dh      = (output_size[1] - new_h) / 2
    top     = int(round(dh - 0.1))
    bottom  = int(round(dh + 0.1))
    left    = int(round(dw - 0.1))
    right   = int(round(dw - 0.1))
    return top, bottom, left, right


def read_deploy_config(cfg_path):
    with open(cfg_path, "r") as f:
        try:
            return ujson.load(f)
        except ValueError as e:
            print("JSON error:", e)
            return {}


def box_center(box):
    return (box[0] + box[2]) / 2, (box[1] + box[3]) / 2


def box_size(box):
    return max(1, box[2] - box[0]), max(1, box[3] - box[1])


def center_distance(box_a, box_b):
    ax, ay = box_center(box_a)
    bx, by = box_center(box_b)
    return ((ax - bx) ** 2 + (ay - by) ** 2) ** 0.5


def box_iou(box_a, box_b):
    x1, y1 = max(box_a[0], box_b[0]), max(box_a[1], box_b[1])
    x2, y2 = min(box_a[2], box_b[2]), min(box_a[3], box_b[3])
    iw, ih = max(0, x2 - x1), max(0, y2 - y1)
    inter = iw * ih
    if inter <= 0:
        return 0
    aw, ah = box_size(box_a)
    bw, bh = box_size(box_b)
    union = aw * ah + bw * bh - inter
    return inter / union if union > 0 else 0


def is_duplicate_box(box_a, box_b):
    if box_iou(box_a, box_b) >= DEDUP_IOU_THRESHOLD:
        return True
    dist = center_distance(box_a, box_b)
    aw, ah = box_size(box_a)
    bw, bh = box_size(box_b)
    limit = max(min(max(aw, ah), max(bw, bh)) * DEDUP_CENTER_RATIO, 10)
    return dist <= limit


def is_duplicate_track(box_a, box_b):
    if box_iou(box_a, box_b) >= TRACK_MERGE_IOU_THRESHOLD:
        return True
    dist = center_distance(box_a, box_b)
    aw, ah = box_size(box_a)
    bw, bh = box_size(box_b)
    limit = max(min(max(aw, ah), max(bw, bh)) * TRACK_MERGE_CENTER_RATIO, 12)
    return dist <= limit


def dedup_detections(detections):
    result = []
    for det in detections:
        dup = False
        for kept in result:
            if is_duplicate_box(det["box"], kept["box"]):
                dup = True
                break
        if not dup:
            result.append(det)
    return result


def merge_tracks(tlist):
    merged = []
    ordered = sorted(tlist,
        key=lambda t: (t["updated"], t["score"], t["hits"], -t["lost"]),
        reverse=True)
    for trk in ordered:
        dup = False
        for kept in merged:
            if is_duplicate_track(trk["box"], kept["box"]):
                dup = True
                if trk["score"] > kept["score"]:
                    kept["box"]  = trk["box"]
                    kept["score"] = trk["score"]
                kept["hits"]    = max(kept["hits"], trk["hits"])
                kept["lost"]    = min(kept["lost"], trk["lost"])
                kept["updated"] = kept["updated"] or trk["updated"]
                break
        if not dup:
            merged.append(trk)
    return merged


def match_limit(box):
    w, h = box_size(box)
    return max(24, min(80, max(w, h) * 1.6))


def valid_ball_box(box):
    w, h = box_size(box)
    if w < MIN_BOX_SIZE or h < MIN_BOX_SIZE:
        return False
    if w > MAX_BOX_SIZE or h > MAX_BOX_SIZE:
        return False
    if max(w, h) / min(w, h) > MAX_ASPECT_RATIO:
        return False
    return True


def select_best_ai_ball(det_boxes):
    best_capture = None
    for det in det_boxes or []:
        score = float(det[1])
        x1 = float(det[2])
        y1 = float(det[3])
        x2 = float(det[4])
        y2 = float(det[5])
        width = x2 - x1
        height = y2 - y1
        if width < MIN_BOX_SIZE or height < MIN_BOX_SIZE:
            continue
        if width > MAX_BOX_SIZE or height > MAX_BOX_SIZE:
            continue
        if max(width, height) / min(width, height) > MAX_ASPECT_RATIO:
            continue
        center_x = (x1 + x2) / 2
        center_y = (y1 + y2) / 2
        roi_x, roi_y, roi_width, roi_height = AI_ROD_ROI
        if (center_x < roi_x or center_x >= roi_x + roi_width or
                center_y < roi_y or center_y >= roi_y + roi_height):
            continue
        if best_capture is None or score > best_capture["score"]:
            best_capture = {
                "box": [x1, y1, x2, y2],
                "cx": int(center_x),
                "cy": int(center_y),
                "score": score,
            }
    return best_capture


def smooth_box(old_box, new_box):
    a = SMOOTH_ALPHA
    return [
        old_box[0] * (1 - a) + new_box[0] * a,
        old_box[1] * (1 - a) + new_box[1] * a,
        old_box[2] * (1 - a) + new_box[2] * a,
        old_box[3] * (1 - a) + new_box[3] * a,
    ]


# ============================================================
# Wi-Fi
# ============================================================

def wifi_scan_channel_rssi(item):
    if hasattr(item, "channel") and hasattr(item, "rssi"):
        return int(item.channel), int(item.rssi)
    if isinstance(item, dict):
        return int(item["channel"]), int(item["rssi"])
    return int(item[2]), int(item[3])


def scan_best_channel():
    """
    启动时选择一次最佳2.4G信道。
    不运行中切换，避免 RTSP 断流。
    """
    sta = None
    try:
        sta = network.WLAN(network.STA_IF)
        if not sta.active():
            sta.active(True)
        result = sta.scan()

        busy = {1: 0, 6: 0, 11: 0}

        for item in result:
            channel, rssi = wifi_scan_channel_rssi(item)
            if channel in busy:
                # RSSI越强，占用权重越高
                busy[channel] += max(0, 100 + rssi)

        best = min(busy, key=busy.get)
        print("Wi-Fi channel scan:", busy, "choose:", best)
        return best
    except BaseException as e:
        print("Wi-Fi scan failed:", e)
        return 6


def select_default_network_device(device_name):
    if hasattr(network, "set_default_dev"):
        if network.set_default_dev(device_name) is False:
            raise RuntimeError(
                "Failed to select network device {}".format(device_name))

def start_wifi():
    if WIFI_MODE == "ap":
        wlan = network.WLAN(network.AP_IF)
        if not wlan.active():
            wlan.active(True)
        try:
            ap_channel = WIFI_AP_CHANNEL
            if ap_channel == 0:
                ap_channel = scan_best_channel()
            wlan.config(ssid=WIFI_AP_SSID, key=WIFI_AP_PASSWORD,
                        channel=ap_channel)
        except TypeError:
            try:
                wlan.config(ssid=WIFI_AP_SSID, key=WIFI_AP_PASSWORD)
            except TypeError:
                wlan.config(ssid=WIFI_AP_SSID,
                            password=WIFI_AP_PASSWORD)
        time.sleep_ms(500)
        select_default_network_device("w1")
        print("Wi-Fi AP:", WIFI_AP_SSID, "password:", WIFI_AP_PASSWORD)
        print("Wi-Fi IP:", wlan.ifconfig()[0])
        return wlan

    wlan = network.WLAN(network.STA_IF)
    if not wlan.active():
        wlan.active(True)
    if not wlan.isconnected():
        print("Connecting Wi-Fi:", WIFI_STA_SSID)
        wlan.connect(WIFI_STA_SSID, WIFI_STA_PASSWORD)
        start_ms = time.ticks_ms()
        while not wlan.isconnected():
            if time.ticks_diff(time.ticks_ms(), start_ms) >= WIFI_CONNECT_MS:
                raise RuntimeError("Wi-Fi connect timeout")
            time.sleep_ms(200)
    select_default_network_device("w0")
    print("Wi-Fi connected, IP:", wlan.ifconfig()[0])
    return wlan


# ============================================================
# VLC H.264/RTSP（WBC 合成画面 + 硬件编码）
# ============================================================

def rtsp_call_succeeded(result):
    # 官方 CanMV API 返回 None；部分兼容固件沿用 C 风格的 0。
    return result is None or result == 0


class LowLatencyRtspH264Server:
    def __init__(self, width, height, port=RTSP_PORT,
                 session_name=RTSP_SESSION):
        self.width = ALIGN_UP(width, 16)
        self.height = height
        self.port = port
        self.session_name = session_name
        self.encoder = Encoder()
        self.venc_chn = VENC_CHN_ID_0
        self.channel_api = True
        self.rtsp = mm.rtsp_server()
        self.running = False
        self.thread_over = True
        self.encoder_created = False
        self.encoder_started = False
        self.rtsp_initialized = False

        # 编码缓冲必须在 MediaManager.init() 前配置。
        try:
            self.encoder.SetOutBufs(
                self.venc_chn, 16, self.width, self.height)
        except TypeError:
            self.channel_api = False
            self.encoder.SetOutBufs(16, self.width, self.height)

    def _encoder_create(self):
        attr = ChnAttrStr(
            self.encoder.PAYLOAD_TYPE_H264,
            self.encoder.H264_PROFILE_MAIN,
            self.width, self.height,
            bit_rate=H264_BITRATE, gopLen=H264_GOP)
        if self.channel_api:
            self.encoder.Create(self.venc_chn, attr)
        else:
            self.encoder.Create(attr)
            self.venc_chn = self.encoder.chn
        self.encoder_created = True

    def _encoder_start(self):
        if self.channel_api:
            self.encoder.Start(self.venc_chn)
        else:
            self.encoder.Start()
        self.encoder_started = True

    def _encoder_send_frame(self, frame_info):
        if self.channel_api:
            return self.encoder.SendFrame(self.venc_chn, frame_info, 100)
        return self.encoder.SendFrame(frame_info, 100)

    def _encoder_get_stream(self, stream):
        if self.channel_api:
            return self.encoder.GetStream(self.venc_chn, stream, 100)
        return self.encoder.GetStream(stream, 100)

    def _encoder_release_stream(self, stream):
        if self.channel_api:
            self.encoder.ReleaseStream(self.venc_chn, stream)
        else:
            self.encoder.ReleaseStream(stream)

    def _send_pack(self, stream, pack_idx):
        send_phy = getattr(
            self.rtsp, "rtspserver_sendvideodata_byphyaddr", None)
        if send_phy is not None and hasattr(stream, "phy_addr"):
            return send_phy(
                self.session_name, stream.phy_addr[pack_idx],
                stream.data_size[pack_idx], 1000)

        # 兼容未暴露物理地址发送接口的旧固件。
        payload = bytes(uctypes.bytearray_at(
            stream.data[pack_idx], stream.data_size[pack_idx]))
        return self.rtsp.rtspserver_sendvideodata(
            self.session_name, payload, len(payload), 1000)

    def _stream_loop(self):
        frame_interval_ms = max(1, 1000 // H264_FPS)
        next_frame_ms = time.ticks_ms()
        try:
            while self.running:
                now_ms = time.ticks_ms()
                wait_ms = time.ticks_diff(next_frame_ms, now_ms)
                if wait_ms > 0:
                    time.sleep_ms(wait_ms)
                next_frame_ms = time.ticks_add(
                    next_frame_ms, frame_interval_ms)

                frame_info = WBCDisplay.writeback_dump(100)
                if not frame_info:
                    continue
                if self._encoder_send_frame(frame_info) != 0:
                    continue

                stream = StreamData()
                if self._encoder_get_stream(stream) != 0:
                    continue
                try:
                    for pack_idx in range(stream.pack_cnt):
                        self._send_pack(stream, pack_idx)
                finally:
                    self._encoder_release_stream(stream)
        except BaseException as e:
            if self.running:
                print("RTSP stream thread stopped:", e)
        self.thread_over = True

    def get_url(self, host=None):
        if host:
            return "rtsp://{}:{}/{}".format(
                host, self.port, self.session_name)
        return self.rtsp.rtspserver_getrtspurl(self.session_name)

    def start(self, host=None):
        if self.running:
            return
        wbc_started = False
        try:
            self._encoder_create()
            init_result = self.rtsp.rtspserver_init(self.port)
            if not rtsp_call_succeeded(init_result):
                raise RuntimeError("RTSP failed to bind port {}".format(
                    self.port))
            self.rtsp_initialized = True
            session_result = self.rtsp.rtspserver_createsession(
                self.session_name,
                mm.multi_media_type.media_h264, False)
            if not rtsp_call_succeeded(session_result):
                raise RuntimeError("RTSP session creation failed")
            self.rtsp.rtspserver_start()
            self._encoder_start()
            if not WBCDisplay.writeback(True):
                raise RuntimeError("start WBC for RTSP failed")
            wbc_started = True

            self.running = True
            self.thread_over = False
            _thread.start_new_thread(self._stream_loop, ())
            print("H264 RTSP for VLC:", self.get_url(host))
        except BaseException:
            self.running = False
            if wbc_started:
                WBCDisplay.writeback(False)
            self._release_resources()
            raise

    def _release_resources(self):
        if self.encoder_started:
            try:
                if self.channel_api:
                    self.encoder.Stop(self.venc_chn)
                else:
                    self.encoder.Stop()
            except BaseException:
                pass
            self.encoder_started = False
        if self.encoder_created:
            try:
                if self.channel_api:
                    self.encoder.Destroy(self.venc_chn)
                else:
                    self.encoder.Destroy()
            except BaseException:
                pass
            self.encoder_created = False
        if self.rtsp_initialized:
            try:
                self.rtsp.rtspserver_stop()
            except BaseException:
                pass
            try:
                self.rtsp.rtspserver_deinit()
            except BaseException:
                pass
            self.rtsp_initialized = False

    def stop(self):
        self.running = False
        start_ms = time.ticks_ms()
        while (not self.thread_over and
               time.ticks_diff(time.ticks_ms(), start_ms) < 1500):
            time.sleep_ms(20)
        try:
            WBCDisplay.writeback(False)
        except BaseException:
            pass
        self._release_resources()


# ============================================================
# 中值滤波
# ============================================================

def median_filter_push(cx, cy):
    global pos_history_x, pos_history_y, pos_hist_idx, pos_hist_full
    pos_history_x[pos_hist_idx] = cx
    pos_history_y[pos_hist_idx] = cy
    pos_hist_idx = (pos_hist_idx + 1) % MEDIAN_WINDOW
    if pos_hist_idx == 0:
        pos_hist_full = True
    if not pos_hist_full:
        return cx, cy
    sx = sorted(pos_history_x)
    sy = sorted(pos_history_y)
    return sx[MEDIAN_WINDOW // 2], sy[MEDIAN_WINDOW // 2]


# ============================================================
# 跟踪
# ============================================================

def update_tracks(det_boxes):
    global tracks
    detections = []
    if det_boxes:
        for det in det_boxes:
            score = float(det[1])
            if score < DETECT_CONF_THRESHOLD:
                continue
            box = [float(det[2]), float(det[3]), float(det[4]), float(det[5])]
            if not valid_ball_box(box):
                continue
            detections.append({"score": score, "box": box})

    for trk in tracks:
        trk["updated"] = False

    detections.sort(key=lambda d: d["score"], reverse=True)
    detections = dedup_detections(detections)
    if len(detections) > MAX_DETECTIONS_PER_FRAME:
        detections = detections[:MAX_DETECTIONS_PER_FRAME]

    for det in detections:
        best_track = None
        best_dist  = 1000000
        for trk in tracks:
            if trk["updated"]:
                continue
            dist = center_distance(trk["box"], det["box"])
            if dist < best_dist and dist <= match_limit(trk["box"]):
                best_dist  = dist
                best_track = trk

        if best_track:
            best_track["box"]   = smooth_box(best_track["box"], det["box"])
            best_track["score"] = best_track["score"] * 0.6 + det["score"] * 0.4
            best_track["hits"]    += 1
            best_track["lost"]    = 0
            best_track["updated"] = True
        elif len(tracks) < MAX_TRACKS:
            tracks.append({
                "score": det["score"],
                "box": det["box"], "hits": 1, "lost": 0, "updated": True})

    kept = []
    for trk in tracks:
        if not trk["updated"]:
            trk["lost"] += 1
        if trk["lost"] <= HOLD_FRAMES:
            kept.append(trk)
    tracks = merge_tracks(kept)
    return tracks


def is_visible_track(trk):
    if trk["lost"] > 0 and trk["hits"] < CONFIRM_FRAMES + 1:
        return False
    if trk["score"] >= HIGH_CONF_THRESHOLD or trk["hits"] >= CONFIRM_FRAMES:
        return True
    return False


# ============================================================
# UART
# ============================================================

def format_deviation_msg(dx, dy, valid):
    if valid:
        out_dx = 0 if abs(dx) <= DEVIATION_DEADZONE else dx
        out_dy = 0 if abs(dy) <= DEVIATION_DEADZONE else dy
        out_dx = max(-999, min(999, out_dx))
        out_dy = max(-999, min(999, out_dy))
        msg = "X:{:+04d},Y:{:+04d}\n".format(out_dx, out_dy)
    else:
        msg = "X:----,Y:----\n"
    return msg.encode("utf-8")


def format_stepper_msg(command):
    return "M:{},R:{},F:{:04d},D:{:+d},A:{:+.2f},T:{:+.2f},E:{}\n".format(
        1 if command.get("zeroed", False) else 0,
        1 if command.get("enabled", False) else 0,
        int(round(command.get("frequency_hz", 0.0))),
        int(command.get("direction", 0)),
        float(command.get("estimated_angle_deg", 0.0)),
        float(command.get("target_angle_deg", 0.0)),
        command.get("fault", "unknown")).encode("utf-8")


# ============================================================
# 坐标转换
# ============================================================

def ai_to_disp(ax, ay):
    dx = int(ax * DISPLAY_WIDTH  // OUT_RGB888P_WIDTH)
    dy = int(ay * DISPLAY_HEIGHT // OUT_RGB888P_HEIGH)
    return dx, dy


def display_to_ai(tx, ty):
    ax = max(0, min(OUT_RGB888P_WIDTH - 1,
                    int(tx * OUT_RGB888P_WIDTH // DISPLAY_WIDTH)))
    ay = max(0, min(OUT_RGB888P_HEIGH - 1,
                    int(ty * OUT_RGB888P_HEIGH // DISPLAY_HEIGHT)))
    return ax, ay


def clamp_pipe_param(value):
    return max(0.0, min(1.0, float(value)))


def pipe_geometry_from_corners(corners):
    """Build a stable left-to-right pipe axis from four rotated-box corners."""
    if corners is None or len(corners) != 4:
        return None
    points = [(float(point[0]), float(point[1])) for point in corners]
    edges = []
    for index in range(4):
        start = points[index]
        end = points[(index + 1) % 4]
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        edges.append((dx * dx + dy * dy, dx, dy))
    longest = max(edges, key=lambda item: item[0])
    shortest = min(edges, key=lambda item: item[0])
    length_px = longest[0] ** 0.5
    width_px = shortest[0] ** 0.5
    if length_px < 1.0 or width_px < 1.0:
        return None
    ux = longest[1] / length_px
    uy = longest[2] / length_px
    # The positive coordinate is always screen-left to screen-right. This
    # removes the 180-degree ambiguity of Blob rotation/corner ordering.
    if ux < 0.0 or (abs(ux) < 0.0001 and uy < 0.0):
        ux = -ux
        uy = -uy
    center_x = sum(point[0] for point in points) / 4.0
    center_y = sum(point[1] for point in points) / 4.0
    half_length = length_px * 0.5
    start_x = center_x - ux * half_length
    start_y = center_y - uy * half_length
    end_x = center_x + ux * half_length
    end_y = center_y + uy * half_length
    ordered_corners = []
    nx = -uy
    ny = ux
    half_width = width_px * 0.5
    for axial, lateral in (
            (-half_length, -half_width),
            (half_length, -half_width),
            (half_length, half_width),
            (-half_length, half_width)):
        ordered_corners.append((
            int(round(center_x + ux * axial + nx * lateral)),
            int(round(center_y + uy * axial + ny * lateral))))
    return {
        "corners": ordered_corners,
        "center": (center_x, center_y),
        "start": (start_x, start_y),
        "end": (end_x, end_y),
        "ux": ux,
        "uy": uy,
        "length_px": length_px,
        "width_px": width_px,
        "px_per_cm": length_px / 25.0,
    }


def project_point_to_pipe(point_x, point_y, geometry, clamp=False):
    start_x, start_y = geometry["start"]
    dx = float(point_x) - start_x
    dy = float(point_y) - start_y
    axial_px = dx * geometry["ux"] + dy * geometry["uy"]
    param = axial_px / geometry["length_px"]
    if clamp:
        param = clamp_pipe_param(param)
    projected_x = start_x + param * geometry["length_px"] * geometry["ux"]
    projected_y = start_y + param * geometry["length_px"] * geometry["uy"]
    lateral_px = dx * (-geometry["uy"]) + dy * geometry["ux"]
    return param, projected_x, projected_y, lateral_px


def measure_pipe_position(ball_x, ball_y, target_param,
                          geometry, pipe_length_cm=25.0):
    if geometry is None or geometry.get("length_px", 0.0) < 1.0:
        return {
            "valid": False,
            "ball_position_cm": 0.0,
            "target_position_cm": 0.0,
            "error_cm": 0.0,
            "lateral_px": 0.0,
            "ball_point": (0, 0),
            "origin_point": (0, 0),
            "target_point": (0, 0),
        }
    target_param = clamp_pipe_param(target_param)
    ball_param, ball_x_on_axis, ball_y_on_axis, lateral_px = (
        project_point_to_pipe(ball_x, ball_y, geometry, True))
    start_x, start_y = geometry["start"]
    origin_x = start_x + geometry["length_px"] * 0.5 * geometry["ux"]
    origin_y = start_y + geometry["length_px"] * 0.5 * geometry["uy"]
    target_x = (start_x + geometry["length_px"] * target_param *
                geometry["ux"])
    target_y = (start_y + geometry["length_px"] * target_param *
                geometry["uy"])
    ball_position_cm = (ball_param - 0.5) * pipe_length_cm
    target_position_cm = (target_param - 0.5) * pipe_length_cm
    return {
        "valid": True,
        "ball_position_cm": ball_position_cm,
        "target_position_cm": target_position_cm,
        "error_cm": target_position_cm - ball_position_cm,
        "lateral_px": lateral_px,
        "ball_param": ball_param,
        "ball_point": (int(round(ball_x_on_axis)), int(round(ball_y_on_axis))),
        "origin_point": (int(round(origin_x)), int(round(origin_y))),
        "target_point": (int(round(target_x)), int(round(target_y))),
    }


def calibration_is_valid(data):
    if not isinstance(data, dict) or data.get("version") != CALIBRATION_VERSION:
        return False
    try:
        target_param = float(data["target_param"])
    except (KeyError, TypeError, ValueError):
        return False
    return 0.0 <= target_param <= 1.0


def load_axis_calibration(path=CALIBRATION_PATH,
                          open_fn=open, json_module=ujson):
    try:
        with open_fn(path, "r") as file_obj:
            data = json_module.load(file_obj)
        if not calibration_is_valid(data):
            return None
        return {"target_param": float(data["target_param"])}
    except (OSError, ValueError, TypeError):
        return None


def save_axis_calibration(path, calibration,
                          temp_path=CALIBRATION_TEMP_PATH,
                          open_fn=open, json_module=ujson, os_module=uos):
    data = {
        "version": CALIBRATION_VERSION,
        "target_param": calibration["target_param"],
    }
    try:
        with open_fn(temp_path, "w") as file_obj:
            json_module.dump(data, file_obj)
        try:
            os_module.remove(path)
        except OSError:
            pass
        os_module.rename(temp_path, path)
        return True
    except (OSError, TypeError, ValueError) as error:
        print("Calibration save failed:", error)
        return False


def new_calibration_state(saved_calibration):
    return {
        "mode": CAL_READY if saved_calibration is not None else CAL_WAIT_TARGET,
        "calibration": saved_calibration,
        "touch_down_ms": -1,
        "touch_latched": False,
    }


def handle_touch_points(cal_state, points, now_ms, geometry=None):
    next_state = dict(cal_state)
    if not points:
        next_state["touch_down_ms"] = -1
        next_state["touch_latched"] = False
        return next_state, False

    point = points[0]
    if next_state["mode"] == CAL_READY:
        if next_state["touch_down_ms"] < 0:
            next_state["touch_down_ms"] = now_ms
        elif time.ticks_diff(
                now_ms, next_state["touch_down_ms"]) >= (
                    TOUCH_RECALIBRATE_HOLD_MS):
            return new_calibration_state(None), False
        return next_state, False

    if (getattr(point, "event", -1) != TOUCH_RELEASE_EVENT or
            next_state["touch_latched"]):
        return next_state, False
    next_state["touch_latched"] = True
    if next_state["mode"] == CAL_WAIT_TARGET and geometry is not None:
        touched_ai = display_to_ai(point.x, point.y)
        target_param = project_point_to_pipe(
            touched_ai[0], touched_ai[1], geometry, True)[0]
        completed = new_calibration_state({"target_param": target_param})
        completed["touch_latched"] = True
        return completed, True

    return new_calibration_state(None), False


def handle_stepper_zero_touch(stepper_state, points, target_ready,
                              target_was_ready, now_ms, release_event,
                              zero_rect):
    next_state = dict(stepper_state)
    point = points[0] if points else None
    inside_zero_rect = False
    if point is not None:
        x, y, width, height = zero_rect
        inside_zero_rect = (
            x <= point.x < x + width and y <= point.y < y + height)
    if (next_state.get("zeroed", False) or not target_ready or
            not target_was_ready or not inside_zero_rect or
            getattr(point, "event", -1) != release_event):
        return next_state
    next_state.update({
        "zeroed": True,
        "frequency_hz": 0.0,
        "direction": 0,
        "motion_sign": 0,
        "estimated_angle_deg": 0.0,
        "target_angle_deg": 0.0,
        "last_update_ms": now_ms,
        "fault": "vision_invalid",
    })
    return next_state


# ============================================================
# OSD绘制
# ============================================================

def detection_circle(bx, by, bw, bh):
    cx = bx + bw // 2
    cy = by + bh // 2
    radius = max(4, max(bw, bh) // 2 + 3)
    return cx, cy, radius


def should_render_osd(frame_number, cadence=2):
    return frame_number % cadence == 0


def tracking_osd_color(tracking_state):
    if tracking_state == TRACK_ACTIVE:
        return (0, 255, 0)
    if tracking_state == TRACK_RECOVER:
        return (255, 0, 0)
    return (255, 255, 0)


def draw_tracking_marker(osd_img, current_control, tracking_state):
    if not current_control["valid"]:
        return
    marker_x, marker_y = ai_to_disp(
        current_control["x"], current_control["y"])
    osd_img.draw_circle(
        marker_x, marker_y, 12,
        color=tracking_osd_color(tracking_state), thickness=3)


def draw_dynamic_pipe(osd_img, geometry, color):
    if geometry is None:
        return
    display_corners = [ai_to_disp(point[0], point[1])
                       for point in geometry["corners"]]
    for index in range(4):
        start = display_corners[index]
        end = display_corners[(index + 1) % 4]
        osd_img.draw_line(
            start[0], start[1], end[0], end[1],
            color=color, thickness=3)
    start = ai_to_disp(geometry["start"][0], geometry["start"][1])
    end = ai_to_disp(geometry["end"][0], geometry["end"][1])
    osd_img.draw_line(
        start[0], start[1], end[0], end[1],
        color=color, thickness=2)


def axis_measurement(current_control, cal_state):
    calibration = cal_state.get("calibration")
    if (cal_state.get("mode") != CAL_READY or
            calibration is None or not current_control["valid"] or
            not pipe_state.get("valid") or
            pipe_state.get("geometry") is None):
        return {
            "valid": False,
            "position_cm": 0.0,
            "ball_position_cm": 0.0,
            "target_position_cm": 0.0,
            "error_cm": 0.0,
            "velocity_cm_s": 0.0,
            "lateral_px": 0.0,
            "ball_point": (0, 0),
            "origin_point": (0, 0),
            "target_point": (0, 0),
        }
    geometry = pipe_state["geometry"]
    measurement = measure_pipe_position(
        current_control["x"], current_control["y"],
        calibration["target_param"], geometry, PIPE_LENGTH_CM)
    velocity_px_ms = (
        current_control["vx"] * geometry["ux"] +
        current_control["vy"] * geometry["uy"])
    measurement["velocity_cm_s"] = (
        velocity_px_ms * 1000.0 / geometry["px_per_cm"])
    measurement["position_cm"] = measurement["ball_position_cm"]
    return measurement


def update_stepper_control(measurement, stepper, stepper_state,
                           now_ms, control_timestamp_ms):
    age_ms = time.ticks_diff(now_ms, control_timestamp_ms)
    vision_fresh = (
        measurement.get("valid", False) and
        age_ms >= 0 and age_ms <= STEPPER_VISION_TIMEOUT_MS)
    command = compute_stepper_command(
        measurement.get("error_cm", 0.0),
        measurement.get("velocity_cm_s", 0.0),
        vision_fresh, stepper_state.get("zeroed", False),
        now_ms, stepper_state,
        STEPPER_KP_ANGLE_DEG_PER_CM, STEPPER_KD_ANGLE_DEG_PER_CM_S,
        STEPPER_ANGLE_TRACK_HZ_PER_DEG, STEPPER_ANGLE_TOLERANCE_DEG,
        STEPPER_DEADBAND_CM, STEPPER_MIN_FREQUENCY_HZ,
        STEPPER_MAX_FREQUENCY_HZ, STEPPER_FREQUENCY_RAMP_HZ_S,
        STEPPER_PULSES_PER_ROD_DEG, STEPPER_ANGLE_LIMIT_DEG,
        STEPPER_VISION_TIMEOUT_MS,
        STEPPER_DIRECTION_INVERT, time.ticks_diff)
    stepper.apply(command)
    return command


def publish_control_outputs(osd_img, capture, color_four, uart_obj, cal_state,
                            stepper, stepper_state):
    render_osd = should_render_osd(
        frame_counter + 1, OSD_EVERY_N_FRAMES)
    if render_osd:
        osd_img.clear()
    measurement = axis_measurement(control_state, cal_state)
    stepper_state = update_stepper_control(
        measurement, stepper, stepper_state,
        time.ticks_ms(), control_state.get("timestamp_ms", 0))
    draw_osd(osd_img, capture, color_four, uart_obj,
             cal_state, measurement, stepper_state, render_osd)
    if render_osd:
        Display.show_image(osd_img, 0, 0, Display.LAYER_OSD3)
    return stepper_state


def draw_osd(osd_img, capture, color_four, uart_obj,
             cal_state, measurement, stepper_state, render_osd=True):
    global frame_counter, tracker_state, current_deviation
    frame_counter += 1

    C_GREEN_TEXT = (0, 255, 0, 255)
    C_WHITE  = (255, 255, 255, 255)
    C_RED    = (0,   0,   255, 255)
    C_CYAN_TEXT = (255, 255, 0, 255)
    C_PIPE = (0, 255, 0)
    C_ORIGIN = (0, 128, 255)
    C_TARGET = (255, 0, 255)
    C_PROJECTED = (255, 255, 0)

    ball_valid = control_state["valid"]
    count = 1 if ball_valid else 0
    best_score = capture["score"] if capture is not None else 0.0
    if render_osd:
        draw_tracking_marker(osd_img, control_state, tracker_state)

    mode = cal_state["mode"]
    calibration = cal_state.get("calibration")

    if mode == CAL_READY and measurement["valid"]:
        geometry = pipe_state["geometry"]
        dx = int(round(measurement["error_cm"] * geometry["px_per_cm"]))
        dy = int(round(measurement["lateral_px"]))
        if abs(dx) <= DEVIATION_DEADZONE:
            dx = 0
        if abs(dy) <= DEVIATION_DEADZONE:
            dy = 0
        current_deviation = {
            "dx": dx,
            "dy": dy,
            "valid": True,
            "position_cm": measurement["position_cm"],
            "target_cm": measurement["target_position_cm"],
            "error_cm": measurement["error_cm"],
            "velocity_cm_s": measurement["velocity_cm_s"],
        }
    else:
        current_deviation = {
            "dx": 0,
            "dy": 0,
            "valid": False,
            "position_cm": 0.0,
            "target_cm": 0.0,
            "error_cm": 0.0,
            "velocity_cm_s": 0.0,
        }

    if render_osd:
        osd_img.draw_string_advanced(
            10, 10, 22, "Ball:{}".format(count),
            color=C_WHITE if ball_valid else C_RED)
        osd_img.draw_string_advanced(
            10, 34, 18, "s:{:.2f}".format(best_score), color=C_WHITE)
        geometry = pipe_state.get("geometry")
        if pipe_state.get("valid") and geometry is not None:
            draw_dynamic_pipe(osd_img, geometry, C_PIPE)
            center_dx, center_dy = ai_to_disp(
                geometry["center"][0], geometry["center"][1])
            osd_img.draw_circle(
                center_dx, center_dy, 12,
                color=C_ORIGIN, thickness=3)
            osd_img.draw_string_advanced(
                center_dx - 8, center_dy - 34, 20, "O", color=C_WHITE)
        else:
            osd_img.draw_string_advanced(
                250, 20, 26, "Green pipe not found", color=C_RED)

        if mode == CAL_WAIT_TARGET:
            osd_img.draw_string_advanced(
                215, 20, 28, "Tap target point", color=C_CYAN_TEXT)
        elif measurement["valid"]:
            ball_dx, ball_dy = ai_to_disp(
                measurement["ball_point"][0], measurement["ball_point"][1])
            origin_dx, origin_dy = ai_to_disp(
                measurement["origin_point"][0], measurement["origin_point"][1])
            target_dx, target_dy = ai_to_disp(
                measurement["target_point"][0], measurement["target_point"][1])
            osd_img.draw_line(
                target_dx, target_dy, ball_dx, ball_dy,
                color=C_PROJECTED, thickness=3)
            osd_img.draw_circle(
                origin_dx, origin_dy, 14,
                color=C_ORIGIN, thickness=3)
            osd_img.draw_circle(
                target_dx, target_dy, 12,
                color=C_TARGET, thickness=3)
            osd_img.draw_circle(
                ball_dx, ball_dy, 9,
                color=C_PROJECTED, thickness=3)
            osd_img.draw_string_advanced(
                target_dx - 8, target_dy - 32, 18, "T", color=C_WHITE)
            osd_img.draw_string_advanced(
                DISPLAY_WIDTH - 240, 10, 22,
                "O:{:+.2f}cm".format(measurement["ball_position_cm"]),
                color=C_GREEN_TEXT if measurement["valid"] else C_RED)
            osd_img.draw_string_advanced(
                DISPLAY_WIDTH - 240, 36, 20,
                "|O-B|:{:.2f}cm".format(
                    abs(measurement["ball_position_cm"])),
                color=C_GREEN_TEXT)
            osd_img.draw_string_advanced(
                DISPLAY_WIDTH - 240, 62, 20,
                "T:{:+.2f}cm".format(measurement["target_position_cm"]),
                color=C_WHITE)
            osd_img.draw_string_advanced(
                DISPLAY_WIDTH - 240, 88, 22,
                "E:{:+.2f}cm".format(measurement["error_cm"]),
                color=C_GREEN_TEXT)
            osd_img.draw_string_advanced(
                DISPLAY_WIDTH - 240, 116, 17,
                "F:{:04d}Hz D:{:+d}".format(
                    int(round(stepper_state.get("frequency_hz", 0.0))),
                    int(stepper_state.get("direction", 0))),
                color=C_WHITE)
            osd_img.draw_string_advanced(
                DISPLAY_WIDTH - 240, 140, 17,
                "A:{:+.2f}/{:+.2f} {}".format(
                    stepper_state.get("estimated_angle_deg", 0.0),
                    stepper_state.get("target_angle_deg", 0.0),
                    stepper_state.get("fault", "unknown")),
                color=C_GREEN_TEXT if stepper_state.get("enabled") else C_WHITE)
            osd_img.draw_string_advanced(
                DISPLAY_WIDTH - 240, 164, 15,
                "Hold 2s: new target", color=C_WHITE)

        if (mode == CAL_READY and
                not stepper_state.get("zeroed", False)):
            zero_x, zero_y, zero_w, zero_h = STEPPER_ZERO_TOUCH_RECT
            osd_img.draw_rectangle(
                zero_x, zero_y, zero_w, zero_h,
                color=C_CYAN_TEXT, thickness=3)
            osd_img.draw_string_advanced(
                zero_x + 42, zero_y + 17, 20,
                "LEVEL ROD - TAP ZERO",
                color=C_CYAN_TEXT)

    # ---- UART发送 ----
    if frame_counter % SEND_EVERY_N_FRAMES == 0:
        uart_obj.write(format_deviation_msg(
            current_deviation["dx"],
            current_deviation["dy"],
            current_deviation["valid"]))
        uart_obj.write(format_stepper_msg(stepper_state))

    # ---- 调试打印 ----
    if frame_counter % PRINT_EVERY_N_FRAMES == 0:
        st = mode
        if current_deviation["valid"]:
            print("[{}] Ball:{} dX:{:+04d} dY:{:+04d} B:{:+.2f}cm T:{:+.2f}cm E:{:+.2f}cm s:{:.2f}".format(
                st, count, current_deviation["dx"], current_deviation["dy"],
                current_deviation["position_cm"],
                current_deviation["target_cm"],
                current_deviation["error_cm"], best_score))
        else:
            print("[{}] Ball:{} Pipe:{} (control invalid)".format(
                st, count, 1 if pipe_state.get("valid") else 0))


# ============================================================
# 主入口
# ============================================================

def detection():
    global tracker_state, control_state, motion_samples
    global pipe_state
    global blob_frame_count, blob_total_ms
    global kpu_validation_count, kpu_total_ms
    global blob_loss_count, kpu_reacquire_count
    global prediction_clamp_count
    print("=== Ball Position (new model) ===")
    wlan = None
    stepper = None
    saved_calibration = load_axis_calibration()
    cal_state = new_calibration_state(saved_calibration)

    deploy_conf = read_deploy_config(config_path)
    kmodel_name   = deploy_conf["kmodel_path"]
    nms_threshold = deploy_conf["nms_threshold"]
    img_size      = deploy_conf["img_size"]
    num_classes   = deploy_conf["num_classes"]
    color_four    = get_colors(num_classes)
    nms_option    = deploy_conf["nms_option"]
    model_type    = deploy_conf["model_type"]
    anchors       = []
    if model_type == "AnchorBaseDet":
        anchors = (deploy_conf["anchors"][0] +
                   deploy_conf["anchors"][1] +
                   deploy_conf["anchors"][2])

    kmodel_frame_size = img_size
    frame_size = [OUT_RGB888P_WIDTH, OUT_RGB888P_HEIGH]
    strides    = [8, 16, 32]
    top, bottom, left, right = two_side_pad_param(
        frame_size, kmodel_frame_size)

    # ---- UART ----
    fpioa = FPIOA()
    fpioa.set_function(3, fpioa.UART1_TXD, ie=1, oe=1)
    fpioa.set_function(4, fpioa.UART1_RXD, ie=1, oe=1)
    uart = UART(UART.UART1, baudrate=UART_BAUDRATE,
                bits=UART.EIGHTBITS, parity=UART.PARITY_NONE,
                stop=UART.STOPBITS_ONE)
    print("UART1 OK, baud:", UART_BAUDRATE)

    # ---- KModel ----
    kpu = nn.kpu()
    kpu.load_kmodel(root_path + kmodel_name)

    # ---- AI2D ----
    ai2d = nn.ai2d()
    ai2d.set_dtype(nn.ai2d_format.NCHW_FMT, nn.ai2d_format.NCHW_FMT,
                   np.uint8, np.uint8)
    ai2d.set_pad_param(True, [0,0,0,0, top,bottom,left,right], 0, [114,114,114])
    ai2d.set_resize_param(True, nn.interp_method.tf_bilinear,
                          nn.interp_mode.half_pixel)
    ai2d_builder = ai2d.build(
        [1,3,OUT_RGB888P_HEIGH,OUT_RGB888P_WIDTH],
        [1,3,kmodel_frame_size[1],kmodel_frame_size[0]])

    # ---- 摄像头 ----
    # 部分 K230 板型默认以 60 FPS 启动；显示和 AI 同时工作时
    # 会导致 snapshot chn(2) failed(3)。固定 30 FPS 可避免缓冲区耗尽。
    sensor = None
    rtsp_server = None
    ai2d_output_tensor = None
    tensor_holder = [None]
    try:
        time.sleep_ms(CAMERA_BOOT_DELAY_MS)
        sensor, osd_img, rtsp_server, blob_channel_available = (
            start_camera_with_blob_fallback(start_camera_pipeline))
        stepper = D36AStepper(fpioa)
        print("D36A ready: STEP=pin13 DIR=pin11 EN=pin12 (disabled)")
        stepper_state = new_stepper_control_state(time.ticks_ms())
        tp = TOUCH(0)
        touch_poll_counter = 0
        if saved_calibration is None:
            print("Touch target selection required")
        else:
            print("Touch target loaded")

        print("Creating AI output tensor")
        data = np.ones(
            (1,3,kmodel_frame_size[1],kmodel_frame_size[0]), dtype=np.uint8)
        ai2d_output_tensor = nn.from_numpy(data)
        tensor_holder[0] = ai2d_output_tensor
        first_ai_frame = True
        wlan = None
        gc_frame_count = 0
        perf_frame_count = 0
        perf_start_ms = time.ticks_ms()
        metrics_start_ms = time.ticks_ms()
        blob_frame_count = 0
        blob_total_ms = 0
        kpu_validation_count = 0
        kpu_total_ms = 0
        blob_loss_count = 0
        kpu_reacquire_count = 0
        prediction_clamp_count = 0
        blob_misses = 0
        ai_failures = 0
        predicted_frames = 0
        tracker_state = TRACK_SEARCH
        roi_anchor = {"x": control_state["x"], "y": control_state["y"]}
        print("AI loop start")

        while True:
            touch_poll_counter += 1
            if touch_poll_counter >= TOUCH_POLL_EVERY_N_FRAMES:
                touch_points = tp.read(1)
                target_was_ready = cal_state.get("mode") == CAL_READY
                stepper_was_zeroed = stepper_state.get("zeroed", False)
                cal_state, should_save_calibration = handle_touch_points(
                    cal_state, touch_points, time.ticks_ms(),
                    pipe_state.get("geometry") if pipe_state.get("valid")
                    else None)
                stepper_state = handle_stepper_zero_touch(
                    stepper_state, touch_points,
                    cal_state.get("mode") == CAL_READY,
                    target_was_ready, time.ticks_ms(),
                    STEPPER_ZERO_TOUCH_EVENT, STEPPER_ZERO_TOUCH_RECT)
                if (stepper_state.get("zeroed") and
                        not stepper_was_zeroed):
                    print("D36A zero accepted; automatic control armed")
                if should_save_calibration:
                    if save_axis_calibration(
                            CALIBRATION_PATH, cal_state["calibration"]):
                        print("Touch calibration saved")
                touch_poll_counter = 0
            with ScopedTiming("total", debug_mode > 0):
                dynamic_roi = blob_tracking_roi(roi_anchor["x"])
                blob_capture = None
                pipe_observation = None
                if should_snapshot_blob_channel(
                        tracker_state, blob_channel_available):
                    blob_start_ms = time.ticks_ms()
                    (blob_capture, pipe_observation,
                     blob_channel_available) = (
                        snapshot_blob_channel(
                            sensor, True,
                            dynamic_roi,
                            roi_anchor["x"], roi_anchor["y"],
                            not pipe_state.get("locked", False)))
                    blob_frame_count += 1
                    blob_total_ms += time.ticks_diff(
                        time.ticks_ms(), blob_start_ms)
                    if not blob_channel_available:
                        tracker_state = TRACK_SEARCH
                        blob_misses = 0
                        ai_failures = 0
                        predicted_frames = 0
                        motion_samples = []
                pipe_state = update_pipe_geometry_state(
                    pipe_state, pipe_observation,
                    PIPE_HOLD_MISSES, PIPE_SMOOTH_ALPHA,
                    PIPE_LOCK_FIRST)

                blob_valid = blob_capture is not None
                if (blob_channel_available and
                        tracker_state == TRACK_ACTIVE and not blob_valid):
                    blob_loss_count += 1
                    blob_misses += 1
                actions = hybrid_frame_actions(
                    tracker_state, frame_counter + 1,
                    blob_channel_available, blob_valid)
                blob_measurement_x = None
                blob_measurement_y = None
                ai_valid = False

                for action in actions:
                    if action == "blob_control":
                        if blob_capture is not None:
                            blob_x = blob_capture["x"] + blob_capture["w"] // 2
                            blob_y = blob_capture["y"] + blob_capture["h"] // 2
                            blob_measurement_x = blob_x
                            blob_measurement_y = blob_y
                            publish_measurement(
                                blob_x, blob_y, "blob", time.ticks_ms())
                            roi_anchor = {
                                "x": control_state["x"],
                                "y": control_state["y"],
                            }
                            blob_misses = 0
                            predicted_frames = 0
                            # Blob control stays first on scheduled KPU
                            # validation frames and is published only once.
                            stepper_state = publish_control_outputs(
                                osd_img, None, color_four, uart, cal_state,
                                stepper, stepper_state)
                        continue

                    capture = None
                    rgb888p_img = None
                    ai2d_input = None
                    ai2d_input_tensor = None
                    results = None
                    out_data = None
                    result = None
                    det_boxes = None
                    is_kpu_validation = (
                        blob_channel_available and
                        tracker_state == TRACK_ACTIVE and blob_valid)
                    try:
                        rgb888p_img = sensor.snapshot(
                            chn=CAM_CHN_ID_2, timeout=2000)
                        if first_ai_frame:
                            print("AI first frame OK")
                            first_ai_frame = False
                            # LCD、摄像头和 AI 均正常后，再尝试启动一次 Wi-Fi。
                            try:
                                wlan = start_wifi()
                            except BaseException as e:
                                print("Wi-Fi disabled after initialization failure:", e)
                            if wlan is not None:
                                try:
                                    rtsp_server.start(wlan.ifconfig()[0])
                                except BaseException as e:
                                    print("RTSP disabled after initialization failure:", e)
                            (metrics_start_ms,
                             blob_frame_count, blob_total_ms,
                             kpu_validation_count, kpu_total_ms,
                             blob_loss_count, kpu_reacquire_count,
                             prediction_clamp_count) = (
                                reset_tracking_metrics_window(
                                    time.ticks_ms()))

                        if rgb888p_img.format() == image.RGBP888:
                            if is_kpu_validation:
                                kpu_start_ms = time.ticks_ms()
                            ai2d_input = rgb888p_img.to_numpy_ref()
                            ai2d_input_tensor = nn.from_numpy(ai2d_input)
                            ai2d_builder.run(
                                ai2d_input_tensor, ai2d_output_tensor)

                            kpu.set_input_tensor(0, ai2d_output_tensor)
                            kpu.run()

                            results = []
                            for i in range(kpu.outputs_size()):
                                out_data = kpu.get_output_tensor(i)
                                result = out_data.to_numpy()
                                result = result.reshape(
                                    result.shape[0] * result.shape[1] *
                                    result.shape[2] * result.shape[3])
                                del out_data
                                out_data = None
                                results.append(result)

                            det_boxes = aicube.anchorbasedet_post_process(
                                results[0], results[1], results[2],
                                kmodel_frame_size, frame_size, strides,
                                num_classes, DETECT_CONF_THRESHOLD,
                                nms_threshold, anchors, nms_option)
                            capture = select_best_ai_ball(det_boxes)
                            if is_kpu_validation:
                                kpu_validation_count += 1
                                kpu_total_ms += time.ticks_diff(
                                    time.ticks_ms(), kpu_start_ms)
                    finally:
                        del det_boxes
                        del result
                        del out_data
                        del results
                        del ai2d_input_tensor
                        del ai2d_input
                        del rgb888p_img

                    ai_valid = capture is not None
                    if (ai_valid and
                            (tracker_state == TRACK_RECOVER or
                             (tracker_state == TRACK_ACTIVE and
                              not blob_valid))):
                        kpu_reacquire_count += 1
                    validation_only = is_kpu_validation
                    if validation_only:
                        control_state, ai_valid, ai_failures = (
                            kpu_validation_outcome(
                                control_state,
                                blob_measurement_x, blob_measurement_y,
                                capture, ai_failures))
                        if ai_valid:
                            (control_state, corrected_anchor,
                             motion_samples) = apply_kpu_validation_anchor(
                                control_state,
                                blob_measurement_x, blob_measurement_y,
                                capture, motion_samples,
                                MOTION_RESET_JUMP_PX)
                            if corrected_anchor is not None:
                                roi_anchor = corrected_anchor
                            (blob_misses, ai_failures,
                             predicted_frames) = ai_capture_counters(
                                tracker_state, blob_valid, blob_misses)
                    else:
                        if ai_valid:
                            if (tracker_state != TRACK_ACTIVE or
                                    not blob_valid):
                                motion_samples = []
                            publish_measurement(
                                capture["cx"], capture["cy"],
                                "kpu", time.ticks_ms())
                            roi_anchor = {
                                "x": capture["cx"], "y": capture["cy"],
                            }
                            blob_misses, ai_failures, predicted_frames = (
                                ai_capture_counters(
                                    tracker_state, blob_valid, blob_misses))
                        elif (blob_channel_available and
                              tracker_state in (
                                  TRACK_ACTIVE, TRACK_RECOVER)):
                            ai_failures += 1
                            predicted_state, predicted_frames = (
                                prediction_for_missed_frame(
                                    control_state, predicted_frames,
                                    time.ticks_ms()))
                            if predicted_state is None:
                                invalidate_control_state()
                            else:
                                control_state = predicted_state
                                roi_anchor = {
                                    "x": control_state["x"],
                                    "y": control_state["y"],
                                }
                        else:
                            invalidate_control_state()

                        stepper_state = publish_control_outputs(
                            osd_img, capture, color_four, uart, cal_state,
                            stepper, stepper_state)

                    perf_frame_count += 1
                    if perf_frame_count >= PERF_EVERY_N_FRAMES:
                        perf_now_ms = time.ticks_ms()
                        perf_elapsed_ms = time.ticks_diff(
                            perf_now_ms, perf_start_ms)
                        if perf_elapsed_ms > 0:
                            print("AI FPS:{:.1f} avg:{:.1f}ms".format(
                                perf_frame_count * 1000.0 / perf_elapsed_ms,
                                perf_elapsed_ms * 1.0 / perf_frame_count))
                        perf_start_ms = perf_now_ms
                        perf_frame_count = 0

                if blob_channel_available:
                    tracker_state = hybrid_transition(
                        tracker_state, blob_valid, ai_valid,
                        blob_misses, ai_failures)
                else:
                    tracker_state = TRACK_SEARCH
                    blob_misses = 0
                    ai_failures = 0
                    predicted_frames = 0

                if (frame_counter % METRICS_EVERY_N_CONTROL_FRAMES == 0):
                    metrics_now_ms = time.ticks_ms()
                    (metrics_line, metrics_start_ms,
                     blob_frame_count, blob_total_ms,
                     kpu_validation_count, kpu_total_ms,
                     blob_loss_count, kpu_reacquire_count,
                     prediction_clamp_count) = tracking_metrics_report(
                        frame_counter, tracker_state, metrics_now_ms,
                        metrics_start_ms, blob_frame_count, blob_total_ms,
                        kpu_validation_count, kpu_total_ms,
                        blob_loss_count, kpu_reacquire_count,
                        prediction_clamp_count)
                    print(metrics_line)

                gc_frame_count += 1
                if gc_frame_count >= GC_EVERY_N_FRAMES:
                    gc.collect()
                    gc_frame_count = 0

    except KeyboardInterrupt:
        print("=== Stop ===")
    except BaseException as e:
        print("=== Runtime error ===", e)
        raise
    finally:
        ai2d_output_tensor = None
        try:
            if stepper is not None:
                stepper.deinit()
        finally:
            cleanup_runtime_resources(rtsp_server, sensor, tensor_holder)
    return 0


if __name__ == "__main__":
    detection()
