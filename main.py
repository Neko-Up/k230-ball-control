# -*- coding: utf-8 -*-
#
# CanMV K230 钢珠定位 — 新模型(单类 gangqiu) + 校准 + UART
# 模型文件放在: /sdcard/mp_deployment_source/
#
# 工作流程：
#   1. 启动 → CALIBRATING：小球放在任意位置保持静止
#   2. 静止约1.5秒 → 锁定为校准零点 → CALIBRATED
#   3. 实时显示偏差 + 累计滚动距离 + UART发送

import gc
import time
import uctypes
import network
import socket
import _thread

import aicube
import image
import nncase_runtime as nn
import ujson
import ulab.numpy as np
from libs.PipeLine import ScopedTiming
from libs.Utils import *
from machine import FPIOA, UART
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
# Wi-Fi（仅联网，不启用视频推流）
# ============================================================
WIFI_MODE         = "ap"       # "ap": K230 开热点；"sta": 连接现有 Wi-Fi
WIFI_AP_SSID      = "K230_BALL"
WIFI_AP_PASSWORD  = "12345678" # 至少 8 位
WIFI_AP_CHANNEL   = 0   # 0=启动时自动选择，选定后固定
WIFI_STA_SSID     = "YOUR_WIFI"
WIFI_STA_PASSWORD = "YOUR_PASSWORD"
WIFI_CONNECT_MS   = 15000

MJPEG_PORT        = 8080
MJPEG_QUALITY     = 60

def format_iso_time(epoch_s=None, millis=0):
    """OSD 使用的 ISO 8601 本地时间（UTC+08:00）。"""
    t = time.localtime() if epoch_s is None else time.localtime(epoch_s)
    return ("%04d-%02d-%02dT%02d:%02d:%02d.%03d+08:00" %
            (t[0], t[1], t[2], t[3], t[4], t[5], millis))
# ============================================================
# 状态机
# ============================================================
STATE_CALIBRATING = 0
STATE_CALIBRATED  = 1
state = STATE_CALIBRATING

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
DISPLAY_LABEL            = "gz"
MERGED_CLASS_ID          = 0       # 新模型只有 gangqiu 一个类
MIN_BOX_SIZE             = 4
MAX_BOX_SIZE             = 170
MAX_ASPECT_RATIO         = 1.8
DEDUP_IOU_THRESHOLD      = 0.35
DEDUP_CENTER_RATIO       = 0.55
TRACK_MERGE_IOU_THRESHOLD = 0.25
TRACK_MERGE_CENTER_RATIO  = 0.75

# ============================================================
# 中值滤波
# ============================================================
MEDIAN_WINDOW = 1
pos_history_x = [0] * MEDIAN_WINDOW
pos_history_y = [0] * MEDIAN_WINDOW
pos_hist_idx  = 0
pos_hist_full = False

# ============================================================
# 校准参数 — 小球在任意位置保持静止即可校准
# ============================================================
STILL_RADIUS          = 15     # 视为"静止"的最大移动范围(px)
CALIB_DURATION_FRAMES = 45     # 约1.5秒稳定
calib_stable_count    = 0
calib_last_cx         = -1
calib_last_cy         = -1
calib_cx              = 0
calib_cy              = 0
calib_done_flash      = 0

# ============================================================
# UART
# ============================================================
UART_BAUDRATE       = 115200
SEND_EVERY_N_FRAMES = 1
DEVIATION_DEADZONE  = 3
PIXEL_TO_MM         = 0.5

total_distance_px = 0.0
last_valid_cx     = -1
last_valid_cy     = -1

tracks            = []
frame_counter     = 0
current_deviation = {"dx": 0, "dy": 0, "valid": False, "dist_mm": 0.0}


# ============================================================
# 工具函数
# ============================================================

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


def scan_best_channel():
    """
    启动时选择一次最佳2.4G信道。
    不运行中切换，避免MJPEG断流。
    """
    try:
        sta = network.WLAN(network.STA_IF)
        sta.active(True)
        result = sta.scan()

        busy = {1: 0, 6: 0, 11: 0}

        for item in result:
            ssid, bssid, channel, rssi, auth, hidden = item
            if channel in busy:
                # RSSI越强，占用权重越高
                busy[channel] += max(0, 100 + rssi)

        best = min(busy, key=busy.get)
        print("Wi-Fi channel scan:", busy, "choose:", best)
        return best
    except BaseException as e:
        print("Wi-Fi scan failed:", e)
        return 6

def start_wifi():
    if WIFI_MODE == "ap":
        wlan = network.WLAN(network.AP_IF)
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
        print("Wi-Fi AP:", WIFI_AP_SSID, "password:", WIFI_AP_PASSWORD)
        print("Wi-Fi IP:", wlan.ifconfig()[0])
        return wlan

    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print("Connecting Wi-Fi:", WIFI_STA_SSID)
        wlan.connect(WIFI_STA_SSID, WIFI_STA_PASSWORD)
        start_ms = time.ticks_ms()
        while not wlan.isconnected():
            if time.ticks_diff(time.ticks_ms(), start_ms) >= WIFI_CONNECT_MS:
                raise RuntimeError("Wi-Fi connect timeout")
            time.sleep_ms(200)
    print("Wi-Fi connected, IP:", wlan.ifconfig()[0])
    return wlan


# ============================================================
# MJPEG（WBC + 硬件 JPEG）
# ============================================================

class LowLatencyMjpegServer:
    def __init__(self, width, height, port=MJPEG_PORT):
        self.width = ALIGN_UP(width, 16)
        self.height = height
        self.port = port
        self.encoder = Encoder()
        self.venc_chn = VENC_CHN_ID_0
        self.channel_api = True
        self.running = False
        self.thread_over = True
        self.server_sock = None
        self.client_sock = None

        # 编码缓冲必须在 MediaManager.init() 前配置。
        try:
            self.encoder.SetOutBufs(
                self.venc_chn, 4, self.width, self.height)
        except TypeError:
            self.channel_api = False
            self.encoder.SetOutBufs(4, self.width, self.height)

    def start(self):
        encoder_created = False
        encoder_started = False
        wbc_started = False
        try:
            attr = ChnAttrStr(
                self.encoder.PAYLOAD_TYPE_JPEG, 0,
                self.width, self.height,
                4000, 15, 15, 15, MJPEG_QUALITY)
            if self.channel_api:
                self.encoder.Create(self.venc_chn, attr)
                encoder_created = True
                self.encoder.Start(self.venc_chn)
            else:
                self.encoder.Create(attr)
                encoder_created = True
                self.venc_chn = self.encoder.chn
                self.encoder.Start()
            encoder_started = True

            if not WBCDisplay.writeback(True):
                raise RuntimeError("start WBC for MJPEG failed")
            wbc_started = True

            addr = socket.getaddrinfo("0.0.0.0", self.port)[0][-1]
            self.server_sock = socket.socket()
            self.server_sock.setsockopt(
                socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_sock.bind(addr)
            self.server_sock.listen(1)
            self.server_sock.settimeout(0.2)

            self.running = True
            self.thread_over = False
            _thread.start_new_thread(self._serve, ())
            print("MJPEG quality:", MJPEG_QUALITY)
        except BaseException:
            self.running = False
            if self.server_sock:
                try:
                    self.server_sock.close()
                except BaseException:
                    pass
                self.server_sock = None
            if wbc_started:
                WBCDisplay.writeback(False)
            if encoder_started:
                try:
                    if self.channel_api:
                        self.encoder.Stop(self.venc_chn)
                    else:
                        self.encoder.Stop()
                except BaseException:
                    pass
            if encoder_created:
                try:
                    if self.channel_api:
                        self.encoder.Destroy(self.venc_chn)
                    else:
                        self.encoder.Destroy()
                except BaseException:
                    pass
            raise

    def _encode_jpeg(self, frame_info):
        stream = StreamData()
        if self.channel_api:
            ret = self.encoder.SendFrame(self.venc_chn, frame_info, 100)
        else:
            ret = self.encoder.SendFrame(frame_info, 100)
        if ret != 0:
            return None

        if self.channel_api:
            ret = self.encoder.GetStream(self.venc_chn, stream, 100)
        else:
            ret = self.encoder.GetStream(stream, 100)
        if ret != 0:
            return None

        parts = []
        for i in range(stream.pack_cnt):
            parts.append(bytes(uctypes.bytearray_at(
                stream.data[i], stream.data_size[i])))
        if self.channel_api:
            self.encoder.ReleaseStream(self.venc_chn, stream)
        else:
            self.encoder.ReleaseStream(stream)
        return b"".join(parts)

    def _serve(self):
        header = (b"HTTP/1.1 200 OK\r\n"
                  b"Cache-Control: no-store, no-cache\r\n"
                  b"Connection: close\r\n"
                  b"Content-Type: multipart/x-mixed-replace; "
                  b"boundary=frame\r\n\r\n")
        while self.running:
            try:
                client, _ = self.server_sock.accept()
            except OSError:
                continue
            self.client_sock = client
            try:
                client.sendall(header)
                while self.running:
                    frame_info = WBCDisplay.writeback_dump(100)
                    if not frame_info:
                        continue
                    jpg = self._encode_jpeg(frame_info)
                    if not jpg:
                        continue
                    client.sendall(
                        b"--frame\r\nContent-Type: image/jpeg\r\n" +
                        b"Content-Length: " + str(len(jpg)).encode() +
                        b"\r\n\r\n")
                    client.sendall(jpg)
                    client.sendall(b"\r\n")
            except OSError:
                pass
            try:
                client.close()
            except BaseException:
                pass
            self.client_sock = None
        self.thread_over = True

    def stop(self):
        self.running = False
        if self.client_sock:
            try:
                self.client_sock.close()
            except BaseException:
                pass
        if self.server_sock:
            try:
                self.server_sock.close()
            except BaseException:
                pass
        start_ms = time.ticks_ms()
        while (not self.thread_over and
               time.ticks_diff(time.ticks_ms(), start_ms) < 1000):
            time.sleep_ms(20)
        WBCDisplay.writeback(False)
        if self.channel_api:
            self.encoder.Stop(self.venc_chn)
            self.encoder.Destroy(self.venc_chn)
        else:
            self.encoder.Stop()
            self.encoder.Destroy()


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


# ============================================================
# 坐标转换
# ============================================================

def ai_to_disp(ax, ay):
    dx = int(ax * DISPLAY_WIDTH  // OUT_RGB888P_WIDTH)
    dy = int(ay * DISPLAY_HEIGHT // OUT_RGB888P_HEIGH)
    return dx, dy


# ============================================================
# OSD绘制
# ============================================================

def detection_circle(bx, by, bw, bh):
    cx = bx + bw // 2
    cy = by + bh // 2
    radius = max(4, max(bw, bh) // 2 + 3)
    return cx, cy, radius


def draw_osd(osd_img, stable_tracks, color_four, uart_obj):
    global frame_counter, state
    global pos_hist_full
    global calib_stable_count, calib_last_cx, calib_last_cy
    global calib_cx, calib_cy, calib_done_flash, current_deviation
    global total_distance_px, last_valid_cx, last_valid_cy
    frame_counter += 1

    C_GREEN  = (0,   255, 0,   255)
    C_BLUE   = (255, 0,   0,   255)
    C_YELLOW = (0,   255, 255, 255)
    C_WHITE  = (255, 255, 255, 255)
    C_RED    = (0,   0,   255, 255)
    C_ORANGE = (0,   165, 255, 255)


    gc_dx, gc_dy = ai_to_disp(GEOM_CENTER_X, GEOM_CENTER_Y)

    # ---- 找最佳球 ----
    count      = 0
    raw_cx     = -1
    raw_cy     = -1
    best_score = 0.0

    for trk in stable_tracks:
        if not is_visible_track(trk):
            continue
        x1, y1, x2, y2 = trk["box"]
        bx, by = ai_to_disp(x1, y1)
        bw     = int((x2 - x1) * DISPLAY_WIDTH  // OUT_RGB888P_WIDTH)
        bh     = int((y2 - y1) * DISPLAY_HEIGHT // OUT_RGB888P_HEIGH)
        if bw <= 0 or bh <= 0:
            continue

        col = color_four[MERGED_CLASS_ID][1:]
        circle_x, circle_y, circle_radius = detection_circle(bx, by, bw, bh)
        osd_img.draw_circle(
            circle_x, circle_y, circle_radius, color=col, thickness=2)

        tcx = int((x1 + x2) / 2)
        tcy = int((y1 + y2) / 2)
        lbl = DISPLAY_LABEL + ("*" if trk["lost"] > 0 else "")
        osd_img.draw_string_advanced(bx, max(0, by - 26), 22, lbl, color=col)
        count += 1
        if trk["score"] > best_score:
            best_score = trk["score"]
            raw_cx     = tcx
            raw_cy     = tcy

    # ---- 中值滤波 ----
    ball_valid = (raw_cx >= 0 and raw_cy >= 0)
    if ball_valid:
        filt_cx, filt_cy = median_filter_push(raw_cx, raw_cy)
    else:
        pos_hist_full = False
        filt_cx, filt_cy = -1, -1

    # ============================================================
    # CALIBRATING — 球在任意位置保持静止即可校准
    # ============================================================
    if state == STATE_CALIBRATING:
        flash_on = ((frame_counter // 15) % 2 == 0)

        # 在画面几何中心画准星 (仅作视觉参考)
        if flash_on:
            osd_img.draw_circle(gc_dx, gc_dy, 40, color=C_GREEN, thickness=3)
            osd_img.draw_circle(gc_dx, gc_dy, 20, color=C_GREEN, thickness=2)
            osd_img.draw_line(gc_dx - 50, gc_dy, gc_dx + 50, gc_dy,
                              color=C_GREEN, thickness=2)
            osd_img.draw_line(gc_dx, gc_dy - 50, gc_dx, gc_dy + 50,
                              color=C_GREEN, thickness=2)

        # 检测球是否静止 (连续两帧移动 < STILL_RADIUS)
        if ball_valid and pos_hist_full:
            if calib_last_cx >= 0:
                move = ((filt_cx - calib_last_cx) ** 2 +
                        (filt_cy - calib_last_cy) ** 2) ** 0.5
                if move <= STILL_RADIUS:
                    calib_stable_count += 1
                else:
                    calib_stable_count = max(0, calib_stable_count - 2)
            calib_last_cx = filt_cx
            calib_last_cy = filt_cy
        else:
            calib_last_cx = -1
            calib_last_cy = -1
            calib_stable_count = max(0, calib_stable_count - 2)

        # 提示文字
        remaining = max(0, CALIB_DURATION_FRAMES - calib_stable_count)
        remaining_s = remaining / 30.0
        if calib_stable_count > 0:
            tip = "HOLD... {:.1f}s".format(remaining_s)
            tip_color = C_YELLOW
        else:
            tip = "HOLD BALL STILL"
            tip_color = C_WHITE

        # 在球所在位置上方显示提示
        if ball_valid and pos_hist_full:
            bx, by = ai_to_disp(filt_cx, filt_cy)
            osd_img.draw_string_advanced(bx - 50, by - 50, 22, tip, color=tip_color)
        else:
            osd_img.draw_string_advanced(gc_dx - 70, gc_dy - 80, 26, tip, color=tip_color)

        # 进度条
        bar_w = 120
        bar_h = 8
        bar_x = gc_dx - bar_w // 2
        bar_y = gc_dy + 70
        progress = min(1.0, calib_stable_count / CALIB_DURATION_FRAMES)
        osd_img.draw_rectangle(bar_x, bar_y, bar_w, bar_h, color=C_WHITE, thickness=1)
        if progress > 0:
            fill_w = int(bar_w * progress)
            osd_img.draw_rectangle(bar_x, bar_y, fill_w, bar_h, color=C_GREEN, thickness=-1)

        # 校准完成
        if calib_stable_count >= CALIB_DURATION_FRAMES:
            state = STATE_CALIBRATED
            calib_cx = filt_cx
            calib_cy = filt_cy
            calib_done_flash = 60
            uart_obj.write(b"CALIB:OK\n")
            print("=== CALIB OK! zero=({},{}) ===".format(calib_cx, calib_cy))

    # ============================================================
    # CALIBRATED — 计算偏差
    # ============================================================
    else:
        cal_dx, cal_dy = ai_to_disp(calib_cx, calib_cy)

        if calib_done_flash > 0:
            calib_done_flash -= 1
            if calib_done_flash % 10 < 5:
                osd_img.draw_string_advanced(cal_dx - 50, cal_dy - 60, 28,
                    "CALIB OK!", color=C_GREEN)

        # 蓝色准星 (校准零点)
        osd_img.draw_circle(cal_dx, cal_dy, 16, color=C_BLUE, thickness=2)
        osd_img.draw_circle(cal_dx, cal_dy, 6,  color=C_BLUE, thickness=2)
        osd_img.draw_line(cal_dx - 25, cal_dy, cal_dx + 25, cal_dy,
                          color=C_BLUE, thickness=2)
        osd_img.draw_line(cal_dx, cal_dy - 25, cal_dx, cal_dy + 25,
                          color=C_BLUE, thickness=2)

        if ball_valid and pos_hist_full:
            dx = filt_cx - calib_cx
            dy = filt_cy - calib_cy
            if abs(dx) <= DEVIATION_DEADZONE:
                dx = 0
            if abs(dy) <= DEVIATION_DEADZONE:
                dy = 0

            if last_valid_cx >= 0:
                step = ((filt_cx - last_valid_cx) ** 2 +
                        (filt_cy - last_valid_cy) ** 2) ** 0.5
                total_distance_px += step
            last_valid_cx = filt_cx
            last_valid_cy = filt_cy

            dist_mm = total_distance_px * PIXEL_TO_MM
            current_deviation = {"dx": dx, "dy": dy, "valid": True, "dist_mm": dist_mm}

            ball_dx, ball_dy = ai_to_disp(filt_cx, filt_cy)
            osd_img.draw_line(ball_dx, ball_dy, cal_dx, cal_dy,
                              color=C_YELLOW, thickness=1)

            info_x = DISPLAY_WIDTH - 150
            osd_img.draw_string_advanced(info_x, 10, 22,
                "dX:{:+04d}".format(dx), color=C_WHITE)
            osd_img.draw_string_advanced(info_x, 34, 22,
                "dY:{:+04d}".format(dy), color=C_WHITE)
            osd_img.draw_string_advanced(info_x, 60, 20,
                "dist:{:.0f}mm".format(dist_mm), color=C_ORANGE)

            osd_img.draw_string_advanced(10, 10, 22,
                "Ball:{}".format(count), color=C_WHITE)
            osd_img.draw_string_advanced(10, 34, 18,
                "s:{:.2f}".format(best_score), color=C_WHITE)
        else:
            last_valid_cx = -1
            last_valid_cy = -1
            current_deviation = {"dx": 0, "dy": 0, "valid": False,
                                 "dist_mm": total_distance_px * PIXEL_TO_MM}
            osd_img.draw_string_advanced(10, 10, 22, "Ball:0", color=C_RED)
            osd_img.draw_string_advanced(DISPLAY_WIDTH - 150, 10, 22,
                "dX:----", color=C_RED)
            osd_img.draw_string_advanced(DISPLAY_WIDTH - 150, 34, 22,
                "dY:----", color=C_RED)

    # ---- UART发送 ----
    if frame_counter % SEND_EVERY_N_FRAMES == 0:
        uart_obj.write(format_deviation_msg(
            current_deviation["dx"],
            current_deviation["dy"],
            current_deviation["valid"]))

    # ---- 调试打印 ----
    if frame_counter % PRINT_EVERY_N_FRAMES == 0:
        st = "CALIB" if state == STATE_CALIBRATED else "CALIBRATING"
        if current_deviation["valid"]:
            print("[{}] Ball:{} dX:{:+04d} dY:{:+04d} dist:{:.0f}mm s:{:.2f}".format(
                st, count, current_deviation["dx"], current_deviation["dy"],
                current_deviation["dist_mm"], best_score))
        else:
            print("[{}] Ball:0 (lost)".format(st))


# ============================================================
# 主入口
# ============================================================

def detection():
    global state
    print("=== Ball Position (new model) ===")
    print("System time:", format_iso_time(time.time(), 0))
    if time.localtime()[0] < 2024:
        print("WARNING: RTC time is not calibrated; displayed time is invalid")
    wlan = None

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
    time.sleep_ms(CAMERA_BOOT_DELAY_MS)
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
    sensor.reset()
    sensor.set_hmirror(False)
    sensor.set_vflip(False)
    sensor.set_framesize(width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT)
    sensor.set_pixformat(PIXEL_FORMAT_YUV_SEMIPLANAR_420)
    # chn0 用于 LCD，chn2 用于 AI。
    sensor.set_framesize(width=OUT_RGB888P_WIDTH, height=OUT_RGB888P_HEIGH,
                         chn=CAM_CHN_ID_2)
    sensor.set_pixformat(PIXEL_FORMAT_RGB_888_PLANAR, chn=CAM_CHN_ID_2)

    # ---- 显示屏 ----
    sensor_bind_info = sensor.bind_info(x=0, y=0, chn=CAM_CHN_ID_0)
    Display.bind_layer(**sensor_bind_info, layer=Display.LAYER_VIDEO1)
    if display_mode == "lcd":
        Display.init(Display.ST7701, to_ide=False)
    else:
        Display.init(Display.LT9611, to_ide=False)
    osd_img = image.Image(DISPLAY_WIDTH, DISPLAY_HEIGHT, image.ARGB8888)
    mjpeg_server = LowLatencyMjpegServer(
        Display.width(), Display.height(), MJPEG_PORT)
    MediaManager.init()
    sensor.run()

    print("Creating AI output tensor")
    data = np.ones((1,3,kmodel_frame_size[1],kmodel_frame_size[0]), dtype=np.uint8)
    ai2d_output_tensor = nn.from_numpy(data)
    first_ai_frame = True
    wlan = None
    gc_frame_count = 0
    perf_frame_count = 0
    perf_start_ms = time.ticks_ms()
    print("AI loop start")

    try:
        while True:
            with ScopedTiming("total", debug_mode > 0):
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
                            mjpeg_server.start()
                            print("MJPEG (with OSD): http://{}:{}/".format(
                                wlan.ifconfig()[0], MJPEG_PORT))
                        except BaseException as e:
                            print("MJPEG disabled after initialization failure:", e)
                if rgb888p_img.format() == image.RGBP888:
                    ai2d_input = rgb888p_img.to_numpy_ref()
                    ai2d_input_tensor = nn.from_numpy(ai2d_input)
                    ai2d_builder.run(ai2d_input_tensor, ai2d_output_tensor)

                    kpu.set_input_tensor(0, ai2d_output_tensor)
                    kpu.run()

                    results = []
                    for i in range(kpu.outputs_size()):
                        out_data = kpu.get_output_tensor(i)
                        result   = out_data.to_numpy()
                        result   = result.reshape(
                            result.shape[0] * result.shape[1] *
                            result.shape[2] * result.shape[3])
                        del out_data
                        results.append(result)

                    det_boxes = aicube.anchorbasedet_post_process(
                        results[0], results[1], results[2],
                        kmodel_frame_size, frame_size, strides,
                        num_classes, DETECT_CONF_THRESHOLD,
                        nms_threshold, anchors, nms_option)

                    stable_tracks = update_tracks(det_boxes)
                    osd_img.clear()
                    draw_osd(osd_img, stable_tracks, color_four, uart)
                    Display.show_image(osd_img, 0, 0, Display.LAYER_OSD3)

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

                    del ai2d_input_tensor
                    gc_frame_count += 1
                    if gc_frame_count >= GC_EVERY_N_FRAMES:
                        gc.collect()
                        gc_frame_count = 0

    except KeyboardInterrupt:
        print("=== Stop ===")
    except BaseException as e:
        print("=== Runtime error ===", e)
        raise

    if mjpeg_server.running:
        mjpeg_server.stop()
    sensor.stop()
    Display.deinit()
    MediaManager.deinit()
    del ai2d_output_tensor
    gc.collect()
    nn.shrink_memory_pool()
    return 0


if __name__ == "__main__":
    detection()
