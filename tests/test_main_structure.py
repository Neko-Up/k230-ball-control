import ast
from pathlib import Path


SOURCE = Path(__file__).parents[1] / "main.py"
TREE = ast.parse(SOURCE.read_text(encoding="utf-8"))
PURE_CONSTANTS = {
    target.id: ast.literal_eval(item.value)
    for item in TREE.body
    if isinstance(item, ast.Assign)
    for target in item.targets
    if isinstance(target, ast.Name)
    and target.id in {
        "MIN_BOX_SIZE", "MAX_BOX_SIZE", "MAX_ASPECT_RATIO",
        "BLOB_THRESHOLDS",
        "BLOB_MIN_PIXELS", "BLOB_MAX_PIXELS", "BLOB_MAX_ASPECT_RATIO",
        "BLOB_MAX_CENTER_DISTANCE",
    }
}


def load_pure_function(name, namespace=None):
    node = next(
        (
            item
            for item in TREE.body
            if isinstance(item, ast.FunctionDef) and item.name == name
        ),
        None,
    )
    assert node is not None, "{} is missing".format(name)
    module = ast.Module(body=[node], type_ignores=[])
    injected_namespace = dict(PURE_CONSTANTS)
    if namespace is not None:
        injected_namespace.update(namespace)
    exec(compile(module, str(SOURCE), "exec"), injected_namespace)
    return injected_namespace[name]


def test_detection_circle_geometry():
    detection_circle = load_pure_function("detection_circle")
    assert detection_circle(10, 20, 20, 12) == (20, 26, 13)
    assert detection_circle(1, 2, 2, 2) == (2, 3, 4)


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


def test_single_ai_capture_rejects_invalid_configured_boxes():
    select_best_ai_ball = load_pure_function(
        "select_best_ai_ball",
        {"MIN_BOX_SIZE": 20, "MAX_BOX_SIZE": 170, "MAX_ASPECT_RATIO": 1.8},
    )
    detections = [
        [0, 0.99, 0, 0, 10, 10],
        [0, 0.80, 0, 0, 40, 20],
    ]
    assert select_best_ai_ball(detections) is None


def test_blob_candidate_prefers_nearest_valid_ball():
    select_blob_candidate = load_pure_function("select_blob_candidate")
    candidates = [
        {"x": 90, "y": 180, "w": 20, "h": 18, "pixels": 240},
        {"x": 200, "y": 180, "w": 22, "h": 20, "pixels": 300},
        {"x": 105, "y": 180, "w": 80, "h": 5, "pixels": 300},
    ]
    result = select_blob_candidate(candidates, 100, 190)
    assert result["x"] == 90


def test_blob_detection_returns_plain_candidate_from_requested_roi():
    select_blob_candidate = load_pure_function("select_blob_candidate")
    detect_blob_measurement = load_pure_function(
        "detect_blob_measurement",
        {"select_blob_candidate": select_blob_candidate},
    )

    class Blob:
        def rect(self):
            return (90, 180, 20, 18)

        def pixels(self):
            return 240

    class Image:
        def __init__(self):
            self.call = None

        def find_blobs(self, thresholds, **kwargs):
            self.call = (thresholds, kwargs)
            return [Blob()]

    img = Image()
    roi = (20, 110, 192, 140)
    result = detect_blob_measurement(img, roi)

    assert result == {"x": 90, "y": 180, "w": 20, "h": 18, "pixels": 240}
    assert img.call == (
        [(0, 70, -20, 20, -20, 20)],
        {
            "roi": roi,
            "pixels_threshold": 40,
            "area_threshold": 40,
            "merge": False,
        },
    )


def test_blob_channel_is_best_effort_and_configured_before_media_init():
    events = []
    sensors = []

    class FakeSensorType:
        RGB565 = "rgb565"

    class FakeSensor:
        def __init__(self, enable_blob_channel):
            self.enable_blob_channel = enable_blob_channel
            self.channels = set()
            sensors.append(self)

        def reset(self):
            events.append(("reset", self.enable_blob_channel))

        def set_hmirror(self, value):
            pass

        def set_vflip(self, value):
            pass

        def set_framesize(self, **kwargs):
            self.channels.add(kwargs.get("chn", 0))

        def set_pixformat(self, pixel_format, chn=0):
            self.channels.add(chn)

        def bind_info(self, **kwargs):
            events.append(("bind_info", self.enable_blob_channel))
            return {"src": self.enable_blob_channel}

        def run(self):
            events.append(("sensor_run", self.enable_blob_channel))
            if self.enable_blob_channel:
                raise RuntimeError("deferred channel 1 failure")

        def stop(self, is_del=False):
            events.append(("sensor_stop", self.enable_blob_channel, is_del))

    class FakeDisplay:
        ST7701 = "lcd"
        LT9611 = "hdmi"
        LAYER_VIDEO1 = 1

        @staticmethod
        def bind_layer(**kwargs):
            events.append(("display_bind", kwargs["src"]))

        @staticmethod
        def init(display_type, to_ide=False):
            events.append(("display_init", display_type))

        @staticmethod
        def width():
            return 800

        @staticmethod
        def height():
            return 480

        @staticmethod
        def deinit():
            events.append(("display_deinit",))

    class FakeMediaManager:
        @staticmethod
        def init():
            events.append(("media_init", sensors[-1].enable_blob_channel))

        @staticmethod
        def deinit():
            events.append(("media_deinit",))

    class FakeImage:
        ARGB8888 = "argb"

        @staticmethod
        def Image(width, height, pixel_format):
            events.append(("osd", width, height, pixel_format))
            return "osd"

    class FakeRtspServer:
        def __init__(self, width, height, port, session):
            events.append(("rtsp", width, height, port, session))

    configure_namespace = {
        "DISPLAY_WIDTH": 800,
        "DISPLAY_HEIGHT": 480,
        "OUT_RGB888P_WIDTH": 640,
        "OUT_RGB888P_HEIGH": 360,
        "PIXEL_FORMAT_YUV_SEMIPLANAR_420": "yuv420",
        "PIXEL_FORMAT_RGB_888_PLANAR": "rgbp888",
        "CAM_CHN_ID_1": 1,
        "CAM_CHN_ID_2": 2,
        "Sensor": FakeSensorType,
    }
    configure_camera_sensor = load_pure_function(
        "configure_camera_sensor", configure_namespace)

    def create_camera_sensor(enable_blob_channel):
        sensor = FakeSensor(enable_blob_channel)
        return configure_camera_sensor(sensor, enable_blob_channel)

    cleanup_camera_start = load_pure_function(
        "cleanup_camera_start",
        {"Display": FakeDisplay, "MediaManager": FakeMediaManager},
    )
    start_camera_pipeline = load_pure_function(
        "start_camera_pipeline",
        {
            "create_camera_sensor": create_camera_sensor,
            "cleanup_camera_start": cleanup_camera_start,
            "Display": FakeDisplay,
            "MediaManager": FakeMediaManager,
            "image": FakeImage,
            "LowLatencyRtspH264Server": FakeRtspServer,
            "display_mode": "lcd",
            "DISPLAY_WIDTH": 800,
            "DISPLAY_HEIGHT": 480,
            "CAM_CHN_ID_0": 0,
            "RTSP_PORT": 8554,
            "RTSP_SESSION": "ball",
        },
    )
    start_camera_with_blob_fallback = load_pure_function(
        "start_camera_with_blob_fallback", {"print": lambda *args: None})

    sensor, osd_img, rtsp_server, blob_available = (
        start_camera_with_blob_fallback(start_camera_pipeline))

    assert sensor is sensors[1]
    assert sensors[0] is not sensors[1]
    assert sensors[0].channels == {0, 1, 2}
    assert sensors[1].channels == {0, 2}
    assert osd_img == "osd"
    assert isinstance(rtsp_server, FakeRtspServer)
    assert blob_available is False
    assert events.index(("media_init", True)) < events.index(("sensor_run", True))
    assert events.index(("sensor_stop", True, True)) < events.index(("reset", False))
    assert events.index(("media_init", False)) < events.index(("sensor_run", False))


def test_blob_setter_failure_releases_partial_sensor_before_fallback():
    class FakeGc:
        @staticmethod
        def collect():
            pass

    class FakeTime:
        @staticmethod
        def sleep_ms(duration):
            pass

    class LegacySensor:
        RGB565 = "rgb565"
        active = False

        def __init__(self, id, fps):
            if LegacySensor.active:
                raise RuntimeError("sensor already initialized")
            LegacySensor.active = True

        def reset(self):
            pass

        def set_hmirror(self, value):
            pass

        def set_vflip(self, value):
            pass

        def set_framesize(self, **kwargs):
            pass

        def set_pixformat(self, pixel_format, chn=0):
            if chn == 1:
                raise RuntimeError("channel 1 unsupported")

        def stop(self, *args, **kwargs):
            if kwargs:
                raise TypeError("legacy stop has no is_del")
            LegacySensor.active = False

    constants = {
        "DISPLAY_WIDTH": 800,
        "DISPLAY_HEIGHT": 480,
        "OUT_RGB888P_WIDTH": 640,
        "OUT_RGB888P_HEIGH": 360,
        "PIXEL_FORMAT_YUV_SEMIPLANAR_420": "yuv420",
        "PIXEL_FORMAT_RGB_888_PLANAR": "rgbp888",
        "CAM_CHN_ID_1": 1,
        "CAM_CHN_ID_2": 2,
        "CAMERA_PROBE_RETRIES": 1,
        "CAMERA_CSI_ID": 2,
        "Sensor": LegacySensor,
        "gc": FakeGc,
        "time": FakeTime,
    }
    configure_camera_sensor = load_pure_function(
        "configure_camera_sensor", constants)
    cleanup_camera_start = load_pure_function(
        "cleanup_camera_start",
        {"Display": None, "MediaManager": None},
    )
    create_camera_sensor = load_pure_function(
        "create_camera_sensor",
        dict(
            constants,
            configure_camera_sensor=configure_camera_sensor,
            cleanup_camera_start=cleanup_camera_start,
        ),
    )
    start_camera_with_blob_fallback = load_pure_function(
        "start_camera_with_blob_fallback", {"print": lambda *args: None})

    def start_attempt(enable_blob_channel):
        return create_camera_sensor(enable_blob_channel), "osd", "rtsp"

    sensor, _, _, blob_available = start_camera_with_blob_fallback(start_attempt)
    assert isinstance(sensor, LegacySensor)
    assert blob_available is False


def test_three_sample_velocity_and_bounded_prediction():
    estimate_velocity = load_pure_function("estimate_velocity")
    predict_position = load_pure_function("predict_position")
    samples = [(100, 50, 0), (104, 50, 20), (110, 52, 40)]
    vx, vy = estimate_velocity(samples)
    assert round(vx, 3) == 0.25
    assert round(vy, 3) == 0.05
    assert predict_position(110, 52, vx, vy, 40, 16) == (120, 54, False)
    assert predict_position(110, 52, 1.0, 0.0, 40, 16) == (126, 52, True)


def test_osd_updates_at_the_requested_cadence():
    should_render_osd = load_pure_function("should_render_osd")
    assert should_render_osd(1, 1) is True
    assert should_render_osd(2, 1) is True
    assert should_render_osd(3, 1) is True
    assert should_render_osd(4, 1) is True


def test_control_ui_is_full_rate_and_rtc_is_removed():
    function_names = {
        item.name for item in TREE.body if isinstance(item, ast.FunctionDef)
    }
    assert "format_iso_time" not in function_names
    text = SOURCE.read_text(encoding="utf-8")
    assert "System time:" not in text
    assert "RTC time is not calibrated" not in text
    assert "OSD_EVERY_N_FRAMES       = 1" in text


def test_uart_updates_are_not_gated_by_osd_rendering():
    draw_osd = next(
        item
        for item in TREE.body
        if isinstance(item, ast.FunctionDef) and item.name == "draw_osd"
    )
    for branch in (node for node in ast.walk(draw_osd) if isinstance(node, ast.If)):
        if not any(
            isinstance(name, ast.Name) and name.id == "render_osd"
            for name in ast.walk(branch.test)
        ):
            continue
        assert not any(
            isinstance(call, ast.Call)
            and isinstance(call.func, ast.Attribute)
            and call.func.attr == "write"
            and isinstance(call.func.value, ast.Name)
            and call.func.value.id == "uart_obj"
            for stmt in branch.body
            for call in ast.walk(stmt)
        )


def test_h264_rtsp_replaces_mjpeg_transport():
    class_names = {
        item.name for item in TREE.body if isinstance(item, ast.ClassDef)
    }
    assert "LowLatencyRtspH264Server" in class_names
    assert "LowLatencyMjpegServer" not in class_names

    assignments = {
        target.id: ast.literal_eval(item.value)
        for item in TREE.body
        if isinstance(item, ast.Assign)
        for target in item.targets
        if isinstance(target, ast.Name)
        and target.id in {"RTSP_PORT", "RTSP_SESSION", "H264_FPS"}
    }
    assert assignments == {
        "RTSP_PORT": 8554,
        "RTSP_SESSION": "ball",
        "H264_FPS": 30,
    }


def test_control_interface_and_model_paths_are_unchanged():
    format_deviation_msg = load_pure_function(
        "format_deviation_msg", {"DEVIATION_DEADZONE": 3})
    assert format_deviation_msg(12, -7, True) == b"X:+012,Y:-007\n"
    assert format_deviation_msg(2, -3, True) == b"X:+000,Y:+000\n"
    assert format_deviation_msg(0, 0, False) == b"X:----,Y:----\n"

    assignments = {
        target.id: ast.literal_eval(item.value)
        for item in TREE.body
        if isinstance(item, ast.Assign)
        for target in item.targets
        if isinstance(target, ast.Name)
        and target.id in {
            "root_path", "config_path", "UART_BAUDRATE",
            "SEND_EVERY_N_FRAMES",
        }
    }
    assert assignments == {
        "root_path": "/sdcard/mp_deployment_source/",
        "config_path": "/sdcard/mp_deployment_source/deploy_config.json",
        "UART_BAUDRATE": 115200,
        "SEND_EVERY_N_FRAMES": 1,
    }


def test_rtsp_void_and_zero_returns_are_successful():
    rtsp_call_succeeded = load_pure_function("rtsp_call_succeeded")
    assert rtsp_call_succeeded(None) is True
    assert rtsp_call_succeeded(0) is True
    assert rtsp_call_succeeded(-1) is False


def test_wifi_scan_supports_firmware_info_objects():
    wifi_scan_channel_rssi = load_pure_function("wifi_scan_channel_rssi")

    class ScanInfo:
        channel = 6
        rssi = -42

    assert wifi_scan_channel_rssi(ScanInfo()) == (6, -42)
    assert wifi_scan_channel_rssi({"channel": 11, "rssi": -70}) == (11, -70)
    assert wifi_scan_channel_rssi(("ssid", b"mac", 1, -55, 0, 0)) == (1, -55)


if __name__ == "__main__":
    test_detection_circle_geometry()
    test_single_ai_capture_selects_highest_valid_confidence()
    test_single_ai_capture_rejects_invalid_configured_boxes()
    test_blob_candidate_prefers_nearest_valid_ball()
    test_blob_detection_returns_plain_candidate_from_requested_roi()
    test_blob_channel_is_best_effort_and_configured_before_media_init()
    test_blob_setter_failure_releases_partial_sensor_before_fallback()
    test_three_sample_velocity_and_bounded_prediction()
    test_osd_updates_at_the_requested_cadence()
    test_control_ui_is_full_rate_and_rtc_is_removed()
    test_uart_updates_are_not_gated_by_osd_rendering()
    test_h264_rtsp_replaces_mjpeg_transport()
    test_control_interface_and_model_paths_are_unchanged()
    test_rtsp_void_and_zero_returns_are_successful()
    test_wifi_scan_supports_firmware_info_objects()
    print("tests: OK")
