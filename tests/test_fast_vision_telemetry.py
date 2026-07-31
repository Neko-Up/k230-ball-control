import ast
import pathlib


SOURCE = pathlib.Path(__file__).parents[1] / "main_two_touch_calibration.py"
TREE = ast.parse(SOURCE.read_text(encoding="utf-8"), filename=str(SOURCE))


def assignment_value(name, namespace=None):
    node = next(
        item.value
        for item in TREE.body
        if isinstance(item, ast.Assign)
        for target in item.targets
        if isinstance(target, ast.Name) and target.id == name
    )
    values = {} if namespace is None else dict(namespace)
    return eval(compile(ast.Expression(node), str(SOURCE), "eval"), values)


def load_function(name):
    node = next(
        item for item in TREE.body
        if isinstance(item, ast.FunctionDef) and item.name == name
    )
    namespace = {}
    exec(compile(ast.Module(body=[node], type_ignores=[]), str(SOURCE), "exec"),
         namespace)
    return namespace[name]


def test_ai_and_blob_channels_use_320_square_coordinates():
    align_up = lambda value, alignment: (
        (value + alignment - 1) // alignment * alignment)
    width = assignment_value("OUT_RGB888P_WIDTH", {"ALIGN_UP": align_up})
    height = assignment_value("OUT_RGB888P_HEIGH")
    assert (width, height) == (320, 320)

    for name in ("AI_ROD_ROI", "BLOB_GLOBAL_ROI", "PIPE_GLOBAL_ROI"):
        x, y, roi_width, roi_height = assignment_value(name)
        assert 0 <= x < width
        assert 0 <= y < height
        assert 0 < roi_width <= width - x
        assert 0 < roi_height <= height - y


def test_runtime_fps_uses_real_elapsed_window_time():
    assert assignment_value("PERF_EVERY_N_FRAMES") == 15
    compute_window_fps = load_function("compute_window_fps")
    assert compute_window_fps(15, 500) == 30.0
    assert compute_window_fps(15, 0) == 0.0


def test_lcd_ball_telemetry_formats_position_velocity_and_fps():
    format_ball_telemetry = load_function("format_ball_telemetry")
    lines = format_ball_telemetry({
        "valid": True,
        "ball_position_cm": -3.25,
        "velocity_cm_s": 12.4,
    }, 28.75)
    assert lines == (
        "P:-3.25cm",
        "BV:+12.4cm/s",
        "AI:28.8FPS",
    )

    assert format_ball_telemetry({"valid": False}, 0.0) == (
        "P:--cm",
        "BV:--cm/s",
        "AI:0.0FPS",
    )


def test_lcd_render_path_consumes_cached_runtime_telemetry():
    source = SOURCE.read_text(encoding="utf-8")
    assert 'runtime_telemetry = {"vision_fps": 0.0}' in source
    assert 'runtime_telemetry["vision_fps"]' in source
    assert "format_ball_telemetry(" in source
