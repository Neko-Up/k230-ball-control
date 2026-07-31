import ast
import pathlib


SOURCE = pathlib.Path(__file__).parents[1] / "main_two_touch_calibration.py"
TREE = ast.parse(SOURCE.read_text(encoding="utf-8"), filename=str(SOURCE))


def load_functions(*names):
    selected = [
        node for node in TREE.body
        if isinstance(node, ast.FunctionDef) and node.name in names
    ]
    namespace = {}
    exec(compile(ast.Module(body=selected, type_ignores=[]), str(SOURCE), "exec"),
         namespace)
    return namespace


class Touch:
    def __init__(self, event, x=400, y=240):
        self.event = event
        self.x = x
        self.y = y


def test_touch_release_requests_zero_only_when_target_was_ready():
    fn = load_functions("handle_stepper_zero_touch")[
        "handle_stepper_zero_touch"]
    state = {"zeroed": False, "frequency_hz": 0.0, "direction": 0}
    zero_rect = (180, 190, 440, 100)
    untouched = fn(state, [Touch(2)], True, False, 20, 2, zero_rect)
    assert untouched["zeroed"] is False
    outside = fn(state, [Touch(2, 20, 20)], True, True, 20, 2, zero_rect)
    assert outside["zeroed"] is False
    armed = fn(state, [Touch(2)], True, True, 20, 2, zero_rect)
    assert armed["zeroed"] is True


def test_visual_loop_only_publishes_target_to_cascade_controller():
    update = next(
        node for node in TREE.body
        if isinstance(node, ast.FunctionDef) and node.name == "update_stepper_control"
    )
    source = ast.unparse(update)
    assert ".set_visual_target(" in source
    assert ".status(" in source
    assert ".apply(" not in source
    assert "stepper.apply" not in source


def test_only_cascade_tick_applies_d36a_commands():
    apply_calls = [
        node for node in ast.walk(TREE)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "apply"
    ]
    assert len(apply_calls) == 1
    cascade = next(
        node for node in TREE.body
        if isinstance(node, ast.ClassDef) and node.name == "RodCascadeController"
    )
    tick = next(
        node for node in cascade.body
        if isinstance(node, ast.FunctionDef) and node.name == "tick"
    )
    assert apply_calls[0] in list(ast.walk(tick))


def test_publish_uses_cascade_status_before_uart_lcd_rendering():
    publish = next(
        node for node in TREE.body
        if isinstance(node, ast.FunctionDef) and node.name == "publish_control_outputs"
    )
    args = [arg.arg for arg in publish.args.args]
    assert "cascade_controller" in args
    assert "stepper" not in args
    source = ast.unparse(publish)
    assert "update_stepper_control" in source
    assert "draw_osd" in source


def test_stepper_uart_frame_exposes_real_encoder_telemetry():
    fn = load_functions("format_stepper_msg")["format_stepper_msg"]
    message = fn({
        "zeroed": True, "enabled": True, "frequency_hz": 345.4,
        "direction": -1, "actual_angle_deg": 0.25,
        "target_angle_deg": -0.40, "actual_velocity_deg_s": 12.3,
        "angle_error_deg": -0.65, "control_rate_hz": 199.5,
        "fault": "none",
    })
    assert message == (
        b"M:1,R:1,F:0345,D:-1,A:+0.25,T:-0.40,V:+12.3,E:-0.65,H:199.5,S:none\n")


def test_lcd_exposes_actual_target_velocity_error_and_rate():
    draw = next(
        node for node in TREE.body
        if isinstance(node, ast.FunctionDef) and node.name == "draw_osd"
    )
    source = ast.unparse(draw)
    assert "A:{:+.2f} T:{:+.2f}" in source
    assert "V:{:+.1f} E:{:+.2f}" in source
    assert "ENC:{} {:3.0f}Hz" in source


def test_detection_wires_encoder_calibration_and_safe_cleanup_order():
    detection = next(
        node for node in TREE.body
        if isinstance(node, ast.FunctionDef) and node.name == "detection"
    )
    source = ast.unparse(detection)
    assert "MS42CGEncoder(fpioa)" in source
    assert "load_encoder_calibration()" in source
    assert "restore_encoder_from_absolute" in source
    assert "RodCascadeController(" in source
    assert "save_encoder_calibration(" in source
    assert source.find("cascade_controller.deinit()") < source.find("encoder.deinit()")
    assert source.find("encoder.deinit()") < source.find("stepper.deinit()")


def test_target_position_is_drawn_outside_measurement_valid_branch():
    draw = next(
        node for node in TREE.body
        if isinstance(node, ast.FunctionDef) and node.name == "draw_osd"
    )
    source = ast.unparse(draw)
    target_label = source.find("T:{:+.2f}cm")
    measurement_branch = source.find("elif measurement['valid']")
    assert target_label >= 0
    assert target_label < measurement_branch


def test_axis_distance_is_drawn_before_zero_prompt_overlay():
    draw = next(
        node for node in TREE.body
        if isinstance(node, ast.FunctionDef) and node.name == "draw_osd"
    )
    source = ast.unparse(draw)
    measurement_branch = source.find("elif measurement['valid']")
    distance_label = source.find("|O-B|:{:.2f}cm")
    zero_prompt = source.find("LEVEL ROD - TAP ZERO")
    assert measurement_branch >= 0
    assert distance_label > measurement_branch
    assert zero_prompt > distance_label

