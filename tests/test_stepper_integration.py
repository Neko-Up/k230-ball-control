import ast
import pathlib


SOURCE = pathlib.Path(__file__).parents[1] / "main_two_touch_calibration.py"
TREE = ast.parse(SOURCE.read_text(encoding="utf-8"), filename=str(SOURCE))


def load_functions(*names, extra=None):
    selected = [
        node for node in TREE.body
        if isinstance(node, ast.FunctionDef) and node.name in names
    ]
    namespace = {} if extra is None else dict(extra)
    exec(compile(ast.Module(body=selected, type_ignores=[]), str(SOURCE), "exec"),
         namespace)
    return namespace


class Touch:
    def __init__(self, event, x=400, y=240):
        self.event = event
        self.x = x
        self.y = y


def test_touch_release_arms_zero_only_when_target_was_already_ready():
    fn = load_functions("handle_stepper_zero_touch")[
        "handle_stepper_zero_touch"]
    state = {"zeroed": False, "estimated_angle_deg": 3.0,
             "frequency_hz": 200.0, "direction": 1,
             "motion_sign": 1, "last_update_ms": 10, "fault": "not_zeroed"}
    zero_rect = (180, 190, 440, 100)
    untouched = fn(state, [Touch(2)], True, False, 20, 2, zero_rect)
    assert untouched["zeroed"] is False
    outside = fn(state, [Touch(2, 20, 20)], True, True, 20, 2,
                 zero_rect)
    assert outside["zeroed"] is False
    armed = fn(state, [Touch(2)], True, True, 20, 2, zero_rect)
    assert armed["zeroed"] is True
    assert armed["estimated_angle_deg"] == 0.0
    assert armed["frequency_hz"] == 0.0


def test_publish_updates_stepper_before_uart_lcd_rendering():
    publish = next(
        node for node in TREE.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "publish_control_outputs"
    )
    args = [arg.arg for arg in publish.args.args]
    assert "stepper" in args
    assert "stepper_state" in args
    calls = [
        node for node in ast.walk(publish)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id in {"update_stepper_control", "draw_osd"}
    ]
    lines = {node.func.id: node.lineno for node in calls}
    assert lines["update_stepper_control"] < lines["draw_osd"]
    assert any(
        isinstance(node, ast.Return) and isinstance(node.value, ast.Name)
        and node.value.id == "stepper_state"
        for node in publish.body
    )


def test_detection_uses_returned_stepper_state_for_every_publish():
    detection = next(
        node for node in TREE.body
        if isinstance(node, ast.FunctionDef) and node.name == "detection"
    )
    publish_calls = [
        node for node in ast.walk(detection)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "publish_control_outputs"
    ]
    assert len(publish_calls) == 2
    for call in publish_calls:
        parent_assignments = [
            node for node in ast.walk(detection)
            if isinstance(node, ast.Assign) and node.value is call
        ]
        assert len(parent_assignments) == 1
        assert any(
            isinstance(target, ast.Name) and target.id == "stepper_state"
            for target in parent_assignments[0].targets
        )


def test_stepper_uart_frame_exposes_realtime_command():
    fn = load_functions("format_stepper_msg")["format_stepper_msg"]
    message = fn({
        "zeroed": True, "enabled": True, "frequency_hz": 345.4,
        "direction": -1, "estimated_angle_deg": 1.25,
        "fault": "none",
    })
    assert message == b"M:1,R:1,F:0345,D:-1,A:+1.25,E:none\n"


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
