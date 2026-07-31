import ast
import pathlib


SOURCE = pathlib.Path(__file__).parents[1] / "main_two_touch_calibration.py"
TREE = ast.parse(SOURCE.read_text(encoding="utf-8"), filename=str(SOURCE))


def assignment_value(name):
    node = next(item.value for item in TREE.body if isinstance(item, ast.Assign)
                for target in item.targets
                if isinstance(target, ast.Name) and target.id == name)
    return ast.literal_eval(node)


def load_function(name):
    node = next(item for item in TREE.body
                if isinstance(item, ast.FunctionDef) and item.name == name)
    namespace = {}
    exec(compile(ast.Module(body=[node], type_ignores=[]), str(SOURCE), "exec"),
         namespace)
    return namespace[name]


def test_edge_status_has_explicit_lcd_uart_labels():
    fn = load_function("format_edge_telemetry")
    assert fn("NORMAL") == "EDGE OK"
    assert fn("EDGE WARN") == "EDGE WARN"
    assert fn("EDGE RESCUE") == "EDGE RESCUE"


def test_osd_is_decimated_but_control_is_not():
    assert assignment_value("OSD_EVERY_N_FRAMES") in (2, 3)
    source = SOURCE.read_text(encoding="utf-8")
    assert "SEND_EVERY_N_FRAMES = 1" in source


def test_unreachable_center_confirmation_state_is_removed():
    source = SOURCE.read_text(encoding="utf-8")
    assert 'BALANCE_WAIT_CENTER = "WAIT_CENTER"' not in source
    assert "CENTER CONFIRM" not in source
    assert "zero is session-only" not in source


def test_cascade_arms_only_from_valid_absolute_calibration():
    detection = next(node for node in TREE.body
                     if isinstance(node, ast.FunctionDef)
                     and node.name == "detection")
    source = ast.unparse(detection)
    assert "restore_encoder_from_absolute" in source
    assert "calibrate_encoder_zero" in source
    assert "armed=encoder_zero_restored" in source
    assert source.count("cascade_controller.arm(") == 1
    assert "encoder_calibration['zero_abs_count']" in source
    assert "saved_encoder_calibration['zero_abs_count']" in source
