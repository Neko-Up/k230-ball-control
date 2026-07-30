import ast
import pathlib


SOURCE = pathlib.Path(__file__).parents[1] / "main_two_touch_calibration.py"
TEXT = SOURCE.read_text(encoding="utf-8")
TREE = ast.parse(TEXT, filename=str(SOURCE))


def assigned_value(name):
    for node in TREE.body:
        if isinstance(node, ast.Assign):
            if any(isinstance(target, ast.Name) and target.id == name
                   for target in node.targets):
                return ast.literal_eval(node.value)
    raise AssertionError("missing assignment: " + name)


def test_d36a_pin_assignment_matches_k230_header():
    assert assigned_value("STEPPER_STEP_IO") == 42
    assert assigned_value("STEPPER_PWM_CHANNEL") == 0
    assert assigned_value("STEPPER_DIR_IO") == 5
    assert assigned_value("STEPPER_EN_IO") == 6
    assert assigned_value("STEPPER_WATCHDOG_TIMER_ID") == -1


def test_d36a_adapter_owns_pwm_direction_and_active_high_enable():
    cls = next(
        node for node in TREE.body
        if isinstance(node, ast.ClassDef) and node.name == "D36AStepper"
    )
    methods = {
        node.name: node for node in cls.body if isinstance(node, ast.FunctionDef)
    }
    assert {"__init__", "apply", "stop", "deinit"} <= set(methods)
    init_text = ast.unparse(methods["__init__"])
    assert "PWM(STEPPER_PWM_CHANNEL" in init_text
    pwm_call = next(
        node for node in ast.walk(methods["__init__"])
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name) and node.func.id == "PWM"
    )
    assert len(pwm_call.args) == 1
    assert pwm_call.keywords == []
    assert "self.pwm.freq(int(STEPPER_MIN_FREQUENCY_HZ))" in init_text
    assert "self.pwm.duty(0)" in init_text
    assert ".enable(" not in init_text
    assert "self.en_pin.value(0)" in init_text
    apply_text = ast.unparse(methods["apply"])
    assert "self.dir_pin.value" in apply_text
    assert "self.pwm.freq" in apply_text
    assert "self.en_pin.value(1)" in apply_text
    assert ".enable(" not in apply_text
    assert "self.last_direction" in apply_text
    assert "self.pwm.duty(50)" in apply_text
    assert "self.watchdog.init" in apply_text
    assert "except Exception" in apply_text
    assert "self.stop(disable=True)" in apply_text
    stop_text = ast.unparse(methods["stop"])
    assert ".enable(" not in stop_text
    assert "self.pwm.duty(0)" in stop_text
    assert "self.en_pin.value(0)" in stop_text
    assert "self.en_pin.value(1)" in stop_text


def test_detection_cleanup_deinitializes_stepper_before_media_cleanup():
    detection = next(
        node for node in TREE.body
        if isinstance(node, ast.FunctionDef) and node.name == "detection"
    )
    calls = [
        node for node in ast.walk(detection)
        if isinstance(node, ast.Call)
    ]
    deinit = [
        node for node in calls
        if isinstance(node.func, ast.Attribute)
        and node.func.attr == "deinit"
        and isinstance(node.func.value, ast.Name)
        and node.func.value.id == "stepper"
    ]
    cleanup = [
        node for node in calls
        if isinstance(node.func, ast.Name)
        and node.func.id == "cleanup_runtime_resources"
    ]
    assert len(deinit) == 1
    assert len(cleanup) == 1
    assert deinit[0].lineno < cleanup[0].lineno
