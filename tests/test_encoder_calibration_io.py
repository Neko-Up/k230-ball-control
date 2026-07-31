import ast
import json
import pathlib


SOURCE = pathlib.Path(__file__).parents[1] / "main_two_touch_calibration.py"


def load_functions(*names):
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"), filename=str(SOURCE))
    selected = [
        node for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name in names
    ]
    namespace = {
        "ujson": json,
        "validate_encoder_calibration": None,
        "ENCODER_COUNTS_PER_REV": 4096,
        "ENCODER_PWM_INVERT": False,
        "ENCODER_CALIBRATION_PATH": "/sdcard/ms42cg_encoder_calibration.json",
    }
    validator = next(
        node for node in tree.body
        if isinstance(node, ast.FunctionDef) and
        node.name == "validate_encoder_calibration"
    )
    namespace.pop("validate_encoder_calibration")
    exec(compile(ast.Module(body=[validator] + selected, type_ignores=[]),
                 str(SOURCE), "exec"), namespace)
    return namespace


def calibration(zero=123, invert=False):
    return {
        "version": 1,
        "encoder_model": "MS42CG",
        "counts_per_rev": 4096,
        "zero_abs_count": zero,
        "pwm_invert": invert,
    }


def test_load_accepts_valid_and_rejects_missing_malformed_or_wrong_schema(tmp_path):
    load = load_functions("load_encoder_calibration")["load_encoder_calibration"]
    valid = tmp_path / "valid.json"
    valid.write_text(json.dumps(calibration()), encoding="utf-8")
    assert load(str(valid)) == calibration()
    assert load(str(tmp_path / "missing.json")) is None
    malformed = tmp_path / "malformed.json"
    malformed.write_text("{", encoding="utf-8")
    assert load(str(malformed)) is None
    wrong = tmp_path / "wrong.json"
    wrong.write_text(json.dumps(calibration(zero=4096)), encoding="utf-8")
    assert load(str(wrong)) is None


def test_save_writes_temp_before_atomic_rename(tmp_path):
    save = load_functions("save_encoder_calibration")["save_encoder_calibration"]
    destination = tmp_path / "encoder.json"
    rename_calls = []

    def rename(source, target):
        rename_calls.append((source, target))
        pathlib.Path(source).replace(target)

    assert save(str(destination), calibration(), rename_fn=rename) is True
    assert json.loads(destination.read_text(encoding="utf-8")) == calibration()
    assert rename_calls == [(str(destination) + ".tmp", str(destination))]


def test_save_failure_never_replaces_good_calibration(tmp_path):
    save = load_functions("save_encoder_calibration")["save_encoder_calibration"]
    destination = tmp_path / "encoder.json"
    destination.write_text(json.dumps(calibration(77)), encoding="utf-8")
    rename_calls = []

    class BrokenFile:
        def __enter__(self):
            return self

        def __exit__(self, *args):
            return False

        def write(self, data):
            raise OSError("storage full")

    assert save(str(destination), calibration(88),
                open_fn=lambda *args, **kwargs: BrokenFile(),
                rename_fn=lambda *args: rename_calls.append(args)) is False
    assert json.loads(destination.read_text(encoding="utf-8"))["zero_abs_count"] == 77
    assert rename_calls == []


def test_calibrate_encoder_zero_builds_separate_valid_record():
    fn = load_functions("calibrate_encoder_zero")["calibrate_encoder_zero"]
    result = fn({"pwm_valid": True, "absolute_count": 4095}, 12)
    assert result == {
        "version": 1,
        "encoder_model": "MS42CG",
        "counts_per_rev": 4096,
        "zero_abs_count": 4095,
        "pwm_invert": False,
        "z_index_count": 12,
    }
    assert fn({"pwm_valid": False, "absolute_count": 4}) is None


class FakeEncoder:
    def __init__(self, absolute_count):
        self.absolute_count = absolute_count
        self.absolute_zero_count = None
        self.count = None
        self.snapshot_count = None


def test_startup_restore_seeds_ab_count_with_circular_difference():
    restore = load_functions(
        "wrapped_encoder_delta", "restore_encoder_from_absolute"
    )["restore_encoder_from_absolute"]
    encoder = FakeEncoder(0)
    assert restore(encoder, calibration(zero=4095),
                   {"pwm_valid": True, "absolute_count": 0}) is True
    assert encoder.count == 1
    encoder = FakeEncoder(4095)
    assert restore(encoder, calibration(zero=0),
                   {"pwm_valid": True, "absolute_count": 4095}) is True
    assert encoder.count == -1
    assert restore(encoder, calibration(),
                   {"pwm_valid": False, "absolute_count": None}) is False
