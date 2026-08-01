import ast
import pathlib


SOURCE = pathlib.Path(__file__).parents[1] / "main_two_touch_calibration.py"


def load_function(name):
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"), filename=str(SOURCE))
    node = next(
        item for item in tree.body
        if isinstance(item, ast.FunctionDef) and item.name == name
    )
    namespace = {}
    exec(compile(ast.Module(body=[node], type_ignores=[]), str(SOURCE), "exec"),
         namespace)
    return namespace[name]


def test_quadrature_forward_reverse_repeat_and_invalid_transitions():
    decode = load_function("quadrature_delta")
    forward = (0b00, 0b01, 0b11, 0b10, 0b00)
    reverse = tuple(reversed(forward))

    assert [decode(a, b) for a, b in zip(forward, forward[1:])] == [1] * 4
    assert [decode(a, b) for a, b in zip(reverse, reverse[1:])] == [-1] * 4
    assert [decode(state, state) for state in range(4)] == [0] * 4
    assert decode(0b00, 0b11) == 0
    assert decode(0b01, 0b10) == 0
    assert decode(0b11, 0b00) == 0
    assert decode(0b10, 0b01) == 0


def test_wrapped_encoder_delta_uses_shortest_circular_distance():
    wrapped = load_function("wrapped_encoder_delta")
    assert wrapped(0, 4095, 4096) == 1
    assert wrapped(4095, 0, 4096) == -1
    assert wrapped(2048, 0, 4096) == -2048
    assert wrapped(100, 200, 4096) == -100


def test_encoder_count_to_angle_uses_quadrature_resolution():
    convert = load_function("encoder_count_to_angle")
    assert convert(1, 4096) == 0.087890625
    assert convert(-1024, 4096) == -90.0
    assert convert(1, 0) == 0.0


def test_pwm_duty_to_count_maps_configured_duty_window():
    convert = load_function("pwm_duty_to_count")
    assert convert(250, 1000, 4096, 0.25, 0.75, False) == 0
    assert convert(500, 1000, 4096, 0.25, 0.75, False) == 2048
    assert convert(750, 1000, 4096, 0.25, 0.75, False) == 4095
    assert convert(250, 1000, 4096, 0.25, 0.75, True) == 4095
    assert convert(0, 0, 4096, 0.25, 0.75, False) is None
    assert convert(100, 1000, 4096, 0.25, 0.75, False) is None
    assert convert(900, 1000, 4096, 0.25, 0.75, False) is None


def valid_calibration():
    return {
        "version": 1,
        "encoder_model": "MS42CG",
        "counts_per_rev": 4096,
        "zero_abs_count": 123,
        "pwm_invert": False,
        "z_index_count": 3000,
    }


def test_validate_encoder_calibration_accepts_only_exact_schema():
    validate = load_function("validate_encoder_calibration")
    expected = valid_calibration()
    assert validate(dict(expected)) == expected

    for key, value in (
            ("version", 2),
            ("encoder_model", "other"),
            ("counts_per_rev", 1024),
            ("zero_abs_count", 4096),
            ("zero_abs_count", -1),
            ("zero_abs_count", 1.5),
            ("pwm_invert", 0),
            ("z_index_count", 4096),
            ("z_index_count", 2.5)):
        bad = valid_calibration()
        bad[key] = value
        assert validate(bad) is None

    without_optional_z = valid_calibration()
    del without_optional_z["z_index_count"]
    assert validate(without_optional_z) == without_optional_z

