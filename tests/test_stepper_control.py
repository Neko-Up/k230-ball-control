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


def base_state(frequency_hz=0.0, direction=0, integral_hz=0.0):
    return {
        "integral_hz": integral_hz,
        "frequency_hz": frequency_hz,
        "direction": direction,
        "last_update_ms": 0,
        "fault": "none",
    }


def command(**overrides):
    values = {
        "target_angle_deg": 0.0,
        "actual_angle_deg": 0.0,
        "actual_velocity_deg_s": 0.0,
        "dt_s": 0.005,
        "state": base_state(),
        "kp_hz_per_deg": 400.0,
        "ki_hz_per_deg_s": 20.0,
        "kd_hz_per_deg_s": 2.0,
        "max_frequency_hz": 800.0,
        "ramp_hz_s": 24000.0,
        "angle_limit_deg": 16.0,
    }
    values.update(overrides)
    return load_function("compute_angle_pid")(**values)


def test_positive_and_negative_angle_errors_select_direction():
    assert command(target_angle_deg=1.0)["direction"] == 1
    assert command(target_angle_deg=-1.0)["direction"] == -1


def test_one_encoder_count_deadband_stops_output():
    result = command(target_angle_deg=0.08)
    assert result["enabled"] is False
    assert result["frequency_hz"] == 0.0
    assert result["fault"] == "angle_deadband"


def test_measured_velocity_provides_derivative_braking():
    unbraked = command(target_angle_deg=1.0, actual_velocity_deg_s=0.0,
                       ramp_hz_s=100000.0)
    braked = command(target_angle_deg=1.0, actual_velocity_deg_s=100.0,
                     ramp_hz_s=100000.0)
    assert braked["frequency_hz"] < unbraked["frequency_hz"]


def test_integral_accumulates_but_does_not_wind_up_at_saturation():
    accumulating = command(target_angle_deg=0.5, dt_s=0.1,
                           ramp_hz_s=100000.0)
    assert accumulating["integral_hz"] > 0.0
    saturated = command(target_angle_deg=16.0, dt_s=1.0,
                        state=base_state(integral_hz=7.0),
                        ramp_hz_s=100000.0)
    assert saturated["frequency_hz"] == 800.0
    assert saturated["integral_hz"] == 7.0


def test_directional_angle_limits_block_only_outward_motion():
    outward = command(target_angle_deg=16.0, actual_angle_deg=16.0)
    inward = command(target_angle_deg=0.0, actual_angle_deg=16.0)
    assert outward["enabled"] is False
    assert outward["fault"] == "angle_limit"
    assert inward["enabled"] is True
    assert inward["direction"] == -1


def test_frequency_is_saturated_and_ramped():
    saturated = command(target_angle_deg=10.0, ramp_hz_s=1000000.0)
    ramped = command(target_angle_deg=10.0, ramp_hz_s=1000.0)
    assert saturated["frequency_hz"] == 800.0
    assert ramped["frequency_hz"] == 5.0


def test_reversal_decelerates_to_zero_before_switching_direction():
    state = base_state(frequency_hz=300.0, direction=1)
    braking = command(target_angle_deg=-1.0, state=state, dt_s=0.005)
    assert braking["direction"] == 1
    assert braking["frequency_hz"] == 180.0
    assert braking["fault"] == "reversing"
    stopped = command(target_angle_deg=-1.0, state=state, dt_s=0.020)
    assert stopped["enabled"] is False
    assert stopped["frequency_hz"] == 0.0
    assert stopped["fault"] == "reversing"


def test_production_control_never_integrates_step_frequency_into_angle():
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"), filename=str(SOURCE))
    production_text = ast.unparse(tree)
    forbidden = (
        "previous_sign * previous_frequency * elapsed_s / pulses_per_degree",
        "frequency_hz * elapsed_s / pulses_per_degree",
    )
    assert all(expression not in production_text for expression in forbidden)
