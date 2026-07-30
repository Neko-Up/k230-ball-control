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


def base_state(now_ms=1000):
    return {
        "frequency_hz": 0.0,
        "direction": 0,
        "estimated_angle_deg": 0.0,
        "last_update_ms": now_ms,
    }


def command(**overrides):
    values = {
        "error_cm": 0.0,
        "velocity_cm_s": 0.0,
        "measurement_valid": True,
        "zeroed": True,
        "now_ms": 1020,
        "state": base_state(),
        "kp_hz_per_cm": 400.0,
        "kd_hz_per_cm_s": 30.0,
        "deadband_cm": 0.2,
        "min_frequency_hz": 120.0,
        "max_frequency_hz": 1800.0,
        "frequency_ramp_hz_s": 4000.0,
        "pulses_per_degree": 8.8889,
        "angle_limit_deg": 8.0,
        "max_motion_ms": 150,
        "direction_invert": False,
        "ticks_diff_fn": lambda now, before: now - before,
    }
    values.update(overrides)
    return load_function("compute_stepper_command")(**values)


def test_deadband_stops_stepper():
    result = command(error_cm=0.1)
    assert result["enabled"] is False
    assert result["frequency_hz"] == 0.0
    assert result["fault"] == "deadband"


def test_position_error_selects_direction_and_ramps_frequency():
    result = command(error_cm=1.0)
    assert result["enabled"] is True
    assert result["direction"] == 1
    assert result["frequency_hz"] == 120.0


def test_derivative_damping_can_reverse_command():
    result = command(error_cm=0.5, velocity_cm_s=10.0,
                     frequency_ramp_hz_s=100000.0)
    assert result["enabled"] is True
    assert result["direction"] == -1


def test_frequency_is_clamped_to_configured_maximum():
    result = command(error_cm=20.0, frequency_ramp_hz_s=1000000.0)
    assert result["frequency_hz"] == 1800.0


def test_previous_motion_is_integrated_before_new_command():
    state = base_state()
    state.update({"frequency_hz": 888.89, "direction": 1})
    result = command(state=state, error_cm=0.0)
    assert abs(result["estimated_angle_deg"] - 2.0) < 0.002


def test_outward_motion_at_angle_limit_is_blocked():
    state = base_state()
    state["estimated_angle_deg"] = 8.0
    result = command(state=state, error_cm=1.0)
    assert result["enabled"] is False
    assert result["fault"] == "angle_limit"


def test_inward_motion_at_angle_limit_is_allowed():
    state = base_state()
    state["estimated_angle_deg"] = 8.0
    result = command(state=state, error_cm=-1.0)
    assert result["enabled"] is True
    assert result["direction"] == -1


def test_invalid_or_unzeroed_state_disables_motor():
    assert command(measurement_valid=False)["fault"] == "vision_invalid"
    assert command(zeroed=False)["fault"] == "not_zeroed"
    assert command(measurement_valid=False)["enabled"] is False
    assert command(zeroed=False)["enabled"] is False


def test_direction_inversion_only_changes_physical_direction():
    normal = command(error_cm=1.0)
    inverted = command(error_cm=1.0, direction_invert=True)
    assert normal["direction"] == 1
    assert inverted["direction"] == -1
    assert normal["frequency_hz"] == inverted["frequency_hz"]


def test_long_frame_integrates_only_watchdog_bounded_motion():
    state = base_state()
    state.update({"frequency_hz": 100.0, "direction": 1})
    result = command(state=state, now_ms=3000, error_cm=0.0)
    assert abs(result["estimated_angle_deg"] - 1.6875) < 0.002


def test_frequency_deceleration_uses_same_ramp_limit():
    state = base_state()
    state.update({"frequency_hz": 500.0, "direction": 1})
    result = command(state=state, error_cm=0.5)
    assert result["frequency_hz"] == 420.0


def test_direction_reversal_decelerates_before_switching_direction():
    state = base_state()
    state.update({"frequency_hz": 500.0, "direction": 1})
    result = command(state=state, error_cm=-1.0)
    assert result["direction"] == 1
    assert result["frequency_hz"] == 420.0
    assert result["fault"] == "reversing"
