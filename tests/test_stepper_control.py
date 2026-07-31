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
        "motion_sign": 0,
        "estimated_angle_deg": 0.0,
        "target_angle_deg": 0.0,
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
        "kp_angle_deg_per_cm": 0.18,
        "kd_angle_deg_per_cm_s": 0.04,
        "edge_boost_deg_per_cm2": 0.0,
        "angle_track_hz_per_deg": 800.0,
        "angle_tolerance_deg": 0.03,
        "deadband_cm": 0.15,
        "min_frequency_hz": 40.0,
        "max_frequency_hz": 500.0,
        "frequency_ramp_hz_s": 1500.0,
        "pulses_per_degree": 8.8889,
        "angle_limit_deg": 1.0,
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
    assert result["target_angle_deg"] == 0.0
    assert result["fault"] == "angle_deadband"


def test_position_error_selects_direction_and_ramps_frequency():
    result = command(error_cm=1.0)
    assert result["enabled"] is True
    assert result["direction"] == 1
    assert result["target_angle_deg"] == 0.18
    assert result["frequency_hz"] == 40.0


def test_derivative_damping_can_reverse_command():
    result = command(error_cm=0.5, velocity_cm_s=10.0,
                     frequency_ramp_hz_s=100000.0)
    assert result["enabled"] is True
    assert result["direction"] == -1


def test_frequency_is_clamped_to_configured_maximum():
    result = command(error_cm=20.0, frequency_ramp_hz_s=1000000.0)
    assert result["target_angle_deg"] == 1.0
    assert result["frequency_hz"] == 500.0


def test_previous_motion_is_integrated_before_new_command():
    state = base_state()
    state.update({"frequency_hz": 88.889, "direction": 1,
                  "motion_sign": 1})
    result = command(state=state, error_cm=0.0)
    assert abs(result["estimated_angle_deg"] - 0.2) < 0.002


def test_outward_motion_at_angle_limit_is_blocked():
    state = base_state()
    state["estimated_angle_deg"] = 1.0
    result = command(state=state, error_cm=20.0)
    assert result["enabled"] is False
    assert result["fault"] == "angle_deadband"


def test_inward_motion_at_angle_limit_is_allowed():
    state = base_state()
    state["estimated_angle_deg"] = 1.0
    result = command(state=state, error_cm=-1.0)
    assert result["enabled"] is True
    assert result["direction"] == -1


def test_invalid_or_unzeroed_state_disables_motor():
    lost = command(measurement_valid=False)
    assert lost["fault"] == "vision_hold"
    assert lost["target_angle_deg"] == 0.0
    assert command(zeroed=False)["fault"] == "not_zeroed"
    assert lost["enabled"] is False
    assert command(zeroed=False)["enabled"] is False


def test_vision_loss_preserves_last_target_angle_for_holding_torque():
    state = base_state()
    state["target_angle_deg"] = -1.4
    result = command(state=state, measurement_valid=False)
    assert result["fault"] == "vision_hold"
    assert result["target_angle_deg"] == -1.4


def test_direction_inversion_only_changes_physical_direction():
    normal = command(error_cm=1.0)
    inverted = command(error_cm=1.0, direction_invert=True)
    assert normal["direction"] == 1
    assert inverted["direction"] == -1
    assert normal["frequency_hz"] == inverted["frequency_hz"]


def test_long_frame_integrates_only_watchdog_bounded_motion():
    state = base_state()
    state.update({"frequency_hz": 100.0, "direction": 1,
                  "motion_sign": 1})
    result = command(state=state, now_ms=3000, error_cm=0.0)
    assert result["estimated_angle_deg"] == 1.0


def test_frequency_deceleration_uses_same_ramp_limit():
    state = base_state()
    state.update({"frequency_hz": 300.0, "direction": 1,
                  "motion_sign": 1})
    result = command(state=state, error_cm=20.0)
    assert result["frequency_hz"] == 270.0


def test_direction_reversal_decelerates_before_switching_direction():
    state = base_state()
    state.update({"frequency_hz": 300.0, "direction": 1,
                  "motion_sign": 1})
    result = command(state=state, error_cm=-1.0)
    assert result["direction"] == 1
    assert result["frequency_hz"] == 270.0
    assert result["fault"] == "reversing"


def test_large_ball_error_clamps_target_angle_to_one_degree():
    result = command(error_cm=20.0, frequency_ramp_hz_s=1000000.0)
    assert result["target_angle_deg"] == 1.0
    assert result["frequency_hz"] <= 500.0


def test_edge_boost_requests_strong_lift_near_pipe_end():
    result = command(
        error_cm=12.5,
        kp_angle_deg_per_cm=0.45,
        kd_angle_deg_per_cm_s=0.06,
        edge_boost_deg_per_cm2=0.06,
        angle_limit_deg=16.0,
        frequency_ramp_hz_s=1000000.0)
    assert abs(result["target_angle_deg"] - 15.0) < 0.001


def test_fast_outward_motion_at_edge_requests_full_recovery_angle():
    result = command(
        error_cm=12.0,
        velocity_cm_s=-20.0,
        kp_angle_deg_per_cm=0.55,
        kd_angle_deg_per_cm_s=0.12,
        edge_boost_deg_per_cm2=0.07,
        angle_limit_deg=16.0,
        max_frequency_hz=800.0,
        frequency_ramp_hz_s=24000.0)
    assert result["target_angle_deg"] == 16.0
    assert result["frequency_hz"] > 400.0


def test_inner_loop_tracks_target_from_estimated_angle():
    state = base_state()
    state["estimated_angle_deg"] = 0.8
    result = command(state=state, error_cm=1.0,
                     frequency_ramp_hz_s=1000000.0)
    assert result["target_angle_deg"] == 0.18
    assert result["direction"] == -1


def test_ball_deadband_returns_nonlevel_rod_toward_zero():
    state = base_state()
    state["estimated_angle_deg"] = 0.5
    result = command(state=state, error_cm=0.1,
                     frequency_ramp_hz_s=1000000.0)
    assert result["target_angle_deg"] == 0.0
    assert result["enabled"] is True
    assert result["direction"] == -1
