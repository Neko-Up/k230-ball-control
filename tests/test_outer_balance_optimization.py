import ast
import pathlib


SOURCE = pathlib.Path(__file__).parents[1] / "main_two_touch_calibration.py"


def load_functions(*names):
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"), filename=str(SOURCE))
    nodes = [node for node in tree.body
             if isinstance(node, ast.FunctionDef) and node.name in names]
    namespace = {}
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(SOURCE), "exec"),
         namespace)
    return namespace


def new_state():
    return {
        "integral_deg": 0.0,
        "last_target_deg": 0.0,
        "last_edge_side": 0,
        "last_valid_ms": -1000,
        "edge_state": "NORMAL",
    }


def test_alpha_beta_filter_uses_real_elapsed_time_for_velocity():
    update = load_functions("alpha_beta_update")["alpha_beta_update"]
    state = update(None, 0.0, 1000)
    state = update(state, 1.0, 1050, alpha=1.0, beta=1.0)
    assert state["position_cm"] == 1.0
    assert 19.9 < state["velocity_cm_s"] < 20.1


def test_integral_bias_is_conditional_and_limited_to_point_three_degree():
    compute = load_functions("compute_outer_balance_target")[
        "compute_outer_balance_target"]
    state = new_state()
    for index in range(100):
        _, state = compute(1.0, 0.0, 0.05, state, -1.0, True, index * 50)
    assert 0.0 < state["integral_deg"] <= 0.3
    _, moving = compute(1.0, 8.0, 0.05, state, -1.0, True, 6000)
    assert moving["integral_deg"] == state["integral_deg"]


def test_near_centre_command_is_small_and_settles_level():
    compute = load_functions("compute_outer_balance_target")[
        "compute_outer_balance_target"]
    angle, state = compute(0.2, 0.0, 0.033, new_state(), -0.2, True, 100)
    assert abs(angle) <= 0.3
    angle, _ = compute(0.02, 0.1, 0.033, state, -0.02, True, 133)
    assert abs(angle) < 0.08


def test_velocity_term_brakes_reversal_instead_of_accelerating_it():
    compute = load_functions("compute_outer_balance_target")[
        "compute_outer_balance_target"]
    unbraked, _ = compute(2.0, 0.0, 0.033, new_state(), -2.0, True, 100)
    braking, _ = compute(2.0, 8.0, 0.033, new_state(), -2.0, True, 100)
    assert braking < unbraked


def test_speed_prediction_enters_edge_warning_before_ball_touches_end():
    compute = load_functions("compute_outer_balance_target")[
        "compute_outer_balance_target"]
    angle, state = compute(-8.5, 18.0, 0.033, new_state(), 8.5, True, 100)
    assert state["edge_state"] in ("EDGE WARN", "EDGE RESCUE")
    assert angle < 0.0


def test_soft_edge_never_allows_outward_command():
    compute = load_functions("compute_outer_balance_target")[
        "compute_outer_balance_target"]
    angle, state = compute(3.0, 0.0, 0.033, new_state(), 10.2, True, 100)
    assert state["edge_state"] == "EDGE WARN"
    assert angle <= 0.0


def test_hard_edge_overrides_pid_with_strong_inward_rescue():
    compute = load_functions("compute_outer_balance_target")[
        "compute_outer_balance_target"]
    right_angle, right = compute(5.0, 12.0, 0.033, new_state(), 11.6,
                                 True, 100)
    left_angle, left = compute(-5.0, -12.0, 0.033, new_state(), -11.6,
                               True, 100)
    assert right["edge_state"] == "EDGE RESCUE"
    assert left["edge_state"] == "EDGE RESCUE"
    assert right_angle <= -3.0
    assert left_angle >= 3.0


def test_edge_loss_retains_side_for_200ms_then_levels():
    compute = load_functions("compute_outer_balance_target")[
        "compute_outer_balance_target"]
    _, state = compute(-10.0, 15.0, 0.033, new_state(), 11.7, True, 100)
    rescue, state = compute(0.0, 0.0, 0.033, state, 0.0, False, 250)
    expired, state = compute(0.0, 0.0, 0.033, state, 0.0, False, 301)
    assert rescue < 0.0
    assert state["edge_state"] == "NORMAL"
    assert expired == 0.0


def test_output_limit_never_exceeds_five_degrees():
    compute = load_functions("compute_outer_balance_target")[
        "compute_outer_balance_target"]
    angle, _ = compute(-30.0, 100.0, 0.1, new_state(), 12.0, True, 100)
    assert abs(angle) <= 5.0
