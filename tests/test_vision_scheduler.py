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


def test_pipe_is_observed_each_startup_frame_then_one_in_ten_when_locked():
    fn = load_functions("should_validate_pipe")["should_validate_pipe"]
    assert all(fn(frame, False) for frame in range(1, 6))
    assert [frame for frame in range(1, 31) if fn(frame, True)] == [10, 20, 30]


def test_tracking_and_recovery_rois_are_bounded_and_small():
    fn = load_functions("ball_tracking_roi")["ball_tracking_roi"]
    assert fn(160, 160, False) == (112, 112, 96, 96)
    assert fn(3, 4, False) == (0, 0, 96, 96)
    assert fn(319, 319, True) == (192, 192, 128, 128)


def test_kpu_runs_every_third_frame_during_normal_tracking():
    fn = load_functions("should_run_kpu_validation")[
        "should_run_kpu_validation"]
    assert [frame for frame in range(1, 10)
            if fn(frame, 0.0, 0.0)] == [3, 6, 9]


def test_edge_position_or_predicted_edge_forces_global_kpu_every_frame():
    functions = load_functions("should_force_global_kpu",
                               "should_run_kpu_validation")
    force = functions["should_force_global_kpu"]
    run = functions["should_run_kpu_validation"]
    assert force(10.1, 0.0) is True
    assert force(8.5, 18.0) is True
    assert force(-8.5, -18.0) is True
    assert force(4.0, 2.0) is False
    assert all(run(frame, 10.2, 0.0) for frame in range(1, 5))


def test_prediction_is_clamped_to_physical_rail_endpoints():
    fn = load_functions("clamp_ball_prediction_cm")[
        "clamp_ball_prediction_cm"]
    assert fn(13.0) == 12.5
    assert fn(-13.0) == -12.5
    assert fn(4.2) == 4.2
