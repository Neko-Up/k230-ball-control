import ast
import pathlib


SOURCE = pathlib.Path(__file__).parents[1] / "main_two_touch_calibration.py"


def load_controller(extra):
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"), filename=str(SOURCE))
    names = {"compute_angle_pid", "encoder_count_to_angle"}
    nodes = [
        node for node in tree.body
        if ((isinstance(node, ast.FunctionDef) and node.name in names) or
            (isinstance(node, ast.ClassDef) and
             node.name == "RodCascadeController"))
    ]
    namespace = dict(extra)
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(SOURCE), "exec"),
         namespace)
    return namespace["RodCascadeController"]


class FakeTimer:
    PERIODIC = 1

    def __init__(self, timer_id):
        self.timer_id = timer_id
        self.kwargs = None
        self.deinitialized = False

    def init(self, **kwargs):
        if "hard" in kwargs:
            raise TypeError("extra keyword arguments given")
        self.kwargs = kwargs

    def deinit(self):
        self.deinitialized = True


class FakeEncoder:
    def __init__(self):
        self.data = {
            "count": 0,
            "angle_deg": 0.0,
            "velocity_deg_s": 0.0,
            "absolute_count": None,
            "pwm_valid": False,
            "z_seen": False,
            "invalid_transitions": 0,
            "last_edge_us": 100000,
        }

    def snapshot(self, now_us):
        return dict(self.data)


class FakeStepper:
    def __init__(self):
        self.commands = []
        self.stops = []

    def apply(self, command):
        self.commands.append(dict(command))

    def stop(self, disable=True, cancel_watchdog=True):
        self.stops.append(disable)


def new_controller():
    clock = {"ms": 100, "us": 100000}
    extra = {
        "Timer": FakeTimer,
        "CASCADE_TIMER_ID": -1,
        "CASCADE_PERIOD_MS": 5,
        "STEPPER_VISION_TIMEOUT_MS": 150,
        "ENCODER_STALL_TIMEOUT_MS": 80,
        "ENCODER_PWM_MISMATCH_DEG": 2.0,
        "ENCODER_COUNTS_PER_REV": 4096,
        "CASCADE_MIN_RATE_HZ": 80.0,
        "CASCADE_RATE_WINDOW_MS": 200,
        "STEPPER_ANGLE_LIMIT_DEG": 16.0,
        "INNER_KP_HZ_PER_DEG": 400.0,
        "INNER_KI_HZ_PER_DEG_S": 20.0,
        "INNER_KD_HZ_PER_DEG_S": 2.0,
        "STEPPER_MAX_FREQUENCY_HZ": 800.0,
        "STEPPER_FREQUENCY_RAMP_HZ_S": 24000.0,
    }
    cls = load_controller(extra)
    encoder = FakeEncoder()
    stepper = FakeStepper()
    controller = cls(
        encoder, stepper,
        ticks_ms_fn=lambda: clock["ms"],
        ticks_us_fn=lambda: clock["us"],
        ticks_diff_fn=lambda now, before: now - before,
        timer_factory=FakeTimer)
    return controller, encoder, stepper, clock


def tick(controller, clock, advance_ms=5):
    clock["ms"] += advance_ms
    clock["us"] += advance_ms * 1000
    controller.tick(None)


def test_controller_owns_single_5ms_soft_timer():
    controller, _, _, _ = new_controller()
    assert controller.timer.timer_id == -1
    assert controller.timer.kwargs["mode"] == FakeTimer.PERIODIC
    assert controller.timer.kwargs["period"] == 5
    assert controller.timer.kwargs["callback"] == controller.tick
    assert "hard" not in controller.timer.kwargs


def test_stale_visual_target_stops_motion():
    controller, _, stepper, clock = new_controller()
    controller.set_visual_target(2.0, clock["ms"] - 151, True)
    tick(controller, clock)
    assert controller.status(clock["ms"])["fault"] == "VISION TIMEOUT"
    assert stepper.stops[-1] is False


def test_commanded_motion_without_encoder_edges_faults():
    controller, encoder, stepper, clock = new_controller()
    controller.set_visual_target(2.0, clock["ms"], True)
    encoder.data["last_edge_us"] = clock["us"] - 81000
    tick(controller, clock)
    assert controller.status(clock["ms"])["fault"] == "ENCODER STALL"
    assert stepper.stops[-1] is True


def test_absolute_pwm_mismatch_faults_ab_feedback():
    controller, encoder, _, clock = new_controller()
    controller.set_visual_target(1.0, clock["ms"], True)
    encoder.data.update({
        "pwm_valid": True,
        "absolute_count": 100,
        "angle_deg": 10.0,
    })
    controller.absolute_zero_count = 100
    tick(controller, clock)
    assert controller.status(clock["ms"])["fault"] == "ENC MISMATCH"


def test_new_invalid_ab_transition_faults():
    controller, encoder, _, clock = new_controller()
    controller.set_visual_target(1.0, clock["ms"], True)
    encoder.data["invalid_transitions"] = 1
    tick(controller, clock)
    assert controller.status(clock["ms"])["fault"] == "AB INVALID"


def test_sustained_callback_rate_below_80hz_faults():
    controller, _, _, clock = new_controller()
    controller.set_visual_target(0.0, clock["ms"], True)
    for _ in range(10):
        tick(controller, clock, advance_ms=25)
        controller.set_visual_target(0.0, clock["ms"], True)
    assert controller.status(clock["ms"])["fault"] == "CTRL SLOW"


def test_valid_feedback_applies_real_angle_pid_and_can_recover():
    controller, encoder, stepper, clock = new_controller()
    controller.set_visual_target(1.0, clock["ms"], True)
    tick(controller, clock)
    assert stepper.commands[-1]["enabled"] is True
    assert stepper.commands[-1]["direction"] == 1
    encoder.data["angle_deg"] = 1.0
    encoder.data["last_edge_us"] = clock["us"]
    controller.set_visual_target(1.0, clock["ms"], True)
    tick(controller, clock)
    assert stepper.commands[-1]["enabled"] is False
    assert controller.status(clock["ms"])["fault"] == "angle_deadband"
