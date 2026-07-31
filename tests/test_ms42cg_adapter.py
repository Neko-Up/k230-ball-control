import ast
import pathlib


SOURCE = pathlib.Path(__file__).parents[1] / "main_two_touch_calibration.py"


class FakeFPIOA:
    GPIO19 = 119
    GPIO20 = 120
    GPIO32 = 132
    GPIO33 = 133

    def __init__(self):
        self.calls = []

    def set_function(self, *args, **kwargs):
        self.calls.append((args, kwargs))


class FakePin:
    IN = 0
    PULL_NONE = 0
    IRQ_BOTH = 3
    registry = {}

    def __init__(self, pin_id, mode, **kwargs):
        self.pin_id = pin_id
        self.mode = mode
        self.level = 0
        self.callback = None
        self.trigger = None
        self.destroyed = False
        self.destroy_calls = 0
        FakePin.registry[pin_id] = self

    def __del__(self):
        self.callback = None
        self.destroyed = True
        self.destroy_calls += 1

    def value(self):
        return self.level

    def irq(self, handler=None, trigger=None):
        if trigger is None:
            raise TypeError("'trigger' argument required")
        if handler is None:
            raise ValueError("invalid callback")
        self.callback = handler
        self.trigger = trigger
        return self

    def edge(self, level):
        self.level = level
        if self.callback is not None:
            self.callback(self)


def load_symbols():
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"), filename=str(SOURCE))
    wanted_functions = {
        "quadrature_delta", "wrapped_encoder_delta",
        "encoder_count_to_angle", "pwm_duty_to_count",
    }
    nodes = [
        item for item in tree.body
        if ((isinstance(item, ast.FunctionDef) and item.name in wanted_functions) or
            (isinstance(item, ast.ClassDef) and item.name == "MS42CGEncoder"))
    ]
    namespace = {
        "Pin": FakePin,
        "ENCODER_A_IO": 19,
        "ENCODER_B_IO": 20,
        "ENCODER_Z_IO": 32,
        "ENCODER_PWM_IO": 33,
        "ENCODER_COUNTS_PER_REV": 4096,
        "ENCODER_PWM_DUTY_MIN": 0.25,
        "ENCODER_PWM_DUTY_MAX": 0.75,
        "ENCODER_PWM_INVERT": False,
        "ENCODER_PWM_SAMPLE_COUNT": 4,
    }
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(SOURCE), "exec"),
         namespace)
    return namespace


def new_encoder():
    FakePin.registry = {}
    clock = {"us": 0}
    symbols = load_symbols()
    encoder = symbols["MS42CGEncoder"](
        FakeFPIOA(), ticks_us_fn=lambda: clock["us"],
        ticks_diff_fn=lambda now, before: now - before)
    return encoder, clock


def set_ab(encoder, clock, a, b, advance_us=1000):
    clock["us"] += advance_us
    encoder.a_pin.level = a
    encoder.b_pin.level = b
    encoder._ab_edge(None)


def test_ab_irq_tracks_forward_reverse_and_invalid_transitions():
    encoder, clock = new_encoder()
    encoder.start_abz()
    for a, b in ((0, 1), (1, 1), (1, 0), (0, 0)):
        set_ab(encoder, clock, a, b)
    assert encoder.count == 4
    for a, b in ((1, 0), (1, 1), (0, 1), (0, 0)):
        set_ab(encoder, clock, a, b)
    assert encoder.count == 0
    set_ab(encoder, clock, 1, 1)
    assert encoder.count == 0
    assert encoder.invalid_transitions == 1


def test_z_irq_records_index_without_changing_position():
    encoder, clock = new_encoder()
    encoder.start_abz()
    encoder.count = 77
    clock["us"] = 12345
    encoder.z_pin.edge(1)
    assert encoder.z_seen is True
    assert encoder.z_index_count == 77
    assert encoder.count == 77


def test_pwm_capture_collects_fixed_samples_and_produces_absolute_count():
    encoder, clock = new_encoder()
    encoder.start_pwm_capture()
    # The first rising edge establishes the PWM period origin.
    for _ in range(5):
        clock["us"] += 500
        encoder.pwm_pin.edge(1)
        clock["us"] += 500
        encoder.pwm_pin.edge(0)
    assert encoder.pwm_capture_active is False
    snapshot = encoder.snapshot(clock["us"] + 1000)
    assert snapshot["pwm_valid"] is True
    assert snapshot["absolute_count"] == 2048


def test_pwm_absolute_angle_can_be_resampled_without_reregistering_irq():
    encoder, clock = new_encoder()
    encoder.start_pwm_capture()
    for _ in range(5):
        clock["us"] += 500
        encoder.pwm_pin.edge(1)
        clock["us"] += 500
        encoder.pwm_pin.edge(0)
    assert encoder.absolute_count == 2048
    assert encoder.pwm_pin.destroyed is False
    encoder.start_pwm_capture()
    for _ in range(5):
        encoder.pwm_pin.edge(1)
        clock["us"] += 250
        encoder.pwm_pin.edge(0)
        clock["us"] += 750
    assert encoder.absolute_count == 0
    assert encoder.pwm_pin.destroyed is False
    encoder.deinit()
    assert encoder.pwm_pin.destroy_calls == 1


def test_snapshot_reports_angle_and_velocity_outside_irq_context():
    encoder, _ = new_encoder()
    encoder.start_abz()
    encoder.snapshot(1000)
    encoder.count = 16
    snapshot = encoder.snapshot(11000)
    assert abs(snapshot["angle_deg"] - 1.40625) < 1e-9
    assert abs(snapshot["velocity_deg_s"] - 140.625) < 1e-6
    assert snapshot["last_edge_us"] == 0


def test_deinit_releases_all_irq_pins_once():
    encoder, _ = new_encoder()
    encoder.start_abz()
    encoder.start_pwm_capture()
    encoder.deinit()
    encoder.deinit()
    assert encoder.a_pin.destroyed is True
    assert encoder.b_pin.destroyed is True
    assert encoder.z_pin.destroyed is True
    assert encoder.pwm_pin.destroyed is True
    assert encoder.a_pin.callback is None
    assert encoder.b_pin.callback is None
    assert encoder.z_pin.callback is None
    assert encoder.pwm_pin.callback is None
    assert encoder.a_pin.destroy_calls == 1
    assert encoder.b_pin.destroy_calls == 1
    assert encoder.z_pin.destroy_calls == 1
    assert encoder.pwm_pin.destroy_calls == 1
