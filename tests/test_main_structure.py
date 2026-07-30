import ast
from pathlib import Path


SOURCE = Path(__file__).parents[1] / "main.py"
TREE = ast.parse(SOURCE.read_text(encoding="utf-8"))


def load_pure_function(name, namespace=None):
    node = next(
        (
            item
            for item in TREE.body
            if isinstance(item, ast.FunctionDef) and item.name == name
        ),
        None,
    )
    assert node is not None, "{} is missing".format(name)
    module = ast.Module(body=[node], type_ignores=[])
    namespace = {} if namespace is None else dict(namespace)
    exec(compile(module, str(SOURCE), "exec"), namespace)
    return namespace[name]


def test_detection_circle_geometry():
    detection_circle = load_pure_function("detection_circle")
    assert detection_circle(10, 20, 20, 12) == (20, 26, 13)
    assert detection_circle(1, 2, 2, 2) == (2, 3, 4)


def test_osd_updates_every_second_ai_frame():
    should_render_osd = load_pure_function("should_render_osd")
    assert should_render_osd(1) is False
    assert should_render_osd(2) is True
    assert should_render_osd(3) is False
    assert should_render_osd(4) is True


def test_uart_updates_are_not_gated_by_osd_rendering():
    draw_osd = next(
        item
        for item in TREE.body
        if isinstance(item, ast.FunctionDef) and item.name == "draw_osd"
    )
    for branch in (node for node in ast.walk(draw_osd) if isinstance(node, ast.If)):
        if not any(
            isinstance(name, ast.Name) and name.id == "render_osd"
            for name in ast.walk(branch.test)
        ):
            continue
        assert not any(
            isinstance(call, ast.Call)
            and isinstance(call.func, ast.Attribute)
            and call.func.attr == "write"
            and isinstance(call.func.value, ast.Name)
            and call.func.value.id == "uart_obj"
            for stmt in branch.body
            for call in ast.walk(stmt)
        )


def test_h264_rtsp_replaces_mjpeg_transport():
    class_names = {
        item.name for item in TREE.body if isinstance(item, ast.ClassDef)
    }
    assert "LowLatencyRtspH264Server" in class_names
    assert "LowLatencyMjpegServer" not in class_names

    assignments = {
        target.id: ast.literal_eval(item.value)
        for item in TREE.body
        if isinstance(item, ast.Assign)
        for target in item.targets
        if isinstance(target, ast.Name)
        and target.id in {"RTSP_PORT", "RTSP_SESSION", "H264_FPS"}
    }
    assert assignments == {
        "RTSP_PORT": 8554,
        "RTSP_SESSION": "ball",
        "H264_FPS": 30,
    }


def test_control_interface_and_model_paths_are_unchanged():
    format_deviation_msg = load_pure_function(
        "format_deviation_msg", {"DEVIATION_DEADZONE": 3})
    assert format_deviation_msg(12, -7, True) == b"X:+012,Y:-007\n"
    assert format_deviation_msg(2, -3, True) == b"X:+000,Y:+000\n"
    assert format_deviation_msg(0, 0, False) == b"X:----,Y:----\n"

    assignments = {
        target.id: ast.literal_eval(item.value)
        for item in TREE.body
        if isinstance(item, ast.Assign)
        for target in item.targets
        if isinstance(target, ast.Name)
        and target.id in {
            "root_path", "config_path", "UART_BAUDRATE",
            "SEND_EVERY_N_FRAMES",
        }
    }
    assert assignments == {
        "root_path": "/sdcard/mp_deployment_source/",
        "config_path": "/sdcard/mp_deployment_source/deploy_config.json",
        "UART_BAUDRATE": 115200,
        "SEND_EVERY_N_FRAMES": 1,
    }


if __name__ == "__main__":
    test_detection_circle_geometry()
    test_osd_updates_every_second_ai_frame()
    test_uart_updates_are_not_gated_by_osd_rendering()
    test_h264_rtsp_replaces_mjpeg_transport()
    test_control_interface_and_model_paths_are_unchanged()
    print("tests: OK")
