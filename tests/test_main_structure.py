import ast
from pathlib import Path


SOURCE = Path(__file__).parents[1] / "main.py"
TREE = ast.parse(SOURCE.read_text(encoding="utf-8"))


def load_pure_function(name):
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
    namespace = {}
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


if __name__ == "__main__":
    test_detection_circle_geometry()
    test_osd_updates_every_second_ai_frame()
    test_uart_updates_are_not_gated_by_osd_rendering()
    print("tests: OK")
