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


if __name__ == "__main__":
    test_detection_circle_geometry()
    print("tests: OK")
