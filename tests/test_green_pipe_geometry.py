import ast
import math
import pathlib
import unittest


SOURCE = pathlib.Path(__file__).parents[1] / "main_two_touch_calibration.py"
PURE_FUNCTIONS = {
    "clamp_pipe_param",
    "pipe_geometry_from_corners",
    "project_point_to_pipe",
    "measure_pipe_position",
    "update_pipe_geometry_state",
    "select_green_pipe_candidate",
}


def load_geometry_functions():
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"), filename=str(SOURCE))
    selected = [
        node for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name in PURE_FUNCTIONS
    ]
    module = ast.Module(body=selected, type_ignores=[])
    namespace = {}
    exec(compile(module, str(SOURCE), "exec"), namespace)
    return namespace


class GreenPipeGeometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.geometry = load_geometry_functions()

    def test_horizontal_pipe_midpoint_is_official_zero(self):
        geometry = self.geometry["pipe_geometry_from_corners"](
            [(100, 100), (500, 100), (500, 140), (100, 140)])
        measurement = self.geometry["measure_pipe_position"](
            300, 120, 0.75, geometry, 25.0)
        self.assertAlmostEqual(measurement["ball_position_cm"], 0.0, places=5)
        self.assertAlmostEqual(measurement["target_position_cm"], 6.25, places=5)
        self.assertAlmostEqual(measurement["error_cm"], 6.25, places=5)
        self.assertEqual(measurement["origin_point"], (300, 120))
        self.assertEqual(measurement["target_point"], (400, 120))

    def test_tilted_pipe_projects_ball_onto_dynamic_centerline(self):
        geometry = self.geometry["pipe_geometry_from_corners"](
            [(90, 90), (410, 250), (390, 290), (70, 130)])
        measurement = self.geometry["measure_pipe_position"](
            236, 199, 0.5, geometry, 25.0)
        self.assertTrue(measurement["valid"])
        self.assertAlmostEqual(measurement["ball_position_cm"], 0.0, delta=0.25)
        self.assertLess(abs(measurement["lateral_px"]), 30.0)

    def test_corner_order_does_not_reverse_signed_coordinate(self):
        forward = self.geometry["pipe_geometry_from_corners"](
            [(100, 100), (500, 100), (500, 140), (100, 140)])
        reversed_geometry = self.geometry["pipe_geometry_from_corners"](
            [(500, 140), (500, 100), (100, 100), (100, 140)])
        first = self.geometry["measure_pipe_position"](
            400, 120, 0.5, forward, 25.0)
        second = self.geometry["measure_pipe_position"](
            400, 120, 0.5, reversed_geometry, 25.0)
        self.assertAlmostEqual(first["ball_position_cm"], 6.25, places=5)
        self.assertAlmostEqual(second["ball_position_cm"], 6.25, places=5)

    def test_target_ratio_follows_translated_pipe(self):
        first_geometry = self.geometry["pipe_geometry_from_corners"](
            [(100, 100), (500, 100), (500, 140), (100, 140)])
        moved_geometry = self.geometry["pipe_geometry_from_corners"](
            [(130, 160), (530, 160), (530, 200), (130, 200)])
        first = self.geometry["measure_pipe_position"](
            400, 120, 0.75, first_geometry, 25.0)
        moved = self.geometry["measure_pipe_position"](
            430, 180, 0.75, moved_geometry, 25.0)
        self.assertAlmostEqual(first["error_cm"], 0.0, places=5)
        self.assertAlmostEqual(moved["error_cm"], 0.0, places=5)
        self.assertEqual(moved["target_point"], (430, 180))

    def test_touch_projection_is_clamped_to_pipe(self):
        geometry = self.geometry["pipe_geometry_from_corners"](
            [(100, 100), (500, 100), (500, 140), (100, 140)])
        project = self.geometry["project_point_to_pipe"]
        self.assertEqual(project(20, 120, geometry, True)[0], 0.0)
        self.assertEqual(project(620, 120, geometry, True)[0], 1.0)

    def test_pipe_state_holds_short_miss_then_invalidates(self):
        geometry = self.geometry["pipe_geometry_from_corners"](
            [(100, 100), (500, 100), (500, 140), (100, 140)])
        update = self.geometry["update_pipe_geometry_state"]
        state = update({"valid": False, "geometry": None, "misses": 0},
                       geometry, 2, 1.0)
        self.assertTrue(state["valid"])
        state = update(state, None, 2, 1.0)
        self.assertTrue(state["valid"])
        state = update(state, None, 2, 1.0)
        self.assertTrue(state["valid"])
        state = update(state, None, 2, 1.0)
        self.assertFalse(state["valid"])

    def test_first_pipe_geometry_is_locked_until_restart(self):
        first = self.geometry["pipe_geometry_from_corners"](
            [(100, 100), (500, 100), (500, 140), (100, 140)])
        jittered = self.geometry["pipe_geometry_from_corners"](
            [(112, 108), (512, 108), (512, 148), (112, 148)])
        update = self.geometry["update_pipe_geometry_state"]
        state = update({"valid": False, "geometry": None, "misses": 0},
                       first, 2, 1.0, True)
        self.assertTrue(state["locked"])
        locked = update(state, jittered, 2, 1.0, True)
        self.assertEqual(locked["geometry"]["center"], first["center"])

    def test_pipe_candidate_rejects_large_sparse_search_box(self):
        select = self.geometry["select_green_pipe_candidate"]
        huge_sparse = {
            "geometry": {"length_px": 520.0, "width_px": 115.0},
            "pixels": 9000,
        }
        real_green_pipe = {
            "geometry": {"length_px": 410.0, "width_px": 38.0},
            "pixels": 12500,
        }
        selected = select(
            [huge_sparse, real_green_pipe],
            min_length_px=180.0,
            min_aspect_ratio=3.0,
            max_width_px=72.0,
            min_fill_ratio=0.45)
        self.assertIs(selected, real_green_pipe["geometry"])


if __name__ == "__main__":
    unittest.main()
