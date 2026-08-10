import importlib.util
import math
from pathlib import Path
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = PACKAGE_ROOT / "cone_nav" / "avoidance_algorithms.py"
SPEC = importlib.util.spec_from_file_location("avoidance_algorithms", MODULE_PATH)
ALGORITHMS = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(ALGORITHMS)
grid_index = ALGORITHMS.grid_index
inside_cone_corridor = ALGORITHMS.inside_cone_corridor
path_clearance = ALGORITHMS.path_clearance
path_collides = ALGORITHMS.path_collides
shifted_path = ALGORITHMS.shifted_path


class AvoidanceAlgorithmsTest(unittest.TestCase):
    def test_grid_index_and_bounds(self):
        self.assertEqual(grid_index(0.05, -0.95, 0.0, -1.0, 0.1, 20, 20), 0)
        self.assertIsNone(grid_index(-0.01, 0.0, 0.0, -1.0, 0.1, 20, 20))

    def test_shifted_path_starts_at_vehicle_and_reaches_offset(self):
        shifted = shifted_path([(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)], 0.5, 1.0)
        self.assertAlmostEqual(shifted[0][1], 0.0)
        self.assertAlmostEqual(shifted[1][1], 0.5)
        self.assertAlmostEqual(shifted[2][1], 0.5)

    def test_collision_check_is_fail_closed_outside_grid(self):
        data = [0] * 100
        data[55] = 100
        self.assertTrue(path_collides([(0.55, 0.55)], data, 0.0, 0.0, 0.1, 10, 10, 50))
        self.assertTrue(path_collides([(1.1, 0.0)], data, 0.0, 0.0, 0.1, 10, 10, 50))
        self.assertFalse(path_collides([(0.1, 0.1)], data, 0.0, 0.0, 0.1, 10, 10, 50))

    def test_cone_corridor_rejects_boundary_crossing(self):
        left = [(1.0, 0.8), (2.0, 0.8)]
        right = [(1.0, -0.8), (2.0, -0.8)]
        self.assertTrue(inside_cone_corridor([(1.0, 0.0)], left, right, 0.2, 0.2))
        self.assertFalse(inside_cone_corridor([(1.0, 0.7)], left, right, 0.2, 0.2))

    def test_clearance(self):
        clearance = path_clearance([(0.0, 0.0), (1.0, 0.0)], [(1.0, 1.0)])
        self.assertTrue(math.isclose(clearance, 1.0))


if __name__ == "__main__":
    unittest.main()
