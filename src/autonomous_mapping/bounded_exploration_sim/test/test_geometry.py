import math
import unittest

from bounded_exploration_sim.geometry import (
    bounded_map_data, inside_rectangle, keepout_mask,
    observation_grid, occupancy_at)


class GeometryTest(unittest.TestCase):
    def test_start_aligned_rectangle(self):
        self.assertTrue(inside_rectangle(4.5, 0, 0, 0, 0, 4.5, 4.5))
        self.assertFalse(inside_rectangle(4.6, 0, 0, 0, 0, 4.5, 4.5))
        self.assertTrue(inside_rectangle(0, 4, 0, 0, math.pi / 2, 4.5, 4.5))
        self.assertFalse(inside_rectangle(0, 5, 0, 0, math.pi / 2, 4.5, 4.5))

    def test_bounded_frontier_map(self):
        data = [-1] * 25
        bounded = bounded_map_data(data, 5, 5, 1, -2.5, -2.5, 0, 0, 0, 0, 1.5, 1.5)
        self.assertEqual(bounded[12], -1)
        self.assertEqual(bounded[0], 100)
        self.assertEqual(bounded[2], 100)

    def test_keepout_ring(self):
        ox, oy, width, height, mask = keepout_mask(0, 0, 0, 4.5, 4.5, 0.5)
        self.assertEqual((width, height), (len(mask) ** .5, len(mask) ** .5))
        self.assertEqual(mask[0], 100)
        center_col = int(-ox / .5)
        center_row = int(-oy / .5)
        self.assertEqual(mask[center_row * width + center_col], 0)

    def test_bad_geometry_fails_closed(self):
        self.assertFalse(inside_rectangle(float('nan'), 0, 0, 0, 0, 5, 5))
        with self.assertRaises(ValueError):
            keepout_mask(0, 0, 0, 0, 5)

    def test_observation_grid_stays_inside_safe_rectangle(self):
        points = observation_grid(0, 0, math.pi / 4, 4.5, 4.5, 2.0, 0.6)
        self.assertEqual(len(points), 16)
        self.assertTrue(all(inside_rectangle(x, y, 0, 0, math.pi / 4, 3.9, 3.9)
                            for x, y in points))

    def test_cost_lookup_supports_rotated_map_origin(self):
        self.assertEqual(occupancy_at([0, 7, 9, 100], 2, 2, 1.0,
                                      0.0, 0.0, math.pi / 2, -0.5, 1.5), 7)
        self.assertIsNone(occupancy_at([0, 7, 9, 100], 2, 2, 1.0,
                                       0.0, 0.0, 0.0, 4.0, 0.0))


if __name__ == '__main__':
    unittest.main()
