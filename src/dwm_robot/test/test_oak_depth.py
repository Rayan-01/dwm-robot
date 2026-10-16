import unittest
import numpy as np
from dwm_perception.oak_source import box_depth


class DepthTests(unittest.TestCase):
    def test_valid_median_resists_outlier(self):
        depth = np.full((40, 60), 800, dtype=np.uint16)
        depth[20, 30] = 9000
        self.assertEqual(box_depth(depth, (10, 10, 50, 30))["z_m"], 0.8)

    def test_missing_or_outside_depth_is_unavailable(self):
        depth = np.zeros((40, 60), dtype=np.uint16)
        self.assertIsNone(box_depth(depth, (10, 10, 50, 30)))
        self.assertIsNone(box_depth(depth, (100, 100, 120, 120)))

    def test_sparse_valid_points_are_not_enough(self):
        depth = np.zeros((40, 60), dtype=np.uint16)
        depth[20, 30] = 500
        self.assertIsNone(box_depth(depth, (10, 10, 50, 30)))

    def test_unsupported_near_depth_is_not_used(self):
        depth = np.full((40, 60), 50, dtype=np.uint16)
        self.assertIsNone(box_depth(depth, (10, 10, 50, 30)))
