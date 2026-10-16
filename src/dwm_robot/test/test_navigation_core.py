import math
import unittest
from dwm_core.navigation_core import (
    WheelOdometry,
    WheelSpeedControl,
    clear_motion,
    route_headings,
    goal_bearing,
)


class NavigationMathTests(unittest.TestCase):
    def test_three_cm_padding_keeps_body_collision_check(self):
        self.assertFalse(clear_motion([[0.30035, 0]], 0, 0))
        self.assertTrue(clear_motion([[0.30035, 0]], 0, 0, padding=0.03))
        self.assertFalse(clear_motion([[0.25035, 0]], 0, 0, padding=0.03))

    def test_left_goal_uses_left_arrival_heading_and_map_left_can_be_behind(self):
        pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        points = [{"x": 0.0, "y": 1.0, "yaw": 0.0}, {"x": -1.0, "y": 1.0, "yaw": 0.0}]
        result = route_headings(points, pose)
        self.assertAlmostEqual(result[0]["yaw"], math.pi / 2)
        self.assertAlmostEqual(result[1]["yaw"], math.pi)
        self.assertAlmostEqual(goal_bearing(pose, points[0]), math.pi / 2)
        self.assertAlmostEqual(abs(goal_bearing(pose, {"x": -1.0, "y": 0.0})), math.pi)
        self.assertEqual(route_headings(points, pose, "fixed")[0]["yaw"], 0.0)
        self.assertEqual(points[0]["yaw"], 0.0)

    def test_gyro_bias_fusion_and_stale_fallback(self):
        o = WheelOdometry(0.4318)
        o.update(0, 0, "a")
        o.update(0, 0, "a", 0.01, 0.1, True)
        self.assertTrue(o.imu_used)
        self.assertAlmostEqual(o.yaw, 0)
        o.update(-0.01, 0.01, "a", 0.21, 0.1, False)
        self.assertAlmostEqual(o.yaw, 0.8 * 0.2 * 0.1 + 0.2 * 0.02 / 0.4318)
        yaw = o.yaw
        o.update(-0.02, 0.02, "a", None, 0.1, False)
        self.assertFalse(o.imu_used)
        self.assertAlmostEqual(o.yaw - yaw, 0.02 / 0.4318)

    def test_straight_spin_and_restart_baseline(self):
        o = WheelOdometry(0.4318)
        o.update(0, 0, "first")
        o.update(0.1, 0.1, "first")
        self.assertAlmostEqual(o.x, 0.1)
        self.assertAlmostEqual(o.y, 0)
        o.update(0.0, 0.2, "first")
        self.assertAlmostEqual(o.yaw, 0.2 / 0.4318)
        x, y, yaw = o.x, o.y, o.yaw
        o.update(0, 0, "restart")
        self.assertEqual((o.x, o.y, o.yaw), (x, y, yaw))

    def test_rear_swing_and_forward_obstacle(self):
        self.assertFalse(clear_motion([[0.39, 0]], 0.12, 0))
        # Rear corner swings right on a left turn; narrow robot-only front checks miss it.
        self.assertFalse(clear_motion([[-0.60, -0.38]], 0, 0.3))
        self.assertTrue(clear_motion([[2, 2]], 0.12, 0.2))
        self.assertFalse(clear_motion([], 0, 0))

    def test_speed_control_is_bounded_and_zero_immediate(self):
        c = WheelSpeedControl(0.4318, 0.14)
        for _ in range(200):
            pair = c.step(0.12, 0.3, (0, 0))
        self.assertTrue(all(0 <= v <= 50 for v in pair))
        self.assertEqual(c.step(0, 0, (12, 12)), (0, 0))
        for _ in range(30):
            pair = c.step(0, 0.3, (0, 0))
        self.assertLess(pair[0], 0)
        self.assertGreater(pair[1], 0)
