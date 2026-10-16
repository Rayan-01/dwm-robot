import unittest
from dwm_core.forward_pass import ForwardPass
import test_detect


class PassMathTests(unittest.TestCase):
    def test_stops_at_relative_one_metre(self):
        p = ForwardPass(5, 8, 0, 1)
        self.assertIsNone(p.step(5.4, 8.4, 1)[1])
        self.assertIn("completed", p.step(6, 9, 2)[1])

    def test_drift_reverse_and_stall_stop(self):
        for l, r, t in [(0.2, 0, 1), (-0.04, 0, 1), (0, 0, 6)]:
            self.assertIsNotNone(ForwardPass(0, 0, 0, 1).step(l, r, t)[1])

    def test_small_drift_corrects_without_reversing_tracks(self):
        p = ForwardPass(0, 0, 0, 1)
        for i in range(20):
            pair, reason = p.step(0.08, 0.05, i * 0.05)
        self.assertIsNone(reason)
        self.assertLess(pair[0], pair[1])
        self.assertGreaterEqual(min(pair), 30)


class PassTransitionTests(unittest.TestCase):
    def setUp(self):
        test_detect.DetectTests.setUp(self)
        self.rt.c["height"] = 360

    tearDown = test_detect.DetectTests.tearDown
    target = test_detect.DetectTests.target

    def begin_pass(self):
        self.rt.c["wheel_diameter_m"] = 0.14
        self.rt.command(
            "detect",
            {"distance_cm": 45, "forward_pass_m": 1, "clean_after_pass": False},
            self.operator,
        )
        self.target(0.485)
        self.rt.engine.tracker.target.box = (300, 310, 340, 357)
        self.rt.follow_tick(self.now)
        self.assertEqual(self.rt.engine.phase, "BEFORE_PASS_PAUSE")
        self.assertEqual(self.rt.engine.tick(self.now), (0, 0, False))
        self.now += 0.99
        e = self.rt.engine
        e.capture_at = e.vision_at = e.browser_at = self.now
        self.rt.follow_tick(self.now)
        self.assertEqual(e.tick(self.now), (0, 0, False))
        self.now += 0.02
        e.capture_at = e.vision_at = e.browser_at = self.now
        self.rt.follow_tick(self.now)
        self.assertEqual(self.rt.engine.phase, "FORWARD_PASS")
        self.assertTrue(e.tick(self.now)[2])

    def test_pass_continues_after_target_leaves_image_and_finishes(self):
        self.begin_pass()
        e = self.rt.engine
        e.tracker.visible = False
        self.rt.detection_depths = {}
        self.now += 0.1
        e.capture_at = e.vision_at = e.browser_at = self.now
        self.rt.follow_tick(self.now)
        self.assertFalse(e.latched)
        self.assertTrue(e.tick(self.now)[2])
        e.left_m += 1
        e.right_m += 1
        self.rt.follow_tick(self.now + 0.05)
        self.assertTrue(e.latched)
        self.assertIn("completed", e.reason)
        self.assertEqual(self.rt.forward_pass_status["estimated_travel_m"], 1)

    def test_stop_cancels_pass(self):
        self.begin_pass()
        self.rt.stop()
        self.assertIsNone(self.rt.follow)
        self.assertEqual(self.rt.engine.tick(self.now), (0, 0, False))

    def test_close_target_at_view_edge_can_be_confirmed(self):
        self.rt.c["wheel_diameter_m"] = 0.14
        self.rt.command(
            "detect", {"distance_cm": 45, "forward_pass_m": 1}, self.operator
        )
        from dwm_core.control import Detection

        d = Detection(44, 0.9, (300, 310, 340, 357))
        depths = {44: {"z_m": 0.485, "valid_fraction": 1}}
        for _ in range(3):
            self.now += 0.05
            e = self.rt.engine
            e.capture_at = e.browser_at = self.rt.depth_at = self.now
            self.rt.track_vision([d], depths, self.now, self.now)
        self.assertEqual(self.rt.engine.tracker.target.id, 44)
