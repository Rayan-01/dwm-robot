import json
import test_detect
import unittest


class NavigationRuntimeTests(unittest.TestCase):
    def setUp(self):
        test_detect.DetectTests.setUp(self)
        self.rt.c["wheel_diameter_m"] = 0.14
        (test_detect.robot.ROOT / "navigation_geometry.json").write_text(
            json.dumps(
                {"lidar_alignment_confirmed": True, "track_centre_spacing_m": 0.4318}
            )
        )

    tearDown = test_detect.DetectTests.tearDown

    def begin(self):
        return self.rt.command("nav_begin", {}, self.operator)["key"]

    def velocity(self, key, **changes):
        data = dict(
            key=key,
            seq=1,
            linear=0.1,
            angular=0.0,
            points=[[2.0, 2.0]],
            scan_fresh=True,
        )
        data.update(changes)
        return self.rt.command("nav_velocity", data, self.operator)

    def test_navigation_times_out_to_zero_and_brush_stays_off(self):
        key = self.begin()
        self.velocity(key)
        self.rt.navigation_tick(self.now + 0.05)
        output = self.rt.engine.tick(self.now + 0.05)
        self.assertGreater(output[0], 0)
        self.assertFalse(output[2])
        self.rt.navigation_tick(self.now + 0.4)
        self.assertEqual(self.rt.engine.tick(self.now + 0.4), (0, 0, False))

    def test_rear_obstacle_and_stale_scan_prevent_motion(self):
        key = self.begin()
        self.assertFalse(
            self.velocity(key, points=[[-0.6, -0.38]], linear=0.0, angular=0.3)["clear"]
        )
        self.rt.navigation_tick(self.now + 0.05)
        self.assertEqual(self.rt.engine.tick(self.now + 0.05), (0, 0, False))
        self.assertFalse(self.velocity(key, seq=2, scan_fresh=False)["clear"])

    def test_old_navigation_session_cannot_restart_or_stop_new_manual_session(self):
        key = self.begin()
        self.rt.stop()
        with self.assertRaises(ValueError):
            self.velocity(key)
        self.rt.command("manual", {}, self.operator)
        self.rt.command("nav_end", {"key": key}, self.operator)
        self.assertFalse(self.rt.engine.latched)

    def test_old_velocity_packet_ignored(self):
        key = self.begin()
        self.velocity(key)
        self.velocity(key, seq=0, points=[])
        self.assertTrue(self.rt.navigation["clear"])
