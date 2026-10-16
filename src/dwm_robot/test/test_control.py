import math
import unittest
from dwm_core.control import (
    Controller,
    DEFAULTS,
    Detection,
    TargetLock,
    auto_ready,
    ground_point,
    validate_config,
)


def config():
    c = dict(DEFAULTS)
    c.update(
        {
            "wheel_diameter_m": 0.15,
            "axle_to_brush_m": 0.45,
            "brush_width_m": 0.45,
            "geometry_verified": True,
            "homography": [1, 0, 0, 0, 1, 0, 0, 0, 1],
            "ground_polygon": [[0, 0], [2, 0], [2, 2], [0, 2]],
            "ground_range": [0, 2, 0, 2],
            "calibration_resolution": [640, 480],
        }
    )
    return c


class ControlTests(unittest.TestCase):
    def setUp(self):
        self.c = config()
        self.e = Controller(self.c)
        self.t = 100.0
        self.fresh()

    def fresh(self):
        self.e.browser_at = self.e.capture_at = self.e.vision_at = (
            self.e.telemetry_at
        ) = self.t
        self.e.vision_ok = self.e.serial_ok = True

    def observe(self, ident=1, conf=0.9, xy=(0, 1), dt=0.08):
        self.t += dt
        self.e.browser_at = self.e.capture_at = self.e.telemetry_at = self.t
        self.e.vision([Detection(ident, conf, (10, 10, 20, 30), xy)], self.t, self.t)
        return self.e.tick(self.t)

    def locked(self, xy=(0, 1)):
        self.e.start("AUTO", self.t)
        for _ in range(3):
            self.observe(xy=xy)

    def test_no_lock_from_single_frame(self):
        self.e.start("AUTO", self.t)
        self.assertEqual(self.observe(), (0, 0, False))
        self.assertIsNone(self.e.tracker.target)

    def test_requires_same_id_consecutive_high_confidence(self):
        self.e.start("AUTO", self.t)
        self.observe()
        self.observe(ident=2)
        self.observe()
        self.observe()
        self.assertIsNone(self.e.tracker.target)
        self.observe()
        self.assertEqual(self.e.tracker.target.id, 1)

    def test_lower_confidence_keeps_existing_lock(self):
        self.locked()
        self.observe(conf=0.15)
        self.assertEqual(self.e.tracker.target.id, 1)
        self.assertEqual(self.e.phase, "APPROACHING")

    def test_missing_target_stops_and_does_not_switch(self):
        self.locked()
        self.assertEqual(self.observe(ident=2), (0, 0, False))
        self.assertEqual(self.e.tracker.target.id, 1)
        self.assertEqual(self.e.phase, "TARGET_MISSING")
        for _ in range(15):
            self.observe(ident=2)
        self.assertTrue(self.e.latched)
        self.assertEqual(self.e.mode, "STOPPED")

    def test_same_id_reacquires_within_timeout(self):
        self.locked()
        self.observe(ident=2)
        out = self.observe()
        self.assertEqual(out, (50, 50, False))

    def test_target_outside_calibration_does_not_move(self):
        self.locked(xy=None)
        self.assertEqual(self.e.output, (0, 0, False))
        self.assertEqual(self.e.phase, "OUTSIDE_CALIBRATION")

    def test_turn_right_for_target_on_right(self):
        self.locked(xy=(0.2, 1))
        self.assertEqual(self.e.output, (45, -45, False))

    def test_camera_freeze_latches_stop(self):
        self.locked()
        self.t += 0.7
        self.e.browser_at = self.e.telemetry_at = self.t
        self.e.tick(self.t)
        self.assertTrue(self.e.latched)
        self.assertIn("stale", self.e.reason)

    def test_inference_freeze_despite_live_capture(self):
        self.locked()
        self.t += 0.7
        self.e.browser_at = self.e.capture_at = self.e.telemetry_at = self.t
        self.e.tick(self.t)
        self.assertTrue(self.e.latched)

    def test_browser_disconnect_stops_manual_brush(self):
        self.e.start("MANUAL", self.t)
        self.e.manual_brush = True
        self.e.manual("forward", self.t)
        self.e.tick(self.t)
        self.assertTrue(self.e.output[2])
        self.t += 1.6
        self.e.capture_at = self.e.vision_at = self.e.telemetry_at = self.t
        self.e.tick(self.t)
        self.assertEqual(self.e.output, (0, 0, False))
        self.assertTrue(self.e.latched)

    def test_manual_direction_deadman(self):
        self.e.start("MANUAL", self.t)
        self.e.manual("forward", self.t)
        self.assertEqual(self.e.tick(self.t), (55, 55, False))
        self.t += 0.4
        self.fresh()
        self.assertEqual(self.e.tick(self.t), (0, 0, False))

    def test_telemetry_loss_stops(self):
        self.locked()
        self.t += 0.6
        self.e.browser_at = self.e.capture_at = self.e.vision_at = self.t
        self.e.tick(self.t)
        self.assertTrue(self.e.latched)

    def test_start_requires_calibration(self):
        self.c["geometry_verified"] = False
        with self.assertRaises(ValueError):
            self.e.start("AUTO", self.t)

    def test_cleaning_waits_for_settled_wheels(self):
        self.e.left_rpm = 10
        self.e.right_rpm = 10
        self.locked(xy=(0, 0.25))
        self.assertEqual(self.e.phase, "SETTLING")
        self.assertEqual(self.e.output, (0, 0, False))

    def test_cleaning_pass_uses_brush_offset_and_completes(self):
        self.locked(xy=(0, 0.25))
        self.assertEqual(self.e.phase, "CLEAN_PASS")
        self.assertAlmostEqual(self.e.pass_distance, 0.25 + 0.45 + 0.12)
        self.assertEqual(self.e.output, (45, 45, True))
        self.e.left_m += self.e.pass_distance
        self.e.right_m += self.e.pass_distance
        self.assertEqual(self.e.tick(self.t), (0, 0, False))
        self.assertTrue(self.e.latched)
        self.assertEqual(self.e.cycles, 1)

    def test_pass_continues_without_visible_target_but_requires_live_vision(self):
        self.locked(xy=(0, 0.25))
        self.t += 0.1
        self.fresh()
        self.e.tracker.visible = False
        self.assertEqual(self.e.tick(self.t), (45, 45, True))
        self.t += 0.7
        self.e.browser_at = self.e.telemetry_at = self.t
        self.assertEqual(self.e.tick(self.t), (0, 0, False))

    def test_pass_drift_stops(self):
        self.locked(xy=(0, 0.25))
        self.e.left_m += 0.06
        self.assertEqual(self.e.tick(self.t), (0, 0, False))
        self.assertIn("drift", self.e.reason)

    def test_pass_timeout_stops_even_with_zero_rpm(self):
        self.locked(xy=(0, 0.25))
        self.t += 21
        self.fresh()
        self.assertEqual(self.e.tick(self.t), (0, 0, False))
        self.assertIn("timed out", self.e.reason)

    def test_stop_is_latched_against_old_direction_packets(self):
        self.e.start("MANUAL", self.t)
        self.e.manual("forward", self.t)
        self.e.stop()
        with self.assertRaises(ValueError):
            self.e.manual("forward", self.t)
        self.assertEqual(self.e.tick(self.t), (0, 0, False))

    def test_odometry_integrates_signed_rpm(self):
        self.e.wheels(60, 60, self.t)
        self.e.wheels(60, 60, self.t + 0.1)
        self.assertAlmostEqual(self.e.left_m, math.pi * 0.15 * 0.1)
        self.e.wheels(float("nan"), 0, self.t + 0.2)
        self.assertTrue(self.e.latched)

    def test_projection_rejects_points_outside_measured_polygon(self):
        self.assertEqual(ground_point((0, 0, 1, 1), self.c), (0.5, 1.0))
        self.assertIsNone(ground_point((3, 0, 4, 1), self.c))

    def test_invalid_numeric_configuration(self):
        for value in (float("nan"), float("inf"), True, 121):
            c = config()
            c["max_pwm"] = value
            with self.assertRaises(ValueError):
                validate_config(c)


if __name__ == "__main__":
    unittest.main()
