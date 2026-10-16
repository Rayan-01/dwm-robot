import argparse
import json
from pathlib import Path
import tempfile
import time
import unittest
from dwm_robot import robot
from dwm_core.control import DEFAULTS, Detection


class DetectTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.oldroot = robot.ROOT
        robot.ROOT = Path(self.temp.name)
        (robot.ROOT / "config.json").write_text(
            json.dumps(dict(DEFAULTS, camera="oak"))
        )
        self.rt = robot.Runtime(
            argparse.Namespace(
                config="config.json",
                camera=None,
                device=None,
                serial=None,
                demo=False,
                hardware=False,
            )
        )
        self.now = time.monotonic()
        e = self.rt.engine
        e.vision_ok = True
        e.capture_at = e.vision_at = self.now
        self.operator = self.rt.claim()["operator"]

    def tearDown(self):
        self.rt.close()
        robot.ROOT = self.oldroot
        self.temp.cleanup()

    def start(self, distance=45):
        self.rt.command(
            "detect", {"distance_cm": distance, "forward_pass_m": 0}, self.operator
        )

    def target(self, z=1):
        rt, e = self.rt, self.rt.engine
        e.tracker.target = Detection(7, 0.9, (300, 200, 340, 250))
        e.tracker.visible = True
        e.tracker.seen_at = rt.depth_at = self.now
        rt.detection_depths = {7: {"z_m": z, "valid_fraction": 1}}

    def test_invalid_close_or_negative_distance_rejected_without_arming(self):
        for distance in [-10, 0.5, 1, 44.9, 301, float("nan"), True]:
            with self.assertRaises(ValueError):
                self.start(distance)
            self.assertTrue(self.rt.engine.latched)

    def test_no_target_keeps_wheels_and_brush_off(self):
        self.start()
        self.rt.follow_tick(self.now)
        self.assertEqual(self.rt.engine.tick(self.now), (0, 0, False))

    def test_follows_confirmed_target_and_stop_cancels_follow(self):
        self.start()
        self.target()
        self.rt.follow_tick(self.now)
        self.assertEqual(self.rt.engine.tick(self.now), (30, 30, False))
        self.rt.stop()
        self.assertIsNone(self.rt.follow)
        self.assertEqual(self.rt.engine.tick(self.now), (0, 0, False))

    def test_missing_depth_pauses_then_resumes_without_brush(self):
        self.start()
        self.target()
        self.rt.detection_depths.clear()
        self.rt.follow_tick(self.now)
        self.assertFalse(self.rt.engine.latched)
        self.assertEqual(self.rt.engine.tick(self.now), (0, 0, False))
        self.assertEqual(self.rt.engine.phase, "REACQUIRING")
        self.target()
        self.rt.follow_tick(self.now + 0.05)
        self.assertEqual(self.rt.engine.tick(self.now + 0.05), (30, 30, False))

    def test_long_depth_loss_keeps_waiting_without_movement(self):
        self.start()
        self.target()
        self.rt.detection_depths.clear()
        self.rt.follow_tick(self.now)
        later = self.now + 70
        e = self.rt.engine
        e.browser_at = e.capture_at = e.vision_at = later
        self.rt.follow_tick(later)
        self.assertFalse(e.latched)
        self.assertEqual(e.tick(later), (0, 0, False))

    def test_filtered_fg_noise_does_not_block_camera_approach(self):
        self.rt.fg_noise_at = self.now
        self.start()
        self.target()
        self.rt.serial_line(b"F,1,100")
        self.rt.follow_tick(self.now)
        self.assertFalse(self.rt.engine.latched)
        self.assertEqual(self.rt.engine.tick(self.now), (30, 30, False))

    def test_long_pause_does_not_consume_movement_time_budget(self):
        self.start()
        self.target()
        self.rt.follow_tick(self.now)
        self.rt.detection_depths.clear()
        self.rt.follow_tick(self.now + 0.05)
        self.now += 70
        e = self.rt.engine
        e.browser_at = e.capture_at = e.vision_at = self.now
        self.target()
        self.rt.follow_tick(self.now)
        self.assertFalse(e.latched)
        self.assertEqual(e.tick(self.now), (30, 30, False))

    def test_standoff_stops_without_brush(self):
        self.start(90)
        self.target(0.85)
        self.rt.follow_tick(self.now)
        self.assertTrue(self.rt.engine.latched)
        self.assertFalse(self.rt.engine.output[2])

    def test_continues_beyond_old_step_limit(self):
        self.start()
        self.target()
        self.rt.follow_tick(self.now)
        self.now += 4
        e = self.rt.engine
        e.browser_at = e.capture_at = e.vision_at = self.now
        self.target(0.8)
        self.rt.follow_tick(self.now)
        self.assertFalse(e.latched)
        output = e.tick(self.now)
        self.assertEqual(output[0], output[1])
        self.assertTrue(30 <= output[0] <= 40)
        self.assertFalse(output[2])

    def test_view_edge_stops(self):
        self.start()
        self.target()
        self.rt.engine.tracker.target.box = (300, 200, 340, self.rt.c["height"])
        self.rt.follow_tick(self.now)
        self.assertTrue(self.rt.engine.latched)

    def test_brush_and_manual_drive_cannot_override_follow(self):
        self.start()
        for action, data in [
            ("brush", {"on": True}),
            ("drive", {"direction": "forward", "seq": 1}),
        ]:
            with self.assertRaises(ValueError):
                self.rt.command(action, data, self.operator)

    def test_heartbeat_loss_stops_follow(self):
        self.start()
        self.target()
        self.rt.engine.browser_at = self.now - 10
        self.rt.follow_tick(self.now)
        self.assertTrue(self.rt.engine.latched)

    def test_changed_id_is_reconfirmed_before_resuming(self):
        self.start()
        self.target()
        self.rt.follow_tick(self.now)
        e = self.rt.engine
        e.tracker.visible = False
        self.rt.follow_tick(self.now + 0.01)
        self.now += 1.2
        e.browser_at = e.capture_at = e.vision_at = self.now
        self.rt.follow_tick(self.now)
        self.assertIsNone(e.tracker.target)
        candidate = Detection(99, 0.9, (300, 200, 340, 250), source="dark_grass")
        depths = {99: {"z_m": 1.6, "valid_fraction": 1}}
        self.rt.detection_depths = depths
        for i in range(3):
            self.now += 0.05
            e.browser_at = e.capture_at = self.rt.depth_at = self.now
            self.rt.track_vision([candidate], depths, self.now, self.now)
            self.rt.follow_tick(self.now)
            output = e.tick(self.now)
            self.assertEqual(output, (30, 30, False) if i == 2 else (0, 0, False))
        self.assertEqual(e.tracker.target.id, 99)

    def test_no_depth_candidate_cannot_be_reselected(self):
        self.start()
        candidate = Detection(99, 0.9, (300, 200, 340, 250))
        for i in range(5):
            self.now += 0.05
            self.rt.track_vision([candidate], {}, self.now, self.now)
        self.assertIsNone(self.rt.engine.tracker.target)

    def test_overlapping_id_change_keeps_target_without_pause(self):
        self.start()
        self.target()
        self.rt.follow_tick(self.now)
        self.rt.detection_depths = {7: {"z_m": 1, "valid_fraction": 1}}
        self.now += 0.05
        e = self.rt.engine
        e.capture_at = e.browser_at = self.rt.depth_at = self.now
        candidate = Detection(99, 0.9, (301, 200, 341, 250), source="dark_grass")
        depths = {99: {"z_m": 1.02, "valid_fraction": 1}}
        self.rt.track_vision([candidate], depths, self.now, self.now)
        self.rt.detection_depths = depths
        self.rt.follow_tick(self.now)
        self.assertEqual(e.tracker.target.id, 7)
        self.assertEqual(e.phase, "APPROACHING")
        self.assertTrue(e.tracker.visible)

    def test_smooth_right_turn_keeps_both_tracks_forward(self):
        self.start()
        self.target()
        self.rt.engine.tracker.target.box = (450, 200, 490, 250)
        for _ in range(20):
            self.now += 0.05
            e = self.rt.engine
            e.capture_at = e.browser_at = e.vision_at = e.tracker.seen_at = (
                self.rt.depth_at
            ) = self.now
            self.rt.follow_tick(self.now)
        left, right, brush = e.tick(self.now)
        self.assertGreater(left, right)
        self.assertGreaterEqual(right, 30)
        self.assertFalse(brush)
