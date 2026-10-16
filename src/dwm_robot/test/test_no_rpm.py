"""Manual-only RPM bypass retains the live-connection and stop checks."""

import argparse, json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from dwm_robot import robot
from dwm_core.control import DEFAULTS


class NoRPMTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.oldroot = robot.ROOT
        robot.ROOT = Path(self.temp.name)
        (robot.ROOT / "config.json").write_text(
            json.dumps(dict(DEFAULTS, use_wheel_rpm=False))
        )
        self.clock = patch("dwm_robot.robot.time.monotonic", return_value=100.0)
        self.now = self.clock.start()
        args = argparse.Namespace(
            config="config.json",
            camera=None,
            device=None,
            serial=None,
            demo=False,
            hardware=True,
        )
        self.rt = robot.Runtime(args)
        self.e = self.rt.engine
        self.e.capture_at = self.e.vision_at = 100.0
        self.e.vision_ok = True
        self.rt.serial_line(b"W,0,99999")
        self.op = self.rt.claim()["operator"]
        self.rt.command("manual", {}, self.op)

    def tearDown(self):
        self.rt.close()
        self.clock.stop()
        robot.ROOT = self.oldroot
        self.temp.cleanup()

    def fresh(self, at):
        self.now.return_value = at
        self.e.browser_at = self.e.capture_at = self.e.vision_at = at

    def test_rpm_spikes_nan_and_noise_are_ignored(self):
        for packet in (b"W,nan,inf", b"W,0,900", b"F,invalid,noise"):
            self.rt.serial_line(packet)
        self.rt.command("drive", {"direction": "forward", "seq": 1}, self.op)
        self.assertEqual(self.e.tick(100.0), (55, 55, False))
        self.assertEqual(self.rt.bad_serial_lines, 0)
        self.assertIsNone(self.e.odom_at)
        self.assertIsNone(self.rt.snapshot()["right_rpm"])

    def test_non_rpm_packets_keep_manual_connection_live(self):
        self.fresh(100.4)
        self.rt.serial_line(b"F,0,100")
        self.rt.command("drive", {"direction": "right", "seq": 1}, self.op)
        self.assertEqual(self.e.tick(100.4), (55, -55, False))
        self.assertLess(self.e.telemetry_at, 0)

    def test_usb_loss_still_stops(self):
        self.fresh(100.6)
        self.assertEqual(self.e.tick(100.6), (0, 0, False))
        self.assertTrue(self.e.latched)
        self.assertIn("Arduino communication stale", self.e.reason)

    def test_direction_release_timeout_still_stops_wheels(self):
        self.e.manual("forward", 100.0)
        self.fresh(100.4)
        self.rt.serial_line(b"F,0,0")
        self.assertEqual(self.e.tick(100.4), (0, 0, False))

    def test_stop_latch_and_brush_off_are_kept(self):
        self.rt.command("brush", {"on": True}, self.op)
        self.rt.command("stop", {}, self.op)
        self.assertEqual(self.e.tick(100.0), (0, 0, False))
        with self.assertRaises(ValueError):
            self.e.manual("forward", 100.0)

    def test_auto_cannot_use_missing_rpm(self):
        with self.assertRaisesRegex(ValueError, "RPM feedback"):
            self.rt.command("auto", {}, self.op)
        self.assertFalse(self.rt.snapshot()["auto_ready"])

    def test_camera_loss_still_stops(self):
        self.fresh(100.7)
        self.rt.serial_line(b"F,0,0")
        self.e.capture_at = 100.0
        self.assertEqual(self.e.tick(100.7), (0, 0, False))
        self.assertTrue(self.e.latched)


if __name__ == "__main__":
    unittest.main()
