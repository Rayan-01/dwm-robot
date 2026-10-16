"""Physical direction and odometry regressions; uses no physical hardware."""

import argparse
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

from dwm_robot import robot
from dwm_core.control import DEFAULTS, Detection, validate_config


class MotorPolarityTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.oldroot = robot.ROOT
        robot.ROOT = Path(self.temp.name)
        config = dict(DEFAULTS, left_motor_sign=-1, right_motor_sign=-1)
        (robot.ROOT / "config.json").write_text(json.dumps(config))
        args = argparse.Namespace(
            config="config.json",
            camera=None,
            device=None,
            serial=None,
            demo=True,
            hardware=False,
        )
        self.rt = robot.Runtime(args)
        self.rt.ser = Mock()

    def tearDown(self):
        self.rt.close()
        robot.ROOT = self.oldroot
        self.temp.cleanup()

    def fresh(self, now):
        engine = self.rt.engine
        engine.browser_at = engine.capture_at = now
        engine.vision_at = now - 0.01
        engine.vision_ok = True

    def test_auto_turn_approach_and_clean_use_same_polarity(self):
        # Ground coordinates choose the movement; serial output reverses both
        # electrical signs while the controller still sees physical forward.
        cases = [
            ((-0.2, 1), "M,45,-45\nR,0\n"),
            ((0.2, 1), "M,-45,45\nR,0\n"),
            ((0, 1), "M,-50,-50\nR,0\n"),
            ((0, 0.25), "M,-45,-45\nR,1\n"),
        ]
        for ground, expected in cases:
            with self.subTest(ground=ground):
                now = 100.0
                self.fresh(now)
                self.rt.engine.stop()
                self.rt.engine.start("AUTO", now)
                for _ in range(3):
                    now += 0.05
                    self.fresh(now)
                    self.rt.engine.vision(
                        [Detection(1, 0.9, (10, 10, 20, 30), ground)], now, now
                    )
                    output = self.rt.engine.tick(now)
                self.rt.write_output(output)
                self.rt.ser.write.assert_called_with(expected.encode())

    def test_forward_feedback_advances_odometry_and_finishes_pass(self):
        self.fresh(100.0)
        self.rt.engine.start("AUTO", 100.0)
        for now in (100.0, 100.05, 100.1):
            self.fresh(now)
            self.rt.engine.vision(
                [Detection(1, 0.9, (10, 10, 20, 30), (0, 0.25))], now, now
            )
            self.rt.engine.tick(now)
        self.assertEqual(self.rt.engine.phase, "CLEAN_PASS")
        for i in range(22):
            now = 100.1 + i * 0.1
            self.fresh(now)
            with patch("dwm_robot.robot.time.monotonic", return_value=now):
                self.rt.serial_line(b"W,-60,-60")
            self.rt.engine.tick(now)
        self.assertAlmostEqual(self.rt.engine.left_m, math.pi * 0.15 * 2.1)
        self.assertEqual(self.rt.engine.left_rpm, 60)
        self.assertEqual(self.rt.engine.cycles, 1)
        self.assertTrue(self.rt.engine.latched)

    def test_reverse_and_turn_feedback_keeps_signed_coordinates(self):
        for raw, expected in [
            (b"W,12,12", (-12, -12)),
            (b"W,12,-12", (-12, 12)),
            (b"W,-12,12", (12, -12)),
        ]:
            self.rt.serial_line(raw)
            self.assertEqual(
                (self.rt.engine.left_rpm, self.rt.engine.right_rpm), expected
            )

    def test_stop_never_becomes_movement(self):
        self.rt.write_output((0, 0, False))
        self.rt.ser.write.assert_called_with(b"M,0,0\nR,0\n")

    def test_bad_motor_polarity_is_rejected(self):
        for key in ("left_motor_sign", "right_motor_sign"):
            for value in (0, 2, -2, True, None, float("nan")):
                with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                    validate_config(dict(DEFAULTS, **{key: value}))


if __name__ == "__main__":
    unittest.main()
