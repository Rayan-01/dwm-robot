import argparse
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from dwm_robot import robot
from dwm_core.control import DEFAULTS


class IMUTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = patch.object(robot, "ROOT", Path(self.tmp.name))
        self.root.start()
        (robot.ROOT / "config.json").write_text(json.dumps(DEFAULTS))
        self.clock = patch("dwm_robot.robot.time.monotonic", return_value=100.0)
        self.now = self.clock.start()
        self.rt = robot.Runtime(
            argparse.Namespace(
                config="config.json",
                camera=None,
                device=None,
                serial=None,
                demo=False,
                hardware=True,
            )
        )

    def tearDown(self):
        self.rt.close()
        self.clock.stop()
        self.root.stop()
        self.tmp.cleanup()

    def test_live_and_stale_sensor(self):
        self.rt.serial_line(b"I,1,0,0,0,0.1,-0.2,0.3,1,2,3")
        imu = self.rt.snapshot()["imu"]
        self.assertTrue(imu["connected"])
        self.assertEqual(imu["data"]["gyro_rad_s"], [0.1, -0.2, 0.3])
        self.assertEqual(imu["data"]["acceleration_m_s2"], [1, 2, 3])
        self.assertEqual(imu["data"]["yaw_deg"], 0)
        self.now.return_value = 100.6
        self.assertFalse(self.rt.snapshot()["imu"]["connected"])

    def test_yaw_quaternion(self):
        self.rt.serial_line(b"I,0.7071,0,0,0.7071,0,0,0,0,0,0")
        self.assertAlmostEqual(
            self.rt.snapshot()["imu"]["data"]["yaw_deg"], 90, places=1
        )

    def test_invalid_optional_sensor_does_not_change_wheel_safety(self):
        self.rt.serial_line(b"W,0,0")
        for packet in (b"I,1", b"I,nan,0,0,0,0,0,0,0,0,0", b"I,0,0,0,0,0,0,0,0,0,0"):
            self.rt.serial_line(packet)
            self.assertFalse(self.rt.snapshot()["imu"]["connected"])
            self.assertTrue(self.rt.engine.serial_ok)
            self.assertEqual(self.rt.bad_serial_lines, 0)
        self.rt.serial_line(b"I,1,0,0,0,0,0,0,0,0,0")
        self.assertTrue(self.rt.snapshot()["imu"]["connected"])
        self.rt.serial_line(b"W,0,900")
        self.assertFalse(self.rt.engine.serial_ok)

    def test_missing_sensor(self):
        self.assertFalse(self.rt.snapshot()["imu"]["connected"])
        self.assertIsNone(self.rt.snapshot()["imu"]["data"])
