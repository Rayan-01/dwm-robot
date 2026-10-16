import argparse, json, tempfile, time, unittest
from pathlib import Path
from unittest.mock import patch
from dwm_robot import robot
from dwm_core.control import DEFAULTS
from dwm_core.motor_protocol import telemetry

STATUS = "T,MOTOR,0,0,0,0,0.0,0,0,0,0,0.0,0,0,0"


class MotorProtocolTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.oldroot = robot.ROOT
        robot.ROOT = Path(self.temp.name)
        (robot.ROOT / "config.json").write_text(json.dumps(DEFAULTS))
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
        self.now = time.monotonic()
        e = self.rt.engine
        e.vision_ok = e.serial_ok = True
        e.vision_at = e.capture_at = e.telemetry_at = e.link_at = self.now
        self.rt.firmware_protocol = "motor_safety_v2"
        self.writes = []
        self.rt.write = self.writes.append
        self.operator = self.rt.claim()["operator"]
        self.rt.serial_line(STATUS.encode())

    def tearDown(self):
        self.rt.close()
        robot.ROOT = self.oldroot
        self.temp.cleanup()

    def test_t_status_parses_phases_and_applied_pwm(self):
        s = telemetry("T,MOTOR,-30,-70,2,1,25.0,-30,-70,3,1,26.0,0,0,0".split(","))
        self.assertEqual(s["left"]["phase"], "STARTING")
        self.assertEqual(s["right"]["phase"], "RUNNING")
        self.assertEqual(s["left"]["actual_pwm"], -70)

    def test_new_enable_does_not_wait_for_legacy_ack(self):
        self.rt.command("manual", {}, self.operator)
        self.assertTrue(self.rt.armed)
        self.assertFalse(self.rt.awaiting_arm)
        self.assertIn("M,0,0\nR,0\nQ\n", self.writes)
        self.assertNotIn("E\n", self.writes)

    def test_motor_fault_is_not_misread_as_fg_noise(self):
        self.rt.command("manual", {}, self.operator)
        self.rt.serial_line(b"F,MOTOR,2,1")
        self.assertTrue(self.rt.engine.latched)
        self.assertEqual(self.rt.motor_fault["name"], "NO_MOTION")
        self.assertEqual(self.rt.bad_serial_lines, 0)
        with self.assertRaisesRegex(ValueError, "motor fault"):
            self.rt.command("manual", {}, self.operator)

    def test_status_fault_latches_stop(self):
        self.rt.serial_line(b"T,MOTOR,30,0,5,0,0,30,0,5,0,0,1,2,0")
        self.assertEqual(self.rt.motor_fault["name"], "START_TIMEOUT")
        self.assertTrue(self.rt.engine.latched)

    def test_clear_requires_stop_and_ack_does_not_restart(self):
        self.rt.command("manual", {}, self.operator)
        with self.assertRaises(ValueError):
            self.rt.command("clear_fault", {}, self.operator)
        self.rt.stop()
        self.rt.command("clear_fault", {}, self.operator)
        self.assertTrue(self.rt.fault_clear_pending)
        self.assertIn("M,0,0\nR,0\nC\nQ\n", self.writes)
        self.rt.serial_line(b"ACK,MOTOR_CLEAR")
        self.assertFalse(self.rt.fault_clear_pending)
        self.assertTrue(self.rt.engine.latched)
        self.assertFalse(self.rt.armed)

    def test_clear_rejects_rotating_wheels(self):
        self.rt.engine.left_rpm = 2
        with self.assertRaises(ValueError):
            self.rt.command("clear_fault", {}, self.operator)

    def test_bad_status_is_rejected(self):
        self.rt.serial_line(b"T,MOTOR,0,0,9,0,0,0,0,0,0,0,0,0,0")
        self.assertEqual(self.rt.bad_serial_lines, 1)
        self.assertTrue(self.rt.engine.latched)

    def test_new_boot_marker_stops(self):
        self.rt.command("manual", {}, self.operator)
        self.rt.serial_line(b"READY")
        self.assertTrue(self.rt.engine.latched)

    def test_status_watchdog_stops_active_host(self):
        self.rt.command("manual", {}, self.operator)
        self.rt.serial_line((STATUS[:-1] + "1").encode())
        self.assertTrue(self.rt.engine.latched)

    def test_connect_recognizes_new_firmware_without_version_command(self):
        class Port:
            def reset_input_buffer(self):
                pass

            def readline(self):
                return (STATUS + "\n").encode()

            def close(self):
                pass

        port = Port()
        # This tests the mocked handshake, not attached USB device discovery.
        self.rt.c["serial_port"] = "mock-arduino"
        self.rt.firmware_protocol = "dwm2"
        with (
            patch("serial.Serial", return_value=port),
            patch("dwm_robot.robot.time.sleep"),
        ):
            self.rt.connect_serial()
        self.assertTrue(self.rt.firmware_ok)
        self.assertEqual(self.rt.firmware_protocol, "motor_safety_v2")
