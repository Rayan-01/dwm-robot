"""Exercise real pyserial over a virtual USB/tty, never physical hardware."""

import argparse
import importlib.util
import json
import os
from pathlib import Path
import select
import tempfile
import threading
import time
import unittest
from dwm_robot import robot
from dwm_core.control import DEFAULTS


@unittest.skipUnless(
    os.name == "posix" and importlib.util.find_spec("serial"),
    "Needs Linux PTY and pyserial",
)
class SerialIntegrationTests(unittest.TestCase):
    def test_real_serial_handshake_arm_movement_then_latched_stop(self):
        for sign in (1, -1):
            with self.subTest(motor_sign=sign):
                self.run_serial_sequence(sign)

    def run_serial_sequence(self, sign):
        master, slave = os.openpty()
        commands = []
        done = threading.Event()

        def fake_nano():
            buffer = b""
            active = False
            enabled = False
            last = 0
            while not done.is_set():
                ready, _, _ = select.select([master], [], [], 0.01)
                if ready:
                    buffer += os.read(master, 4096)
                    while b"\n" in buffer:
                        raw, buffer = buffer.split(b"\n", 1)
                        cmd = raw.decode(errors="replace").strip()
                        commands.append(cmd)
                        if cmd == "V":
                            active = True
                            os.write(master, b"K,DWM2\n")
                        elif cmd == "E":
                            enabled = True
                            os.write(master, b"A,ENABLED\n")
                        elif cmd == "D":
                            enabled = False
                            os.write(master, b"A,STOP\n")
                if active and time.monotonic() - last > 0.05:
                    os.write(master, b"W,0.00,0.00\n")
                    last = time.monotonic()

        thread = threading.Thread(target=fake_nano, daemon=True)
        thread.start()
        old = robot.ROOT
        try:
            with tempfile.TemporaryDirectory() as directory:
                robot.ROOT = Path(directory)
                config = dict(DEFAULTS, left_motor_sign=sign, right_motor_sign=sign)
                (robot.ROOT / "config.json").write_text(json.dumps(config))
                args = argparse.Namespace(
                    config="config.json",
                    camera=None,
                    device=None,
                    serial=os.ttyname(slave),
                    demo=False,
                    hardware=True,
                )
                rt = robot.Runtime(args)
                try:
                    rt.connect_serial()
                    self.assertTrue(rt.firmware_ok)
                    rt.spawn(rt.serial_reader)
                    rt.spawn(rt.control_loop)
                    deadline = time.monotonic() + 1
                    while rt.engine.telemetry_at < 0 and time.monotonic() < deadline:
                        time.sleep(0.02)
                    now = time.monotonic()
                    rt.engine.capture_at = rt.engine.vision_at = now
                    rt.engine.vision_ok = True
                    op = rt.claim()["operator"]
                    rt.command("manual", {}, op)
                    directions = [
                        ("forward", 55, 55),
                        ("reverse", -55, -55),
                        ("left", -55, 55),
                        ("right", 55, -55),
                        ("stop", 0, 0),
                    ]
                    for seq, (direction, left, right) in enumerate(directions, 1):
                        now = time.monotonic()
                        rt.engine.capture_at = rt.engine.vision_at = now
                        rt.command("heartbeat", {}, op)
                        before = len(commands)
                        rt.command("drive", {"direction": direction, "seq": seq}, op)
                        expected = "M,%d,%d" % (left * sign, right * sign)
                        deadline = time.monotonic() + 0.3
                        while (
                            expected not in commands[before:]
                            and time.monotonic() < deadline
                        ):
                            time.sleep(0.01)
                        self.assertIn(expected, commands[before:])
                    rt.command("brush", {"on": True}, op)
                    time.sleep(0.06)
                    self.assertIn("R,1", commands)
                    rt.command("stop", {}, op)
                    time.sleep(0.1)
                    self.assertTrue(rt.engine.latched)
                    self.assertFalse(rt.armed)
                    pos = len(commands) - 1 - commands[::-1].index("D")
                    self.assertTrue(
                        all(
                            not cmd.startswith("M,") and cmd != "R,1"
                            for cmd in commands[pos + 1 :]
                        )
                    )
                    with self.assertRaises(ValueError):
                        rt.command("drive", {"direction": "forward", "seq": 6}, op)
                finally:
                    rt.close()
        finally:
            done.set()
            thread.join(timeout=0.5)
            os.close(master)
            os.close(slave)
            robot.ROOT = old


if __name__ == "__main__":
    unittest.main()
