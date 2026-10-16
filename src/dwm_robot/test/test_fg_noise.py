"""Test the actual Arduino edge handlers and host noise reporting."""

import argparse
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import time
import unittest

from dwm_robot import robot
from dwm_core.control import DEFAULTS


class EdgeFilterTests(unittest.TestCase):
    @unittest.skipUnless(shutil.which("g++"), "g++ not available")
    def test_actual_isrs_reject_pwm_bursts_and_preserve_valid_edges(self):
        source = (
            Path(__file__).resolve().parents[1]
            / "firmware/dwm_robot_safe/dwm_robot_safe.ino"
        ).read_text()
        counters = source[
            source.index("volatile unsigned long fg1Count") : source.index(
                "unsigned long previousFG1"
            )
        ]
        handlers = source[
            source.index("void fg1ISR()") : source.index("// IMMEDIATE MOTOR 1 COAST")
        ]
        # Match the AVR's 32-bit unsigned long, including micros() rollover.
        code = "#include <cstdint>\n#include <cassert>\nuint32_t clockUs;\nuint32_t micros(){return clockUs;}\n"
        code += (counters + handlers).replace("unsigned long", "uint32_t")
        code += r"""
int main(){
  clockUs=10000;fg1ISR();fg2ISR();
  assert(fg1Count==1&&fg2Count==1);
  // 1000 Hz chatter continues for 100 ms: never count it once per 2 ms.
  for(int i=0;i<100;i++){clockUs+=1000;fg1ISR();fg2ISR();}
  assert(fg1Count==1&&fg2Count==1&&fg1Rejected==100&&fg2Rejected==100);
  clockUs+=5000;fg1ISR();fg2ISR();
  assert(fg1Count==2&&fg2Count==2);
  // A close bounce is rejected; the next real 5 ms edge is accepted.
  clockUs+=100;fg1ISR();fg2ISR();
  clockUs+=4900;fg1ISR();fg2ISR();
  assert(fg1Count==3&&fg2Count==3&&fg2Rejected==101);
  // Ordinary high speeds are not clamped to the host's RPM stop limit.
  for(int i=0;i<20;i++){clockUs+=2500;fg1ISR();fg2ISR();}
  assert(fg1Count==23&&fg2Count==23);
  clockUs=0xfffffc00;fg1ISR();fg2ISR();
  clockUs+=5000;fg1ISR();fg2ISR();
  assert(fg1Count==25&&fg2Count==25);
}
"""
        with tempfile.TemporaryDirectory() as directory:
            cpp = Path(directory) / "edge.cpp"
            exe = Path(directory) / "edge"
            cpp.write_text(code)
            subprocess.run(
                ["g++", "-std=c++11", "-Wall", "-Wextra", str(cpp), "-o", str(exe)],
                check=True,
                capture_output=True,
            )
            subprocess.run([str(exe)], check=True, capture_output=True)


class FeedbackNoiseTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.oldroot = robot.ROOT
        robot.ROOT = Path(self.temp.name)
        (robot.ROOT / "config.json").write_text(json.dumps(DEFAULTS))
        args = argparse.Namespace(
            config="config.json",
            camera=None,
            device=None,
            serial=None,
            demo=True,
            hardware=False,
        )
        self.rt = robot.Runtime(args)
        now = time.monotonic()
        self.rt.engine.capture_at = self.rt.engine.vision_at = now
        self.rt.engine.vision_ok = True
        self.op = self.rt.claim()["operator"]

    def tearDown(self):
        self.rt.close()
        robot.ROOT = self.oldroot
        self.temp.cleanup()

    def test_noise_warns_and_blocks_auto_but_keeps_manual_deadman(self):
        self.rt.command("manual", {}, self.op)
        self.rt.serial_line(b"F,0,100")
        state = self.rt.snapshot()
        self.assertTrue(state["connection"]["feedback_noise"])
        self.assertFalse(state["auto_ready"])
        self.assertFalse(state["stopped"])
        with self.assertRaisesRegex(ValueError, "feedback noise"):
            self.rt.command("auto", {}, self.op)
        self.assertTrue(self.rt.engine.latched)

    def test_auto_stops_when_noise_appears(self):
        self.rt.command("auto", {}, self.op)
        self.rt.serial_line(b"F,0,1")
        self.assertTrue(self.rt.engine.latched)
        self.assertIn("feedback noise", self.rt.engine.reason)

    def test_stationary_counter_does_not_keep_warning_active(self):
        self.rt.serial_line(b"F,0,100")
        self.rt.fg_noise_at -= 3
        self.rt.serial_line(b"F,0,100")
        self.assertFalse(self.rt.snapshot()["connection"]["feedback_noise"])

    def test_rpm_limit_is_still_enforced(self):
        self.rt.command("manual", {}, self.op)
        self.rt.serial_line(b"W,0,600")
        self.assertTrue(self.rt.engine.latched)
        self.assertEqual(self.rt.bad_serial_lines, 1)
        self.assertIsNone(self.rt.engine.odom_at)

    def test_invalid_fg_diagnostics_stop(self):
        self.rt.serial_line(b"F,0,-2")
        self.assertEqual(self.rt.bad_serial_lines, 1)
        self.assertTrue(self.rt.engine.latched)


if __name__ == "__main__":
    unittest.main()
