import argparse, json, tempfile, time, unittest, shutil, subprocess
from pathlib import Path
from dwm_robot import robot
from dwm_core.control import DEFAULTS


class DriveRPMTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.oldroot = robot.ROOT
        robot.ROOT = Path(self.temp.name)
        (robot.ROOT / "config.json").write_text(
            json.dumps(dict(DEFAULTS, left_motor_sign=-1, right_motor_sign=-1))
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

    def tearDown(self):
        self.rt.close()
        robot.ROOT = self.oldroot
        self.temp.cleanup()

    def test_motor_and_output_rpm_are_separate(self):
        self.rt.serial_line(b"N,-3000,-2000")
        self.rt.serial_line(b"W,-150,-100")
        s = self.rt.snapshot()
        self.assertEqual(s["drive_motor_rpm"]["left"], 3000)
        self.assertEqual(s["left_rpm"], 150)
        self.assertEqual(s["drive_motor_rpm"]["gear_ratio"], 20)
        self.assertFalse(s["drive_motor_rpm"]["calibration_verified"])

    def test_stale_motor_rpm_is_not_displayed_as_live(self):
        self.rt.serial_line(b"N,100,200")
        self.rt.motor_rpm_at -= 2
        self.assertIsNone(self.rt.snapshot()["drive_motor_rpm"]["left"])

    def test_missing_motor_telemetry_is_not_invented_from_wheels(self):
        self.rt.serial_line(b"W,5,5")
        self.assertIsNone(self.rt.snapshot()["drive_motor_rpm"]["left"])

    def test_bad_motor_rpm_is_rejected(self):
        for line in (b"N,nan,0", b"N,3001,0", b"N,1"):
            self.rt.serial_line(line)
        self.assertEqual(self.rt.bad_serial_lines, 3)

    @unittest.skipUnless(shutil.which("g++"), "g++ unavailable")
    def test_actual_firmware_rpm_equation_uses_20_to_1(self):
        source = (
            Path(__file__).resolve().parents[1]
            / "firmware/dwm_robot_safe/dwm_robot_safe.ino"
        ).read_text()
        start = source.index("void updateSpeed()")
        opening = source.index("{", start)
        depth = 1
        end = opening + 1
        while depth:
            if source[end] == "{":
                depth += 1
            elif source[end] == "}":
                depth -= 1
            end += 1
        function = source[start:end]
        import re

        ratio = re.search(r"const float GEAR_RATIO = ([\d.]+);", source).group(1)
        poles = re.search(r"const float POLE_PAIRS = ([\d.]+);", source).group(1)
        stub = (
            """#include <cmath>
#include <cassert>
unsigned long clockMS=0,lastSpeedSample=0,fg1Count=0,fg2Count=0,previousFG1=0,previousFG2=0;
const unsigned long SPEED_SAMPLE_INTERVAL=100;
float fg1RawHz=0,fg2RawHz=0,fg1Hz=0,fg2Hz=0,motor1RPM=0,motor2RPM=0,wheel1RPM=0,wheel2RPM=0;
int motor1Direction=1,motor2Direction=-1;
unsigned long millis(){return clockMS;} void noInterrupts(){} void interrupts(){}
"""
            + f"const float GEAR_RATIO={ratio},POLE_PAIRS={poles};\n"
            + function
            + """
int main(){assert(GEAR_RATIO==20);
for(int i=0;i<30;i++){clockMS+=100;fg1Count+=20;fg2Count+=20;updateSpeed();}
assert(std::fabs(motor1RPM-3000)<.1);assert(std::fabs(motor2RPM+3000)<.1);
assert(std::fabs(wheel1RPM-150)<.01);assert(std::fabs(wheel2RPM+150)<.01);}
"""
        )
        with tempfile.TemporaryDirectory() as d:
            cpp = Path(d) / "rpm.cpp"
            exe = Path(d) / "rpm"
            cpp.write_text(stub)
            subprocess.run(
                ["g++", "-std=c++11", str(cpp), "-o", str(exe)],
                check=True,
                capture_output=True,
            )
            subprocess.run([str(exe)], check=True, capture_output=True)
