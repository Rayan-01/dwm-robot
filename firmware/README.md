# Arduino motor controller

`dwm_robot_safe/dwm_robot_safe.ino` is the DWM2 firmware used in the recorded
working configuration. The host recognizes its serial telemetry and commands;
the Arduino applies motor outputs, reads FG pulses and reports sensor data.

| Setting | Value |
| --- | --- |
| Serial rate | 115200 baud |
| Gear ratio | 20:1 |
| FG conversion | Four pulses per motor revolution configured |
| Brush relay | D11, active-low |
| Serial command watchdog | 500 ms |

Required Arduino libraries are Wire, Adafruit Unified Sensor, Adafruit BNO055
and Adafruit ADS1X15. Select the physical AVR board and processor before flashing.
See [hardware](../docs/hardware.md) for pin assignments. Colcon and ROS launch
files do not compile or upload the Arduino sketch.
