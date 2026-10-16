"""Telemetry from the supplied DWM MOTOR SAFETY v2 firmware."""

import math

PHASES = ("OFF", "WAIT_STOP", "STARTING", "RUNNING", "LOW_COMMAND", "FAULT")
FAULTS = ("NONE", "START_TIMEOUT", "NO_MOTION", "BAD_CONFIG")


def integer(value, low, high):
    n = int(value)
    if str(n) != value or not low <= n <= high:
        raise ValueError("Motor telemetry integer outside range")
    return n


def fault(code, wheel):
    code = integer(code, 0, 3)
    wheel = integer(wheel, 0, 2)
    return {"code": code, "name": FAULTS[code], "wheel": wheel}


def telemetry(fields):
    if fields[:2] != ["T", "MOTOR"] or len(fields) != 15:
        raise ValueError("Expected T,MOTOR with 13 motor status values")
    wheels = []
    for offset in (2, 7):
        target = integer(fields[offset], -255, 255)
        actual = integer(fields[offset + 1], -255, 255)
        phase = integer(fields[offset + 2], 0, 5)
        enabled = bool(integer(fields[offset + 3], 0, 1))
        hz = float(fields[offset + 4])
        if not math.isfinite(hz) or hz < 0:
            raise ValueError("Invalid raw FG frequency")
        wheels.append(
            {
                "target_pwm": target,
                "actual_pwm": actual,
                "phase": PHASES[phase],
                "enabled": enabled,
                "fg_hz": hz,
            }
        )
    return {
        "left": wheels[0],
        "right": wheels[1],
        "fault": fault(fields[12], fields[13]),
        "timed_out": bool(integer(fields[14], 0, 1)),
    }
