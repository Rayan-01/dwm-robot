"""Robot decisions. No hardware or ML imports; monotonic seconds throughout."""

import math
from dataclasses import dataclass

DEFAULTS = {
    "camera": "/dev/video0",
    "width": 640,
    "height": 480,
    "imgsz": 640,
    "model": "models/best-v2.pt",
    "device": "auto",
    "serial_port": "auto",
    "baud": 115200,
    "use_wheel_rpm": True,
    "left_motor_sign": 1,
    "right_motor_sign": 1,
    "detection_conf": 0.10,
    "lock_conf": 0.60,
    "confirm_frames": 3,
    "target_hold_s": 1.0,
    "vision_timeout_s": 0.65,
    "browser_timeout_s": 1.5,
    "telemetry_timeout_s": 0.5,
    "max_pwm": 90,
    "manual_pwm": 55,
    "approach_pwm": 50,
    "clean_pwm": 45,
    "turn_pwm": 45,
    "align_tolerance_m": 0.04,
    "pass_trigger_m": 0.35,
    "wheel_diameter_m": 0.0,
    "wheel_scale": 1.0,
    "axle_to_brush_m": 0.0,
    "brush_margin_m": 0.12,
    "brush_width_m": 0.0,
    "max_clean_travel_m": 1.5,
    "max_clean_time_s": 20.0,
    "max_approach_time_s": 60.0,
    "max_wheel_rpm": 150.0,
    "max_pass_drift_m": 0.05,
    "gear_ratio": 20.0,
    "homography": [],
    "ground_polygon": [],
    "ground_range": [],
    "calibration_resolution": [],
    "geometry_verified": False,
}


def finite_number(value):
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(value)
    )


def validate_config(c):
    for key, default in DEFAULTS.items():
        if isinstance(default, (int, float)) and not isinstance(default, bool):
            if not finite_number(c[key]):
                raise ValueError("Invalid numeric setting: " + key)
    for key in ("left_motor_sign", "right_motor_sign"):
        if c[key] not in (-1, 1):
            raise ValueError(key + " must be -1 or 1")
    for key in ("width", "height", "imgsz", "baud", "confirm_frames"):
        if int(c[key]) != c[key] or c[key] <= 0:
            raise ValueError(key + " must be a positive integer")
    if not 0.01 <= c["detection_conf"] <= c["lock_conf"] <= 1:
        raise ValueError(
            "Confidence thresholds must satisfy 0.01 <= detection <= lock <= 1"
        )
    if not 1 <= c["confirm_frames"] <= 30:
        raise ValueError("confirm_frames must be between 1 and 30")
    if not 1 <= c["max_pwm"] <= 120:
        raise ValueError("max_pwm must be 1..120")
    for key in ("manual_pwm", "approach_pwm", "clean_pwm", "turn_pwm"):
        if not 1 <= c[key] <= c["max_pwm"]:
            raise ValueError(key + " exceeds PWM limit")
    for key in (
        "browser_timeout_s",
        "vision_timeout_s",
        "telemetry_timeout_s",
        "target_hold_s",
    ):
        if not 0.1 <= c[key] <= 5:
            raise ValueError(key + " must be 0.1..5 seconds")
    for key in (
        "align_tolerance_m",
        "pass_trigger_m",
        "brush_margin_m",
        "max_clean_travel_m",
        "max_pass_drift_m",
    ):
        if not 0 < c[key] <= 5:
            raise ValueError(key + " must be positive and <= 5 metres")
    if not 0 < c["max_clean_time_s"] <= 60 or not 0 < c["max_approach_time_s"] <= 180:
        raise ValueError("Invalid motion timeout")
    if not 0 < c["wheel_scale"] <= 5 or not 0 < c["max_wheel_rpm"] <= 1000:
        raise ValueError("Invalid odometry setting")
    if not 1 <= c["gear_ratio"] <= 200:
        raise ValueError("Gear ratio must be 1..200")
    if not isinstance(c["geometry_verified"], bool):
        raise ValueError("geometry_verified must be true or false")
    if not isinstance(c["use_wheel_rpm"], bool):
        raise ValueError("use_wheel_rpm must be true or false")
    if c["homography"]:
        h = c["homography"]
        if len(h) != 9 or not all(finite_number(v) for v in h):
            raise ValueError("Homography needs 9 finite numbers")
        poly = c["ground_polygon"]
        if len(poly) != 4 or any(
            len(p) != 2 or not all(finite_number(v) for v in p) for p in poly
        ):
            raise ValueError("Invalid ground polygon")
        if len(c["ground_range"]) != 4 or not all(
            finite_number(v) for v in c["ground_range"]
        ):
            raise ValueError("Invalid ground range")
        if len(c["calibration_resolution"]) != 2:
            raise ValueError("Invalid calibration resolution")
    for key in ("wheel_diameter_m", "axle_to_brush_m", "brush_width_m"):
        if not 0 <= c[key] <= 3:
            raise ValueError(key + " must be 0..3 metres")
    return c


def auto_ready(c):
    return (
        c["use_wheel_rpm"]
        and bool(c["homography"])
        and c["geometry_verified"]
        and 0.01 <= c["wheel_diameter_m"] <= 1
        and 0.01 <= c["axle_to_brush_m"] <= 2
        and c["brush_width_m"] > 2 * c["align_tolerance_m"]
        and c["pass_trigger_m"] + c["axle_to_brush_m"] + c["brush_margin_m"]
        <= c["max_clean_travel_m"]
    )


def in_polygon(u, v, poly):
    # Convex calibration quadrilateral, including its boundary.
    signs = []
    for a, b in zip(poly, poly[1:] + poly[:1]):
        cross = (b[0] - a[0]) * (v - a[1]) - (b[1] - a[1]) * (u - a[0])
        if abs(cross) > 1e-8:
            signs.append(cross > 0)
    return bool(signs) and all(s == signs[0] for s in signs)


def ground_point(box, c):
    # Bottom-centre approximates where the object meets a flat ground plane.
    u, v = (box[0] + box[2]) / 2, box[3]
    h = c["homography"]
    if not h or not in_polygon(u, v, c["ground_polygon"]):
        return None
    den = h[6] * u + h[7] * v + h[8]
    if abs(den) < 1e-8:
        return None
    x, y = (h[0] * u + h[1] * v + h[2]) / den, (h[3] * u + h[4] * v + h[5]) / den
    lo_x, hi_x, lo_y, hi_y = c["ground_range"]
    if not all(math.isfinite(q) for q in (x, y)) or not (
        lo_x <= x <= hi_x and lo_y <= y <= hi_y
    ):
        return None
    return x, y


@dataclass
class Detection:
    id: int
    confidence: float
    box: tuple
    ground: object = None
    source: str = "model"


class TargetLock:
    def __init__(self, c):
        self.c = c
        self.reset()

    def reset(self):
        self.target = None
        self.seen_at = -1e9
        self.counts = {}
        self.visible = False

    def update(self, detections, captured_at):
        self.visible = False
        if self.target is not None:
            match = next((d for d in detections if d.id == self.target.id), None)
            if match is not None:
                self.target, self.seen_at, self.visible = match, captured_at, True
            # Never silently substitute a different track ID.
            return
        self.counts = {
            d.id: self.counts.get(d.id, 0) + 1
            for d in detections
            if d.confidence >= self.c["lock_conf"]
        }
        candidates = [
            d
            for d in detections
            if self.counts.get(d.id, 0) >= self.c["confirm_frames"]
        ]
        if candidates:
            # Nearest calibrated target; confidence is the fallback when uncalibrated.
            candidates.sort(
                key=lambda d: (
                    d.ground is None,
                    d.ground[1] if d.ground else -d.confidence,
                )
            )
            self.target = candidates[0]
            self.seen_at, self.visible = captured_at, True


class Controller:
    def __init__(self, c, dry_run=False):
        self.c = c
        self.dry_run = dry_run
        self.tracker = TargetLock(c)
        self.mode = "STOPPED"
        self.phase = "STOPPED"
        self.reason = "Press Enable manual or Start auto"
        self.latched = True
        self.browser_at = -1e9
        self.capture_at = -1e9
        self.vision_at = -1e9
        self.vision_ok = False
        self.serial_ok = False
        self.telemetry_at = -1e9
        self.link_at = -1e9
        self.odom_at = None
        self.left_rpm = self.right_rpm = 0.0
        self.left_m = self.right_m = 0.0
        self.manual_at = -1e9
        self.manual_pair = (0, 0)
        self.manual_brush = False
        self.started_at = 0.0
        self.pass_at = 0.0
        self.pass_origin = (0.0, 0.0)
        self.pass_distance = 0.0
        self.output = (0, 0, False)
        self.cycles = 0

    def stop(self, reason="Stopped by operator"):
        self.mode, self.phase, self.reason = "STOPPED", "STOPPED", reason
        self.latched = True
        self.manual_pair = (0, 0)
        self.manual_brush = False
        self.output = (0, 0, False)

    def healthy(self, now):
        if now - self.browser_at > self.c["browser_timeout_s"]:
            return "Browser heartbeat lost"
        if (
            not self.vision_ok
            or now - self.capture_at > self.c["vision_timeout_s"]
            or now - self.vision_at > self.c["vision_timeout_s"]
        ):
            return "Camera or inference stale"
        if not self.dry_run and not self.serial_ok:
            return "Arduino link unavailable or invalid wheel telemetry; check connection diagnostics"
        if (
            not self.dry_run
            and not self.c["use_wheel_rpm"]
            and now - self.link_at > self.c["telemetry_timeout_s"]
        ):
            return "Arduino communication stale; check USB connection"
        if (
            not self.dry_run
            and self.c["use_wheel_rpm"]
            and now - self.telemetry_at > self.c["telemetry_timeout_s"]
        ):
            return "Wheel telemetry stale; check Arduino connection diagnostics"
        return None

    def start(self, mode, now):
        if mode not in ("MANUAL", "AUTO"):
            raise ValueError("Invalid mode")
        if mode == "AUTO" and not self.c["use_wheel_rpm"]:
            raise ValueError(
                "Auto cleaning requires wheel RPM feedback; manual control is available"
            )
        error = self.healthy(now)
        if error:
            raise ValueError(error)
        if mode == "AUTO" and not auto_ready(self.c):
            raise ValueError(
                "Complete ground calibration, geometry and direction/odometry verification first"
            )
        self.tracker.reset()
        self.output = (0, 0, False)
        self.manual_pair = (0, 0)
        self.manual_brush = False
        self.latched = False
        self.mode = mode
        self.phase = "SEARCHING" if mode == "AUTO" else "MANUAL"
        self.reason = (
            "Waiting for confirmed target"
            if mode == "AUTO"
            else "Hold a direction button to drive"
        )
        self.started_at = now

    def vision(self, detections, captured_at, now):
        if captured_at <= self.vision_at:
            return
        self.vision_at = captured_at
        self.vision_ok = True
        self.tracker.update(detections, captured_at)

    def wheels(self, left_rpm, right_rpm, now):
        if not self.c["use_wheel_rpm"]:
            return
        if not all(
            math.isfinite(x) and abs(x) <= self.c["max_wheel_rpm"]
            for x in (left_rpm, right_rpm)
        ):
            self.stop("Invalid wheel telemetry")
            self.serial_ok = False
            self.odom_at = None
            return
        if self.odom_at is not None:
            dt = now - self.odom_at
            if 0 < dt <= self.c["telemetry_timeout_s"]:
                scale = (
                    math.pi * self.c["wheel_diameter_m"] * self.c["wheel_scale"] / 60
                )
                # Trapezoidal integration includes coasting after a stop.
                self.left_m += (self.left_rpm + left_rpm) * 0.5 * scale * dt
                self.right_m += (self.right_rpm + right_rpm) * 0.5 * scale * dt
        self.left_rpm, self.right_rpm = left_rpm, right_rpm
        self.telemetry_at = self.odom_at = now
        self.link_at = now
        # Link recovery never clears the latched stop or enables movement.
        self.serial_ok = True

    def manual(self, direction, now):
        if self.mode != "MANUAL" or self.latched:
            raise ValueError("Enable manual first")
        p = int(self.c["manual_pwm"])
        mapping = {
            "forward": (p, p),
            "reverse": (-p, -p),
            "left": (-p, p),
            "right": (p, -p),
            "stop": (0, 0),
        }
        if direction not in mapping:
            raise ValueError("Unknown direction")
        self.manual_pair, self.manual_at = mapping[direction], now

    def tick(self, now):
        self.output = (0, 0, False)
        if self.latched:
            return self.output
        error = self.healthy(now)
        if error:
            self.stop(error)
            return self.output
        if self.mode == "MANUAL":
            pair = self.manual_pair if now - self.manual_at < 0.35 else (0, 0)
            self.output = (*pair, self.manual_brush)
            return self.output
        if self.phase == "CLEAN_PASS":
            left = self.left_m - self.pass_origin[0]
            right = self.right_m - self.pass_origin[1]
            travel = (left + right) / 2
            if now - self.pass_at > self.c["max_clean_time_s"]:
                self.stop("Cleaning pass timed out; inspect robot")
            elif (
                abs(left - right) > self.c["max_pass_drift_m"]
                or min(left, right) < -0.02
            ):
                self.stop("Cleaning pass drift/reverse detected")
            elif travel >= self.pass_distance:
                self.cycles += 1
                self.stop("Pass completed; check collection, then start next target")
            else:
                self.output = (int(self.c["clean_pwm"]), int(self.c["clean_pwm"]), True)
            return self.output
        if now - self.started_at > self.c["max_approach_time_s"]:
            self.stop("Search/approach time limit reached")
            return self.output
        lock = self.tracker
        if lock.target is None:
            self.phase = "SEARCHING"
            self.reason = "No confirmed target; robot stays still"
            return self.output
        age = now - lock.seen_at
        if age > self.c["target_hold_s"]:
            self.stop("Locked target lost; press Start auto to select again")
            return self.output
        if not lock.visible or age > self.c["vision_timeout_s"]:
            self.phase, self.reason = (
                "TARGET_MISSING",
                "Keeping target ID; wheels stopped",
            )
            return self.output
        xy = lock.target.ground
        if xy is None:
            self.phase, self.reason = (
                "OUTSIDE_CALIBRATION",
                "Target outside measured ground area; stopped",
            )
            return self.output
        x, y = xy
        if y <= 0:
            self.stop("Target behind axle / invalid ground location")
            return self.output
        if abs(x) > self.c["align_tolerance_m"]:
            self.phase, self.reason = (
                "ALIGNING",
                "Aligning the selected target with the rear brush",
            )
            p = int(self.c["turn_pwm"])
            self.output = (p, -p, False) if x > 0 else (-p, p, False)
            return self.output
        if y > self.c["pass_trigger_m"]:
            self.phase, self.reason = "APPROACHING", "Approaching the same track ID"
            p = int(self.c["approach_pwm"])
            self.output = (p, p, False)
            return self.output
        # Stop before committing a straight pass. Avoid carrying turn inertia into it.
        self.phase, self.reason = "SETTLING", "Waiting for wheel rotation to settle"
        if abs(self.left_rpm) > 1 or abs(self.right_rpm) > 1:
            return self.output
        distance = y + self.c["axle_to_brush_m"] + self.c["brush_margin_m"]
        if distance > self.c["max_clean_travel_m"]:
            self.stop("Required cleaning distance exceeds configured limit")
            return self.output
        self.pass_distance = distance
        self.pass_origin = self.left_m, self.right_m
        self.pass_at = now
        self.phase, self.reason = (
            "CLEAN_PASS",
            "Brush on; measured forward pass (target may leave front camera)",
        )
        p = int(self.c["clean_pwm"])
        self.output = (p, p, True)
        return self.output

    def status(self, now):
        t = self.tracker.target
        return {
            "mode": self.mode,
            "phase": self.phase,
            "reason": self.reason,
            "stopped": self.latched,
            "dry_run": self.dry_run,
            "auto_ready": auto_ready(self.c),
            "output": list(self.output),
            "cycles": self.cycles,
            "vision_age_s": (
                round(max(0, now - self.vision_at), 2) if self.vision_at > 0 else None
            ),
            "serial_ok": self.serial_ok,
            "wheel_rpm_enabled": self.c["use_wheel_rpm"],
            "left_rpm": round(self.left_rpm, 2) if self.c["use_wheel_rpm"] else None,
            "right_rpm": round(self.right_rpm, 2) if self.c["use_wheel_rpm"] else None,
            "odometry_left_m": round(self.left_m, 3),
            "odometry_right_m": round(self.right_m, 3),
            "travel_m": round(
                (
                    (self.left_m - self.pass_origin[0])
                    + (self.right_m - self.pass_origin[1])
                )
                / 2,
                3,
            ),
            "pass_distance_m": round(self.pass_distance, 3),
            "target": (
                None
                if t is None
                else {
                    "id": t.id,
                    "confidence": round(t.confidence, 3),
                    "source": t.source,
                    "box": list(t.box),
                    "visible": self.tracker.visible,
                    "age_s": round(max(0, now - self.tracker.seen_at), 2),
                    "ground": t.ground,
                }
            ),
        }
