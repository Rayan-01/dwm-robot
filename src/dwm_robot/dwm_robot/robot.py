#!/usr/bin/env python3
"""Local robot server: one capture worker, one tracker, one control loop."""

import argparse
import hmac
import json
import logging
import math
import os
from pathlib import Path
import secrets
import signal
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit
from urllib.request import Request, urlopen
from urllib.error import HTTPError, URLError

from dwm_core.control import (
    DEFAULTS,
    Controller,
    Detection,
    ground_point,
    validate_config,
)

from dwm_core.paths import runtime_root, package_share, model_path

ROOT = runtime_root()
LOG = logging.getLogger("poop_robot")


def atomic_json(path, obj):
    temp = path.with_suffix(".tmp")
    temp.write_text(json.dumps(obj, indent=2) + "\n")
    os.replace(str(temp), str(path))


class Runtime:
    def __init__(self, args):
        self.args = args
        self.config_path = ROOT / args.config
        self.c = dict(DEFAULTS)
        self.c.update(json.loads(self.config_path.read_text()))
        if args.camera is not None:
            self.c["camera"] = args.camera
        if args.device:
            self.c["device"] = args.device
        if args.serial:
            self.c["serial_port"] = args.serial
        validate_config(self.c)
        self.demo = args.demo
        self.dry_run = not args.hardware
        if self.demo:
            self.c.update(
                {
                    "wheel_diameter_m": 0.15,
                    "axle_to_brush_m": 0.45,
                    "brush_width_m": 0.45,
                    "geometry_verified": True,
                    "homography": [0.002, 0, -0.64, 0, -0.003, 1.6, 0, 0, 1],
                    "ground_polygon": [[0, 0], [640, 0], [640, 480], [0, 480]],
                    "ground_range": [-0.64, 0.64, 0.16, 1.6],
                    "calibration_resolution": [640, 480],
                    "width": 640,
                    "height": 480,
                }
            )
        self.engine = Controller(self.c, self.dry_run)
        self.mu = threading.RLock()
        self.done = threading.Event()
        self.frame_cv = threading.Condition()
        self.frame = None
        self.frame_at = -1e9
        self.frame_seq = 0
        self.jpeg = None
        self.jpeg_seq = 0
        self.jpeg_at = -1e9
        self.camera_fps = 0.0
        self.preview_detections = []
        self.depth_frame = None
        self.depth_jpeg = None
        self.depth_at = -1e9
        self.depth_skew_s = None
        self.camera_info = {"source": self.c["camera"]}
        self.detection_depths = {}
        self.follow = None
        self.navigation = None
        self.instance_id = secrets.token_hex(8)
        self.forward_pass_status = None
        self.cleaning_status = None
        self.jpeg_cv = threading.Condition()
        self.cap = None
        self.ser = None
        self.firmware_ok = False
        self.firmware_protocol = "dwm2"
        self.motor_status = None
        self.motor_rpm = None
        self.motor_rpm_at = -1e9
        self.motor_status_at = -1e9
        self.motor_fault = {"code": 0, "name": "NONE", "wheel": 0}
        self.fault_clear_pending = False
        self.armed = False
        self.awaiting_arm = False
        self.arm_deadline = 0.0
        self.operator = None
        self.operator_seen = -1e9
        self.drive_seq = -1
        self.error = ""
        self.fps = 0.0
        self.battery = None
        self.imu = None
        self.imu_at = None
        self.imu_error = ""
        self.serial_reader_alive = False
        self.serial_line_at = None
        self.serial_lines = 0
        self.wheel_packets = 0
        self.bad_serial_lines = 0
        self.serial_error = ""
        self.fg_rejected = [0, 0]
        self.fg_noise_at = -1e9
        self.threads = []

    def spawn(self, target):
        thread = threading.Thread(target=target, daemon=True, name=target.__name__)
        self.threads.append(thread)
        thread.start()

    def write(self, data):
        # Called only under self.mu. A web stop cannot interleave with a later
        # stale movement packet, and the serial timeout bounds lock contention.
        if self.ser is not None:
            self.ser.write(data.encode("ascii"))

    def write_output(self, output):
        # Keep the controller in physical forward coordinates. Apply wiring
        # polarity only at the serial boundary, for manual AND automatic moves.
        left, right, brush = output
        left *= self.c["left_motor_sign"]
        right *= self.c["right_motor_sign"]
        self.write("M,%d,%d\nR,%d\n" % (left, right, int(brush)))

    def stop(self, reason="Stopped by operator"):
        with self.mu:
            self.follow = None
            self.navigation = None
            if self.cleaning_status is not None:
                self.cleaning_status["active"] = False
            self.engine.stop(reason)
            self.armed = self.awaiting_arm = False
            try:
                self.write("D\nR,0\n")
            except Exception as exc:
                self.error = "Stop serial write failed: " + str(exc)
                self.engine.serial_ok = False

    def fail(self, reason):
        LOG.error(reason)
        self.error = reason
        self.stop(reason)

    def connect_serial(self):
        if self.dry_run:
            return
        import serial
        import serial.tools.list_ports

        port = self.c["serial_port"]
        if port == "auto":
            ports = list(serial.tools.list_ports.comports())
            matches = [
                p.device for p in ports if p.vid in (0x2341, 0x2A03, 0x1A86, 0x0403)
            ]
            if len(matches) != 1:
                raise RuntimeError(
                    "Cannot choose Arduino uniquely. Use --serial /dev/serial/by-id/...; candidates: "
                    + str(matches)
                )
            port = matches[0]
        self.ser = serial.Serial(
            port, self.c["baud"], timeout=0.05, write_timeout=0.1, exclusive=True
        )
        time.sleep(2.5)  # Nano resets on serial open; no other workers started yet.
        self.ser.reset_input_buffer()
        self.write("D\nR,0\nV\nQ\n")
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            line = self.ser.readline().decode("ascii", errors="replace").strip()
            if line == "K,DWM2":
                self.firmware_ok = True
                break
            if line.startswith("T,MOTOR,"):
                from dwm_core.motor_protocol import telemetry

                telemetry(line.split(","))
                self.firmware_protocol = "motor_safety_v2"
                self.serial_line(line.encode("ascii"))
                self.firmware_ok = True
                break
        if not self.firmware_ok:
            self.write("D\nR,0\n")
            self.ser.close()
            self.ser = None
            raise RuntimeError(
                "Supported Arduino firmware not detected (DWM2 or MOTOR SAFETY v2). Check firmware and serial port."
            )
        self.engine.serial_ok = True
        self.engine.link_at = time.monotonic()
        LOG.info("Arduino ready at %s (%s)", port, self.firmware_protocol)

    def serial_reader(self):
        # Drain ALL telemetry, not just one line per control iteration.
        pending = b""
        self.serial_reader_alive = True
        try:
            while not self.done.is_set():
                chunk = self.ser.read(max(1, min(self.ser.in_waiting, 4096)))
                pending += chunk
                if len(pending) > 16384:
                    raise RuntimeError("Serial telemetry overflow")
                while b"\n" in pending:
                    line, pending = pending.split(b"\n", 1)
                    self.serial_line(line)
        except Exception as exc:
            with self.mu:
                self.engine.serial_ok = False
                self.serial_error = "Arduino USB read failed: " + str(exc)
                self.fail(self.serial_error)
        finally:
            self.serial_reader_alive = False

    def serial_line(self, line):
        fields = line.decode("ascii", errors="replace").strip().split(",")
        now = time.monotonic()
        with self.mu:
            self.serial_line_at = now
            self.serial_lines += 1
            try:
                if fields[:1] == ["N"]:
                    if len(fields) != 3:
                        raise ValueError("Expected N,left_motor_rpm,right_motor_rpm")
                    values = [float(v) for v in fields[1:]]
                    if not all(
                        math.isfinite(v)
                        and abs(v) <= self.c["max_wheel_rpm"] * self.c["gear_ratio"]
                        for v in values
                    ):
                        raise ValueError("Motor RPM outside configured limit")
                    self.motor_rpm = [
                        values[0] * self.c["left_motor_sign"],
                        values[1] * self.c["right_motor_sign"],
                    ]
                    self.motor_rpm_at = self.engine.link_at = now
                    return
                if fields[:2] == ["T", "MOTOR"]:
                    from dwm_core.motor_protocol import telemetry

                    status = telemetry(fields)
                    self.motor_status, self.motor_status_at = status, now
                    self.motor_fault = status["fault"]
                    self.engine.link_at = now
                    if self.motor_fault["code"]:
                        self.stop(
                            "Arduino motor fault: "
                            + self.motor_fault["name"]
                            + " (wheel "
                            + str(self.motor_fault["wheel"])
                            + ")"
                        )
                    elif status["timed_out"] and self.armed:
                        self.stop("Arduino motor-command watchdog expired")
                    return
                if fields[:2] == ["F", "MOTOR"]:
                    from dwm_core.motor_protocol import fault

                    if len(fields) != 4:
                        raise ValueError("Expected F,MOTOR,code,wheel")
                    self.motor_fault = fault(fields[2], fields[3])
                    if self.motor_fault["code"]:
                        self.stop(
                            "Arduino motor fault: "
                            + self.motor_fault["name"]
                            + " (wheel "
                            + str(self.motor_fault["wheel"])
                            + ")"
                        )
                    self.engine.link_at = now
                    return
                if fields == ["ACK", "MOTOR_CLEAR"]:
                    self.motor_fault = {"code": 0, "name": "NONE", "wheel": 0}
                    self.fault_clear_pending = False
                    self.stop("Motor fault cleared; choose a mode to start")
                    return
                if self.firmware_protocol == "motor_safety_v2" and fields == ["READY"]:
                    self.stop("Arduino restarted; choose a mode to start")
                    return
                if not self.c["use_wheel_rpm"] and fields[:1] in (["W"], ["F"]):
                    # Temporary manual-only mode: wheel feedback is ignored,
                    # but incoming Arduino packets still prove USB is alive.
                    self.engine.link_at = now
                    self.engine.serial_ok = True
                    self.serial_error = ""
                    if fields[:1] == ["W"]:
                        self.wheel_packets += 1
                    return
                if fields[:1] == ["W"]:
                    if len(fields) != 3:
                        raise ValueError("Expected W,left_rpm,right_rpm")
                    left, right = float(fields[1]), float(fields[2])
                    if not all(
                        math.isfinite(v) and abs(v) <= self.c["max_wheel_rpm"]
                        for v in (left, right)
                    ):
                        raise ValueError("Wheel RPM outside configured limit")
                    # Firmware signs RPM using commanded electrical direction.
                    # Convert it back to physical forward before odometry.
                    self.engine.wheels(
                        left * self.c["left_motor_sign"],
                        right * self.c["right_motor_sign"],
                        now,
                    )
                    self.wheel_packets += 1
                    self.serial_error = ""
                elif fields[:1] == ["F"]:
                    if len(fields) != 3 or not all(v.isdigit() for v in fields[1:]):
                        raise ValueError("Expected F,left_rejected,right_rejected")
                    counts = [int(v) for v in fields[1:]]
                    if any(v > 4294967295 for v in counts):
                        raise ValueError("FG counter outside uint32 range")
                    if any(new > old for new, old in zip(counts, self.fg_rejected)):
                        self.fg_noise_at = now
                        if self.engine.mode == "AUTO" and not self.engine.latched:
                            self.stop(
                                "Wheel feedback noise detected; inspect FG wiring before auto clean"
                            )
                    self.fg_rejected = counts
                elif fields[:1] == ["I"]:
                    # IMU is an optional display sensor, independent of wheel safety.
                    try:
                        if len(fields) != 11:
                            raise ValueError("Expected I,qw,qx,qy,qz,gx,gy,gz,ax,ay,az")
                        values = [float(v) for v in fields[1:]]
                        if not all(math.isfinite(v) for v in values):
                            raise ValueError("Non-finite IMU reading")
                        norm = math.sqrt(sum(v * v for v in values[:4]))
                        if not 0.5 <= norm <= 1.5:
                            raise ValueError("Invalid orientation quaternion")
                        w, x, y, z = [v / norm for v in values[:4]]
                        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
                        pitch = math.asin(max(-1, min(1, 2 * (w * y - z * x))))
                        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
                        self.imu = {
                            "quaternion": values[:4],
                            "gyro_rad_s": values[4:7],
                            "acceleration_m_s2": values[7:10],
                            "roll_deg": round(math.degrees(roll), 1),
                            "pitch_deg": round(math.degrees(pitch), 1),
                            "yaw_deg": round(math.degrees(yaw), 1),
                        }
                        self.imu_at = now
                        self.imu_error = ""
                    except ValueError as exc:
                        self.imu_error = str(exc)
                elif fields[:1] == ["B"]:
                    if len(fields) != 3:
                        raise ValueError("Expected B,voltage,percent")
                    vals = [float(v) for v in fields[1:]]
                    if not all(math.isfinite(v) for v in vals):
                        raise ValueError("Invalid battery telemetry")
                    self.battery = {"voltage": vals[0], "percent": vals[1]}
                elif fields == ["A", "ENABLED"] and self.awaiting_arm:
                    self.armed, self.awaiting_arm = True, False
                elif fields[:1] in (["ERR"], ["FAILSAFE"]):
                    self.fault_clear_pending = False
                    self.stop("Arduino: " + ",".join(fields))
                elif fields == ["K", "DWM2"]:
                    self.stop("Arduino reset; re-enable explicitly")
                if fields[:1] in (["W"], ["F"], ["B"], ["A"]):
                    self.engine.link_at = now
            except ValueError as exc:
                self.bad_serial_lines += 1
                self.serial_error = "Malformed Arduino telemetry: " + str(exc)
                self.engine.serial_ok = False
                self.engine.odom_at = None
                LOG.warning("%s (%r)", self.serial_error, fields)
                self.stop(self.serial_error)

    def control_loop(self):
        try:
            while not self.done.wait(0.05):
                now = time.monotonic()
                with self.mu:
                    if self.demo:
                        left, right, _ = self.engine.output
                        # Demo dynamics only; never used for real travel estimation.
                        self.engine.wheels(left * 0.45, right * 0.45, now)
                    was_latched = self.engine.latched
                    if self.follow is not None:
                        self.follow_tick(now)
                    if self.navigation is not None:
                        self.navigation_tick(now)
                    output = self.engine.tick(now)
                    if self.awaiting_arm and now > self.arm_deadline:
                        self.stop("Arduino did not acknowledge Enable")
                    elif self.engine.latched and (
                        not was_latched or self.armed or self.awaiting_arm
                    ):
                        self.stop(self.engine.reason)
                    elif not self.dry_run and self.armed and not self.engine.latched:
                        self.write_output(output)
        except Exception as exc:
            self.fail("Control worker failed: " + str(exc))

    def navigation_tick(self, now):
        n, e = self.navigation, self.engine
        if e.latched:
            self.navigation = None
            return
        e.manual_brush = False
        if now - n["at"] > 0.35 or not n["clear"]:
            e.manual_pair = (0, 0)
            n["control"].output = [0.0, 0.0]
            n["control"].integral = [0.0, 0.0]
            e.phase, e.reason = (
                "NAV_WAIT",
                "Waiting for fresh navigation command and lidar clearance",
            )
        else:
            e.manual_pair = n["control"].step(n["v"], n["w"], (e.left_rpm, e.right_rpm))
            e.phase, e.reason = "WAYPOINT", "Following Nav2 waypoint route"
        e.manual_at = now

    def follow_tick(self, now):
        """Follow OAK depth to a standoff; hardware/browser watchdogs still apply."""
        f, e = self.follow, self.engine
        if e.latched:
            self.follow = None
            return
        error = e.healthy(now)
        if error:
            self.stop(error)
            return
        e.manual_brush = False
        e.manual_pair = (0, 0)
        if f.get("prepass_pause"):
            e.phase, e.reason = (
                "BEFORE_PASS_PAUSE",
                "Close to poop; drive stopped for 1 second before brush and forward pass",
            )
            if max(abs(e.left_rpm), abs(e.right_rpm)) <= 1:
                if f.get("prepass_still_at") is None:
                    f["prepass_still_at"] = now
            else:
                f["prepass_still_at"] = None
            if now - f["prepass_started"] > 5:
                self.stop("Drive wheels did not settle before forward pass")
                return
            if f.get("prepass_still_at") is None or now - f["prepass_still_at"] < 1:
                return
            from dwm_core.forward_pass import ForwardPass

            f["prepass_pause"] = False
            f["pass"] = ForwardPass(e.left_m, e.right_m, now, f["pass_distance"])
            self.forward_pass_status = f["pass"].status()
        if f.get("cleaning_stage") == "settling":
            e.phase, e.reason = (
                "WHEELS_SETTLING",
                "Drive wheels stopped; waiting 2 seconds before cleaning",
            )
            if max(abs(e.left_rpm), abs(e.right_rpm)) <= 1:
                if f.get("settled_at") is None:
                    f["settled_at"] = now
                if now - f["settled_at"] >= 2:
                    f["cleaning_stage"] = "running"
                    f["cleaning_at"] = now
            else:
                f["settled_at"] = None
            if now - f["settling_started"] > 5:
                self.stop("Drive wheels did not settle; cleaning not started")
            if f["cleaning_stage"] != "running":
                return
        if f.get("cleaning_stage") == "running":
            elapsed = now - f["cleaning_at"]
            self.cleaning_status = {
                "duration_s": 20,
                "remaining_s": round(max(0, 20 - elapsed), 1),
                "active": elapsed < 20,
            }
            if elapsed >= 20:
                self.stop("Forward pass and 20-second cleaning completed")
            elif max(abs(e.left_rpm), abs(e.right_rpm)) > 1:
                self.stop("Drive wheel movement detected during stationary cleaning")
                self.cleaning_status["active"] = False
            else:
                e.manual_brush = True
                e.phase, e.reason = (
                    "CLEANING_STATIONARY",
                    "Drive wheels off; cleaning motor on for 20 seconds",
                )
            return
        if f.get("pass") is not None:
            pair, reason = f["pass"].step(e.left_m, e.right_m, now)
            self.forward_pass_status = f["pass"].status()
            if reason == "Forward pass completed; brush off" and f["clean_after_pass"]:
                f["cleaning_stage"] = "settling"
                f["settling_started"] = now
                f["settled_at"] = None
                self.cleaning_status = {
                    "duration_s": 20,
                    "remaining_s": 20,
                    "active": False,
                }
                e.phase, e.reason = (
                    "WHEELS_SETTLING",
                    "Drive wheels stopped; waiting to start 20-second cleaning",
                )
            elif reason:
                self.stop(reason)
            else:
                e.manual_pair, e.manual_at = pair, now
                e.manual_brush = True
                e.phase, e.reason = (
                    "FORWARD_PASS",
                    "Brush on; going over poop approximately 1 metre forward",
                )
            return
        lock = e.tracker
        if (
            f["moving"] is None
            and lock.target is None
            and f.get("missing_at") is None
            and now - f["started"] > 8
        ):
            self.stop("No confirmed target; robot stayed still")
            return
        if lock.target is None:
            f["smoother"].reset()
            e.phase, e.reason = (
                "DETECTING",
                "Confirming visible poop with fresh depth; wheels stopped",
            )
            return
        d = lock.target
        depth = self.detection_depths.get(d.id)
        if (
            not lock.visible
            or d.confidence < self.c["lock_conf"]
            or now - lock.seen_at > 0.3
            or now - self.depth_at > 0.3
            or not depth
        ):
            f["smoother"].reset()
            if f.get("missing_at") is None:
                f["missing_at"] = now
            if now - f["missing_at"] >= 1:
                # Detector IDs can change after occlusion or switching model/fallback.
                # Release the stale ID, then require normal multi-frame confirmation.
                lock.reset()
                f.pop("previous_depth", None)
            e.phase, e.reason = (
                "REACQUIRING",
                "Paused; reconfirming visible poop and fresh depth",
            )
            return
        if f.get("missing_at") is not None and f["moving"] is not None:
            f["moving"] += now - f["missing_at"]
        f["missing_at"] = None
        if (
            f["moving"] is not None
            and now - f["moving"] > self.c["max_approach_time_s"]
        ):
            self.stop("Approach fault timeout; check target and drive")
            return
        z = depth["z_m"]
        if not math.isfinite(z) or not 0.2 <= z <= 3:
            self.stop("Target depth outside supported range")
            return
        at_edge = d.box[3] >= self.c["height"] * 0.95
        offset = ((d.box[0] + d.box[2]) / 2 - self.c["width"] / 2) / self.c["width"]
        pass_entry = f["pass_distance"] > 0 and (
            z <= f["distance"] or (at_edge and z <= min(0.65, f["distance"] + 0.08))
        )
        if pass_entry and abs(offset) <= 0.12:
            f["prepass_pause"] = True
            f["prepass_started"] = now
            f["prepass_still_at"] = (
                now if max(abs(e.left_rpm), abs(e.right_rpm)) <= 1 else None
            )
            e.phase, e.reason = (
                "BEFORE_PASS_PAUSE",
                "Close to poop; drive stopped for 1 second before brush and forward pass",
            )
            return
        if at_edge:
            self.stop("Target near camera view edge; inspect before continuing")
            return
        if z <= f["distance"] and f["pass_distance"] == 0:
            self.stop("Selected camera distance reached; brush off")
            return
        if f["moving"] is None:
            f["moving"], f["previous_depth"] = now, z
        if abs(z - f.get("previous_depth", z)) > 0.4:
            self.stop("Target depth changed abruptly; approach stopped")
            return
        f["previous_depth"] = z
        e.manual_pair = f["smoother"].step(offset, z, f["distance"], now)
        e.manual_at = now
        e.phase, e.reason = "APPROACHING", "Smooth camera approach; brush off"

    def track_vision(self, detections, depths, at, now):
        eligible = detections
        if self.follow is not None:
            eligible = [
                d
                for d in detections
                if depths.get(d.id)
                and math.isfinite(depths[d.id]["z_m"])
                and 0.2 <= depths[d.id]["z_m"] <= 3
                and (
                    d.box[3] < self.c["height"] * 0.95
                    or (
                        self.follow["pass_distance"] > 0
                        and depths[d.id]["z_m"]
                        <= min(0.65, self.follow["distance"] + 0.08)
                    )
                )
            ]
            target = self.engine.tracker.target
            if target is not None and not any(d.id == target.id for d in eligible):
                from dwm_core.vision_approach import overlap

                old_depth = self.detection_depths.get(target.id)
                # Keep identity through a model/fallback ID change only when a
                # single box overlaps strongly and measured depth agrees.
                matches = [
                    d
                    for d in eligible
                    if overlap(target.box, d.box) > 0.4
                    and old_depth
                    and abs(depths[d.id]["z_m"] - old_depth["z_m"]) < 0.15
                    and now - self.engine.tracker.seen_at < 0.3
                ]
                if len(matches) == 1:
                    d = matches[0]
                    mapped = Detection(
                        target.id, d.confidence, d.box, d.ground, d.source
                    )
                    eligible = [mapped if v.id == d.id else v for v in eligible]
                    depths[target.id] = depths[d.id]
        self.engine.vision(eligible, at, now)

    def capture_loop(self):
        if self.c["camera"] == "oak":
            self.oak_capture_loop()
            return
        import cv2

        try:
            camera = self.c["camera"]
            if isinstance(camera, str) and camera.isdigit():
                camera = int(camera)
            self.cap = cv2.VideoCapture(camera, cv2.CAP_V4L2)
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.c["width"])
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.c["height"])
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self.cap.set(cv2.CAP_PROP_FPS, 30)
            if not self.cap.isOpened():
                raise RuntimeError("Cannot open webcam " + str(camera))
            previous = time.monotonic()
            while not self.done.is_set():
                ok, frame = self.cap.read()
                if not ok:
                    raise RuntimeError("Webcam read failed; restart after checking USB")
                at = time.monotonic()
                instant = 1 / max(1e-6, at - previous)
                previous = at
                self.camera_fps = (
                    instant
                    if not self.camera_fps
                    else 0.9 * self.camera_fps + 0.1 * instant
                )
                if frame.shape[1::-1] != (self.c["width"], self.c["height"]):
                    raise RuntimeError(
                        "Webcam resolution differs from config. Set supported width/height and recalibrate."
                    )
                with self.mu:
                    self.engine.capture_at = at
                with self.frame_cv:
                    self.frame, self.frame_at = frame, at
                    self.frame_seq += 1
                    self.frame_cv.notify_all()
        except Exception as exc:
            with self.mu:
                self.engine.vision_ok = False
            self.fail("Camera failed: " + str(exc))

    def oak_capture_loop(self):
        import cv2
        import numpy as np
        from dwm_perception.oak_source import OakSource

        source = None
        try:
            source = OakSource()
            self.camera_info = dict(source.info, source="OAK-D Pro")
            previous = time.monotonic()
            while not self.done.is_set():
                frame, depth, at, skew = source.read()
                if frame.shape[1::-1] != (self.c["width"], self.c["height"]):
                    raise RuntimeError("OAK RGB resolution differs from configuration")
                now = time.monotonic()
                instant = 1 / max(1e-6, now - previous)
                previous = now
                self.camera_fps = (
                    instant
                    if not self.camera_fps
                    else 0.9 * self.camera_fps + 0.1 * instant
                )
                shade = (
                    255 - np.clip(depth.astype(np.float32) / 3000 * 255, 0, 255)
                ).astype(np.uint8)
                visual = cv2.applyColorMap(shade, cv2.COLORMAP_TURBO)
                visual[depth == 0] = 0
                ok, encoded = cv2.imencode(
                    ".jpg", visual, [cv2.IMWRITE_JPEG_QUALITY, 50]
                )
                with self.mu:
                    self.engine.capture_at = at
                    self.depth_at, self.depth_skew_s = at, skew
                    if ok:
                        self.depth_jpeg = encoded.tobytes()
                with self.frame_cv:
                    self.frame, self.depth_frame, self.frame_at = frame, depth, at
                    self.frame_seq += 1
                    self.frame_cv.notify_all()
        except Exception as exc:
            with self.mu:
                self.engine.vision_ok = False
            self.fail("OAK camera failed: " + str(exc))
        finally:
            if source is not None:
                source.close()

    def publish_jpeg(self, frame, captured_at=None):
        import cv2

        ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 50])
        if ok:
            with self.jpeg_cv:
                self.jpeg = encoded.tobytes()
                self.jpeg_seq += 1
                self.jpeg_at = (
                    captured_at if captured_at is not None else time.monotonic()
                )
                self.jpeg_cv.notify_all()

    def preview_loop(self):
        """Encode fresh camera frames independently of detector speed."""
        import cv2

        seq = -1
        try:
            while not self.done.is_set():
                with self.frame_cv:
                    self.frame_cv.wait_for(
                        lambda: self.done.is_set() or self.frame_seq != seq, timeout=0.5
                    )
                    if self.done.is_set():
                        break
                    if self.frame is None or seq == self.frame_seq:
                        continue
                    frame, at, seq = self.frame.copy(), self.frame_at, self.frame_seq
                now = time.monotonic()
                with self.mu:
                    state = self.engine.status(now)
                    depths = dict(self.detection_depths)
                    detections = (
                        list(self.preview_detections)
                        if now - self.engine.vision_at < self.c["vision_timeout_s"]
                        else []
                    )
                for detection in detections:
                    x1, y1, x2, y2 = map(int, detection.box)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (180, 180, 180), 1)
                    distance = depths.get(detection.id)
                    label = (
                        "possible poop (dark on grass)"
                        if detection.source == "dark_grass"
                        else "dog_poop %.2f" % detection.confidence
                    )
                    if distance:
                        label += " | %.2f m" % distance["z_m"]
                    cv2.putText(
                        frame,
                        label,
                        (max(0, x1), max(80, y1 - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.45,
                        (180, 180, 180),
                        1,
                    )
                self.draw_status(frame, state)
                self.publish_jpeg(frame, at)
        except Exception as exc:
            self.fail("Camera preview failed: " + str(exc))

    def inference_loop(self):
        import cv2

        try:
            import torch
            from ultralytics import YOLO
            from dwm_perception.dark_grass import DarkGrassFallback

            fallback = DarkGrassFallback()
            model = YOLO(str(model_path(self.c["model"])))
            if model.names.get(0) != "dog_poop":
                raise RuntimeError(
                    "Expected class 0 = dog_poop; verify model class map"
                )
            device = self.c["device"]
            if device == "auto":
                device = "0" if torch.cuda.is_available() else "cpu"
            if device == "cpu":
                LOG.warning(
                    "CPU inference: check FPS and vision timeout before hardware use"
                )
            seq = -1
            previous = time.monotonic()
            while not self.done.is_set():
                with self.frame_cv:
                    self.frame_cv.wait_for(
                        lambda: self.done.is_set() or self.frame_seq != seq, timeout=0.5
                    )
                    if self.frame is None or seq == self.frame_seq:
                        continue
                    frame, at, seq = self.frame.copy(), self.frame_at, self.frame_seq
                    depth = (
                        None if self.depth_frame is None else self.depth_frame.copy()
                    )
                # No queued frames: always consume the latest capture.
                result = model.track(
                    frame,
                    persist=True,
                    tracker=str(
                        package_share("dwm_perception") / "config/botsort.yaml"
                    ),
                    conf=self.c["detection_conf"],
                    imgsz=self.c["imgsz"],
                    classes=[0],
                    device=device,
                    verbose=False,
                )[0]
                detections = []
                if result.boxes is not None and result.boxes.id is not None:
                    boxes = result.boxes.xyxy.cpu().tolist()
                    ids = result.boxes.id.int().cpu().tolist()
                    confs = result.boxes.conf.cpu().tolist()
                    with self.mu:
                        for box, ident, conf in zip(boxes, ids, confs):
                            detections.append(
                                Detection(
                                    ident, conf, tuple(box), ground_point(box, self.c)
                                )
                            )
                now = time.monotonic()
                detections += fallback.detect(
                    frame,
                    at,
                    model_present=bool(detections)
                    or bool(result.boxes is not None and len(result.boxes)),
                )
                depths = {}
                if depth is not None:
                    from dwm_perception.oak_source import box_depth

                    depths = {d.id: box_depth(depth, d.box) for d in detections}
                with self.mu:
                    self.track_vision(detections, depths, at, now)
                    self.preview_detections = detections
                    self.detection_depths = depths
                    state = self.engine.status(now)
                dt = max(1e-6, now - previous)
                previous = now
                instant = 1 / dt
                self.fps = instant if not self.fps else 0.9 * self.fps + 0.1 * instant
        except Exception as exc:
            with self.mu:
                self.engine.vision_ok = False
            self.fail("Model/tracker failed: " + str(exc))

    def draw_status(self, frame, state):
        import cv2

        target = state["target"]
        if target:
            x1, y1, x2, y2 = [int(v) for v in target["box"]]
            visible = target["visible"] and target["age_s"] < self.c["vision_timeout_s"]
            color = (80, 255, 80) if visible else (0, 190, 255)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 3)
            label = "LOCK #%s" % target["id"] if visible else "LAST SEEN (not current)"
            cv2.putText(
                frame,
                label,
                (max(0, x1), max(24, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                color,
                2,
            )
        cv2.rectangle(frame, (0, 0), (frame.shape[1], 66), (25, 30, 35), -1)
        cv2.putText(
            frame,
            "%s | %.1f FPS | %s"
            % (state["phase"], self.fps, "DRY RUN" if self.dry_run else "HARDWARE"),
            (12, 26),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            1,
        )
        cv2.putText(
            frame,
            "STOP from dashboard or physical emergency switch",
            (12, 51),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (100, 190, 255),
            1,
        )

    def demo_loop(self):
        import cv2
        import numpy as np

        try:
            start_travel = 0.0
            while not self.done.wait(0.08):
                now = time.monotonic()
                frame = np.zeros((480, 640, 3), dtype=np.uint8)
                frame[:] = (53, 88, 56)
                for yy in range(100, 480, 40):
                    cv2.line(frame, (0, yy), (640, yy), (65, 102, 65), 1)
                with self.mu:
                    self.engine.capture_at = now
                    progress = (
                        self.engine.left_m + self.engine.right_m
                    ) / 2 - start_travel
                    yy = min(470, int(235 + max(0, progress) / 0.003))
                    box = (295, yy - 38, 345, yy)
                    cv2.ellipse(
                        frame, (320, yy - 15), (24, 14), 0, 0, 360, (35, 43, 64), -1
                    )
                    d = Detection(1, 0.91, box, ground_point(box, self.c))
                    self.engine.vision([d], now, now)
                    state = self.engine.status(now)
                self.fps = 12.5
                self.draw_status(frame, state)
                cv2.putText(
                    frame,
                    "SIMULATED ROBOT / NO HARDWARE OUTPUT",
                    (60, 100),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (150, 220, 255),
                    1,
                )
                self.publish_jpeg(frame)
        except Exception as exc:
            self.fail("Demo failed: " + str(exc))

    def claim(self, operator=""):
        now = time.monotonic()
        with self.mu:
            same_owner = bool(operator) and hmac.compare_digest(
                operator, self.operator or ""
            )
            if (
                self.operator
                and not same_owner
                and now - self.operator_seen < self.c["browser_timeout_s"]
            ):
                raise ValueError(
                    "Another browser owns controls. Stop and close it, then wait 2 seconds."
                )
            self.stop("New browser control session; explicitly enable")
            self.operator = secrets.token_urlsafe(18)
            self.operator_seen = self.engine.browser_at = now
            self.drive_seq = -1
            return {"operator": self.operator}

    def command(self, action, data, operator):
        now = time.monotonic()
        with self.mu:
            if action == "stop":
                reasons = {
                    "button": "Stopped with dashboard Stop button",
                    "keyboard": "Stopped with Space key",
                    "visibility": "Dashboard hidden; robot and brush stopped",
                    "page_closed": "Dashboard closed or reloaded; robot and brush stopped",
                    "connection": "Dashboard heartbeat failed; robot and brush stopped",
                    "cancelled_action": "Pending dashboard action cancelled; robot and brush stopped",
                    "brush_change": "Wheels stopped for brush control",
                }
                cause = data.get("cause")
                reason = (
                    reasons.get(cause, "Stopped by operator")
                    if isinstance(cause, str)
                    else "Stopped by operator"
                )
                LOG.info("Stop request: %s", reason)
                self.stop(reason)
                return {"ok": True}
            if not operator or not hmac.compare_digest(operator, self.operator or ""):
                raise ValueError("Connect this browser to claim controls first")
            if action == "heartbeat":
                if now - self.operator_seen > self.c["browser_timeout_s"]:
                    self.stop("Operator session expired; reconnect browser")
                    self.operator = None
                    raise ValueError("Operator session expired; reconnect")
                self.operator_seen = self.engine.browser_at = now
                return {"ok": True}
            if now - self.operator_seen > self.c["browser_timeout_s"]:
                self.stop("Operator heartbeat expired")
                raise ValueError("Operator heartbeat expired; reconnect")
            if action == "nav_begin":
                from dwm_core.navigation_core import WheelSpeedControl

                geometry = json.loads((ROOT / "navigation_geometry.json").read_text())
                if not geometry.get("lidar_alignment_confirmed"):
                    raise ValueError(
                        "Confirm lidar forward alignment on the waypoint page"
                    )
                if not self.c["use_wheel_rpm"] or self.c["wheel_diameter_m"] <= 0:
                    raise ValueError(
                        "Navigation requires wheel feedback and measured sprocket diameter"
                    )
                self.command("manual", {}, operator)
                key = secrets.token_urlsafe(18)
                self.navigation = dict(
                    key=key,
                    at=-1e9,
                    v=0,
                    w=0,
                    clear=False,
                    seq=-1,
                    margin=geometry.get("collision_margin_m", 0.06),
                    control=WheelSpeedControl(
                        geometry["track_centre_spacing_m"], self.c["wheel_diameter_m"]
                    ),
                )
                return {"key": key}
            if action == "nav_end":
                if self.navigation and hmac.compare_digest(
                    str(data.get("key", "")), self.navigation["key"]
                ):
                    self.stop("Waypoint navigation ended")
                return {"ok": True}
            if action == "nav_velocity":
                n = self.navigation
                if not n or not hmac.compare_digest(str(data.get("key", "")), n["key"]):
                    raise ValueError("Navigation session ended")
                seq = data.get("seq")
                if not isinstance(seq, int) or isinstance(seq, bool) or seq < 0:
                    raise ValueError("Navigation velocity requires a sequence number")
                if seq <= n["seq"]:
                    return {"ok": True, "ignored": "Older navigation packet"}
                v, w = data.get("linear"), data.get("angular")
                if any(
                    isinstance(x, bool)
                    or not isinstance(x, (int, float))
                    or not math.isfinite(x)
                    for x in (v, w)
                ):
                    raise ValueError("Invalid navigation velocity")
                if abs(v) > 0.16 or abs(w) > 0.45:
                    raise ValueError("Navigation velocity outside configured bounds")
                points = data.get("points", [])
                if not isinstance(points, list) or len(points) > 3600:
                    raise ValueError("Invalid lidar points")
                if any(
                    not isinstance(p, list)
                    or len(p) != 2
                    or any(
                        isinstance(x, bool)
                        or not isinstance(x, (int, float))
                        or not math.isfinite(x)
                        for x in p
                    )
                    for p in points
                ):
                    raise ValueError("Invalid lidar coordinates")
                from dwm_core.navigation_core import clear_motion

                n.update(
                    at=now,
                    v=v,
                    w=w,
                    seq=seq,
                    clear=data.get("scan_fresh") is True
                    and clear_motion(points, v, w, padding=n["margin"]),
                )
                return {"ok": True, "clear": n["clear"]}
            if action in ("manual", "auto", "detect"):
                if self.firmware_protocol == "motor_safety_v2":
                    if self.motor_fault["code"]:
                        raise ValueError(
                            "Arduino motor fault "
                            + self.motor_fault["name"]
                            + "; stop and clear motor fault first"
                        )
                    if now - self.motor_status_at > 0.65:
                        raise ValueError(
                            "Arduino motor status stale; check USB connection"
                        )
                if action == "detect":
                    distance = data.get("distance_cm", 45)
                    pass_distance = data.get("forward_pass_m", 1.0)
                    clean_after = data.get("clean_after_pass", True)
                    if not isinstance(clean_after, bool):
                        raise ValueError("clean_after_pass must be true or false")
                    if (
                        isinstance(pass_distance, bool)
                        or not isinstance(pass_distance, (int, float))
                        or not math.isfinite(pass_distance)
                        or not 0 <= pass_distance <= 1
                    ):
                        raise ValueError("Forward pass must be 0..1 metres")
                    if pass_distance > 0 and (
                        not self.c["use_wheel_rpm"] or self.c["wheel_diameter_m"] <= 0
                    ):
                        raise ValueError(
                            "Forward pass requires wheel RPM and measured sprocket diameter"
                        )
                    if (
                        isinstance(distance, bool)
                        or not isinstance(distance, (int, float))
                        or not math.isfinite(distance)
                        or not 45 <= distance <= 300
                    ):
                        raise ValueError(
                            "Supported stop distance is 45–300 cm from the camera. Smaller or negative front gaps need calibration."
                        )
                    if self.c["camera"] != "oak":
                        raise ValueError("Auto detect approach requires OAK depth")
                self.stop("Arming requested")
                if action == "auto" and now - self.fg_noise_at <= 2.0:
                    raise ValueError(
                        "Wheel feedback noise detected; inspect FG wiring before auto clean"
                    )
                self.engine.start("AUTO" if action == "auto" else "MANUAL", now)
                if action == "detect":
                    from dwm_core.vision_approach import SmoothApproach

                    self.follow = {
                        "distance": distance / 100,
                        "started": now,
                        "moving": None,
                        "smoother": SmoothApproach(),
                        "pass_distance": pass_distance,
                        "pass": None,
                        "clean_after_pass": clean_after,
                    }
                    self.forward_pass_status = None
                    self.cleaning_status = None
                    self.engine.phase, self.engine.reason = (
                        "DETECTING",
                        "Looking for confirmed poop; brush off",
                    )
                if not self.dry_run:
                    try:
                        if self.firmware_protocol == "motor_safety_v2":
                            self.write("M,0,0\nR,0\nQ\n")
                            self.armed = True
                            self.awaiting_arm = False
                        else:
                            self.awaiting_arm = True
                            self.arm_deadline = now + 0.8
                            self.write("E\n")
                    except Exception:
                        self.stop("Arduino Enable write failed")
                        raise
            elif action == "clear_fault":
                if self.firmware_protocol != "motor_safety_v2":
                    raise ValueError("Fault clear requires MOTOR SAFETY v2 firmware")
                if not self.engine.latched:
                    raise ValueError("Stop robot before clearing motor fault")
                if (
                    now - self.engine.telemetry_at > self.c["telemetry_timeout_s"]
                    or max(abs(self.engine.left_rpm), abs(self.engine.right_rpm)) > 1
                ):
                    raise ValueError(
                        "Wait for stopped wheels and fresh RPM before clearing fault"
                    )
                self.write("M,0,0\nR,0\nC\nQ\n")
                self.fault_clear_pending = True
            elif action == "drive":
                if self.navigation is not None:
                    raise ValueError("Stop waypoint navigation before manual driving")
                if self.follow is not None:
                    raise ValueError("Stop Auto detect before manual driving")
                seq = data.get("seq")
                if not isinstance(seq, int) or isinstance(seq, bool) or seq < 0:
                    raise ValueError("drive requires a nonnegative integer seq")
                if seq <= self.drive_seq:
                    return {"ok": True, "ignored": "Older drive packet"}
                self.drive_seq = seq
                self.engine.manual(data.get("direction"), now)
            elif action == "brush":
                if self.follow is not None:
                    raise ValueError("Stop Auto detect before switching the brush")
                if self.engine.mode != "MANUAL" or self.engine.latched:
                    raise ValueError("Enable manual first")
                if not isinstance(data.get("on"), bool):
                    raise ValueError("on must be true or false")
                self.engine.manual_brush = data["on"]
            elif action == "settings":
                if not self.engine.latched:
                    raise ValueError("Stop before changing setup")
                allowed = (
                    "wheel_diameter_m",
                    "wheel_scale",
                    "axle_to_brush_m",
                    "brush_width_m",
                    "brush_margin_m",
                    "pass_trigger_m",
                    "geometry_verified",
                    "manual_pwm",
                    "approach_pwm",
                    "clean_pwm",
                    "turn_pwm",
                )
                if set(data) - set(allowed):
                    raise ValueError("Unsupported setup key")
                c = dict(self.c)
                c.update(data)
                validate_config(c)
                self.c.update(c)
                if not self.demo:
                    atomic_json(self.config_path, self.c)
            elif action == "calibrate":
                self.calibrate(data)
            else:
                raise ValueError("Unknown action")
            return {"ok": True}

    def calibrate(self, data):
        import cv2
        import numpy as np

        if not self.engine.latched:
            raise ValueError("Stop before calibrating")
        points = data.get("points", [])
        width, near, far = [float(data[k]) for k in ("width_m", "near_m", "far_m")]
        if not all(math.isfinite(x) for x in (width, near, far)) or not (
            0.1 <= width <= 3 and 0 < near < far <= 5
        ):
            raise ValueError("Invalid measured rectangle")
        if near > self.c["pass_trigger_m"] or far <= self.c["pass_trigger_m"]:
            raise ValueError(
                "Calibration rectangle must include the pass trigger distance"
            )
        arr = np.asarray(points, dtype=np.float32)
        if arr.shape != (4, 2) or not np.isfinite(arr).all():
            raise ValueError("Click exactly four corners")
        if (
            (arr[:, 0] < 0).any()
            or (arr[:, 0] >= self.c["width"]).any()
            or (arr[:, 1] < 0).any()
            or (arr[:, 1] >= self.c["height"]).any()
        ):
            raise ValueError("Points outside camera image")
        if (
            not cv2.isContourConvex(arr.reshape(-1, 1, 2))
            or abs(cv2.contourArea(arr)) < 400
        ):
            raise ValueError(
                "Corners must form a non-crossing quadrilateral, in the stated order"
            )
        world = np.float32(
            [[-width / 2, far], [width / 2, far], [width / 2, near], [-width / 2, near]]
        )
        h = cv2.getPerspectiveTransform(arr, world)
        if not np.isfinite(h).all() or abs(np.linalg.det(h)) < 1e-12:
            raise ValueError("Degenerate ground calibration")
        # Direction/order sanity: far corners are normally higher in the image.
        if (
            float(arr[:2, 1].mean()) >= float(arr[2:, 1].mean())
            or arr[0, 0] >= arr[1, 0]
            or arr[3, 0] >= arr[2, 0]
        ):
            raise ValueError("Click far-left, far-right, near-right, near-left")
        self.c.update(
            {
                "homography": h.flatten().tolist(),
                "ground_polygon": arr.tolist(),
                "ground_range": [-width / 2, width / 2, near, far],
                "calibration_resolution": [self.c["width"], self.c["height"]],
                "geometry_verified": False,
            }
        )
        if not self.demo:
            atomic_json(self.config_path, self.c)

    def snapshot(self):
        with self.mu:
            now = time.monotonic()
            s = self.engine.status(now)
            s["instance_id"] = self.instance_id
            s["navigation_active"] = self.navigation is not None
            s["detections"] = (
                [
                    {
                        "id": d.id,
                        "confidence": round(d.confidence, 3),
                        "box": list(d.box),
                        "source": d.source,
                        "depth": self.detection_depths.get(d.id),
                    }
                    for d in self.preview_detections
                ]
                if now - self.engine.vision_at <= self.c["vision_timeout_s"]
                else []
            )
            feedback_noise = now - self.fg_noise_at <= 2.0
            s["auto_ready"] = s["auto_ready"] and not feedback_noise
            age = lambda at: (
                None if at is None or at < 0 else round(max(0, now - at), 3)
            )
            imu_age = age(self.imu_at)
            s["camera"] = dict(
                self.camera_info,
                depth_age_s=age(self.depth_at),
                depth_skew_s=self.depth_skew_s,
            )
            s["detection_threshold"] = self.c["detection_conf"]
            s["motor_driver"] = {
                "protocol": self.firmware_protocol,
                "status": self.motor_status,
                "age_s": age(self.motor_status_at),
                "fault": self.motor_fault,
                "clear_pending": self.fault_clear_pending,
            }
            s["forward_pass"] = self.forward_pass_status
            s["cleaning"] = self.cleaning_status
            motor_fresh = now - self.motor_rpm_at <= self.c["telemetry_timeout_s"]
            s["drive_motor_rpm"] = {
                "left": self.motor_rpm[0] if motor_fresh else None,
                "right": self.motor_rpm[1] if motor_fresh else None,
                "age_s": age(self.motor_rpm_at),
                "gear_ratio": self.c["gear_ratio"],
                "fg_pulses_per_motor_rev_assumed": 4,
                "calibration_verified": False,
            }
            s["imu"] = {
                "connected": imu_age is not None
                and imu_age <= 0.5
                and not self.imu_error,
                "age_s": imu_age,
                "error": self.imu_error,
                "data": self.imu,
            }
            s.update(
                {
                    "fps": round(self.fps, 1),
                    "camera_fps": round(self.camera_fps, 1),
                    "preview_age_ms": (
                        None
                        if self.jpeg_at < 0
                        else round(max(0, now - self.jpeg_at) * 1000)
                    ),
                    "error": self.error,
                    "battery": self.battery,
                    "firmware_ok": self.firmware_ok,
                    "armed": self.armed,
                    "operator_connected": bool(self.operator)
                    and now - self.operator_seen <= self.c["browser_timeout_s"],
                    "demo": self.demo,
                    "connection": {
                        "serial_reader_alive": self.serial_reader_alive,
                        "serial_error": self.serial_error,
                        "serial_lines": self.serial_lines,
                        "wheel_packets": self.wheel_packets,
                        "bad_serial_lines": self.bad_serial_lines,
                        "serial_age_s": age(self.serial_line_at),
                        "wheel_age_s": age(self.engine.telemetry_at),
                        "link_age_s": age(self.engine.link_at),
                        "browser_age_s": age(self.operator_seen),
                        "wheel_timeout_s": self.c["telemetry_timeout_s"],
                    },
                    "setup": {
                        k: self.c[k]
                        for k in (
                            "wheel_diameter_m",
                            "wheel_scale",
                            "axle_to_brush_m",
                            "brush_width_m",
                            "brush_margin_m",
                            "pass_trigger_m",
                            "geometry_verified",
                            "manual_pwm",
                            "approach_pwm",
                            "clean_pwm",
                            "turn_pwm",
                        )
                    },
                    "calibrated": bool(self.c["homography"]),
                    "image_size": [self.c["width"], self.c["height"]],
                }
            )
            s["connection"].update(
                {
                    "fg_rejected": list(self.fg_rejected),
                    "feedback_noise": feedback_noise,
                }
            )
            return s

    def start_workers(self):
        if self.c["homography"] and self.c["calibration_resolution"] != [
            self.c["width"],
            self.c["height"],
        ]:
            raise ValueError(
                "Resolution changed: clear homography or recalibrate before starting"
            )
        self.connect_serial()
        if not self.dry_run:
            self.spawn(self.serial_reader)
        self.spawn(self.control_loop)
        if self.demo:
            self.spawn(self.demo_loop)
        else:
            self.spawn(self.capture_loop)
            self.spawn(self.preview_loop)
            self.spawn(self.inference_loop)

    def close(self):
        self.stop("Server shutting down")
        self.done.set()
        with self.frame_cv:
            self.frame_cv.notify_all()
        with self.jpeg_cv:
            self.jpeg_cv.notify_all()
        for thread in self.threads:
            # Let the OAK device close before Python tears down native SDK threads.
            thread.join(
                timeout=(
                    8.0
                    if thread.name == "capture_loop" and self.c["camera"] == "oak"
                    else 1.0
                )
            )
        # Avoid blocking on cap.release() if driver is wedged in a read.
        if self.ser:
            self.ser.close()


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def navigation_proxy(self, path, data=None):
        # Fixed loopback service; no client-supplied host or arbitrary endpoints.
        suffix = path[len("/api/navigation/") :]
        endpoint = suffix.split("?", 1)[0]
        reads = ("state", "map", "route")
        writes = (
            "route",
            "alignment",
            "mapping",
            "navigation",
            "save_map",
            "initial",
            "confirm_pose",
            "run",
            "mapping_drive",
            "cancel",
            "stop_session",
        )
        if endpoint not in (writes if data is not None else reads):
            self.response(404, {"error": "Unknown navigation endpoint"})
            return
        request = Request(
            "http://127.0.0.1:8091/" + suffix,
            data=json.dumps(data).encode() if data is not None else None,
            headers={
                "Content-Type": "application/json",
                "X-Operator": self.headers.get("X-Operator", ""),
            },
        )
        try:
            with urlopen(request, timeout=4) as reply:
                self.response(200, json.loads(reply.read()))
        except HTTPError as error:
            self.response(error.code, json.loads(error.read()))
        except (URLError, TimeoutError, OSError):
            self.response(
                503,
                {
                    "error": "Navigation sensor service unavailable; camera controls remain available"
                },
            )

    def log_message(self, fmt, *args):
        # Log only the path, never request headers or query parameters.
        LOG.debug("HTTP %s %s", self.command, urlsplit(self.path).path)

    @property
    def rt(self):
        return self.server.runtime

    def response(self, status, payload, content_type="application/json"):
        if content_type == "application/json" and not isinstance(payload, bytes):
            payload = json.dumps(payload, allow_nan=False).encode()
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Referrer-Policy", "no-referrer")
        try:
            self.end_headers()
            self.wfile.write(payload)
        except (BrokenPipeError, ConnectionResetError):
            # A browser timeout is a transport disconnect, not a hardware fault.
            # Hardware and operator watchdogs still run independently.
            self.close_connection = True

    def do_GET(self):
        path = urlsplit(self.path).path
        if path.startswith("/api/navigation/"):
            self.navigation_proxy(self.path)
            return
        if path in (
            "/",
            "/camera",
            "/app.js",
            "/style.css",
            "/navigation",
            "/navigation.js",
        ):
            name = {
                "/": "index.html",
                "/camera": "index.html",
                "/app.js": "app.js",
                "/style.css": "style.css",
                "/navigation": "navigation.html",
                "/navigation.js": "navigation.js",
            }[path]
            kind = {
                "/": "text/html",
                "/camera": "text/html",
                "/app.js": "text/javascript",
                "/style.css": "text/css",
                "/navigation": "text/html",
                "/navigation.js": "text/javascript",
            }[path]
            self.response(
                200, (package_share("dwm_robot") / "web" / name).read_bytes(), kind
            )
            return
        if path == "/api/state":
            self.response(200, self.rt.snapshot())
        elif path == "/snapshot.jpg":
            with self.rt.jpeg_cv:
                jpeg = self.rt.jpeg
            self.response(
                200 if jpeg else 503,
                jpeg or b"No camera frame yet",
                "image/jpeg" if jpeg else "text/plain",
            )
        elif path == "/depth.jpg":
            with self.rt.mu:
                jpeg = self.rt.depth_jpeg
            self.response(
                200 if jpeg else 503,
                jpeg or b"Depth camera starting",
                "image/jpeg" if jpeg else "text/plain",
            )
        elif path == "/stream":
            self.send_response(200)
            self.send_header(
                "Content-Type", "multipart/x-mixed-replace; boundary=frame"
            )
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "close")
            self.end_headers()
            self.close_connection = True
            seq = -1
            try:
                self.connection.settimeout(2)
                while not self.rt.done.is_set():
                    with self.rt.jpeg_cv:
                        self.rt.jpeg_cv.wait_for(
                            lambda: self.rt.done.is_set() or self.rt.jpeg_seq != seq,
                            timeout=1,
                        )
                        jpeg, current = self.rt.jpeg, self.rt.jpeg_seq
                    if jpeg is None or seq == current:
                        continue
                    seq = current
                    self.wfile.write(
                        b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                        + str(len(jpeg)).encode()
                        + b"\r\n\r\n"
                        + jpeg
                        + b"\r\n"
                    )
                    self.wfile.flush()
            except (OSError, TimeoutError):
                pass
        else:
            self.response(404, {"error": "Not found"})

    def do_POST(self):
        try:
            origin = self.headers.get("Origin")
            if self.headers.get("Sec-Fetch-Site") == "cross-site" or (
                origin
                and (
                    urlsplit(origin).scheme != "http"
                    or urlsplit(origin).netloc != self.headers.get("Host")
                )
            ):
                self.close_connection = True
                self.response(403, {"error": "Cross-origin control denied"})
                return
            if self.headers.get_content_type() != "application/json":
                self.close_connection = True
                self.response(
                    415, {"error": "Use application/json for control requests"}
                )
                return
            length = int(self.headers.get("Content-Length", "0"))
            request_path = urlsplit(self.path).path
            max_length = (
                128000
                if request_path == "/api/nav_velocity"
                else (16000 if request_path == "/api/navigation/route" else 8192)
            )
            if not 0 <= length <= max_length:
                raise ValueError("Request too large")
            data = json.loads(self.rfile.read(length) or b"{}")
            if not isinstance(data, dict):
                raise ValueError("JSON object required")
            path = urlsplit(self.path).path
            if path.startswith("/api/navigation/"):
                self.navigation_proxy(self.path, data)
                return
            if path == "/api/claim":
                result = self.rt.claim(self.headers.get("X-Operator", ""))
            elif path.startswith("/api/"):
                result = self.rt.command(
                    path[5:], data, self.headers.get("X-Operator", "")
                )
            else:
                self.response(404, {"error": "Not found"})
                return
            self.response(200, result)
        except (ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
            self.response(400, {"error": str(exc)})
        except Exception as exc:
            self.rt.fail("API failed: " + str(exc))
            self.response(500, {"error": str(exc)})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group()
    group.add_argument(
        "--hardware",
        action="store_true",
        help="Enable real Arduino writes; requires DWM2 firmware",
    )
    group.add_argument(
        "--dry-run", action="store_true", help="No Arduino serial writes (default)"
    )
    group.add_argument(
        "--demo",
        action="store_true",
        help="Synthetic camera/odometry; never opens hardware",
    )
    parser.add_argument("--camera", help="oak, /dev/video0, or camera index")
    parser.add_argument("--serial", help="Arduino device or auto")
    parser.add_argument("--device", help="auto, cpu, or CUDA device index")
    parser.add_argument("--config", default="config.json")
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", default=8080, type=int)
    args = parser.parse_args()
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s"
    )
    rt = Runtime(args)
    server = ThreadingHTTPServer((args.bind, args.port), Handler)
    server.daemon_threads = True
    server.runtime = rt

    def shutdown(signum, frame):
        rt.stop("Shutdown signal")
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    try:
        rt.start_workers()
        print(
            "\nOpen http://192.168.10.25:%d (use the Jetson's current IP)" % args.port
        )
        print("Dashboard: no login token; controls available on this network")
        print(
            "Mode: "
            + (
                "HARDWARE / STOPPED until enabled"
                if args.hardware
                else "DRY RUN / no hardware writes"
            ),
            flush=True,
        )
        server.serve_forever(poll_interval=0.2)
    finally:
        rt.close()
        server.server_close()


if __name__ == "__main__":
    main()
