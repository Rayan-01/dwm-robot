"""Planar odometry, wheel speed control and swept rectangular clearance."""

import math


def route_headings(points, start, mode="path"):
    """Choose incoming-leg headings unless an explicit fixed heading is requested."""
    result = []
    previous = dict(start)
    for point in points:
        p = dict(point)
        dx, dy = p["x"] - previous["x"], p["y"] - previous["y"]
        if mode == "path":
            p["yaw"] = (
                math.atan2(dy, dx)
                if math.hypot(dx, dy) > 0.001
                else previous.get("yaw", 0.0)
            )
        result.append(p)
        previous = p
    return result


def goal_bearing(pose, goal):
    angle = math.atan2(goal["y"] - pose["y"], goal["x"] - pose["x"]) - pose["yaw"]
    return math.atan2(math.sin(angle), math.cos(angle))


class WheelOdometry:
    def __init__(self, separation):
        self.separation = separation
        self.x = self.y = self.yaw = 0.0
        self.previous = None
        self.source = None
        self.gyro_bias = None
        self.imu_used = False
        self.angular_rate = 0.0

    def update(self, left, right, source, gyro_z=None, dt=None, stationary=False):
        if not all(math.isfinite(v) for v in (left, right)):
            raise ValueError("Nonfinite wheel travel")
        self.imu_used = False
        if self.previous is None or source != self.source:
            self.gyro_bias = None
            self.previous, self.source = (left, right), source
            return
        dl, dr = left - self.previous[0], right - self.previous[1]
        self.previous = left, right
        if max(abs(dl), abs(dr)) > 0.25:
            raise ValueError("Wheel travel discontinuity")
        distance, angle = (dl + dr) / 2, (dr - dl) / self.separation
        if (
            gyro_z is not None
            and math.isfinite(gyro_z)
            and abs(gyro_z) < 2
            and dt is not None
            and 0 < dt <= 0.3
        ):
            if stationary and abs(gyro_z) < 0.02:
                self.gyro_bias = (
                    gyro_z
                    if self.gyro_bias is None
                    else 0.98 * self.gyro_bias + 0.02 * gyro_z
                )
            if self.gyro_bias is not None:
                # Use angular rate, not magnetic compass heading near the motors.
                angle = 0.8 * (gyro_z - self.gyro_bias) * dt + 0.2 * angle
                if stationary and abs(gyro_z - self.gyro_bias) < 0.02:
                    angle = 0.0
                self.imu_used = True
        self.angular_rate = angle / dt if dt is not None and 0 < dt <= 0.3 else 0.0
        sinc = math.sin(angle / 2) / (angle / 2) if abs(angle) > 1e-8 else 1
        self.x += distance * sinc * math.cos(self.yaw + angle / 2)
        self.y += distance * sinc * math.sin(self.yaw + angle / 2)
        self.yaw = math.atan2(math.sin(self.yaw + angle), math.cos(self.yaw + angle))


def clear_motion(
    points, v, w, front=0.26035, rear=0.65405, half_width=0.254, padding=0.06
):
    """Check the swept full body, including rear swing, over 0.8 seconds.

    Return False for an obstacle in the expanded body at any sampled pose.
    The ROS collision monitor additionally applies velocity-dependent braking.
    """
    if not points:
        return False
    for step in range(9):
        t = step * 0.1
        yaw = w * t
        x = v * math.sin(yaw) / w if abs(w) > 1e-5 else v * t
        y = v * (1 - math.cos(yaw)) / w if abs(w) > 1e-5 else 0
        co, si = math.cos(yaw), math.sin(yaw)
        for px, py in points:
            dx, dy = px - x, py - y
            bx, by = co * dx + si * dy, -si * dx + co * dy
            if (
                -rear - padding <= bx <= front + padding
                and abs(by) <= half_width + padding
            ):
                return False
    return True


class WheelSpeedControl:
    def __init__(self, separation, diameter):
        self.separation, self.diameter = separation, diameter
        self.integral = [0.0, 0.0]
        self.output = [0.0, 0.0]

    def step(self, v, w, feedback, dt=0.05):
        targets = [
            (v - w * self.separation / 2) * 60 / (math.pi * self.diameter),
            (v + w * self.separation / 2) * 60 / (math.pi * self.diameter),
        ]
        result = []
        for i, target in enumerate(targets):
            if abs(target) < 0.15:
                self.integral[i] = 0
                self.output[i] = 0
            else:
                error = target - feedback[i]
                self.integral[i] = max(-8, min(8, self.integral[i] + error * dt * 0.5))
                value = (
                    math.copysign(22, target)
                    + 1.1 * target
                    + 0.4 * error
                    + self.integral[i]
                )
                # Do not reverse a wheel merely to correct a speed overshoot.
                value = (
                    max(0, min(50, value)) if target > 0 else min(0, max(-50, value))
                )
                self.output[i] += max(-30 * dt, min(30 * dt, value - self.output[i]))
            result.append(round(self.output[i]))
        return tuple(result)
