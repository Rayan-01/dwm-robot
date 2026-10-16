#!/usr/bin/env python3
"""ROS sensor bridge, map storage and ordered waypoint execution."""

import base64
import json
import math
from pathlib import Path
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit, parse_qs

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    DurabilityPolicy,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Path as NavPath
from tf2_ros import Buffer, TransformListener
from dwm_navigation.map_store import MapStore
from dwm_navigation.waypoint_store import RouteStore
from dwm_navigation.navigation_stack import NavigationStack
from dwm_core.navigation_core import goal_bearing

from dwm_core.paths import runtime_root

ROOT = runtime_root()


class NavigationView(NavigationStack, Node):
    def __init__(self):
        super().__init__("poop_robot_navigation_view")
        self.mu = threading.RLock()
        self.scan = None
        self.scan_at = 0
        self.grid = None
        self.plan = []
        self.store = MapStore(ROOT / "maps")
        self.routes = RouteStore(ROOT / "routes")
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.create_subscription(
            LaserScan, "/scan", self.on_scan, qos_profile_sensor_data
        )
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(OccupancyGrid, "/map", self.on_map, qos)
        self.create_subscription(NavPath, "/plan", self.on_plan, 1)
        self.setup_navigation()
        node = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *_):
                pass

            def reply(self, code, data):
                body = json.dumps(data, allow_nan=False).encode()
                self.send_response(code)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                try:
                    self.wfile.write(body)
                except (BrokenPipeError, ConnectionResetError):
                    pass

            def do_GET(self):
                try:
                    url = urlsplit(self.path)
                    name = parse_qs(url.query).get("name", [""])[0]
                    if url.path == "/state":
                        return self.reply(200, node.state())
                    if url.path == "/map":
                        return self.reply(200, node.store.read(name))
                    if url.path == "/route":
                        return self.reply(200, node.routes.read(name))
                    self.reply(404, {"error": "Unknown navigation endpoint"})
                except (ValueError, KeyError, OSError) as error:
                    self.reply(400, {"error": str(error)})

            def do_POST(self):
                try:
                    size = int(self.headers.get("Content-Length", 0))
                    if not 0 < size <= 16000:
                        raise ValueError("Invalid request size")
                    data = json.loads(self.rfile.read(size))
                    if not isinstance(data, dict):
                        raise ValueError("Object required")
                    if self.path != "/route":
                        result = node.request_command(
                            self.path.lstrip("/"),
                            data,
                            self.headers.get("X-Operator", ""),
                        )
                        return self.reply(200, result)
                    # Require a real, readable map before persisting a map-bound draft.
                    node.store.read(data.get("map_name", ""))
                    with node.mu:
                        result = node.routes.save(data)
                    self.reply(200, result)
                except Exception as error:
                    self.reply(400, {"error": str(error)})

        self.server = ThreadingHTTPServer(("127.0.0.1", 8091), Handler)
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

    def on_scan(self, msg):
        with self.mu:
            self.scan, self.scan_at = msg, time.monotonic()

    def on_plan(self, msg):
        if msg.header.frame_id != "map":
            return
        with self.mu:
            self.plan = [
                [p.pose.position.x, p.pose.position.y]
                for p in msg.poses[:: max(1, len(msg.poses) // 1000)]
            ]

    def on_map(self, msg):
        w, h = msg.info.width, msg.info.height
        if msg.header.frame_id != "map" or not 0 < w * h <= 4_000_000:
            return
        pixels = bytearray(w * h)
        for y in range(h):
            for x in range(w):
                v = msg.data[(h - 1 - y) * w + x]
                pixels[y * w + x] = (
                    205 if v < 0 else (0 if v >= 65 else (254 if v <= 19 else 205))
                )
        q = msg.info.origin.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        with self.mu:
            self.map_distance = None
            self.grid = dict(
                width=w,
                height=h,
                pixels=base64.b64encode(pixels).decode(),
                metadata=dict(
                    resolution=msg.info.resolution,
                    origin=[
                        msg.info.origin.position.x,
                        msg.info.origin.position.y,
                        yaw,
                    ],
                    negate=0,
                    free_thresh=0.196,
                    occupied_thresh=0.65,
                ),
            )
            if self.message.startswith("Starting "):
                self.message = (
                    "Mapping active; use the hold controls to explore, then save the map."
                    if self.mode == "mapping"
                    else "Map loaded; set the current robot position and heading."
                )

    def state(self):
        with self.mu:
            scan, age = self.scan, time.monotonic() - self.scan_at
            geometry = json.loads((ROOT / "navigation_geometry.json").read_text())
            points = []
            if scan and age < 0.75:
                for i in range(0, len(scan.ranges), max(1, len(scan.ranges) // 1200)):
                    r = scan.ranges[i]
                    if (
                        math.isfinite(r)
                        and max(0.05, scan.range_min) <= r <= scan.range_max
                    ):
                        a = scan.angle_min + i * scan.angle_increment
                        points.append(
                            [round(r * math.cos(a), 3), round(r * math.sin(a), 3)]
                        )
            return dict(
                scan=dict(
                    points=points,
                    frame=scan.header.frame_id if scan else None,
                    age_s=round(age, 3) if scan else None,
                    fresh=bool(scan and age < 0.75),
                    hz=(
                        round(1 / scan.scan_time, 1)
                        if scan and scan.scan_time > 0
                        else None
                    ),
                ),
                maps=self.store.list(),
                live_map=self.grid,
                plan=self.plan,
                mode=self.mode,
                active_map_name=self.active_map,
                running=self.running,
                waypoint_index=self.index,
                waypoint_count=len(self.route),
                actual_route=self.route,
                command_velocity=dict(
                    linear=self.velocity[0], angular=self.velocity[1]
                ),
                goal_bearing_deg=(
                    math.degrees(goal_bearing(self.robot_pose, self.route[self.index]))
                    if self.robot_pose and self.index < len(self.route)
                    else None
                ),
                initial_set=self.initial_set,
                localization_confirmed=self.localization_confirmed,
                collision_monitor_active=self.collision_active,
                lidar_alignment_confirmed=bool(
                    self.geometry.get("lidar_alignment_confirmed")
                ),
                map_match=round(self.map_match(), 2),
                body_points=self.body_points(),
                odometry=dict(
                    x=self.odom.x,
                    y=self.odom.y,
                    yaw=self.odom.yaw,
                    estimated=True,
                    imu_used=self.odom.imu_used,
                    gyro_bias_rad_s=self.odom.gyro_bias,
                ),
                robot=self.robot_pose,
                poop=[],
                navigation_ready=self.ready(),
                body=dict(
                    length_m=geometry["body_length_m"], width_m=geometry["body_width_m"]
                ),
                geometry=geometry,
                message=self.message,
            )


def main():
    rclpy.init()
    node = NavigationView()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_navigation()
        node.server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
