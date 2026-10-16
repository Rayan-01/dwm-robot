"""ROS navigation integration; all motor writes go through robot.py's lease."""

from concurrent.futures import Future
from collections import deque
import json
import math
import os
from pathlib import Path
from queue import Queue, Empty
import signal
import subprocess
import threading
import time
from urllib.request import Request, urlopen

import numpy as np
from scipy.ndimage import distance_transform_edt
import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import (
    Twist,
    TransformStamped,
    PoseStamped,
    PoseWithCovarianceStamped,
)
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from lifecycle_msgs.srv import GetState
from std_srvs.srv import Empty as EmptyService
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from dwm_core.navigation_core import WheelOdometry, route_headings
from dwm_navigation.waypoint_store import validate_route

from dwm_core.paths import runtime_root

ROOT = runtime_root()


class NavigationStack:
    def setup_navigation(self):
        self.geometry = json.loads((ROOT / "navigation_geometry.json").read_text())
        self.odom = WheelOdometry(self.geometry["track_centre_spacing_m"])
        self.odom_tick_at = None
        self.odom_pub = self.create_publisher(Odometry, "/odometry/filtered", 10)
        self.broadcaster = TransformBroadcaster(self)
        self.static = StaticTransformBroadcaster(self)
        self.send_mount()
        self.initial_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self.nav = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.collision_state = self.create_client(
            GetState, "/collision_monitor/get_state"
        )
        self.collision_active = False
        self.collision_future = None
        self.collision_check_at = 0
        self.nomotion = self.create_client(EmptyService, "/request_nomotion_update")
        self.nomotion_at = 0
        self.nomotion_future = None
        self.create_subscription(Twist, "/cmd_vel_safe", self.on_velocity, 1)
        self.commands = Queue(maxsize=20)
        self.done = threading.Event()
        self.robot_state = None
        self.robot_at = 0
        self.robot_pose = None
        self.localization_confirmed = False
        self.initial_set = False
        self.initial_pending = None
        self.initial_retry_at = 0
        self.mode = "idle"
        self.active_map = None
        self.message = (
            "Ready: verify lidar alignment, then start mapping or load a saved map."
        )
        self.process = None
        self.logfile = None
        self.goal = None
        self.pending = False
        self.epoch = 0
        self.route = []
        self.motion_history = deque(maxlen=400)
        self.index = 0
        self.running = False
        self.operator = ""
        self.key = None
        self.drive_sequence = 0
        self.velocity = (0.0, 0.0)
        self.velocity_at = 0
        self.mapping_velocity = (0.0, 0.0)
        self.mapping_until = 0
        self.map_distance = None
        self.create_timer(0.1, self.navigation_tick)
        self.robot_worker = threading.Thread(target=self.poll_robot, daemon=True)
        self.robot_worker.start()

    def send_mount(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = "base_footprint", "laser"
        t.transform.translation.x = self.geometry["lidar"]["track_centre_x_m"]
        yaw = self.geometry["lidar"]["yaw_rad"] or 0.0
        t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw / 2), math.cos(
            yaw / 2
        )
        # A planar projection for 2D navigation, not measured 3D camera geometry.
        self.static.sendTransform(t)

    def rpc_robot(self, action, data=None):
        request = Request(
            "http://127.0.0.1:8080/api/" + action,
            data=None if data is None else json.dumps(data).encode(),
            headers={"Content-Type": "application/json", "X-Operator": self.operator},
        )
        with urlopen(request, timeout=0.6) as response:
            return json.load(response)

    def poll_robot(self):
        while not self.done.is_set():
            started = time.monotonic()
            try:
                s = self.rpc_robot("state")
                with self.mu:
                    self.robot_state, self.robot_at = s, time.monotonic()
                    key, driving = (
                        self.key,
                        self.running or self.mapping_until > time.monotonic(),
                    )
                    v, w = (
                        self.mapping_velocity
                        if self.mode == "mapping"
                        else self.velocity
                    )
                    fresh_velocity = (
                        self.mode == "mapping"
                        or time.monotonic() - self.velocity_at < 0.35
                    )
                    points = self.body_points()
                    scan_fresh = (
                        self.scan is not None and time.monotonic() - self.scan_at < 0.5
                    )
                    if self.running:
                        self.motion_history.append(
                            dict(
                                time_s=time.time(),
                                pose=self.robot_pose,
                                waypoint_index=self.index,
                                command=dict(linear=v, angular=w),
                                output=s["output"],
                                wheel_rpm=[s["left_rpm"], s["right_rpm"]],
                                imu=s["imu"],
                                map_match=self.map_match(),
                            )
                        )
                if key:
                    self.drive_sequence += 1
                    if not driving or not fresh_velocity:
                        v, w = 0.0, 0.0
                    self.rpc_robot(
                        "nav_velocity",
                        dict(
                            key=key,
                            seq=self.drive_sequence,
                            linear=v,
                            angular=w,
                            points=points,
                            scan_fresh=scan_fresh,
                        ),
                    )
            except Exception as error:
                with self.mu:
                    if self.key:
                        self.key = None
                        self.running = False
                        self.message = "Navigation motor session ended: " + str(error)
                        self.epoch += 1
                        if self.goal:
                            self.goal.cancel_goal_async()
                        self.goal = None
                        self.pending = False
            self.done.wait(max(0.01, 0.1 - (time.monotonic() - started)))

    def on_velocity(self, msg):
        with self.mu:
            self.velocity = (
                max(-0.12, min(0.12, msg.linear.x)),
                max(-0.3, min(0.3, msg.angular.z)),
            )
            self.velocity_at = time.monotonic()

    def body_points(self):
        if not self.scan:
            return []
        yaw = self.geometry["lidar"]["yaw_rad"] or 0
        offset = self.geometry["lidar"]["track_centre_x_m"]
        result = []
        for i in range(0, len(self.scan.ranges), max(1, len(self.scan.ranges) // 3200)):
            r = self.scan.ranges[i]
            if (
                math.isfinite(r)
                and max(0.05, self.scan.range_min) <= r <= self.scan.range_max
            ):
                a = self.scan.angle_min + i * self.scan.angle_increment + yaw
                result.append(
                    [round(offset + r * math.cos(a), 3), round(r * math.sin(a), 3)]
                )
        return result

    def navigation_tick(self):
        with self.mu:
            now = time.monotonic()
            if (
                self.mode == "navigation"
                and self.initial_set
                and now - self.nomotion_at > 1
                and time.monotonic() - self.scan_at < 0.5
            ):
                if self.nomotion_future is None or self.nomotion_future.done():
                    if self.nomotion.service_is_ready():
                        self.nomotion_future = self.nomotion.call_async(
                            EmptyService.Request()
                        )
                        self.nomotion_at = now
            if self.mode == "navigation" and now - self.collision_check_at > 1:
                if self.collision_future is not None and self.collision_future.done():
                    try:
                        self.collision_active = (
                            self.collision_future.result().current_state.id == 3
                        )
                    except Exception:
                        self.collision_active = False
                    self.collision_future = None
                if (
                    self.collision_future is None
                    and self.collision_state.service_is_ready()
                ):
                    self.collision_future = self.collision_state.call_async(
                        GetState.Request()
                    )
                    self.collision_check_at = now
            s = self.robot_state
            if (
                s
                and now - self.robot_at < 0.4
                and s["connection"]["wheel_age_s"] is not None
                and s["connection"]["wheel_age_s"] < 0.5
            ):
                try:
                    imu = s.get("imu", {})
                    gyro = None
                    if imu.get("connected") and imu.get("age_s", 1) < 0.2:
                        angles = imu["data"]
                        if (
                            abs(angles.get("roll_deg", 180)) < 15
                            and abs(angles.get("pitch_deg", 180)) < 15
                        ):
                            gyro = angles["gyro_rad_s"][2]
                    dt = (
                        now - self.odom_tick_at
                        if self.odom_tick_at is not None
                        else None
                    )
                    stationary = max(
                        abs(s["left_rpm"]), abs(s["right_rpm"])
                    ) < 0.5 and s["output"][:2] == [0, 0]
                    self.odom.update(
                        s["odometry_left_m"],
                        s["odometry_right_m"],
                        s.get("instance_id"),
                        gyro,
                        dt,
                        stationary,
                    )
                    self.odom_tick_at = now
                    stamp = self.get_clock().now().to_msg()
                    t = TransformStamped()
                    t.header.stamp = stamp
                    t.header.frame_id = "odom"
                    t.child_frame_id = "base_footprint"
                    t.transform.translation.x, t.transform.translation.y = (
                        self.odom.x,
                        self.odom.y,
                    )
                    t.transform.rotation.z, t.transform.rotation.w = math.sin(
                        self.odom.yaw / 2
                    ), math.cos(self.odom.yaw / 2)
                    self.broadcaster.sendTransform(t)
                    msg = Odometry()
                    msg.header = t.header
                    msg.child_frame_id = t.child_frame_id
                    msg.pose.pose.position.x, msg.pose.pose.position.y = (
                        self.odom.x,
                        self.odom.y,
                    )
                    msg.pose.pose.orientation = t.transform.rotation
                    vl, vr = [
                        s[k] * math.pi * self.geometry["sprocket_diameter_m"] / 60
                        for k in ("left_rpm", "right_rpm")
                    ]
                    msg.twist.twist.linear.x = (vl + vr) / 2
                    msg.twist.twist.angular.z = (vr - vl) / self.geometry[
                        "track_centre_spacing_m"
                    ]
                    if self.odom.imu_used:
                        msg.twist.twist.angular.z = self.odom.angular_rate
                    msg.pose.covariance[0] = msg.pose.covariance[7] = 0.05
                    msg.pose.covariance[35] = 0.1
                    msg.twist.covariance[0] = 0.05
                    msg.twist.covariance[35] = 0.1
                    self.odom_pub.publish(msg)
                except ValueError as error:
                    self.message = str(error)
                    self.cancel_route()
            self.robot_pose = None
            if self.mode != "idle":
                try:
                    t = self.tf.lookup_transform(
                        "map", "base_footprint", rclpy.time.Time()
                    )
                    age = (
                        self.get_clock().now()
                        - rclpy.time.Time.from_msg(t.header.stamp)
                    ).nanoseconds / 1e9
                    if abs(age) < 0.8:
                        q = t.transform.rotation
                        self.robot_pose = dict(
                            x=t.transform.translation.x,
                            y=t.transform.translation.y,
                            yaw=math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z),
                        )
                except Exception:
                    pass
            if self.initial_pending is not None:
                if self.robot_pose is not None:
                    self.initial_pending = None
                elif now - self.initial_retry_at > 1 and self.mode == "navigation":
                    self.initial_pending.header.stamp = self.get_clock().now().to_msg()
                    self.initial_pub.publish(self.initial_pending)
                    self.initial_retry_at = now
            if self.running and not self.ready():
                self.cancel_route()
                self.message = "Localization or lidar unavailable; route stopped"
            if self.process and self.process.poll() is not None:
                code = self.process.returncode
                self.cancel_route()
                self.process = None
                self.mode = "idle"
                self.message = (
                    f"ROS stack exited ({code}); inspect navigation-session.log"
                )
            for _ in range(4):
                try:
                    action, data, operator, future = self.commands.get_nowait()
                except Empty:
                    break
                if not future.set_running_or_notify_cancel():
                    continue
                try:
                    future.set_result(self.navigation_command(action, data, operator))
                except Exception as error:
                    future.set_exception(error)

    def request_command(self, action, data, operator):
        f = Future()
        self.commands.put_nowait((action, data, operator, f))
        try:
            return f.result(timeout=3)
        except Exception:
            f.cancel()
            raise

    def launch_stack(self, mode, name=None):
        if self.mode != "idle":
            raise ValueError("Stop the current map session before switching")
        if not self.scan or time.monotonic() - self.scan_at > 0.5:
            raise ValueError("Lidar scan unavailable")
        if not self.robot_state or time.monotonic() - self.robot_at > 0.5:
            raise ValueError("Robot odometry unavailable")
        names = {n for n, _ in self.get_node_names_and_namespaces()}
        if names.intersection({"slam_toolbox", "amcl", "controller_server"}):
            raise ValueError("Another ROS navigation stack is already running")
        if mode == "mapping":
            args = [
                "ros2",
                "launch",
                "dwm_bringup",
                "mapping.launch.py",
                "start_robot:=false",
            ]
        else:
            self.store.read(name)
            args = [
                "ros2",
                "launch",
                "dwm_bringup",
                "navigation.launch.py",
                "start_robot:=false",
                "map:=" + str(self.store.directory(name) / "map.yaml"),
            ]
        self.logfile = (ROOT / "logs/navigation-session.log").open("ab", buffering=0)
        self.process = subprocess.Popen(
            args, stdout=self.logfile, stderr=self.logfile, start_new_session=True
        )
        self.mode, self.active_map = mode, name
        self.initial_set = self.localization_confirmed = False
        self.grid = None
        self.map_distance = None
        self.plan = []
        self.message = "Starting " + mode

    def pose_message(self, data):
        values = [data.get(k, 0) for k in ("x", "y", "yaw")]
        if any(
            isinstance(v, bool)
            or not isinstance(v, (int, float))
            or not math.isfinite(v)
            for v in values
        ):
            raise ValueError("Invalid pose")
        p = PoseStamped()
        p.header.frame_id = "map"
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x, p.pose.position.y = values[:2]
        p.pose.orientation.z, p.pose.orientation.w = math.sin(values[2] / 2), math.cos(
            values[2] / 2
        )
        return p

    def ready(self):
        return bool(
            self.mode == "navigation"
            and self.nav.server_is_ready()
            and self.grid
            and self.robot_pose
            and self.initial_set
            and self.localization_confirmed
            and self.geometry.get("lidar_alignment_confirmed")
            and self.collision_active
            and self.map_match() >= 0.6
            and time.monotonic() - self.scan_at < 0.5
            and time.monotonic() - self.robot_at < 0.4
        )

    def cancel_route(self):
        self.epoch += 1
        if self.goal:
            self.goal.cancel_goal_async()
        self.goal = None
        self.pending = False
        self.running = False
        self.velocity = (0.0, 0.0)
        if self.key:
            try:
                self.rpc_robot("nav_end", {"key": self.key})
            except Exception:
                pass
        self.key = None
        self.mapping_until = 0
        if self.motion_history:
            target = ROOT / "logs/navigation-last-run.json"
            temp = target.with_suffix(".tmp")
            temp.write_text(
                json.dumps(
                    dict(
                        map_name=self.active_map,
                        route=self.route,
                        records=list(self.motion_history),
                    ),
                    indent=2,
                )
            )
            temp.replace(target)

    def send_next(self):
        if self.index >= len(self.route):
            self.cancel_route()
            self.message = "All waypoints reached"
            return
        if not self.ready():
            self.cancel_route()
            self.message = "Localization or lidar lost; route stopped"
            return
        self.pending = True
        goal = NavigateToPose.Goal()
        goal.pose = self.pose_message(self.route[self.index])
        epoch = self.epoch
        self.nav.send_goal_async(goal).add_done_callback(
            lambda f: self.accepted(f, epoch)
        )
        self.message = f"Navigating to waypoint {self.index+1} of {len(self.route)}"

    def accepted(self, future, epoch):
        with self.mu:
            handle = future.result()
            if epoch != self.epoch:
                if handle.accepted:
                    handle.cancel_goal_async()
                return
            self.pending = False
            if not handle.accepted:
                self.cancel_route()
                self.message = "Nav2 rejected waypoint"
                return
            self.goal = handle
            handle.get_result_async().add_done_callback(
                lambda f: self.completed(f, epoch)
            )

    def completed(self, future, epoch):
        with self.mu:
            if epoch != self.epoch:
                return
            status = future.result().status
            self.goal = None
            if status == 4:
                self.index += 1
                self.send_next()
            else:
                self.cancel_route()
                self.message = (
                    f"Waypoint {self.index+1} could not be reached (status {status})"
                )

    def navigation_command(self, action, data, operator):
        if action == "alignment":
            if self.running or self.key:
                raise ValueError("Stop motion before changing lidar alignment")
            degrees = data.get("yaw_degrees")
            if (
                isinstance(degrees, bool)
                or not isinstance(degrees, (int, float))
                or not math.isfinite(degrees)
                or abs(degrees) > 180
            ):
                raise ValueError("Yaw must be -180..180 degrees")
            previous = self.geometry["lidar"]["yaw_rad"] or 0.0
            if self.mode != "idle" and abs(previous - math.radians(degrees)) > 0.001:
                raise ValueError(
                    "End the map session before changing the lidar angle; create a fresh map after changing it"
                )
            self.geometry["lidar"]["yaw_rad"] = math.radians(degrees)
            self.geometry["lidar_alignment_confirmed"] = data.get("confirmed") is True
            temp = ROOT / "navigation_geometry.tmp"
            temp.write_text(json.dumps(self.geometry, indent=2))
            temp.replace(ROOT / "navigation_geometry.json")
            changed = abs(previous - math.radians(degrees)) > 0.001
            self.send_mount()
            if changed:
                self.localization_confirmed = False
            self.message = (
                "Lidar angle changed; create a fresh map."
                if changed
                else "Lidar alignment confirmed; current map position confirmation is preserved."
            )
        elif action in ("mapping", "navigation"):
            self.launch_stack(action, data.get("name"))
        elif action == "save_map":
            if self.mode != "mapping" or not self.grid:
                raise ValueError("No live map to save")
            self.store.save(data["name"], self.grid)
            self.message = "Map saved: " + data["name"]
        elif action == "initial":
            if self.mode != "navigation" or not self.grid or self.running:
                raise ValueError("Load a map and stop motion before setting the pose")
            p = self.pose_message(data)
            m = PoseWithCovarianceStamped()
            m.header = p.header
            m.pose.pose = p.pose
            m.pose.covariance[0] = m.pose.covariance[7] = 0.04
            m.pose.covariance[35] = 0.04
            self.initial_pending = m
            self.initial_retry_at = 0
            self.initial_pub.publish(m)
            self.initial_set = True
            self.localization_confirmed = False
            self.message = "Initial pose sent. Check robot arrow and lidar against the map, then confirm localization."
        elif action == "confirm_pose":
            if not self.initial_set or not self.robot_pose:
                raise ValueError(
                    "Set an initial robot pose and wait for localization first"
                )
            if self.map_match() < 0.6:
                raise ValueError(
                    "Lidar does not align with this map. Correct robot position/heading or create a new map."
                )
            self.localization_confirmed = True
            self.message = "Localized; select waypoints and Run route"
        elif action == "run":
            if not self.ready():
                raise ValueError(
                    "Load current map, set initial pose, verify lidar alignment and confirm localization first"
                )
            if self.running:
                raise ValueError("A route is already running")
            route = validate_route(data)
            if route["map_name"] != self.active_map:
                raise ValueError("Route belongs to a different map")
            self.operator = operator
            self.key = self.rpc_robot("nav_begin", {})["key"]
            self.route = route_headings(
                route["waypoints"], self.robot_pose, route["heading_mode"]
            )
            self.index, self.running = 0, True
            self.motion_history.clear()
            self.velocity = (0.0, 0.0)
            self.velocity_at = 0
            self.epoch += 1
            self.send_next()
        elif action == "mapping_drive":
            if self.mode != "mapping" or not self.geometry.get(
                "lidar_alignment_confirmed"
            ):
                raise ValueError("Start mapping and confirm lidar alignment first")
            direction = data.get("direction")
            options = {
                "forward": (0.08, 0.0),
                "back": (-0.08, 0.0),
                "left": (0.0, 0.2),
                "right": (0.0, -0.2),
                "stop": (0.0, 0.0),
            }
            if direction not in options:
                raise ValueError("Invalid mapping direction")
            self.operator = operator
            if not self.key:
                self.key = self.rpc_robot("nav_begin", {})["key"]
            self.mapping_velocity = options[direction]
            self.mapping_until = time.monotonic() + 0.3
        elif action == "cancel":
            self.cancel_route()
            self.message = "Route cancelled"
        elif action == "stop_session":
            self.cancel_route()
            if self.process:
                old = self.process
                os.killpg(old.pid, signal.SIGINT)

                def reap():
                    try:
                        old.wait(timeout=8)
                    except subprocess.TimeoutExpired:
                        try:
                            os.killpg(old.pid, signal.SIGTERM)
                        except ProcessLookupError:
                            pass
                        old.wait(timeout=5)

                threading.Thread(target=reap, daemon=True).start()
                # Do not block ROS callbacks waiting for subprocess teardown.
                self.process = None
            if self.logfile:
                self.logfile.close()
                self.logfile = None
            self.mode = "idle"
            self.active_map = None
            self.grid = None
            self.plan = []
            self.robot_pose = None
            self.localization_confirmed = False
            self.initial_pending = None
            self.collision_active = False
            self.message = "Map session stopped"
        else:
            raise ValueError("Unknown navigation command")
        return {"ok": True}

    def map_match(self):
        if not self.grid or not self.robot_pose:
            return 0.0
        if self.map_distance is None:
            a = np.frombuffer(
                __import__("base64").b64decode(self.grid["pixels"]), dtype=np.uint8
            ).reshape(self.grid["height"], self.grid["width"])
            self.map_distance = (
                distance_transform_edt(a > 100) * self.grid["metadata"]["resolution"]
            )
        p = self.robot_pose
        r = self.grid["metadata"]["resolution"]
        ox, oy, angle = self.grid["metadata"]["origin"]
        hits = valid = 0
        for bx, by in self.body_points()[::5]:
            wx = p["x"] + math.cos(p["yaw"]) * bx - math.sin(p["yaw"]) * by - ox
            wy = p["y"] + math.sin(p["yaw"]) * bx + math.cos(p["yaw"]) * by - oy
            x = int((math.cos(angle) * wx + math.sin(angle) * wy) / r)
            y = (
                self.grid["height"]
                - 1
                - int((-math.sin(angle) * wx + math.cos(angle) * wy) / r)
            )
            if 0 <= x < self.grid["width"] and 0 <= y < self.grid["height"]:
                valid += 1
                hits += self.map_distance[y, x] < 0.2
        return hits / valid if valid >= 30 else 0.0

    def close_navigation(self):
        self.cancel_route()
        self.done.set()
        self.robot_worker.join(timeout=2)
        if self.process:
            try:
                os.killpg(self.process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
