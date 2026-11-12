#!/usr/bin/env python3
"""relay_bridge — the relay's view of the G1, into ROS2 for Nav2. Runs on PC2, read-only.

    /odom       nav_msgs/Odometry   Unitree's leg odometry (relay GET /odom/stream, ~20 Hz)
    tf          odom -> base_link   the same pose
    /obstacles  sensor_msgs/PointCloud2 in `odom`, the relay's obstacle map from the Mid-360s
                (GET /lidar/stream, what the app's Navigation page draws)
    /scan       sensor_msgs/LaserScan in `base_link`, the same map as 360 beams from the robot
                (to the nearest obstacle, inf when clear) so Nav2's costmaps can also CLEAR
    asks out loud for the way (one of assets/blocked_es_*.wav, short: "Disculpe.", "Permiso."...,
                to the relay's POST /audio/say) when Nav2's
                tree starts waiting for a blocked path (/behavior_tree_log), once per 20 s
    /cmd_vel    geometry_msgs/Twist from Nav2 -> the relay's POST /walk/move, which it accepts
                ONLY during a walk it handed to Nav2 (its clamp and 1 s dead-man still hold; the
                operator's stop and drive pad cancel that walk and Nav2's goal with it)
    HTTP        127.0.0.1:BRIDGE_PORT (8095), the relay's token: how the relay hands Nav2 a walk
                (its WALK_NAV2_URL) — GET /status, POST /goto {dx, dy, dyaw}, /plan, /cancel —
                so the app's Navigation page and agent drive Nav2 through /walk/control unchanged

Why these and not FAST-LIO2 (2026-11-11, on the robot): the leg odometry is right turning in
place, where FAST-LIO2 still invents 0.6-1 m, and its straight-line error is small at Nav2's
slow speeds (1 % at 0.14 m/s on 2026-11-05). It is Nav2's `odom`; a global correction
(map -> odom) can come later. It reads the relay over HTTP on this machine and never touches
Unitree's DDS bus: ROS2 stays local (ROS_LOCALHOST_ONLY, its own domain, see run.sh).

    python3 relay_bridge.py            (env: RELAY_URL, RELAY_TOKEN_FILE, BRIDGE_PORT)
"""
import http.client
import json
import math
import os
import struct
import sys
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav2_msgs.msg import BehaviorTreeLog
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from tf2_ros import TransformBroadcaster

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import relay_parse as rp

RELAY_URL = os.environ.get("RELAY_URL", "http://127.0.0.1:8092")
TOKEN_FILE = os.environ.get("RELAY_TOKEN_FILE", os.path.expanduser("~/.relay_token"))
BRIDGE_PORT = int(os.environ.get("BRIDGE_PORT", "8095"))   # 8094: the video pipeline
RETRY_S = 2.0
ODOM_FRESH_S = 0.5
ASSETS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "assets")


def stamp(t):
    return Time(seconds=t).to_msg()


class RelayBridge:
    def __init__(self, node):
        self.node = node
        with open(TOKEN_FILE) as f:
            self.token = f.read().strip()
        self.odom_pub = node.create_publisher(Odometry, "odom", 10)
        self.obst_pub = node.create_publisher(PointCloud2, "obstacles", 2)
        self.scan_pub = node.create_publisher(LaserScan, "scan", 2)
        self.tf = TransformBroadcaster(node)
        self.cmd, self.cmd_t = None, 0.0
        self.cmd_lock = threading.Lock()
        node.create_subscription(Twist, "cmd_vel", self.on_cmd_vel, 10)
        self.blocked_wavs = []
        for name in sorted(os.listdir(ASSETS)):
            if name.startswith("blocked_es_") and name.endswith(".wav"):
                with open(os.path.join(ASSETS, name), "rb") as f:
                    self.blocked_wavs.append(f.read())
        self.last_said, self.last_phrase = None, None
        node.create_subscription(BehaviorTreeLog, "behavior_tree_log", self.on_bt_log, 10)
        # Nav2, for the relay (serve_http): one goal at a time.
        self.pose, self.pose_t = None, 0.0
        self.nav_lock = threading.Lock()
        self.goal_handle, self.nav = None, {"state": "idle", "blocked": False, "path": None,
                                            "reason": None}
        self.goal = None                    # (x, y, yaw) in odom of the running goal
        self.navigate = ActionClient(node, NavigateToPose, "navigate_to_pose")
        self.planner = ActionClient(node, ComputePathToPose, "compute_path_to_pose")
        node.create_subscription(Path, "plan", self.on_plan, 2)
        self.refused_at = 0.0

    def lines(self, route):
        """Lines of a relay stream, forever: it reconnects after any failure."""
        while rclpy.ok():
            try:
                req = urllib.request.Request(RELAY_URL + route,
                                             headers={"Authorization": "Bearer " + self.token})
                with urllib.request.urlopen(req, timeout=5) as r:
                    for raw in r:
                        yield raw.decode("utf-8", "replace")
            except Exception as exc:
                self.node.get_logger().warning(f"{route}: {exc}; again in {RETRY_S:g} s")
            time.sleep(RETRY_S)

    def run_odom(self):
        for line in self.lines("/odom/stream"):
            o = rp.parse_odom(line)
            if o is None:
                continue
            qx, qy, qz, qw = rp.yaw_to_quat(o["yaw"])
            msg = Odometry()
            msg.header.stamp = stamp(o["t"])
            msg.header.frame_id = "odom"
            msg.child_frame_id = "base_link"
            p = msg.pose.pose
            p.position.x, p.position.y = o["x"], o["y"]
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx, qy, qz, qw
            msg.twist.twist.linear.x, msg.twist.twist.linear.y = o["vx"], o["vy"]
            msg.twist.twist.angular.z = o["wz"]
            self.odom_pub.publish(msg)
            self.pose, self.pose_t = (o["x"], o["y"], o["yaw"]), time.monotonic()
            tf = TransformStamped()
            tf.header = msg.header
            tf.child_frame_id = "base_link"
            tf.transform.translation.x, tf.transform.translation.y = o["x"], o["y"]
            tf.transform.rotation = p.orientation
            self.tf.sendTransform(tf)

    def run_obstacles(self):
        fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate("xyz")]
        for line in self.lines("/lidar/stream"):
            got = rp.parse_obstacles(line)
            if got is None:
                continue
            t, pts, pose = got
            msg = PointCloud2()
            msg.header.stamp = stamp(t)
            msg.header.frame_id = "odom"
            msg.height, msg.width = 1, len(pts)
            msg.fields, msg.is_bigendian = fields, False
            msg.point_step, msg.row_step = 12, 12 * len(pts)
            msg.is_dense = True
            msg.data = b"".join(struct.pack("<fff", *p) for p in pts)
            self.obst_pub.publish(msg)
            scan = LaserScan()
            scan.header.stamp = msg.header.stamp
            scan.header.frame_id = "base_link"
            scan.angle_min, scan.angle_increment = 0.0, 2 * math.pi / rp.SCAN_BEAMS
            scan.angle_max = scan.angle_increment * (rp.SCAN_BEAMS - 1)
            scan.range_min, scan.range_max = rp.SCAN_MIN_M, rp.SCAN_MAX_M
            scan.ranges = rp.obstacles_to_scan(pts, pose)
            self.scan_pub.publish(scan)


    def on_cmd_vel(self, msg):
        with self.cmd_lock:
            self.cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
            self.cmd_t = time.monotonic()

    def on_bt_log(self, msg):
        now = time.monotonic()
        events = [(e.node_name, e.current_status) for e in msg.event_log]
        with self.nav_lock:
            self.nav["blocked"] = rp.blocked_after(events, self.nav["blocked"])
            done = (self.nav["blocked"] and self.nav["state"] == "running"
                    and rp.arrived_blocked(self.fresh_pose(), self.goal))
            handle = self.goal_handle if done else None
            if done:
                # Blocked right at the goal: arrived (rp.ARRIVED_BLOCKED_M). Forget the handle
                # first, so its "canceled" result does not overwrite this.
                self.goal_handle = None
                self.nav.update(state="succeeded", blocked=False,
                                reason="stopped within reach of the goal: something just past it")
        if handle is not None:
            handle.cancel_goal_async()
            self.node.get_logger().info("blocked at the goal: arrived")
            return
        if rp.should_say(events, now, self.last_said):
            self.last_said = now
            threading.Thread(target=self.say_blocked, daemon=True).start()

    def say_blocked(self):
        """One of the short phrases (never the same twice running) through the robot's speaker;
        the relay validates the WAV."""
        if not self.blocked_wavs:
            return
        self.last_phrase = rp.pick_phrase(len(self.blocked_wavs), self.last_phrase)
        try:
            req = urllib.request.Request(RELAY_URL + "/audio/say",
                                         data=self.blocked_wavs[self.last_phrase],
                                         headers={"Authorization": "Bearer " + self.token,
                                                  "Content-Type": "audio/wav"})
            with urllib.request.urlopen(req, timeout=10) as r:
                r.read()
            self.node.get_logger().info("asked for the way")
        except Exception as exc:          # a voice that fails must not stop the navigation
            self.node.get_logger().warning(f"/audio/say: {exc}")

    def run_cmd(self):
        """Nav2's newest velocity to the relay, rp.CMD_HZ times a second (rp.next_command)."""
        host = RELAY_URL.split("://", 1)[-1]
        conn, stopped = None, True       # nothing sent at start: the drive pad keeps the robot
        while rclpy.ok():
            with self.cmd_lock:
                cmd, cmd_t = self.cmd, self.cmd_t
            body, stopped_next = rp.next_command(cmd, cmd_t, time.monotonic(), stopped)
            if body is not None:
                try:
                    conn = conn or http.client.HTTPConnection(host, timeout=1.0)
                    conn.request("POST", "/walk/move", json.dumps(body),
                                 {"Authorization": "Bearer " + self.token,
                                  "Content-Type": "application/json"})
                    r = conn.getresponse()
                    r.read()
                    if r.status != 200 and time.monotonic() - self.refused_at > 5.0:
                        # 409: no walk handed to Nav2 runs — the relay keeps the robot still.
                        self.refused_at = time.monotonic()
                        self.node.get_logger().warning(
                            f"/walk/move {body['verb']}: HTTP {r.status}")
                    stopped = stopped_next
                except Exception as exc:
                    self.node.get_logger().warning(f"/walk/move: {exc}")
                    conn = None
            else:
                stopped = stopped_next
            time.sleep(1.0 / rp.CMD_HZ)


    # -- Nav2 for the relay (HTTP) ----------------------------------------- #
    def on_plan(self, msg):
        with self.nav_lock:
            if self.nav["state"] == "running":
                self.nav["path"] = rp.downsample_path(
                    [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses])

    def fresh_pose(self):
        if self.pose is None or time.monotonic() - self.pose_t > ODOM_FRESH_S:
            return None
        return self.pose

    def pose_msg(self, x, y, yaw):
        msg = PoseStamped()
        msg.header.frame_id = "odom"
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = x, y
        _, _, msg.pose.orientation.z, msg.pose.orientation.w = rp.yaw_to_quat(yaw)
        return msg

    @staticmethod
    def wait(fut, timeout):
        """Block this (HTTP) thread until an rclpy future is done; the main thread spins."""
        done = threading.Event()
        fut.add_done_callback(lambda _f: done.set())
        return done.wait(timeout)

    def api_status(self):
        with self.nav_lock:
            out = dict(self.nav)
        out["ready"] = self.navigate.server_is_ready() and self.fresh_pose() is not None
        return 200, out

    def api_goto(self, body):
        pose = self.fresh_pose()
        if pose is None:
            return 503, {"ok": False, "error": "no fresh odometry"}
        if not self.navigate.server_is_ready():
            return 503, {"ok": False, "error": "Nav2 is not up (g1_nav/nav.sh)"}
        try:
            dx, dy, dyaw = (float(body.get(k, 0) or 0) for k in ("dx", "dy", "dyaw"))
        except (TypeError, ValueError):
            return 400, {"ok": False, "error": "dx/dy/dyaw must be numbers"}
        x, y, yaw = rp.goal_from_relative(pose, dx, dy, dyaw)
        goal = NavigateToPose.Goal()
        goal.pose = self.pose_msg(x, y, yaw)
        fut = self.navigate.send_goal_async(goal)
        if not self.wait(fut, 5.0) or fut.result() is None or not fut.result().accepted:
            return 503, {"ok": False, "error": "Nav2 did not accept the goal"}
        handle = fut.result()
        with self.nav_lock:
            self.goal_handle, self.goal = handle, (x, y, yaw)
            self.nav = {"state": "running", "blocked": False, "path": None, "reason": None}
        handle.get_result_async().add_done_callback(lambda f, h=handle: self.on_result(h, f))
        self.node.get_logger().info(f"goal from the relay: {dx:.2f} m fwd, {dy:.2f} m left")
        return 200, {"ok": True, "goal": {"x": x, "y": y, "yaw": yaw}}

    def on_result(self, handle, fut):
        with self.nav_lock:
            if self.goal_handle is not handle:
                return                     # an older goal ending after a newer one started
            res = fut.result()
            self.nav["state"] = rp.nav_state(res.status if res is not None else 6)
            self.nav["blocked"] = False
            if self.nav["state"] == "aborted":
                self.nav["reason"] = "no way there, or the way stayed blocked"

    def api_cancel(self):
        with self.nav_lock:
            handle = self.goal_handle
            running = self.nav["state"] == "running"
        if handle is not None and running:
            self.wait(handle.cancel_goal_async(), 3.0)
        return 200, {"ok": True}

    def api_plan(self, body):
        pose = self.fresh_pose()
        if pose is None:
            return 503, {"ok": False, "error": "no fresh odometry"}
        if not self.planner.server_is_ready():
            return 503, {"ok": False, "error": "Nav2's planner is not up (g1_nav/nav.sh)"}
        try:
            dx, dy, dyaw = (float(body.get(k, 0) or 0) for k in ("dx", "dy", "dyaw"))
        except (TypeError, ValueError):
            return 400, {"ok": False, "error": "dx/dy/dyaw must be numbers"}
        x, y, yaw = rp.goal_from_relative(pose, dx, dy, dyaw)
        goal = ComputePathToPose.Goal()
        goal.pose = self.pose_msg(x, y, yaw)
        goal.planner_id = "GridBased"
        fut = self.planner.send_goal_async(goal)
        if not self.wait(fut, 3.0) or fut.result() is None or not fut.result().accepted:
            return 503, {"ok": False, "error": "Nav2's planner did not take it"}
        res = fut.result().get_result_async()
        if not self.wait(res, 5.0) or res.result() is None:
            return 503, {"ok": False, "error": "Nav2's planner did not answer"}
        poses = res.result().result.path.poses
        if not poses:
            return 200, {"ok": False, "error": "no way there"}
        start = {"x": pose[0], "y": pose[1], "yaw": pose[2]}
        return 200, {"ok": True, "start": start, "goal": {"x": x, "y": y, "yaw": yaw},
                     "path": rp.downsample_path([(p.pose.position.x, p.pose.position.y)
                                                 for p in poses])}

    def serve_http(self):
        bridge = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *_a):
                pass

            def reply(self, code, obj):
                data = json.dumps(obj).encode()
                self.send_response(code)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            def authorised(self):
                return self.headers.get("Authorization", "") == "Bearer " + bridge.token

            def do_GET(self):
                if not self.authorised():
                    return self.reply(401, {"error": "unauthorised"})
                if self.path.split("?")[0] == "/status":
                    return self.reply(*bridge.api_status())
                return self.reply(404, {"error": "not found"})

            def do_POST(self):
                if not self.authorised():
                    return self.reply(401, {"error": "unauthorised"})
                try:
                    n = int(self.headers.get("Content-Length", "0"))
                    body = json.loads(self.rfile.read(n) or b"{}")
                except ValueError:
                    return self.reply(400, {"error": "bad json"})
                route = {"/goto": lambda: bridge.api_goto(body),
                         "/plan": lambda: bridge.api_plan(body),
                         "/cancel": bridge.api_cancel}.get(self.path.split("?")[0])
                if route is None:
                    return self.reply(404, {"error": "not found"})
                return self.reply(*route())

        ThreadingHTTPServer(("127.0.0.1", BRIDGE_PORT), Handler).serve_forever()


def main():
    rclpy.init()
    node = rclpy.create_node("relay_bridge")
    bridge = RelayBridge(node)
    for target in (bridge.run_odom, bridge.run_obstacles, bridge.run_cmd, bridge.serve_http):
        threading.Thread(target=target, daemon=True).start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
