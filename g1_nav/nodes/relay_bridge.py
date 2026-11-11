#!/usr/bin/env python3
"""relay_bridge — the relay's view of the G1, into ROS2 for Nav2. Runs on PC2, read-only.

    /odom       nav_msgs/Odometry   Unitree's leg odometry (relay GET /odom/stream, ~20 Hz)
    tf          odom -> base_link   the same pose
    /obstacles  sensor_msgs/PointCloud2 in `odom`, the relay's obstacle map from the Mid-360s
                (GET /lidar/stream, what the app's Navigation page draws)
    /cmd_vel    geometry_msgs/Twist from Nav2 -> the relay's POST /cmd `move` (its clamp, rate
                limit and 1 s dead-man still hold; the operator's stop and drive pad win)

Why these and not FAST-LIO2 (2026-11-11, on the robot): the leg odometry is right turning in
place, where FAST-LIO2 still invents 0.6-1 m, and its straight-line error is small at Nav2's
slow speeds (1 % at 0.14 m/s on 2026-11-05). It is Nav2's `odom`; a global correction
(map -> odom) can come later. It reads the relay over HTTP on this machine and never touches
Unitree's DDS bus: ROS2 stays local (ROS_LOCALHOST_ONLY, its own domain, see run.sh).

    python3 relay_bridge.py            (env: RELAY_URL, RELAY_TOKEN_FILE)
"""
import http.client
import json
import os
import struct
import sys
import threading
import time
import urllib.request

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import TransformBroadcaster

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import relay_parse as rp

RELAY_URL = os.environ.get("RELAY_URL", "http://127.0.0.1:8092")
TOKEN_FILE = os.environ.get("RELAY_TOKEN_FILE", os.path.expanduser("~/.relay_token"))
RETRY_S = 2.0


def stamp(t):
    return Time(seconds=t).to_msg()


class RelayBridge:
    def __init__(self, node):
        self.node = node
        with open(TOKEN_FILE) as f:
            self.token = f.read().strip()
        self.odom_pub = node.create_publisher(Odometry, "odom", 10)
        self.obst_pub = node.create_publisher(PointCloud2, "obstacles", 2)
        self.tf = TransformBroadcaster(node)
        self.cmd, self.cmd_t = None, 0.0
        self.cmd_lock = threading.Lock()
        node.create_subscription(Twist, "cmd_vel", self.on_cmd_vel, 10)

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
            t, pts = got
            msg = PointCloud2()
            msg.header.stamp = stamp(t)
            msg.header.frame_id = "odom"
            msg.height, msg.width = 1, len(pts)
            msg.fields, msg.is_bigendian = fields, False
            msg.point_step, msg.row_step = 12, 12 * len(pts)
            msg.is_dense = True
            msg.data = b"".join(struct.pack("<fff", *p) for p in pts)
            self.obst_pub.publish(msg)


    def on_cmd_vel(self, msg):
        with self.cmd_lock:
            self.cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
            self.cmd_t = time.monotonic()

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
                    conn.request("POST", "/cmd", json.dumps(body),
                                 {"Authorization": "Bearer " + self.token,
                                  "Content-Type": "application/json"})
                    r = conn.getresponse()
                    r.read()
                    if r.status != 200:
                        self.node.get_logger().warning(f"/cmd {body['verb']}: HTTP {r.status}")
                    stopped = stopped_next
                except Exception as exc:
                    self.node.get_logger().warning(f"/cmd: {exc}")
                    conn = None
            else:
                stopped = stopped_next
            time.sleep(1.0 / rp.CMD_HZ)


def main():
    rclpy.init()
    node = rclpy.create_node("relay_bridge")
    bridge = RelayBridge(node)
    for target in (bridge.run_odom, bridge.run_obstacles, bridge.run_cmd):
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
