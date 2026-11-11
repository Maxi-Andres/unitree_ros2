"""Parsing for relay_bridge.py (no ROS, so it is tested anywhere).

The relay's streams are what the app's Navigation page already uses and what worked on the
robot (2026-11-11): Unitree's leg odometry (GET /odom/stream, ~20 Hz) and the relay's own
obstacle map from the Mid-360s (GET /lidar/stream, the page's 2 Hz / 10 cm copy). Both are in
the same odometry frame, whose origin is wherever the robot booted.
"""
import json
import math

STALE_MS = 500          # older than this and a line is not used (the bridge is falling behind)
OBSTACLE_Z = 0.5        # m: obstacle cells become points this high, inside Nav2's height band


def parse_odom(line):
    """{t, x, y, yaw, vx, vy, wz} from an /odom/stream line, or None when unusable.

    Planar on purpose: Unitree's z drifts (+26 cm in one session, 2026-11-11) and Nav2 runs in
    two dimensions. vx, vy, wz are passed as the robot's message carries them."""
    try:
        d = json.loads(line)
        if d.get("age_ms", -1) < 0 or d["age_ms"] > STALE_MS:
            return None
        out = {k: float(d[k]) for k in ("t", "x", "y", "yaw")}
        for k in ("vx", "vy", "wz"):
            out[k] = float(d.get(k, 0.0))
    except (ValueError, KeyError, TypeError):
        return None
    if not all(math.isfinite(v) for v in out.values()):
        return None
    return out


def parse_obstacles(line):
    """(t, [(x, y, z), ...], (x, y, yaw)) from a /lidar/stream line — the obstacle points and the
    robot's pose when they were seen, both in the odometry frame — or None when unusable: not
    fresh, or not in the odometry frame ("odom": 0 = the robot's own frame)."""
    try:
        d = json.loads(line)
        if d.get("age_ms", -1) < 0 or d["age_ms"] > STALE_MS or not d.get("odom"):
            return None
        res = float(d.get("res", 0.05))
        t = float(d["t"])
        pts = [(ix * res, iy * res, OBSTACLE_Z) for ix, iy in d.get("cells", ())]
        pose = tuple(float(v) for v in d["pose"][:3])
    except (ValueError, KeyError, TypeError):
        return None
    if len(pose) != 3:
        return None
    return t, pts, pose


def yaw_to_quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


# Nav2's /cmd_vel -> the relay's POST /cmd. The relay keeps its own clamp, rate limit and 1 s
# dead-man (and the operator's stop and drive pad win over it); this side only decides what to
# send, CMD_HZ times a second: the newest velocity while it is fresh and not zero, then ONE
# stop_move, then nothing — so the relay is not flooded with stops and the drive pad is free.
CMD_HZ = 10.0
CMD_STALE_S = 0.5
CMD_ZERO = 1e-3


def next_command(cmd, cmd_t, now, stopped):
    """(body or None, stopped) for the relay this tick. `cmd` is the newest (vx, vy, vyaw) from
    Nav2 or None, `cmd_t` when it came, `stopped` whether a stop_move was already sent since
    the robot last moved."""
    fresh = cmd is not None and now - cmd_t <= CMD_STALE_S
    if fresh and any(abs(v) > CMD_ZERO for v in cmd):
        vx, vy, vyaw = cmd
        return {"verb": "move", "vx": round(vx, 3), "vy": round(vy, 3), "vyaw": round(vyaw, 3)}, False
    if not stopped:
        return {"verb": "stop_move"}, True
    return None, True


# THE OBSTACLE MAP AS A LASER SCAN, for Nav2's costmaps. The map only says where something is,
# never where nothing is, so a costmap fed only those cells would keep a person who walked past
# for good. A 360-beam scan from the robot — each beam to the nearest obstacle in its direction,
# or out to SCAN_MAX_M when there is none — lets the costmap clear along every beam.
SCAN_BEAMS = 360
SCAN_MIN_M = 0.3
SCAN_MAX_M = 5.5          # the relay's obstacle map reaches 6 m; a clear beam reports past this


def obstacles_to_scan(points, pose, beams=SCAN_BEAMS, rmax=SCAN_MAX_M):
    """Ranges (len `beams`, beam 0 straight ahead, counter-clockwise, robot frame) from obstacle
    points (x, y, _) in the odometry frame and the robot's pose (x, y, yaw) there. A beam with
    nothing nearer than `rmax` is inf: Nav2 reads that as clear out to its raytrace range."""
    px, py, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    ranges = [math.inf] * beams
    step = 2 * math.pi / beams
    for x, y, _z in points:
        dx, dy = x - px, y - py
        fx, fy = c * dx + s * dy, -s * dx + c * dy           # into the robot's frame
        r = math.hypot(fx, fy)
        if r < SCAN_MIN_M or r > rmax:
            continue
        i = int(round((math.atan2(fy, fx) % (2 * math.pi)) / step)) % beams
        if r < ranges[i]:
            ranges[i] = r
    return ranges
