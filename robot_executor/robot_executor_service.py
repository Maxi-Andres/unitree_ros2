#!/usr/bin/env python3
"""
robot_executor_service.py — AI-VL Phase 2 skill executor (Unitree, ROS2 transport).

Closes the voice loop: it receives the skill JSON that AI-VL's /command produced
(`{robot, skill, params}`) over HTTP and MAKES THE ROBOT DO IT by publishing the
matching Unitree command over ROS2. Runs inside the unitree_ros2 devcontainer (that
is where ROS2 + the robot's DDS connection live); the AI-VL backend forwards to it.

    frontend button -> backend /api/execute -> THIS /execute -> ROS2 -> 🤖 robot

Design:
- **Transport-abstracted** (the recorded Phase-2 decision): a thin `RobotTransport`
  interface with a `Go2Ros2Transport` implementation (rclpy). A G1 / SDK transport can
  be added later without touching the HTTP layer.
- **Safety:** `SAFE_MODE` (default on) blocks acrobatics (flips, handstand, walk
  upright). `DRY_RUN` builds and logs the command WITHOUT publishing — use it to test
  the plumbing without moving the robot.

Run (inside the devcontainer, after sourcing the ROS2 env so DDS + unitree_api are up):
    source /workspace/setup.sh
    python3 /workspace/robot_executor/robot_executor_service.py
Config comes from robot_executor/.env (see .env.example) or the environment.

Endpoints:
    GET  /health              -> {ok, robot, safe_mode, dry_run}
    POST /execute {robot, skill, params} -> {ok, robot, skill, detail, ...}
"""
import json
import os
import threading
import time
from abc import ABC, abstractmethod
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import go2_commands


# --------------------------------------------------------------------------- #
# Config (.env or environment)
# --------------------------------------------------------------------------- #
def _load_dotenv(path):
    """Minimal .env loader (no dependency). KEY=VALUE per line; # comments ignored."""
    if not os.path.exists(path):
        return
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, _, value = line.partition("=")
            os.environ.setdefault(key.strip(), value.strip().strip('"').strip("'"))


def _as_bool(value, default):
    if value is None:
        return default
    return str(value).strip().lower() in ("1", "true", "yes", "on", "si", "sí")


_HERE = os.path.dirname(os.path.abspath(__file__))
_load_dotenv(os.path.join(_HERE, ".env"))

EXECUTOR_HOST = os.environ.get("EXECUTOR_HOST", "0.0.0.0")
EXECUTOR_PORT = int(os.environ.get("EXECUTOR_PORT", "8090"))
DEFAULT_ROBOT = os.environ.get("DEFAULT_ROBOT", "go2")
ROBOT_IP = os.environ.get("ROBOT_IP", "192.168.123.161")  # informational (health/ping)
# Default OFF. This is only the FALLBACK: each /execute request may carry its own
# `safe_mode` (the page toggle), which overrides this per request.
SAFE_MODE = _as_bool(os.environ.get("SAFE_MODE"), False)
DRY_RUN = _as_bool(os.environ.get("DRY_RUN"), False)
MOVE_RATE_HZ = float(os.environ.get("MOVE_RATE_HZ", "10"))
DEFAULT_STEP_S = float(os.environ.get("DEFAULT_STEP_S", "2.0"))
# Hard ceiling on a single bounded move, so a bad duration can't run the robot away.
MAX_STEP_S = float(os.environ.get("MAX_STEP_S", "10.0"))


# --------------------------------------------------------------------------- #
# Transport interface
# --------------------------------------------------------------------------- #
class RobotTransport(ABC):
    """Actuation channel to one robot. Swap ROS2 <-> SDK behind this interface."""

    @abstractmethod
    def execute(self, skill: str, params: dict) -> dict:
        """Perform `skill`. Returns {ok, detail, ...}. Never raises for a normal
        unsupported skill — returns {ok: False, ...} instead."""

    def shutdown(self) -> None:  # optional
        pass


class Go2Ros2Transport(RobotTransport):
    """Drives the Go2 by publishing unitree_api/msg/Request to /api/sport/request."""

    def __init__(self, dry_run: bool):
        self._dry_run = dry_run
        self._move_lock = threading.Lock()
        self._move_stop = threading.Event()
        self._move_thread = None
        self._node = None
        self._pub = None
        self._Request = None
        if not dry_run:
            self._init_ros()

    def _init_ros(self):
        import rclpy
        from unitree_api.msg import Request
        if not rclpy.ok():
            rclpy.init()
        self._rclpy = rclpy
        self._Request = Request
        self._node = rclpy.create_node("aivl_robot_executor_go2")
        self._pub = self._node.create_publisher(Request, "/api/sport/request", 10)

    def _publish(self, api_id: int, parameter: dict | None):
        """Build + publish one sport Request (or just log it in dry-run)."""
        param_str = json.dumps(parameter) if parameter is not None else ""
        if self._dry_run:
            print(f"[DRY_RUN] would publish /api/sport/request "
                  f"api_id={api_id} parameter={param_str!r}", flush=True)
            return
        req = self._Request()
        req.header.identity.api_id = api_id
        req.parameter = param_str
        self._pub.publish(req)

    def _stop_move_loop(self):
        """Signal any running move loop to end and join it."""
        self._move_stop.set()
        thread = self._move_thread
        if thread and thread.is_alive():
            thread.join(timeout=1.0)
        self._move_thread = None

    def _run_move_loop(self, vx, vy, vyaw, deadline):
        """Re-publish Move at MOVE_RATE_HZ until stopped or the deadline passes, then
        (for a bounded move) StopMove. A held velocity needs re-sending; this also
        means a crash/stop reliably halts the robot."""
        period = 1.0 / MOVE_RATE_HZ
        try:
            while not self._move_stop.is_set():
                if deadline is not None and time.monotonic() >= deadline:
                    break
                self._publish(go2_commands.MOVE_API_ID,
                              {"x": vx, "y": vy, "z": vyaw})
                time.sleep(period)
        finally:
            # Always stop when a bounded move ends; a continuous move is halted by the
            # explicit 'stop' skill (which also calls StopMove).
            if deadline is not None:
                self._publish(go2_commands.STOPMOVE_API_ID, None)

    def _start_move(self, vx, vy, vyaw, duration, continuous):
        with self._move_lock:
            self._stop_move_loop()
            self._move_stop = threading.Event()
            deadline = None
            if not continuous:
                step = duration if duration else DEFAULT_STEP_S
                step = max(0.1, min(step, MAX_STEP_S))
                deadline = time.monotonic() + step
            self._move_thread = threading.Thread(
                target=self._run_move_loop, args=(vx, vy, vyaw, deadline), daemon=True)
            self._move_thread.start()

    def execute(self, skill: str, params: dict) -> dict:
        intent = go2_commands.resolve(skill, params or {})
        kind = intent["kind"]

        if kind == "unsupported":
            return {"ok": False, "detail": intent["reason"]}

        if kind == "stop":
            with self._move_lock:
                self._stop_move_loop()
            self._publish(go2_commands.STOPMOVE_API_ID, None)
            return {"ok": True, "detail": "StopMove", "api_id": go2_commands.STOPMOVE_API_ID}

        if kind == "move":
            self._start_move(intent["vx"], intent["vy"], intent["vyaw"],
                             intent["duration"], intent["continuous"])
            mode = "continuous (until 'stop')" if intent["continuous"] else \
                f"{intent['duration'] or DEFAULT_STEP_S:.1f}s step"
            return {"ok": True, "detail": f"Move vx={intent['vx']} vy={intent['vy']} "
                    f"vyaw={intent['vyaw']} ({mode})", "api_id": go2_commands.MOVE_API_ID}

        # single
        self._publish(intent["api_id"], intent["parameter"])
        return {"ok": True, "detail": f"sport api_id={intent['api_id']}",
                "api_id": intent["api_id"], "parameter": intent["parameter"]}

    def shutdown(self):
        try:
            with self._move_lock:
                self._stop_move_loop()
            if not self._dry_run and self._node is not None:
                self._publish(go2_commands.STOPMOVE_API_ID, None)
                self._node.destroy_node()
                if self._rclpy.ok():
                    self._rclpy.shutdown()
        except Exception:
            pass


class UnsupportedRobotTransport(RobotTransport):
    """Placeholder for a robot with no transport yet (e.g. G1 over ROS2/SDK)."""

    def __init__(self, robot: str):
        self._robot = robot

    def execute(self, skill: str, params: dict) -> dict:
        return {"ok": False, "detail": f"no transport implemented for robot "
                f"'{self._robot}' yet (only 'go2' is wired)"}


# --------------------------------------------------------------------------- #
# Transport registry — one live transport per robot (built lazily)
# --------------------------------------------------------------------------- #
_TRANSPORTS: dict[str, RobotTransport] = {}
_TRANSPORTS_LOCK = threading.Lock()


def _get_transport(robot: str) -> RobotTransport:
    with _TRANSPORTS_LOCK:
        if robot not in _TRANSPORTS:
            if robot == "go2":
                _TRANSPORTS[robot] = Go2Ros2Transport(dry_run=DRY_RUN)
            else:
                _TRANSPORTS[robot] = UnsupportedRobotTransport(robot)
        return _TRANSPORTS[robot]


# --------------------------------------------------------------------------- #
# HTTP server
# --------------------------------------------------------------------------- #
class ExecutorHandler(BaseHTTPRequestHandler):
    def _send(self, code, payload):
        body = json.dumps(payload).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *_args):  # quieter default logging
        pass

    def do_GET(self):
        if self.path.rstrip("/") == "/health":
            self._send(200, {"ok": True, "service": "robot_executor",
                             "default_robot": DEFAULT_ROBOT, "safe_mode": SAFE_MODE,
                             "dry_run": DRY_RUN, "robot_ip": ROBOT_IP})
        else:
            self._send(404, {"ok": False, "error": "not found"})

    def do_POST(self):
        if self.path.rstrip("/") != "/execute":
            self._send(404, {"ok": False, "error": "not found"})
            return
        try:
            length = int(self.headers.get("Content-Length", 0))
            body = json.loads(self.rfile.read(length) or b"{}")
        except (ValueError, json.JSONDecodeError):
            self._send(400, {"ok": False, "error": "invalid JSON body"})
            return

        robot = (body.get("robot") or DEFAULT_ROBOT).strip()
        skill = (body.get("skill") or "").strip()
        params = body.get("params") or {}

        if not skill or skill == "unknown":
            self._send(400, {"ok": False, "error": "no executable skill"})
            return

        # Per-request safe_mode (page toggle) overrides the env default when present.
        req_safe = body.get("safe_mode")
        effective_safe = req_safe if isinstance(req_safe, bool) else SAFE_MODE
        if effective_safe and skill in go2_commands.DANGEROUS_SKILLS:
            self._send(403, {"ok": False, "blocked": True, "robot": robot,
                             "skill": skill,
                             "error": f"'{skill}' blocked by SAFE_MODE (acrobatic). "
                             "Turn SAFE_MODE off to allow."})
            return

        try:
            result = _get_transport(robot).execute(skill, params)
        except Exception as e:  # never let a transport error kill the server
            self._send(502, {"ok": False, "robot": robot, "skill": skill,
                             "error": f"transport error: {e}"})
            return

        code = 200 if result.get("ok") else 422
        self._send(code, {"robot": robot, "skill": skill, "dry_run": DRY_RUN, **result})


def main():
    print(f"robot_executor: robot={DEFAULT_ROBOT} safe_mode={SAFE_MODE} "
          f"dry_run={DRY_RUN} listening on {EXECUTOR_HOST}:{EXECUTOR_PORT}", flush=True)
    server = ThreadingHTTPServer((EXECUTOR_HOST, EXECUTOR_PORT), ExecutorHandler)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        for transport in _TRANSPORTS.values():
            transport.shutdown()
        server.server_close()


if __name__ == "__main__":
    main()
