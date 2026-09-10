#!/usr/bin/env python3
"""
robot_camera_bridge.py — stream a robot's camera into AI-VL as the "Live" source.

Reads the robot camera over ROS2 (camera_sources.py), and — while streaming is ON —
forwards each JPEG frame to the AI-VL backend over a WebSocket. The backend fans it
straight out to the monitors (no YOLO) for a MINIMUM-LATENCY view of what the robot
sees. A tiny HTTP control lets the Monitor page start/stop it.

    🤖 camera --ROS2--> THIS (decode->JPEG) --WS--> backend /ws/robot-cam --> monitors

Runs inside the unitree_ros2 devcontainer (ROS2 + robot DDS live there); host-networked
so it reaches the backend at wss://localhost:8443. Start it (usually via
run_camera_bridge.sh, which sources the ROS2 env):
    source /workspace/setup.sh
    python3 /workspace/robot_camera_bridge/robot_camera_bridge.py

Endpoints (control):
    GET  /health   -> {ok, robot, streaming, connected}
    GET  /status   -> {streaming, connected, frames_sent}
    POST /start    -> begin streaming
    POST /stop     -> stop streaming
"""
import json
import os
import ssl
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import camera_sources
import rclpy
import websocket  # websocket-client


def _load_dotenv(path):
    if not os.path.exists(path):
        return
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line and not line.startswith("#") and "=" in line:
                k, _, v = line.partition("=")
                os.environ.setdefault(k.strip(), v.strip().strip('"').strip("'"))


def _as_bool(v, default):
    return default if v is None else str(v).strip().lower() in ("1", "true", "yes", "on")


_HERE = os.path.dirname(os.path.abspath(__file__))
_load_dotenv(os.path.join(_HERE, ".env"))

# Whether the operator asked for streaming, remembered across process restarts.
#
# WHY: run_camera_bridge.sh supervises this process now, but a supervisor that brings the
# bridge back NOT streaming leaves the drive view just as dark as no supervisor at all —
# the frontend only sends /start when the page mounts, so nothing re-asks after a crash.
# The flag records the operator's INTENT, so a restart resumes what they wanted. It is
# written only by /start and /stop, never by the shutdown path, so exiting cleanly does
# not look like "the operator turned it off".
_STREAMING_FLAG = os.path.join(_HERE, ".streaming")


def _remember_streaming(on):
    try:
        if on:
            open(_STREAMING_FLAG, "w").close()
        elif os.path.exists(_STREAMING_FLAG):
            os.remove(_STREAMING_FLAG)
    except OSError as exc:                     # never let bookkeeping break the stream
        print(f"[camera] could not record streaming state: {exc}", flush=True)


ROBOT = os.environ.get("CAMERA_ROBOT", "go2")           # go2 | g1 | stream | test
# "stream" needs no DDS: it reads the video that already left the robot (see
# camera_sources.HttpStreamSource). Use it whenever the robot is not on this subnet.
BACKEND_WS_URL = os.environ.get("BACKEND_WS_URL", "wss://localhost:8443/ws/robot-cam")
CONTROL_HOST = os.environ.get("CAMERA_CONTROL_HOST", "0.0.0.0")  # noqa: S104  # known finding P0-1: binds broadly, no auth yet
CONTROL_PORT = int(os.environ.get("CAMERA_CONTROL_PORT", "8091"))
START_STREAMING = _as_bool(os.environ.get("START_STREAMING"), False)
# Passed through to the camera source (fps, resolution, quality, topic…).
SOURCE_CFG = {k: os.environ[k] for k in (
    "GO2_VIDEO_FPS", "GO2_RESOLUTION", "JPEG_QUALITY",
    "G1_CAMERA_SOURCE", "G1_IMAGE_TOPIC", "G1_VIDEO_FPS", "G1_RESOLUTION",
    "STREAM_URL", "STREAM_FPS", "STREAM_RESOLUTION", "STREAM_QUALITY",
    "TEST_FPS") if k in os.environ}


class SourceManager:
    """Owns the active camera source and switches it (go2|g1|test) at RUNTIME. The
    switch (close old + build new: destroys/creates ROS subs & timers) must run on
    the ROS executor thread, so /config only records a `pending` robot and a
    supervisor timer applies it. fps/resolution/quality are forwarded to the source."""

    def __init__(self, node, on_frame, cfg, robot):
        self._node = node
        self._on_frame = on_frame
        self._cfg = cfg
        self._lock = threading.Lock()
        self._robot = robot
        self._pending = None
        self._source = camera_sources.build_source(
            robot, node, on_frame, cfg, logger=node.get_logger())
        node.create_timer(0.5, self._supervise)

    def _supervise(self):
        with self._lock:
            pending, self._pending = self._pending, None
        if not pending or pending == self._robot:
            return
        try:
            self._source.close()
        except Exception:
            pass
        try:
            self._source = camera_sources.build_source(
                pending, self._node, self._on_frame, self._cfg,
                logger=self._node.get_logger())
            self._robot = pending
            self._node.get_logger().info(f"camera source switched to '{pending}'")
        except Exception as e:
            self._node.get_logger().error(f"camera switch to '{pending}' failed: {e}")

    def request_robot(self, robot):
        if robot not in ("go2", "g1", "stream", "test"):
            return False
        with self._lock:
            self._pending = robot
        return True

    def set_params(self, **kw):
        setter = getattr(self._source, "set_params", None)
        if setter:
            setter(**kw)

    def get_params(self):
        getter = getattr(self._source, "get_params", None)
        return {"robot": self._robot, **(getter() if getter else {})}


class CameraBridge:
    """Holds the WS to the backend and gates frame forwarding by a streaming flag.

    on_frame() runs on the ROS callback thread; start()/stop() on the control-HTTP
    thread — a lock serializes access to the socket. A failed send drops the socket
    and it reconnects (throttled) on the next frame."""

    def __init__(self):
        self._lock = threading.Lock()
        self._ws = None
        self._streaming = False
        self._last_attempt = 0.0
        self._frames_sent = 0

    def _connect_locked(self):
        if self._ws is not None:
            return
        now = time.monotonic()
        if now - self._last_attempt < 2.0:   # throttle reconnects
            return
        self._last_attempt = now
        try:
            self._ws = websocket.create_connection(
                BACKEND_WS_URL, sslopt={"cert_reqs": ssl.CERT_NONE}, timeout=5,
                enable_multithread=True)
            # IMPORTANT: create_connection's timeout also becomes the socket's recv
            # timeout. Clear it (blocking) so the reader's recv() blocks indefinitely
            # instead of dying after 5 s of idle — otherwise it stops answering the
            # server's keepalive PINGs and we get dropped (~40 s later).
            self._ws.settimeout(None)
            # Reader thread: we only PUSH frames, but the server sends keepalive
            # PINGs. websocket-client auto-replies PONG while blocked in recv(), so
            # draining the socket here is what keeps the connection alive.
            threading.Thread(target=self._reader, args=(self._ws,),
                             daemon=True).start()
            print(f"[camera] connected to {BACKEND_WS_URL}", flush=True)
        except Exception as e:
            self._ws = None
            print(f"[camera] WS connect failed: {e}", flush=True)

    def _reader(self, ws):
        """Drain incoming frames on `ws` (auto-PONGs keepalive pings). Exits when the
        socket closes/replaced. The server never sends data here, so recv() just
        blocks and answers pings until the connection ends."""
        try:
            while True:
                ws.recv()
        except Exception:
            pass  # closed or replaced -> reconnect happens on the next frame send

    def _close_locked(self):
        if self._ws is not None:
            try:
                self._ws.close()
            except Exception:
                pass
            self._ws = None

    def start(self):
        with self._lock:
            self._streaming = True
            self._last_attempt = 0.0
            self._connect_locked()

    def stop(self):
        with self._lock:
            self._streaming = False
            self._close_locked()

    def on_frame(self, jpeg):
        if not self._streaming:
            return
        with self._lock:
            if self._ws is None:
                self._connect_locked()
                if self._ws is None:
                    return
            try:
                self._ws.send_binary(jpeg)
                self._frames_sent += 1
            except Exception as e:
                print(f"[camera] WS send failed: {e}", flush=True)
                self._close_locked()

    def status(self):
        with self._lock:
            return {"streaming": self._streaming, "connected": self._ws is not None,
                    "frames_sent": self._frames_sent}


class ControlHandler(BaseHTTPRequestHandler):
    bridge: CameraBridge = None  # set in main
    mgr: SourceManager = None    # set in main

    def _params(self):
        """Current camera params (robot/fps/resolution/quality)."""
        return self.mgr.get_params() if self.mgr else {}

    def _send(self, code, payload):
        body = json.dumps(payload).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *_a):
        pass

    def do_GET(self):
        path = self.path.rstrip("/")
        if path == "/health":
            self._send(200, {"ok": True, "service": "robot_camera_bridge",
                             "robot": ROBOT, **self.bridge.status(), **self._params()})
        elif path == "/status":
            self._send(200, {**self.bridge.status(), **self._params()})
        else:
            self._send(404, {"ok": False, "error": "not found"})

    def do_POST(self):
        path = self.path.rstrip("/")
        if path == "/start":
            self.bridge.start()
            _remember_streaming(True)
            self._send(200, {"ok": True, "robot": ROBOT, **self.bridge.status()})
        elif path == "/stop":
            self.bridge.stop()
            _remember_streaming(False)
            self._send(200, {"ok": True, "robot": ROBOT, **self.bridge.status()})
        elif path == "/config":
            # {robot?, fps?, resolution?, quality?} — switch the camera source robot
            # and/or tune the live source.
            try:
                length = int(self.headers.get("Content-Length", 0))
                body = json.loads(self.rfile.read(length) or b"{}")
            except (ValueError, json.JSONDecodeError):
                self._send(400, {"ok": False, "error": "invalid JSON body"})
                return
            if body.get("robot"):
                self.mgr.request_robot(str(body["robot"]))
            self.mgr.set_params(fps=body.get("fps"), resolution=body.get("resolution"),
                                quality=body.get("quality"))
            self._send(200, {"ok": True, **self._params()})
        else:
            self._send(404, {"ok": False, "error": "not found"})


def main():
    rclpy.init()
    node = rclpy.create_node("aivl_robot_camera_bridge")
    bridge = CameraBridge()
    try:
        mgr = SourceManager(node, bridge.on_frame, SOURCE_CFG, ROBOT)
    except Exception as e:
        node.get_logger().error(f"camera source '{ROBOT}' failed to init: {e}")
        raise

    ControlHandler.bridge = bridge
    ControlHandler.mgr = mgr
    server = ThreadingHTTPServer((CONTROL_HOST, CONTROL_PORT), ControlHandler)
    threading.Thread(target=server.serve_forever, daemon=True).start()

    resumed = os.path.exists(_STREAMING_FLAG)
    if START_STREAMING or resumed:
        bridge.start()
        if resumed:
            print("[camera] resuming streaming after a restart", flush=True)
    # Report the ACTUAL state, not the env default: since a restart resumes the operator's
    # last intent, printing START_STREAMING here said "streaming=False" on a process that
    # was in fact streaming.
    print(f"robot_camera_bridge: robot={ROBOT} control=:{CONTROL_PORT} "
          f"streaming={bridge.status()['streaming']} -> {BACKEND_WS_URL}", flush=True)

    stopped_externally = False
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        # Something tore the ROS context down under us. Until 2026-09-10 this escaped as a
        # traceback, and since the launcher used `exec` with no supervisor the drive view
        # just went dark until a human noticed. Exit non-zero instead so the supervision
        # loop in run_camera_bridge.sh brings the bridge straight back.
        stopped_externally = True
    finally:
        bridge.stop()
        server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if stopped_externally:
        print("[camera] ROS context shut down externally; exiting for a restart",
              file=sys.stderr, flush=True)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
