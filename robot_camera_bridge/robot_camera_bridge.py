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
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
import websocket  # websocket-client

import camera_sources


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

ROBOT = os.environ.get("CAMERA_ROBOT", "go2")           # go2 | g1 | test
BACKEND_WS_URL = os.environ.get("BACKEND_WS_URL", "wss://localhost:8443/ws/robot-cam")
CONTROL_HOST = os.environ.get("CAMERA_CONTROL_HOST", "0.0.0.0")
CONTROL_PORT = int(os.environ.get("CAMERA_CONTROL_PORT", "8091"))
START_STREAMING = _as_bool(os.environ.get("START_STREAMING"), False)
# Passed through to the camera source (resolution, quality, topic…).
SOURCE_CFG = {k: os.environ[k] for k in (
    "GO2_RESOLUTION", "JPEG_QUALITY", "G1_IMAGE_TOPIC", "TEST_FPS") if k in os.environ}


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
            # Reader thread: we only PUSH frames, but the server sends keepalive
            # PINGs. websocket-client auto-replies PONG while blocked in recv(), so
            # draining the socket here is what keeps the connection alive (without it
            # the server drops us with "keepalive ping timeout").
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
                             "robot": ROBOT, **self.bridge.status()})
        elif path == "/status":
            self._send(200, self.bridge.status())
        else:
            self._send(404, {"ok": False, "error": "not found"})

    def do_POST(self):
        path = self.path.rstrip("/")
        if path == "/start":
            self.bridge.start()
            self._send(200, {"ok": True, "robot": ROBOT, **self.bridge.status()})
        elif path == "/stop":
            self.bridge.stop()
            self._send(200, {"ok": True, "robot": ROBOT, **self.bridge.status()})
        else:
            self._send(404, {"ok": False, "error": "not found"})


def main():
    rclpy.init()
    node = rclpy.create_node("aivl_robot_camera_bridge")
    bridge = CameraBridge()
    try:
        camera_sources.build_source(
            ROBOT, node, bridge.on_frame, SOURCE_CFG, logger=node.get_logger())
    except Exception as e:
        node.get_logger().error(f"camera source '{ROBOT}' failed to init: {e}")
        raise

    ControlHandler.bridge = bridge
    server = ThreadingHTTPServer((CONTROL_HOST, CONTROL_PORT), ControlHandler)
    threading.Thread(target=server.serve_forever, daemon=True).start()

    if START_STREAMING:
        bridge.start()
    print(f"robot_camera_bridge: robot={ROBOT} control=:{CONTROL_PORT} "
          f"streaming={START_STREAMING} -> {BACKEND_WS_URL}", flush=True)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.stop()
        server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
