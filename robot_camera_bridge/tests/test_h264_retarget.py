"""`H264Relay.retarget` — switching the drive branch from one robot to the other.

WHY IT EXISTS. The operator picks Go2 or G1 in the app, and each robot serves its own
`/h264`. Before retarget() the relay was built once, at start-up, against one robot's URL, so
picking the other robot changed the camera everywhere EXCEPT the drive view, which kept
showing — or trying to reach — the first robot. Found 2026-10-01 with the Go2 powered off: the
relay retried a dead Go2 every second while the G1 was streaming.

WHAT MUST HOLD:
  1. after retarget(), frames come from the NEW robot, in the same relay (one thread: the UDP
     path binds a fixed port, and a second relay would race the first for it);
  2. a switch away from a robot that is DOWN does not wait out the reconnect backoff.

Two real HTTP servers on loopback play the two robots; the backend websocket is a fake that
records what it is sent. TCP path only — the UDP path needs a robot to grant a lease.
"""
from __future__ import annotations

import struct
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import h264_relay

B = b"--ThisRandomString"


def part(payload):
    return (B + b"\r\nContent-Type: video/x-h264\r\nX-Capture: 1.5\r\nContent-Length: "
            + str(len(payload)).encode() + b"\r\n\r\n" + payload + b"\r\n")


def robot(tag):
    """A fake robot `/h264`: one multipart part every 50 ms whose payload names the robot."""
    class H(BaseHTTPRequestHandler):
        def do_GET(self):
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=ThisRandomString")
            self.end_headers()
            try:
                while True:
                    self.wfile.write(part(tag))
                    self.wfile.flush()
                    time.sleep(0.05)
            except OSError:
                pass

        def log_message(self, *_a):
            pass

    srv = ThreadingHTTPServer(("127.0.0.1", 0), H)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv, f"http://127.0.0.1:{srv.server_address[1]}/h264"


class FakeWS:
    def __init__(self, sink):
        self.sink = sink

    def send_binary(self, data):
        self.sink.append(data[struct.calcsize("<d"):])

    def close(self):
        pass


def relay_to(url, sink, monkeypatch):
    def connect(self):
        self._ws = FakeWS(sink)
    monkeypatch.setattr(h264_relay.H264Relay, "_connect_backend", connect)
    return h264_relay.H264Relay(url, "ws://unused", udp_port=0)


def wait_for(pred, timeout=5.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if pred():
            return True
        time.sleep(0.02)
    return False


def test_after_retarget_frames_come_from_the_new_robot(monkeypatch):
    go2_srv, go2 = robot(b"GO2")
    g1_srv, g1 = robot(b"G1")
    sink = []
    r = relay_to(go2, sink, monkeypatch)
    try:
        assert wait_for(lambda: b"GO2" in sink), "never relayed the first robot"
        r.retarget(g1)
        assert wait_for(lambda: sink and sink[-1] == b"G1"), f"still relaying {sink[-1:]}"
        n = len(sink)
        time.sleep(0.3)
        assert all(x == b"G1" for x in sink[n:]), "old robot's frames after the switch"
    finally:
        r._stop.set()
        go2_srv.shutdown()
        g1_srv.shutdown()


def test_switching_away_from_a_dead_robot_does_not_wait_out_the_backoff(monkeypatch):
    dead = "http://127.0.0.1:9/h264"           # discard port: connection refused at once
    g1_srv, g1 = robot(b"G1")
    sink = []
    r = relay_to(dead, sink, monkeypatch)
    try:
        time.sleep(1.5)                         # let it fail and grow its backoff
        t0 = time.monotonic()
        r.retarget(g1)
        assert wait_for(lambda: b"G1" in sink, timeout=3.0), "switch waited out the backoff"
        assert time.monotonic() - t0 < 3.0
    finally:
        r._stop.set()
        g1_srv.shutdown()


def test_retarget_to_the_same_url_is_a_no_op(monkeypatch):
    srv, url = robot(b"GO2")
    sink = []
    r = relay_to(url, sink, monkeypatch)
    try:
        r.retarget(url)
        assert not r._switch.is_set()
    finally:
        r._stop.set()
        srv.shutdown()
