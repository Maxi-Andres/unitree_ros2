"""POST /walk/control and GET /odom/stream: the executor passes the walk to a point and the
odometry to the relay, where the loop runs (relay_server.Walk).

What matters: the token is added HERE, the body reaches the relay untouched, its refusals keep
their status, it is G1-only, and DRY_RUN refuses `goto` but never a cancel. A fake relay stands
in for the robot.
"""
from __future__ import annotations

import http.client
import json
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import robot_executor_service as svc

TOKEN = "relay-test-token"  # noqa: S105  # a test fixture, not a secret


class FakeRelay(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *_a):
        pass

    def do_GET(self):
        self.server.seen.append((self.path, self.headers.get("Authorization"), None))
        self.send_response(200)
        self.send_header("Content-Type", "application/x-ndjson")
        self.send_header("Connection", "close")
        self.end_headers()
        for k in range(3):
            self.wfile.write(json.dumps({"x": k, "y": 0, "yaw": 0, "age_ms": 20}).encode() + b"\n")
            self.wfile.flush()

    def do_POST(self):
        n = int(self.headers.get("Content-Length", "0"))
        body = json.loads(self.rfile.read(n) or b"{}")
        self.server.seen.append((self.path, self.headers.get("Authorization"), body))
        if self.server.refuse:
            code, out = 409, {"ok": False, "error": "a walk is already running"}
        else:
            code, out = 200, {"ok": True, "walk": {"state": "running"}}
        data = json.dumps(out).encode()
        self.send_response(code)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)


def _serve(handler):
    srv = ThreadingHTTPServer(("127.0.0.1", 0), handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv


@pytest.fixture
def relay(monkeypatch):
    srv = _serve(FakeRelay)
    srv.seen = []
    srv.refuse = False
    monkeypatch.setenv("G1_RELAY_URL", f"http://127.0.0.1:{srv.server_address[1]}")
    monkeypatch.setenv("G1_RELAY_TOKEN", TOKEN)
    monkeypatch.setattr(svc, "DRY_RUN", False)
    yield srv
    srv.shutdown()


@pytest.fixture
def executor():
    srv = _serve(svc.ExecutorHandler)
    yield srv
    srv.shutdown()


def _req(srv, method, path, body=None):
    conn = http.client.HTTPConnection("127.0.0.1", srv.server_address[1], timeout=15)
    conn.request(method, path, json.dumps(body) if body is not None else None,
                 {"Content-Type": "application/json"})
    r = conn.getresponse()
    raw = r.read()
    conn.close()
    return r.status, raw


def test_goto_reaches_the_relay_with_the_token_added_here(relay, executor):
    body = {"op": "goto", "dx": 2.0, "dy": 0, "dyaw": 0}
    code, raw = _req(executor, "POST", "/walk/control?robot=g1", body)
    assert code == 200 and json.loads(raw)["walk"]["state"] == "running"
    assert relay.seen == [("/walk/control", f"Bearer {TOKEN}", body)]


def test_a_refusal_keeps_its_status(relay, executor):
    relay.refuse = True
    code, raw = _req(executor, "POST", "/walk/control?robot=g1", {"op": "goto", "dx": 1})
    assert code == 409 and "already" in json.loads(raw)["error"]


def test_g1_only(relay, executor):
    code, _raw = _req(executor, "POST", "/walk/control?robot=go2", {"op": "goto", "dx": 1})
    assert code == 400 and relay.seen == []


def test_dry_run_refuses_goto_but_never_a_cancel(relay, executor, monkeypatch):
    monkeypatch.setattr(svc, "DRY_RUN", True)
    assert _req(executor, "POST", "/walk/control?robot=g1", {"op": "goto", "dx": 1})[0] == 409
    assert _req(executor, "POST", "/walk/control?robot=g1", {"op": "cancel"})[0] == 200
    assert [b["op"] for _p, _a, b in relay.seen] == ["cancel"]


def test_the_odometry_stream_passes_through(relay, executor):
    code, raw = _req(executor, "GET", "/odom/stream?robot=g1")
    assert code == 200
    assert [json.loads(ln)["x"] for ln in raw.splitlines()] == [0, 1, 2]
    assert relay.seen[0][:2] == ("/odom/stream", f"Bearer {TOKEN}")
