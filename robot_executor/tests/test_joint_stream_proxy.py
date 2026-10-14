"""GET /joints/stream: the executor passes the relay's joint state through, token and all.

What matters here: the relay token is added by THIS process (it never reaches the browser),
lines arrive as the relay sent them, every failure before the first line is a JSON error with
a status the page can show, and the stream is G1-only. A fake relay stands in for the robot.
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
WRONG = "not-the-token"
LINES = [b'{"t":1,"q":[0.1]}\n', b'{"t":2,"q":[0.2]}\n', b'{"stale":true}\n']


class FakeRelay(BaseHTTPRequestHandler):
    def log_message(self, *_a):
        pass

    def do_GET(self):
        self.server.seen_auth = self.headers.get("Authorization")
        if self.path != "/joints/stream":
            self.send_response(404)
            self.end_headers()
            return
        if self.headers.get("Authorization") != f"Bearer {TOKEN}":
            body = b'{"error": "unauthorised"}'
            self.send_response(401)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        self.send_response(200)
        self.send_header("Content-Type", "application/x-ndjson")
        self.end_headers()
        for line in LINES:
            self.wfile.write(line)
            self.wfile.flush()


def _serve(handler):
    srv = ThreadingHTTPServer(("127.0.0.1", 0), handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv


@pytest.fixture
def relay():
    srv = _serve(FakeRelay)
    yield srv
    srv.shutdown()


@pytest.fixture
def executor():
    srv = _serve(svc.ExecutorHandler)
    yield srv
    srv.shutdown()


def _get(srv, path):
    conn = http.client.HTTPConnection("127.0.0.1", srv.server_address[1], timeout=5)
    conn.request("GET", path)
    return conn.getresponse()


def _point_at(monkeypatch, relay, token=TOKEN):
    monkeypatch.setenv("G1_RELAY_URL", f"http://127.0.0.1:{relay.server_address[1]}")
    monkeypatch.delenv("G1_RELAY_TOKEN", raising=False)   # the shared token is the one used
    monkeypatch.delenv("G1_RELAY_TOKEN_FILE", raising=False)
    monkeypatch.setenv("RELAY_TOKEN", token)


def test_lines_pass_through_with_the_token_added_here(monkeypatch, relay, executor):
    _point_at(monkeypatch, relay)
    r = _get(executor, "/joints/stream?robot=g1")
    assert r.status == 200
    assert r.getheader("Content-Type") == "application/x-ndjson"
    assert r.read().splitlines(keepends=True) == LINES
    assert relay.seen_auth == f"Bearer {TOKEN}"


def test_relay_refusal_is_a_json_502(monkeypatch, relay, executor):
    _point_at(monkeypatch, relay, token=WRONG)
    r = _get(executor, "/joints/stream?robot=g1")
    assert r.status == 502
    body = json.loads(r.read())
    assert body["ok"] is False and "401" in body["error"]
    assert WRONG not in body["error"], "never echo a secret"


def test_relay_unreachable_is_a_json_502(monkeypatch, executor):
    monkeypatch.setenv("G1_RELAY_URL", "http://127.0.0.1:9")   # discard port: nothing listens
    monkeypatch.setenv("G1_RELAY_TOKEN", TOKEN)
    r = _get(executor, "/joints/stream?robot=g1")
    assert r.status == 502
    assert "unreachable" in json.loads(r.read())["error"]


def test_no_relay_configured_is_a_json_503(monkeypatch, executor):
    monkeypatch.delenv("G1_RELAY_URL", raising=False)
    r = _get(executor, "/joints/stream?robot=g1")
    assert r.status == 503
    assert "G1_RELAY_URL" in json.loads(r.read())["error"]


def test_go2_has_no_joint_stream(monkeypatch, relay, executor):
    _point_at(monkeypatch, relay)
    r = _get(executor, "/joints/stream?robot=go2")
    assert r.status == 400
    assert getattr(relay, "seen_auth", None) is None, "the relay must not even be asked"


def test_the_g1_uses_its_own_relay_token(monkeypatch, relay, executor):
    """Catches the 2026-10-13 bug: _relay_for read only the shared RELAY_TOKEN, so the G1 —
    which has G1_RELAY_TOKEN — was asked with the Go2's token and refused."""
    monkeypatch.setenv("G1_RELAY_URL", f"http://127.0.0.1:{relay.server_address[1]}")
    monkeypatch.setenv("RELAY_TOKEN", WRONG)          # the shared one: the Go2's
    monkeypatch.setenv("G1_RELAY_TOKEN", TOKEN)
    r = _get(executor, "/joints/stream?robot=g1")
    assert r.status == 200
    assert relay.seen_auth == f"Bearer {TOKEN}"
