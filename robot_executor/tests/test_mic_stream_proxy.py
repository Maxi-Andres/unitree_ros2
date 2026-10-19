"""GET /audio/mic/stream: the executor passes the G1's microphone through, token and all.

The relay token is added by THIS process (it never reaches the browser), the audio arrives as
the relay sent it, failures before the first byte are JSON errors with a status, and hanging up
here hangs up on the relay — which is what switches the robot's mic off. A fake relay stands in.
"""
from __future__ import annotations

import http.client
import json
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import robot_executor_service as svc

TOKEN = "relay-test-token"  # noqa: S105  # a test fixture, not a secret
WRONG = "not-the-token"
PACKET = b"\x05\x00" * 2560          # 160 ms of 16 kHz s16le, what the G1 sends


class FakeRelay(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *_a):
        pass

    def do_GET(self):
        self.server.seen_auth = self.headers.get("Authorization")
        if self.headers.get("Authorization") != f"Bearer {TOKEN}":
            body = b'{"error": "unauthorised"}'
            self.send_response(401)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        self.send_response(200)
        self.send_header("Content-Type", "audio/L16;rate=16000;channels=1")
        self.send_header("Connection", "close")
        self.end_headers()
        self.server.hung_up = False
        try:
            for _ in range(500):                 # ~5 s, unless the executor hangs up first
                self.wfile.write(PACKET)
                self.wfile.flush()
                time.sleep(0.01)
        except (BrokenPipeError, ConnectionResetError):
            self.server.hung_up = True


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
    return conn, conn.getresponse()


def _point_at(monkeypatch, relay, token=TOKEN):
    monkeypatch.setenv("G1_RELAY_URL", f"http://127.0.0.1:{relay.server_address[1]}")
    monkeypatch.setenv("G1_RELAY_TOKEN", token)


def test_audio_passes_through_with_the_token_added_here(monkeypatch, relay, executor):
    _point_at(monkeypatch, relay)
    conn, r = _get(executor, "/audio/mic/stream?robot=g1")
    assert r.status == 200
    assert r.getheader("Content-Type") == "audio/L16;rate=16000;channels=1"
    assert r.read(3 * len(PACKET)) == PACKET * 3
    assert relay.seen_auth == f"Bearer {TOKEN}"
    conn.close()


def test_hanging_up_here_hangs_up_on_the_relay(monkeypatch, relay, executor):
    """Catches: the app switching the robot's mic off while the executor keeps the relay's
    stream open — and with it the robot's mic."""
    _point_at(monkeypatch, relay)
    conn, r = _get(executor, "/audio/mic/stream?robot=g1")
    r.read(len(PACKET))
    r.close()          # the response holds the socket: closing only the connection is not a hang-up
    conn.close()
    deadline = time.time() + 5
    while not getattr(relay, "hung_up", False) and time.time() < deadline:
        time.sleep(0.05)
    assert relay.hung_up


def test_relay_refusal_is_a_json_502_without_the_secret(monkeypatch, relay, executor):
    _point_at(monkeypatch, relay, token=WRONG)
    _conn, r = _get(executor, "/audio/mic/stream?robot=g1")
    assert r.status == 502
    body = json.loads(r.read())
    assert body["ok"] is False and "401" in body["error"] and WRONG not in body["error"]


def test_the_go2_has_no_mic_stream(monkeypatch, relay, executor):
    _point_at(monkeypatch, relay)
    _conn, r = _get(executor, "/audio/mic/stream?robot=go2")
    assert r.status == 400
    assert getattr(relay, "seen_auth", None) is None, "the relay must not even be asked"
