"""/audio/*: the executor passes the Go2's speaker routes through to the relay.

What matters: the relay token is added HERE (it never reaches the browser), the WAV bytes
arrive untouched, and the relay's status code survives — a WAV the relay refuses must be an
error at this layer too, not a 200 with an error inside. A fake relay stands in for the robot.
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
WAV = b"RIFF\x24\x00\x00\x00WAVEfmt " + bytes(range(40))


class FakeRelay(BaseHTTPRequestHandler):
    def log_message(self, *_a):
        pass

    def _reply(self, code, obj):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _record(self):
        n = int(self.headers.get("Content-Length", 0) or 0)
        self.server.seen.append((self.command, self.path, self.headers.get("Authorization"),
                                 self.headers.get("Content-Type"), self.rfile.read(n)))

    def do_GET(self):
        self._record()
        self._reply(200, {"ok": True, "volume": 7})

    def do_POST(self):
        self._record()
        if self.path == "/audio/say" and self.server.seen[-1][4] != WAV:
            return self._reply(400, {"ok": False, "error": "not a WAV file"})
        self._reply(200, {"ok": True, "seconds": 1.5})


def _serve(handler):
    srv = ThreadingHTTPServer(("127.0.0.1", 0), handler)
    srv.seen = []
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv


@pytest.fixture
def relay(monkeypatch):
    srv = _serve(FakeRelay)
    monkeypatch.setenv("GO2_RELAY_URL", f"http://127.0.0.1:{srv.server_address[1]}")
    monkeypatch.delenv("GO2_RELAY_TOKEN", raising=False)
    monkeypatch.delenv("GO2_RELAY_TOKEN_FILE", raising=False)
    monkeypatch.setenv("RELAY_TOKEN", TOKEN)
    yield srv
    srv.shutdown()


@pytest.fixture
def executor():
    srv = _serve(svc.ExecutorHandler)
    yield srv
    srv.shutdown()


def _call(srv, method, path, body=None, ctype="application/json"):
    conn = http.client.HTTPConnection("127.0.0.1", srv.server_address[1], timeout=5)
    conn.request(method, path, body=body, headers={"Content-Type": ctype})
    r = conn.getresponse()
    return r.status, json.loads(r.read())


def test_the_wav_reaches_the_relay_untouched_with_the_token_added_here(relay, executor):
    code, out = _call(executor, "POST", "/audio/say?robot=go2", WAV, "audio/wav")
    assert (code, out["ok"], out["seconds"]) == (200, True, 1.5)
    method, path, auth, ctype, body = relay.seen[-1]
    assert (method, path, auth, ctype, body) == \
        ("POST", "/audio/say", f"Bearer {TOKEN}", "audio/wav", WAV)


def test_a_refused_wav_stays_an_error(relay, executor):
    code, out = _call(executor, "POST", "/audio/say?robot=go2", b"mp3 bytes", "audio/mpeg")
    assert code == 400 and out["ok"] is False and "not a WAV" in out["error"]


def test_volume_is_read_and_set_through_the_relay(relay, executor):
    assert _call(executor, "GET", "/audio/volume?robot=go2")[1]["volume"] == 7
    _call(executor, "POST", "/audio/volume?robot=go2", b'{"level": 3}')
    assert relay.seen[-1][1:] == ("/audio/volume", f"Bearer {TOKEN}", "application/json",
                                  b'{"level": 3}')


def test_head_led_goes_through_the_relay(relay, executor):
    _call(executor, "POST", "/audio/led?robot=go2", b'{"r": 255, "g": 0, "b": 0}')
    assert relay.seen[-1][1:] == ("/audio/led", f"Bearer {TOKEN}", "application/json",
                                  b'{"r": 255, "g": 0, "b": 0}')


def test_a_robot_without_a_relay_says_so(monkeypatch, executor):
    monkeypatch.delenv("G1_RELAY_URL", raising=False)
    code, out = _call(executor, "POST", "/audio/stop?robot=g1", b"")
    assert code == 503 and "no relay configured" in out["error"]
