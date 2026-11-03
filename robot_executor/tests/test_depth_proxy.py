"""GET /camera/depth: the executor passes the G1's depth snapshot through, token and all.

The relay token is added by THIS process (it never reaches the browser), the PNG and its
X-Depth-Info arrive as the relay sent them, and the relay's refusals keep their status. A fake
relay stands in.
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
PNG = b"\x89PNG\r\n\x1a\n" + b"\x01" * 100
INFO = json.dumps({"width": 848, "height": 480, "scale_m": 0.001})


class FakeRelay(BaseHTTPRequestHandler):
    def log_message(self, *_a):
        pass

    def do_GET(self):
        self.server.seen = (self.path, self.headers.get("Authorization"))
        if self.server.off:
            body = b'{"ok": false, "error": "depth snapshots are off on this robot"}'
            self.send_response(404)
        else:
            body = PNG
            self.send_response(200)
            self.send_header("X-Depth-Info", INFO)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def _serve(handler):
    srv = ThreadingHTTPServer(("127.0.0.1", 0), handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv


@pytest.fixture
def relay(monkeypatch):
    srv = _serve(FakeRelay)
    srv.off = False
    monkeypatch.setenv("G1_RELAY_URL", f"http://127.0.0.1:{srv.server_address[1]}")
    monkeypatch.setenv("G1_RELAY_TOKEN", TOKEN)
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
    r = conn.getresponse()
    return r.status, r.getheader("X-Depth-Info"), r.read()


def test_the_png_passes_through_with_the_token_added_here(relay, executor):
    code, info, body = _get(executor, "/camera/depth?robot=g1&frames=3")
    assert (code, info, body) == (200, INFO, PNG)
    assert relay.seen == ("/camera/depth?frames=3", f"Bearer {TOKEN}")


def test_a_refusal_keeps_its_status_and_reason(relay, executor):
    relay.off = True
    code, _info, body = _get(executor, "/camera/depth?robot=g1")
    assert code == 404 and "off on this robot" in json.loads(body)["error"]
