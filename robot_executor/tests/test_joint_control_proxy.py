"""POST /joints/control: the executor passes the Joints page's arm control to the relay.

This path moves the robot, so what matters: the body reaches the relay untouched with the
token added HERE (it never reaches the browser), the relay's refusals keep their status, it is
G1-only, and a DRY_RUN executor refuses everything except a release. A fake relay stands in
for the robot.
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
    def log_message(self, *_a):
        pass

    def do_POST(self):
        n = int(self.headers.get("Content-Length", "0"))
        body = json.loads(self.rfile.read(n) or b"{}")
        self.server.seen.append((self.path, self.headers.get("Authorization"), body))
        if body.get("op") == "take" and self.server.refuse:
            code, out = 409, {"ok": False, "reply": "err fsm 1 not allowed", "state": "off"}
        else:
            code, out = 200, {"ok": True, "reply": body.get("op"), "state": "active", "w": 1.0}
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


def _post(srv, path, body):
    conn = http.client.HTTPConnection("127.0.0.1", srv.server_address[1], timeout=15)
    conn.request("POST", path, json.dumps(body), {"Content-Type": "application/json"})
    r = conn.getresponse()
    out = json.loads(r.read() or b"{}")
    conn.close()
    return r.status, out


def test_the_body_reaches_the_relay_with_the_token_added_here(relay, executor):
    body = {"op": "target", "session": "abc", "joint": "left_elbow", "q": 1.2}
    code, out = _post(executor, "/joints/control?robot=g1", body)
    assert code == 200 and out["ok"] and out["state"] == "active"
    assert relay.seen == [("/joints/control", f"Bearer {TOKEN}", body)]


def test_a_refused_take_keeps_its_status(relay, executor):
    relay.refuse = True
    code, out = _post(executor, "/joints/control?robot=g1", {"op": "take"})
    assert code == 409 and out["ok"] is False and "not allowed" in out["reply"]


def test_go2_has_no_joint_control(relay, executor):
    code, _ = _post(executor, "/joints/control?robot=go2", {"op": "take"})
    assert code == 400
    assert relay.seen == [], "the relay must not even be asked"


def test_dry_run_refuses_all_but_release(relay, executor, monkeypatch):
    monkeypatch.setattr(svc, "DRY_RUN", True)
    code, out = _post(executor, "/joints/control?robot=g1", {"op": "take"})
    assert code == 409 and "DRY_RUN" in out["error"]
    assert relay.seen == []
    code, _ = _post(executor, "/joints/control?robot=g1", {"op": "release", "session": "abc"})
    assert code == 200, "a release must always get through"


def test_relay_unreachable_is_a_json_502(monkeypatch, executor):
    monkeypatch.setenv("G1_RELAY_URL", "http://127.0.0.1:9")   # discard port: nothing listens
    monkeypatch.setenv("G1_RELAY_TOKEN", TOKEN)
    monkeypatch.setattr(svc, "DRY_RUN", False)
    code, out = _post(executor, "/joints/control?robot=g1", {"op": "take"})
    assert code == 502 and "unreachable" in out["error"]


def test_bad_json_is_a_400(relay, executor):
    conn = http.client.HTTPConnection("127.0.0.1", executor.server_address[1], timeout=5)
    conn.request("POST", "/joints/control?robot=g1", "[1, 2]",
                 {"Content-Type": "application/json"})
    r = conn.getresponse()
    assert r.status == 400
    conn.close()
    assert relay.seen == []
