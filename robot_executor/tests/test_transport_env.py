"""POST /transport writes ONLY what it was sent, and a ping address does not restart anything.

THE INCIDENT (2026-10-01): G1_TRANSPORT was set to `relay` by hand in .env while the executor
was running. Saving the G1's online-check address from the UI then sent {robot, ping_ip}; the
handler refilled the missing `mode` from the PROCESS environment — still `dds`, from before the
edit — and wrote G1_TRANSPORT=dds back over it. The G1 silently fell off its relay.
"""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import robot_executor_service as svc


class Fake:
    def __init__(self):
        self.sent = None

    def _send(self, code, payload):
        self.sent = (code, payload)

    class wfile:
        @staticmethod
        def flush():
            pass


@pytest.fixture
def env(tmp_path, monkeypatch):
    f = tmp_path / ".env"
    f.write_text("G1_TRANSPORT=relay\nG1_RELAY_URL=http://192.168.51.115:8092\n")
    monkeypatch.setattr(svc, "ENV_PATH", str(f))
    monkeypatch.setenv("G1_TRANSPORT", "dds")          # the STALE process copy
    restarts = []
    monkeypatch.setattr(svc, "_restart_self", lambda: restarts.append(1))
    monkeypatch.setattr(svc.threading, "Thread",
                        lambda target, daemon=False: type("T", (), {"start": lambda s: target()})())
    return f, restarts


def call(body):
    h = Fake()
    svc.ExecutorHandler._handle_transport(h, body)
    return h.sent


def test_saving_the_ping_address_keeps_the_hand_set_transport(env):
    f, restarts = env
    code, res = call({"robot": "g1", "ping_ip": "192.168.51.115"})
    assert code == 200 and res["restarting"] is False
    text = f.read_text()
    assert "G1_TRANSPORT=relay" in text and "G1_TRANSPORT=dds" not in text
    assert "G1_PING_IP=192.168.51.115" in text
    assert restarts == []


def test_changing_the_transport_writes_it_and_restarts(env):
    f, restarts = env
    code, _ = call({"robot": "g1", "mode": "dds"})
    assert code == 200
    assert "G1_TRANSPORT=dds" in f.read_text()
    assert restarts == [1]


def test_an_empty_request_changes_nothing(env):
    f, _ = env
    before = f.read_text()
    code, _ = call({"robot": "g1"})
    assert code == 400
    assert f.read_text() == before


def test_a_relay_robot_without_a_ping_address_is_checked_at_its_relay_host(monkeypatch):
    monkeypatch.setenv("G1_TRANSPORT", "relay")
    monkeypatch.setenv("G1_RELAY_URL", "http://192.168.51.115:8092")
    monkeypatch.delenv("G1_PING_IP", raising=False)
    monkeypatch.setattr(svc, "_relay_health", lambda url: {"ok": True})
    assert svc._transport_config()["g1"]["ping_ip"] == "192.168.51.115"


def test_an_explicit_ping_address_wins(monkeypatch):
    monkeypatch.setenv("G1_TRANSPORT", "relay")
    monkeypatch.setenv("G1_RELAY_URL", "http://192.168.51.115:8092")
    monkeypatch.setenv("G1_PING_IP", "192.168.123.164")
    monkeypatch.setattr(svc, "_relay_health", lambda url: {"ok": True})
    assert svc._transport_config()["g1"]["ping_ip"] == "192.168.123.164"


def test_a_hand_edit_of_env_is_noticed(tmp_path, monkeypatch):
    f = tmp_path / ".env"
    f.write_text("G1_TRANSPORT=dds\n")
    monkeypatch.setattr(svc, "ENV_PATH", str(f))
    svc._ENV_SEEN["mtime"] = svc._env_mtime()
    assert not svc._env_changed_outside()
    import os
    import time
    time.sleep(0.01)
    f.write_text("G1_TRANSPORT=relay\n")                  # someone runs sed on it
    os.utime(f, ns=(time.time_ns(), time.time_ns()))
    assert svc._env_changed_outside()


def test_the_executors_own_write_is_not_a_hand_edit(tmp_path, monkeypatch):
    f = tmp_path / ".env"
    f.write_text("G1_TRANSPORT=relay\n")
    monkeypatch.setattr(svc, "ENV_PATH", str(f))
    svc._ENV_SEEN["mtime"] = svc._env_mtime()
    svc._set_env_keys({"G1_PING_IP": "192.168.51.115"})
    assert not svc._env_changed_outside()
