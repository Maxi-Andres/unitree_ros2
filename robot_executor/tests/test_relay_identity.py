"""A relayed command must reach THE RIGHT ROBOT.

2026-10-07: the G1's relay URL ended up pointing at the Go2's relay (a Robot page field kept
the previous robot's value through a robot switch), so a command for the humanoid would have
been carried out by the dog. The relay reports its own robot_model in /health; the executor
now refuses when it does not match. Every test names the defect it catches.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import robot_executor_service as svc


def transport(monkeypatch, robot, relay_says):
    t = svc.RelayTransport(robot, "http://stub:8092", "tok", dry_run=False)
    asked = []
    monkeypatch.setattr(svc, "_relay_health",
                        lambda url: (asked.append(url), relay_says)[1])
    sent = []
    monkeypatch.setattr(t, "_post", lambda body: (sent.append(body), {"ok": True})[1])
    monkeypatch.setattr(t, "_udp_stop", lambda: None)
    return t, sent, asked


def test_a_command_for_the_g1_is_refused_by_the_go2_relay(monkeypatch):
    """The defect: the dog executing the humanoid's command."""
    t, sent, _ = transport(monkeypatch, "g1", {"ok": True, "robot_model": "go2"})
    res = t.execute("stop", {})
    assert res["ok"] is False and "go2's relay" in res["detail"]
    assert sent == [], "something was sent to the wrong robot"


def test_the_matching_relay_gets_the_command(monkeypatch):
    """The check must not block a robot that is configured correctly."""
    t, sent, _ = transport(monkeypatch, "go2", {"ok": True, "robot_model": "go2"})
    assert t.execute("stop", {})["ok"] is True
    assert sent and sent[-1]["verb"] == "stop_move"


def test_a_relay_that_cannot_say_is_let_through(monkeypatch):
    """An older relay with no robot_model, or one that is unreachable right now, must not
    lock the robot out — refusing needs a positive mismatch."""
    t, sent, _ = transport(monkeypatch, "go2", {"ok": False, "error": "timed out"})
    assert t.execute("stop", {})["ok"] is True
    assert sent


def test_the_answer_is_cached_not_asked_per_command(monkeypatch):
    """A /health round trip per command would add latency to every move."""
    t, _, asked = transport(monkeypatch, "go2", {"ok": True, "robot_model": "go2"})
    for _ in range(5):
        t.execute("stop", {})
    assert len(asked) == 1
