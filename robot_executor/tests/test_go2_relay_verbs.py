"""The executor's relay path for the Go2's gestures, and what it tells the drive pad.

WHY: until 2026-10-01 the Go2's relay map stopped at hello, so the pad's stretch, heart,
scrape and pose came back "not available over the relay" over LTE. Pose is the odd one: an
on/off skill, while the relay's verbs carry no arguments, so each side is its own verb.

Dry-run transport: nothing leaves the machine; _post is wrapped to record the verb.
"""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import robot_executor_service as svc


def go2_transport(monkeypatch):
    t = svc.RelayTransport("go2", "http://stub", "tok", dry_run=True)
    sent = []
    real = t._post
    monkeypatch.setattr(t, "_post", lambda body: (sent.append(body.get("verb")), real(body))[1])
    return t, sent


@pytest.mark.parametrize("skill", ["stretch", "scrape", "heart"])
def test_go2_gestures_reach_the_relay_as_their_verb(skill, monkeypatch):
    t, sent = go2_transport(monkeypatch)
    assert t.execute(skill, {})["ok"]
    assert sent == [skill]


@pytest.mark.parametrize("params,verb", [
    ({"on": True}, "pose_on"), ({"on": False}, "pose_off"), ({}, "pose_on"),
])
def test_pose_on_and_off_are_separate_verbs(params, verb, monkeypatch):
    t, sent = go2_transport(monkeypatch)
    assert t.execute("pose", params)["ok"]
    assert sent == [verb]


@pytest.mark.parametrize("skill,params,verb", [
    ("front_flip", {}, "front_flip"), ("dance1", {}, "dance1"),
    ("handstand", {"on": True}, "handstand_on"), ("walk_upright", {"on": False},
                                                  "walk_upright_off"),
    ("set_gait", {"gait": "free_walk"}, "gait_free_walk"),
])
def test_go2_acrobatics_and_gaits_reach_the_relay(skill, params, verb, monkeypatch):
    """Since 2026-10-01 the relay carries them; safe mode is what holds them back (below)."""
    t, sent = go2_transport(monkeypatch)
    assert t.execute(skill, params)["ok"]
    assert sent == [verb]


def test_safe_mode_still_holds_back_every_go2_acrobatic():
    import go2_commands
    for skill in ("front_flip", "back_flip", "left_flip", "front_jump", "front_pounce",
                  "handstand", "walk_upright", "dance1", "dance2", "damp"):
        assert skill in go2_commands.DANGEROUS_SKILLS


def test_an_unknown_gait_is_refused_before_the_relay(monkeypatch):
    t, sent = go2_transport(monkeypatch)
    assert not t.execute("set_gait", {"gait": "moonwalk"})["ok"]
    assert sent == []


def test_the_pad_gets_every_skill_when_the_relay_did_not_answer():
    allowed = svc._relay_allowed_skills("go2", {"ok": False, "error": "unreachable"})
    assert allowed == sorted(svc.RelayTransport.VERB_FOR_SKILL_BY_ROBOT["go2"])


def test_the_pad_drops_what_an_unpulled_relay_does_not_accept():
    """A relay still on the old allowlist refuses the new verbs; the pad must not offer them."""
    old = ["stop_move", "stand_up", "stand_down", "damp", "balance_stand", "recovery_stand",
           "sit", "rise_sit", "hello", "keepalive"]
    allowed = svc._relay_allowed_skills("go2", {"ok": True, "verbs": old})
    assert "hello" in allowed and "stop" in allowed
    assert not {"stretch", "scrape", "heart", "pose"} & set(allowed)


def test_pose_needs_both_of_its_verbs():
    verbs = ["stop_move", "pose_on"]
    assert "pose" not in svc._relay_allowed_skills("go2", {"verbs": verbs})
    assert "pose" in svc._relay_allowed_skills("go2", {"verbs": [*verbs, "pose_off"]})
