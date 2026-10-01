"""The executor's relay path for the G1: each skill reaches the G1 relay as the right verb.

WHY: until 2026-10-01 RelayTransport resolved every skill with go2_commands and mapped it with
one Go2 table, so a G1 on the relay could not stand, walk or squat — every skill came back
"not available over the relay", or resolved as the Go2's. The G1's verbs are its own, measured
on THIS robot (g1_commands.FSM_IDS), and the dangerous ones must stay unreachable remotely.

Dry-run transport: nothing leaves the machine; _post is wrapped to record the verb.
"""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import robot_executor_service as svc


def g1_transport(monkeypatch):
    t = svc.RelayTransport("g1", "http://stub", "tok", dry_run=True)
    sent = []
    real = t._post
    monkeypatch.setattr(t, "_post", lambda body: (sent.append(body.get("verb")), real(body))[1])
    return t, sent


@pytest.mark.parametrize("skill,verb", [
    ("stand_up", "stand_up"), ("walk_waist", "walk_waist"), ("start", "start"),
    ("squat", "squat"),
    ("lie_up", "lie_up"), ("balance_stand", "balance_stand"), ("wave_hand", "wave_hand"),
])
def test_g1_skills_reach_the_relay_as_their_verb(skill, verb, monkeypatch):
    t, sent = g1_transport(monkeypatch)
    res = t.execute(skill, {})
    assert res["ok"], res
    assert sent == [verb]


@pytest.mark.parametrize("skill", ["squat_sdk", "sit", "set_fsm_id", "set_speed_mode",
                                   "switch_mode"])
def test_what_misbehaved_or_takes_a_number_never_reaches_the_relay(skill, monkeypatch):
    t, sent = g1_transport(monkeypatch)
    res = t.execute(skill, {"fsm_id": 0})
    assert not res["ok"], res
    assert sent == []


@pytest.mark.parametrize("params,verb", [
    ({"action": "hug"}, "action_hug"), ({"action": "shake_hand"}, "action_shake_hand"),
    ({"action": "high_wave"}, "arm_high_wave"),
])
def test_g1_arm_actions_pick_their_own_verb(params, verb, monkeypatch):
    t, sent = g1_transport(monkeypatch)
    assert t.execute("arm_action", params)["ok"]
    assert sent == [verb]


def test_high_five_is_refused_before_the_relay(monkeypatch):
    """It started the robot falling backwards (2026-10-01)."""
    t, sent = g1_transport(monkeypatch)
    res = t.execute("arm_action", {"action": "high_five"})
    assert not res["ok"] and sent == []


def test_safe_mode_still_holds_back_what_drops_the_g1():
    import g1_commands
    assert {"damp", "zero_torque", "dance"} <= g1_commands.DANGEROUS_SKILLS


def test_the_pad_marks_arm_actions_one_by_one():
    """arm_action is one block on the pad but one relay verb per action."""
    vals = svc._relay_allowed_values("g1", {"verbs": ["stop_move", "action_hug"]})
    assert vals["arm_action"]["action"] == ["hug"]
    assert "arm_action" in svc._relay_allowed_skills("g1", {"verbs": ["action_hug"]})
    assert "arm_action" not in svc._relay_allowed_skills("g1", {"verbs": ["stop_move"]})


def test_a_7404_from_the_relay_tells_the_operator_to_switch_to_run():
    hint = svc._relay_error_hint("g1", {"ok": False, "reply": "err action_hug 7404"})
    assert "Run" in hint
    assert svc._relay_error_hint("g1", {"ok": True, "reply": "ok action_hug 0"}) == ""


def test_the_pad_offers_the_walk_of_the_waist_the_relay_reports():
    """Which walk is valid depends on the app's waist lock, which only the relay knows (its
    G1_WAIST_LOCK). The executor maps both; the relay's /health verbs pick the one shown."""
    locked = ["stop_move", "stand_up", "start", "wave_hand"]
    allowed = svc._relay_allowed_skills("g1", {"verbs": locked})
    assert "start" in allowed and "walk_waist" not in allowed


def test_g1_stop_is_a_stop_move(monkeypatch):
    t, sent = g1_transport(monkeypatch)
    assert t.execute("stop", {})["ok"]
    assert sent == ["stop_move"]


def test_the_go2_table_is_unchanged(monkeypatch):
    t = svc.RelayTransport("go2", "http://stub", "tok", dry_run=True)
    assert t.VERB_FOR_SKILL["stand_down"] == "stand_down"
    assert "walk_waist" not in t.VERB_FOR_SKILL


# These clear EVERY token variable first: the executor loads its real .env at import, so a
# machine with a G1 token configured would otherwise decide the result — and the assertion
# would print that real token. Compared with `is True` so a failure never shows a value.
TOKEN_VARS = ("G1_RELAY_TOKEN", "G1_RELAY_TOKEN_FILE", "GO2_RELAY_TOKEN",
              "GO2_RELAY_TOKEN_FILE", "RELAY_TOKEN", "RELAY_TOKEN_FILE")


def _clean(monkeypatch):
    for v in TOKEN_VARS:
        monkeypatch.delenv(v, raising=False)


def test_a_robot_with_its_own_token_uses_it(monkeypatch, tmp_path):
    _clean(monkeypatch)
    f = tmp_path / "g1_token"
    f.write_text("g1-secret\n")
    monkeypatch.setenv("G1_RELAY_TOKEN_FILE", str(f))
    monkeypatch.setenv("RELAY_TOKEN", "shared")
    assert (svc._relay_token("g1") == "g1-secret") is True
    assert (svc._relay_token("go2") == "shared") is True     # the Go2 is untouched


def test_the_per_robot_value_beats_its_file(monkeypatch, tmp_path):
    _clean(monkeypatch)
    f = tmp_path / "g1_token"
    f.write_text("from-file\n")
    monkeypatch.setenv("G1_RELAY_TOKEN", "from-env")
    monkeypatch.setenv("G1_RELAY_TOKEN_FILE", str(f))
    assert (svc._relay_token("g1") == "from-env") is True


def test_without_a_per_robot_token_the_shared_one_is_used(monkeypatch):
    _clean(monkeypatch)
    monkeypatch.setenv("RELAY_TOKEN", "shared")
    assert (svc._relay_token("g1") == "shared") is True
