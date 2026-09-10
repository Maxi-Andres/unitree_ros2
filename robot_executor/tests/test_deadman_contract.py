"""The dead-man contract that ALL THREE transports must satisfy.

Every test here names the defect it catches. Nothing touches DDS, ROS2, rclpy, a socket or a
robot: each transport is built with `dry_run=True` (which skips `_init_ros()` / the network)
and its one outbound seam is replaced by a recorder.

WHY A SHARED SUITE INSTEAD OF THREE. The dead-man is implemented three times —
`Go2Ros2Transport`, `G1Ros2Transport`, `RelayTransport` — and the loop, the deadline, the
lock and the thread are the same code in all three. Only the two publish primitives differ:

    | transport | "keep moving"                                  | "halt"                        |
    |-----------|------------------------------------------------|-------------------------------|
    | Go2 ROS2  | _publish(MOVE_API_ID, {x,y,z})                 | _publish(STOPMOVE_API_ID)     |
    | G1 ROS2   | _set_velocity(vx,vy,vyaw, _PUB_DUR)            | _set_velocity(0,0,0, 1.0)     |
    | relay     | _post({"verb":"move", ...})                    | _post({"verb":"stop_move"})   |

Writing three parallel suites would trade one duplication for another. Parametrising over the
three states the contract ONCE, and that statement is exactly the specification the planned
template-method base class has to meet: after the refactor this file must stay green without
edits, which is what makes the refactor provably behaviour-preserving.

The per-transport `_Seam` adapters below are the *measure* of the duplication. When the base
class lands, they collapse to one.

THE TWO RELAY XFAILS ARE GONE — FIXED 2026-09-10. Parametrising surfaced that the three
copies had drifted apart: the relay's loop was NOT a copy of the other two. Both defects
were confirmed against the real robot by sniffing tcp/8092 during teleop (154 of 155 moves
preceded by an injected halt), then fixed by copying the `if reached_deadline` guard and the
step clamp from the ROS2 transports. `strict=True` did its job — the markers turned into
hard failures the moment the code was right, and were removed. All three transports now
pass every row. Analysis: robot-splunk-docs/FRENO-INYECTADO.md
"""
from __future__ import annotations

import sys
import threading
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import g1_commands
import go2_commands
import robot_executor_service as svc


# --------------------------------------------------------------------------- #
# Harness: one adapter per transport, hiding only WHAT a move/halt looks like
# --------------------------------------------------------------------------- #
class _Seam:
    """Records outbound traffic and classifies each event as a move or a halt."""

    def __init__(self):
        self._lock = threading.Lock()
        self.events: list[str] = []          # "move" | "halt" | "other"

    def _add(self, kind: str) -> None:
        with self._lock:
            self.events.append(kind)

    def snapshot(self) -> list[str]:
        with self._lock:
            return list(self.events)

    def count(self, kind: str) -> int:
        return self.snapshot().count(kind)

    def wait_for(self, kind: str, timeout: float = 2.0) -> bool:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if kind in self.snapshot():
                return True
            time.sleep(0.005)
        return False

    def wait_until_at_least(self, kind: str, n: int, timeout: float = 2.0) -> bool:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if self.count(kind) >= n:
                return True
            time.sleep(0.005)
        return False


def _go2_seam():
    t = svc.Go2Ros2Transport(dry_run=True)
    seam = _Seam()

    def publish(api_id, parameter=None):
        if api_id == go2_commands.MOVE_API_ID:
            seam._add("move")
        elif api_id == go2_commands.STOPMOVE_API_ID:
            seam._add("halt")
        else:
            seam._add("other")

    t._publish = publish
    return t, seam


def _g1_seam():
    t = svc.G1Ros2Transport(dry_run=True)
    seam = _Seam()

    # The G1 has no distinct StopMove api_id: it halts by commanding zero velocity. So the
    # classification is on the VALUES, not on a message type — a difference the base class
    # will have to keep, because it is the robot's protocol, not a style choice.
    def set_velocity(vx, vy, vyaw, duration):
        moving = any(abs(v) > 1e-9 for v in (vx, vy, vyaw))
        seam._add("move" if moving else "halt")

    t._set_velocity = set_velocity
    return t, seam


def _relay_seam():
    t = svc.RelayTransport("go2", "http://stub", "tok", dry_run=True)
    seam = _Seam()

    def post(body):
        verb = body.get("verb")
        seam._add("move" if verb == "move" else "halt" if verb == "stop_move" else "other")
        return {"ok": True, "reply": "ok stub"}

    t._post = post
    return t, seam


TRANSPORTS = [
    pytest.param(_go2_seam, id="go2-ros2"),
    pytest.param(_g1_seam, id="g1-ros2"),
    pytest.param(_relay_seam, id="relay"),
]


@pytest.fixture(autouse=True)
def fast(monkeypatch):
    """Compress the production timings. These are module globals read at call time.

    `RelayTransport._REFRESH_S` is a class attribute (0.4 s in production) rather than a
    global, so it needs its own patch or every relay test would wait nearly half a second
    between publishes.
    """
    monkeypatch.setattr(svc, "MOVE_RATE_HZ", 100.0)   # 10 ms per publish
    monkeypatch.setattr(svc, "DEFAULT_STEP_S", 0.05)
    monkeypatch.setattr(svc, "MAX_STEP_S", 0.20)
    monkeypatch.setattr(svc.RelayTransport, "_REFRESH_S", 0.01)
    monkeypatch.setattr(svc.G1Ros2Transport, "_PUB_DUR", 0.05)


@pytest.fixture
def build(request):
    """Yields a factory, and guarantees no move loop survives into the next test."""
    made = []

    def _factory(make):
        t, seam = make()
        made.append(t)
        return t, seam

    yield _factory
    for t in made:
        with t._move_lock:
            t._stop_move_loop()


# --------------------------------------------------------------------------- #
# The contract every transport must meet
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("make", TRANSPORTS)
def test_a_bounded_move_halts_itself_without_anyone_asking(build, make):
    """THE reason the dead-man exists: a client that goes quiet must not leave the robot
    walking. If the deadline branch breaks, the robot walks until it hits something."""
    t, seam = build(make)
    t.execute("move", {"vx": 0.3, "duration_s": 0.05, "continuous": False})
    assert seam.wait_for("halt"), (
        f"no halt within the timeout; sent {seam.snapshot()}"
    )


@pytest.mark.parametrize("make", TRANSPORTS)
def test_a_bounded_move_moves_before_it_halts(build, make):
    """The defect: a deadline so eager the robot never actually moves."""
    t, seam = build(make)
    t.execute("move", {"vx": 0.3, "duration_s": 0.05, "continuous": False})
    assert seam.wait_for("halt")
    ev = seam.snapshot()
    assert ev.index("move") < ev.index("halt"), f"move must precede halt; got {ev}"


@pytest.mark.parametrize("make", TRANSPORTS)
def test_the_halt_is_the_last_thing_sent(build, make):
    """The defect: a move racing out after the halt, restarting the robot."""
    t, seam = build(make)
    t.execute("move", {"vx": 0.3, "duration_s": 0.05, "continuous": False})
    assert seam.wait_for("halt")
    time.sleep(0.08)
    assert seam.snapshot()[-1] == "halt", f"something followed the halt: {seam.snapshot()}"


@pytest.mark.parametrize("make", TRANSPORTS)
def test_a_continuous_move_never_halts_itself(build, make):
    """`continuous=True` means no deadline. A pad holding a stick needs this.

    (That `continuous` DEFAULTS to True is the separate P0 — see
    test_move_without_continuous_is_bounded in test_command_resolution.py.)
    """
    t, seam = build(make)
    t.execute("move", {"vx": 0.3, "continuous": True})
    assert seam.wait_until_at_least("move", 3), f"the loop is not sending: {seam.snapshot()}"
    # Wait past the deadline a BOUNDED move would have had; asserting straight after the
    # first few moves is a race that lets a broken `if not continuous:` pass.
    time.sleep(svc.DEFAULT_STEP_S * 2 + 0.05)
    assert seam.count("halt") == 0, (
        f"a continuous move must not halt itself; sent {seam.snapshot()}"
    )


@pytest.mark.parametrize("make", TRANSPORTS)
def test_an_explicit_stop_halts_a_continuous_move(build, make):
    """With no deadline, `stop` is the ONLY way out. If this breaks the robot is unstoppable
    through the normal API."""
    t, seam = build(make)
    t.execute("move", {"vx": 0.3, "continuous": True})
    assert seam.wait_until_at_least("move", 2)
    t.execute("stop", {})
    assert seam.snapshot()[-1] == "halt", f"stop did not halt: {seam.snapshot()}"
    before = seam.count("move")
    time.sleep(0.08)
    assert seam.count("move") == before, "the loop kept sending after stop"


@pytest.mark.parametrize("make", TRANSPORTS)
def test_the_deadline_still_fires_after_the_refreshes_stop(build, make):
    """Superseding must not disable the dead-man. A refresh loop that silently cleared the
    deadline would leave the robot walking forever once the client froze."""
    t, seam = build(make)
    for _ in range(3):
        t.execute("move", {"vx": 0.3, "duration_s": 0.05, "continuous": False})
        time.sleep(0.01)
    assert seam.wait_for("halt"), (
        f"the dead-man did not fire after the refreshes stopped: {seam.snapshot()}"
    )


@pytest.mark.parametrize("make", TRANSPORTS)
def test_an_unsupported_skill_is_refused_without_sending_anything(build, make):
    t, seam = build(make)
    out = t.execute("teleport", {})
    assert out["ok"] is False
    assert seam.snapshot() == [], f"an unsupported skill sent something: {seam.snapshot()}"


# --------------------------------------------------------------------------- #
# Where the three copies HAD drifted — fixed 2026-09-10, kept as regression rows
#
# These two were the concrete cost of the duplication: the relay's loop was not a copy of
# the other two any more, and nobody noticed because nothing compared them. They ran as
# xfail(strict=True) until the guard and the clamp were copied over from the ROS2
# transports. They stay parametrised across all three so the drift cannot come back, and
# the planned template-method base class keeps them passing by construction.
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("make", [
    pytest.param(_go2_seam, id="go2-ros2"),
    pytest.param(_g1_seam, id="g1-ros2"),
    pytest.param(_relay_seam, id="relay"),
])
def test_a_new_move_supersedes_the_previous_one_without_injecting_a_halt(build, make):
    """The invariant that makes teleop smooth, and the one a refactor breaks first.

    Spelled out in `Go2Ros2Transport._run_move_loop`'s docstring: "StopMove is published ONLY
    when the deadline is actually reached — NOT when the loop is superseded by a newer move."
    A pad refreshing a short bounded move every tick must produce continuous motion.
    """
    t, seam = build(make)
    for _ in range(4):
        t.execute("move", {"vx": 0.3, "duration_s": 0.20, "continuous": False})
        time.sleep(0.02)   # well inside the 0.20 s deadline
    assert seam.count("halt") == 0, (
        f"a refresh injected a halt: {seam.snapshot()}"
    )
    assert seam.count("move") >= 2, f"the refreshes sent nothing: {seam.snapshot()}"


@pytest.mark.parametrize("make", [
    pytest.param(_go2_seam, id="go2-ros2"),
    pytest.param(_g1_seam, id="g1-ros2"),
    pytest.param(_relay_seam, id="relay"),
])
def test_an_absurd_duration_is_clamped_to_max_step(build, make):
    """The defect: trusting the caller's duration, so `duration_s: 60` is a de-facto
    continuous move the dead-man never ends.

    With MAX_STEP_S compressed to 0.20 s, a request for 60 s must still halt promptly.
    """
    t, seam = build(make)
    t.execute("move", {"vx": 0.3, "duration_s": 60.0, "continuous": False})
    assert seam.wait_for("halt", timeout=1.0), (
        f"a 60 s duration outlived MAX_STEP_S={svc.MAX_STEP_S}; sent {seam.snapshot()}"
    )


# --------------------------------------------------------------------------- #
# The wrong-key trap, on every transport
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("make", TRANSPORTS)
def test_the_field_is_duration_s_and_a_wrong_key_still_bounds_the_move(build, make):
    """`resolve()` reads `params["duration_s"]` — NOT `params["duration"]`.

    The real callers use `duration_s` (`ControlPage.tsx:145`, and the interpreter's skill
    schema in `command_common.py`). The first draft of these tests used the wrong key, so it
    silently exercised the default-step path and kept passing with the clamp deleted; found
    by mutation-testing, not by reading. Asserted here so nobody "fixes" it by quietly
    accepting both spellings — and, more importantly, so an unrecognised duration still
    yields a BOUNDED move via DEFAULT_STEP_S rather than an unbounded one.
    """
    t, seam = build(make)
    for mod in (go2_commands, g1_commands):
        assert mod.resolve("move", {"vx": 0.3, "duration": 999.0,
                                    "continuous": False})["duration"] is None, (
            f"{mod.__name__} honoured the wrong key"
        )
    t.execute("move", {"vx": 0.3, "duration": 999.0, "continuous": False})
    assert seam.wait_for("halt", timeout=1.0), (
        f"an unrecognised duration key produced an unbounded move: {seam.snapshot()}"
    )


@pytest.mark.parametrize("make", TRANSPORTS)
def test_a_zero_duration_falls_back_to_the_default_step(build, make):
    """The defect: `duration_s: 0` yielding a deadline already in the past, so the robot gets
    a halt and never moves — a command that silently does nothing."""
    t, seam = build(make)
    t.execute("move", {"vx": 0.3, "duration_s": 0, "continuous": False})
    assert seam.wait_for("move"), f"nothing moved: {seam.snapshot()}"
    assert seam.wait_for("halt"), f"never halted: {seam.snapshot()}"


@pytest.mark.parametrize("make", TRANSPORTS)
def test_stopping_the_loop_leaves_no_thread_behind(build, make):
    """The defect: the executor exiting while the robot is still being told to move.

    Whatever teardown a transport does about ROS2 or HTTP, it must not leave a live loop.
    """
    t, seam = build(make)
    t.execute("move", {"vx": 0.3, "continuous": True})
    assert seam.wait_until_at_least("move", 2)
    with t._move_lock:
        t._stop_move_loop()
    before = seam.count("move")
    time.sleep(0.08)
    assert seam.count("move") == before, "the move loop survived _stop_move_loop()"
    assert t._move_thread is None


# --------------------------------------------------------------------------- #
# Transport-specific: the reply shape the UI reads
# --------------------------------------------------------------------------- #
def test_go2_stop_reports_the_stopmove_api_id(build):
    """Guards the reply the UI shows, so 'stop' can never look like a no-op.

    Not parametrised: the api_id in the reply is the Go2's sport protocol. The G1 answers a
    different shape and the relay answers the relay's — asserting one shape across all three
    would be asserting a fiction.
    """
    t, seam = build(_go2_seam)
    out = t.execute("stop", {})
    assert out["ok"] is True
    assert out["api_id"] == go2_commands.STOPMOVE_API_ID
    assert seam.snapshot() == ["halt"]
