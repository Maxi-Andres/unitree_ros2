"""The dead-man loop — the mechanism that stops the robot when nobody is talking to it.

Every test here names the defect it catches. **Nothing touches DDS, ROS2, rclpy or a robot.**
`robot_executor_service` imports with no third-party dependency at all (`rclpy` is imported
lazily inside functions, never at module level) and starts no threads on import, and
`Go2Ros2Transport(dry_run=True)` skips `_init_ros()` entirely. So the whole file runs with the
robot powered off, in well under a second.

WHY THIS FILE EXISTS. `robot_executor_service.py` is 781 statements and was at **0%
coverage** — the tests only ever exercised the pure `resolve()` layer in
`{go2,g1}_commands.py`. That zero was not a technical barrier, it was an oversight: this file
needed no new machinery to write. It matters because the dead-man is the code that guarantees
a frozen or crashed client cannot leave the robot walking, and it is implemented **three
times** (Go2 ROS2, G1 ROS2, relay) with ~70% duplication. Refactoring it into a template
method was deferred as "needs the robot"; with these tests it does not.

HOW THE SEAM WORKS. `_publish(api_id, parameter)` is the single point where the transport
talks to the outside world. Replacing it with a recorder turns the whole loop into a pure
function of time, so every assertion below is about the *sequence of api_ids* the robot would
have received. `1008` is Move, `1003` is StopMove.

TIMING. `MOVE_RATE_HZ` and the step bounds are module globals read at call time, so the tests
compress them (100 Hz, 0.05 s steps) instead of sleeping for the production 2 s default.
Waits are polled with a deadline, never a bare `sleep` sized to the expected duration — that
is the classic flaky-test shape.
"""
from __future__ import annotations

import sys
import threading
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import go2_commands
import robot_executor_service as svc

MOVE = go2_commands.MOVE_API_ID        # 1008
STOP = go2_commands.STOPMOVE_API_ID    # 1003


# --------------------------------------------------------------------------- #
# Harness
# --------------------------------------------------------------------------- #
class _Recorder:
    """Collects what the transport would have published, thread-safely.

    The move loop runs on its own thread, so an unsynchronised list would make these tests
    flaky in a way that looks like a product bug.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self.calls: list[tuple[int, dict | None]] = []

    def __call__(self, api_id, parameter=None):
        with self._lock:
            self.calls.append((api_id, parameter))

    @property
    def ids(self) -> list[int]:
        with self._lock:
            return [api_id for api_id, _ in self.calls]

    def count(self, api_id: int) -> int:
        return self.ids.count(api_id)

    def wait_for(self, api_id: int, timeout: float = 2.0) -> bool:
        """Poll until `api_id` shows up. Returns False on timeout — never raises, so the
        caller's assertion is the one that reports the failure."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if api_id in self.ids:
                return True
            time.sleep(0.005)
        return False

    def wait_until_at_least(self, api_id: int, n: int, timeout: float = 2.0) -> bool:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if self.count(api_id) >= n:
                return True
            time.sleep(0.005)
        return False


@pytest.fixture
def fast(monkeypatch):
    """Compress the production timings so the suite runs in milliseconds, not seconds."""
    monkeypatch.setattr(svc, "MOVE_RATE_HZ", 100.0)   # 10 ms per publish
    monkeypatch.setattr(svc, "DEFAULT_STEP_S", 0.05)
    monkeypatch.setattr(svc, "MAX_STEP_S", 0.20)


@pytest.fixture
def go2(fast):
    """A Go2 transport with the outside world replaced by a recorder.

    `dry_run=True` is what makes this possible with no ROS2 installed: `__init__` skips
    `_init_ros()`, so no `unitree_api.msg` import and no rclpy context.
    """
    t = svc.Go2Ros2Transport(dry_run=True)
    rec = _Recorder()
    t._publish = rec
    try:
        yield t, rec
    finally:
        # Never leave a move loop running into the next test: it would publish into a dead
        # recorder and make an unrelated test flake.
        with t._move_lock:
            t._stop_move_loop()


# --------------------------------------------------------------------------- #
# The bounded move: the dead-man actually fires
# --------------------------------------------------------------------------- #
def test_a_bounded_move_stops_itself_without_anyone_asking(go2):
    """THE defect this whole file exists for: a bounded move that never stops.

    If the deadline branch breaks, the robot keeps receiving Move forever and walks until it
    hits something. That is the failure the dead-man exists to prevent.
    """
    t, rec = go2
    t.execute("move", {"vx": 0.3, "duration_s": 0.05, "continuous": False})
    assert rec.wait_for(STOP), (
        f"no StopMove within the timeout; published: {rec.ids}. A bounded move that never "
        "issues StopMove leaves the robot walking after the client goes quiet."
    )


def test_a_bounded_move_publishes_move_before_it_stops(go2):
    """The defect: a deadline so eager that the robot never actually moves."""
    t, rec = go2
    t.execute("move", {"vx": 0.3, "duration_s": 0.05, "continuous": False})
    assert rec.wait_for(STOP)
    assert rec.ids.index(MOVE) < rec.ids.index(STOP), (
        f"Move must precede StopMove; got {rec.ids}"
    )


def test_stopmove_is_the_last_thing_published(go2):
    """The defect: a Move racing out after the StopMove, restarting the robot.

    The loop publishes StopMove in a `finally`, so nothing may follow it.
    """
    t, rec = go2
    t.execute("move", {"vx": 0.3, "duration_s": 0.05, "continuous": False})
    assert rec.wait_for(STOP)
    time.sleep(0.08)  # give any stray publish time to land
    assert rec.ids[-1] == STOP, f"something was published after StopMove: {rec.ids}"


# --------------------------------------------------------------------------- #
# Continuous: no deadline, and therefore no self-stop
# --------------------------------------------------------------------------- #
def test_a_continuous_move_never_stops_itself(go2):
    """The documented behaviour of `continuous=True`: the loop has no deadline.

    This is the flip side of the P0. `continuous` defaulting to True is the defect (see
    test_move_without_continuous_is_bounded in test_command_resolution.py); `continuous=True`
    when the caller *asks* for it is correct, and must keep working — a teleop pad holding a
    stick needs it.
    """
    t, rec = go2
    t.execute("move", {"vx": 0.3, "continuous": True})
    assert rec.wait_until_at_least(MOVE, 3), f"the loop is not publishing: {rec.ids}"
    # Wait PAST the deadline a bounded move would have had. Asserting right after the first
    # few Moves is a race: mutation-testing this file showed that a broken `if not
    # continuous:` (deadline always set) still passed, because the assertion ran at ~30 ms
    # and the would-be deadline was at 50 ms.
    time.sleep(svc.DEFAULT_STEP_S * 2 + 0.05)
    assert rec.count(STOP) == 0, (
        f"a continuous move must not stop itself; published {rec.ids}"
    )


def test_an_explicit_stop_halts_a_continuous_move(go2):
    """The defect: a continuous move that cannot be stopped.

    With no deadline, `stop` is the ONLY way out. If this breaks, the robot is unstoppable
    through the normal API.
    """
    t, rec = go2
    t.execute("move", {"vx": 0.3, "continuous": True})
    assert rec.wait_until_at_least(MOVE, 2)
    t.execute("stop", {})
    assert rec.ids[-1] == STOP, f"stop did not halt the robot: {rec.ids}"
    before = rec.count(MOVE)
    time.sleep(0.08)
    assert rec.count(MOVE) == before, "the move loop kept publishing after stop"


# --------------------------------------------------------------------------- #
# Superseding: the subtle invariant that makes teleop smooth
# --------------------------------------------------------------------------- #
def test_a_new_move_supersedes_the_previous_one_without_injecting_a_stop(go2):
    """The defect: a StopMove between every teleop refresh, making the robot stutter.

    This is the invariant the docstring in `_run_move_loop` spells out and the one most
    likely to be broken by a well-meaning refactor: "StopMove is published ONLY when the
    deadline is actually reached — NOT when the loop is superseded by a newer move." A pad
    refreshing a short bounded move every tick must produce continuous motion, not
    move-stop-move-stop.
    """
    t, rec = go2
    for _ in range(4):
        t.execute("move", {"vx": 0.3, "duration_s": 0.20, "continuous": False})
        time.sleep(0.02)   # well inside the 0.20 s deadline
    assert rec.count(STOP) == 0, (
        f"a refresh injected a StopMove: {rec.ids}. Each refresh must cancel the previous "
        "loop silently, or teleop stutters."
    )
    assert rec.count(MOVE) >= 2, f"the refreshes published nothing: {rec.ids}"


def test_the_deadline_still_fires_after_the_refreshes_stop(go2):
    """The other half of the same invariant: superseding must not disable the dead-man.

    A refresh loop that silently cleared the deadline would look identical to the test above
    while leaving the robot walking forever once the client froze.
    """
    t, rec = go2
    for _ in range(3):
        t.execute("move", {"vx": 0.3, "duration_s": 0.05, "continuous": False})
        time.sleep(0.01)
    # Client "freezes" here: no more refreshes.
    assert rec.wait_for(STOP), (
        f"the dead-man did not fire after the refreshes stopped: {rec.ids}"
    )


# --------------------------------------------------------------------------- #
# The step bounds: a client cannot ask for an unbounded move
# --------------------------------------------------------------------------- #
def test_an_absurd_duration_is_clamped_to_max_step(go2, monkeypatch):
    """The defect: trusting the caller's duration, so `duration: 99999` is a de-facto
    continuous move that the dead-man never ends.

    The clamp is `max(0.1, min(step, MAX_STEP_S))`. With MAX_STEP_S compressed to 0.20 s, a
    request for 60 s must still stop.
    """
    t, rec = go2
    t.execute("move", {"vx": 0.3, "duration_s": 60.0, "continuous": False})
    assert rec.wait_for(STOP, timeout=1.0), (
        f"a 60 s duration outlived MAX_STEP_S={svc.MAX_STEP_S}; published {rec.ids}"
    )


def test_a_zero_duration_falls_back_to_the_default_step(go2):
    """The defect: `duration_s: 0` producing a deadline already in the past, so the robot gets
    a StopMove and never moves at all — a command that silently does nothing."""
    t, rec = go2
    t.execute("move", {"vx": 0.3, "duration_s": 0, "continuous": False})
    assert rec.wait_for(MOVE), f"nothing moved: {rec.ids}"
    assert rec.wait_for(STOP), f"never stopped: {rec.ids}"


def test_the_field_is_duration_s_and_a_wrong_key_still_bounds_the_move(go2):
    """The trap that made the first version of this file test nothing.

    `resolve()` reads `params["duration_s"]` — NOT `params["duration"]`. The real callers use
    `duration_s` (`ControlPage.tsx:145`, and the interpreter's skill schema in
    `command_common.py`). A test written with the wrong key silently exercised the
    default-step path instead of the clamp, and kept passing with the clamp deleted — found
    by mutation-testing this file, not by reading it.

    Two things are asserted. First, that the wrong key is ignored rather than honoured, so
    nobody "fixes" this by quietly accepting both spellings. Second — the part that matters
    for safety — that an unrecognised duration still yields a BOUNDED move via
    DEFAULT_STEP_S, never an unbounded one. Fail-safe, which is the right default here.
    """
    t, rec = go2
    resolved = go2_commands.resolve("move", {"vx": 0.3, "duration": 999.0,
                                            "continuous": False})
    assert resolved["duration"] is None, (
        f"the wrong key must not be honoured; resolve returned {resolved}"
    )
    t.execute("move", {"vx": 0.3, "duration": 999.0, "continuous": False})
    assert rec.wait_for(STOP, timeout=1.0), (
        f"an unrecognised duration key produced an unbounded move: {rec.ids}"
    )


# --------------------------------------------------------------------------- #
# execute() contract
# --------------------------------------------------------------------------- #
def test_an_unsupported_skill_is_refused_without_publishing(go2):
    """The defect: an unknown skill reaching the robot as some default api_id."""
    t, rec = go2
    out = t.execute("teleport", {})
    assert out["ok"] is False
    assert rec.ids == [], f"an unsupported skill published something: {rec.ids}"


def test_stop_reports_the_stopmove_api_id(go2):
    """Guards the reply the UI shows, so 'stop' can never look like a no-op."""
    t, rec = go2
    out = t.execute("stop", {})
    assert out["ok"] is True
    assert out["api_id"] == STOP
    assert rec.ids == [STOP]


def test_shutdown_leaves_no_move_loop_running(go2):
    """The defect: the executor exiting while the robot is still being told to move.

    Whatever `shutdown()` does about ROS2, it must not leave a live loop behind.
    """
    t, rec = go2
    t.execute("move", {"vx": 0.3, "continuous": True})
    assert rec.wait_until_at_least(MOVE, 2)
    with t._move_lock:
        t._stop_move_loop()
    before = rec.count(MOVE)
    time.sleep(0.08)
    assert rec.count(MOVE) == before, "the move loop survived _stop_move_loop()"
    assert t._move_thread is None
