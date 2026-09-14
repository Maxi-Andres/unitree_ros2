"""The lag watchdog in `RtspStreamSource` — the guard on the rule that cost the most here.

THE RULE: a *pull* reader slower than its source does not settle at a lower frame rate. The
backlog upstream grows without bound and the latency climbs forever — it does not plateau.
That is not theory: with the bridge reading RTSP the drive view reached ~8 SECONDS and kept
going, and reverting the source URL was what stopped it. The failure is invisible in the
first seconds and only obvious after minutes, which is precisely how it shipped.

WHAT THE WATCHDOG DOES. It compares two ELAPSED times, never two clocks:

    lag = (our monotonic seconds since the first frame)
        - (the stream's own presentation seconds over those same frames)

Both series start at the same frame, so the unknown offset between the two clocks cancels
and only their rates are compared. No NTP, no agreement with the robot, nothing to sync.

Measured against the live stream on 2026-09-14 this reads +0.01 s and holds there, because
mediamtx runs on the same machine as the bridge — the RTSP hop is loopback and never crosses
the robot link. These tests exist for the day that stops being true: a source moved off-box,
a slower workstation, a second consumer. They assert the watchdog SPEAKS UP then.

Nothing here touches a socket, a robot or OpenCV: the source is built without running
`__init__` (which would open a stream and start a reader thread). See `conftest.py`.
"""
from __future__ import annotations

import camera_sources
import pytest


def approx(value, expected, tol=1e-6):
    """`pytest.approx` is deliberately NOT used here.

    It probes `sys.modules["numpy"]`, which conftest stubs out as an empty module, and dies
    with `AttributeError: module 'numpy' has no attribute 'isscalar'`. That is the stub
    working as designed — see conftest's docstring — so the fix is to not need numpy, not to
    grow the stub."""
    return abs(value - expected) <= tol


class _Log:
    """Captures the warnings the source would have logged."""

    def __init__(self):
        self.warnings: list[str] = []

    def info(self, msg):    # pragma: no cover - the source only info()s on construction
        pass

    def warn(self, msg):
        self.warnings.append(msg)


@pytest.fixture
def source():
    """An `RtspStreamSource` with no stream behind it and no reader thread running."""
    src = object.__new__(camera_sources.RtspStreamSource)
    src._fps = 15.0
    src._res = "native"
    src._quality = 75
    src._passed = 0.0
    src._lag = 0.0
    src._lag_peak = 0.0
    src._last_lag_warn = 0.0
    src._log = _Log()
    return src


def _feed(src, seconds, lag_per_s, step=1.0, monkeypatch=None):
    """Play `seconds` of stream at a fixed lag accumulation rate.

    lag_per_s = 0.0 is a reader keeping up; 0.5 is one consuming two seconds of wall time
    for every one second of video, i.e. falling a second behind every two.
    """
    t = 0.0
    while t < seconds:
        t += step
        src._track_lag(elapsed_s=t, stream_s=t * (1.0 - lag_per_s))


def test_keeping_up_stays_silent(source, monkeypatch):
    """A reader at the source's rate must not warn — that is the normal, measured case.

    If this ever fails the watchdog is crying wolf, and a warning nobody believes is worse
    than no warning: the ~8 s incident was missed because nothing said anything at all.
    """
    monkeypatch.setattr(camera_sources.time, "monotonic", lambda: 1e6)
    _feed(source, seconds=600, lag_per_s=0.0)
    assert approx(source._lag, 0.0, tol=1e-9)
    assert source._log.warnings == []


def test_falling_behind_is_reported(source, monkeypatch):
    """A reader consuming slower than its source must say so, and say how far behind.

    The number matters as much as the alarm: "behind" is a shrug, "4.5 s behind" tells the
    operator the drive view cannot be steered by.
    """
    clock = [1e6]
    monkeypatch.setattr(camera_sources.time, "monotonic", lambda: clock[0])
    for t in range(1, 81):
        clock[0] += 1.0
        source._track_lag(elapsed_s=float(t), stream_s=t * 0.5)   # half speed
    assert approx(source._lag, 40.0)
    assert source._log.warnings, "a reader 40 s behind its source warned nobody"
    # The first warning fires the moment the lag CROSSES the threshold, so it reports 1.0s
    # -- the crossing is the useful alarm, not some later round number.
    # Anchored on "is " because "31.0s behind" CONTAINS "1.0s behind" -- an unanchored
    # substring check here passes on the wrong message.
    assert "is 1.0s behind" in source._log.warnings[0]
    # And a later one must carry the GROWN number. This is the whole point: a bounded
    # problem would keep reporting ~1 s, so a rising figure is what distinguishes "slow"
    # from "falling behind forever", which is the distinction that matters.
    assert len(source._log.warnings) > 1, "the lag grew for 80 s and only one line said so"
    assert "is 31.0s behind" in source._log.warnings[-1]


def test_warning_is_throttled(source, monkeypatch):
    """One warning per `_LAG_WARN_EVERY_S`, not one per frame.

    At 14 fps an unthrottled warning is 14 lines a second for as long as the fault lasts —
    which buries the reconnects and stream-ended lines that say WHY it went bad.
    """
    clock = [1e6]
    monkeypatch.setattr(camera_sources.time, "monotonic", lambda: clock[0])
    # 120 s of stream at 14 fps, badly behind the whole way.
    for i in range(1, 120 * 14 + 1):
        clock[0] += 1.0 / 14.0
        source._track_lag(elapsed_s=i / 14.0, stream_s=i / 14.0 * 0.5)
    expected = 120 / camera_sources.RtspStreamSource._LAG_WARN_EVERY_S
    assert len(source._log.warnings) <= expected + 1
    assert len(source._log.warnings) >= expected - 1


def test_peak_survives_a_recovery(source, monkeypatch):
    """The peak must outlive the fault, because triage always arrives after it.

    Current lag answers "is it bad now"; the peak answers "was it ever", which is the only
    question you can still ask once the stream has reconnected and looks fine.
    """
    monkeypatch.setattr(camera_sources.time, "monotonic", lambda: 1e6)
    source._track_lag(elapsed_s=10.0, stream_s=6.0)    # 4 s behind
    source._track_lag(elapsed_s=20.0, stream_s=20.0)   # caught up
    assert approx(source._lag, 0.0)
    assert approx(source._lag_peak, 4.0)


def test_params_expose_the_lag(source):
    """`get_params()` feeds the bridge's /status, so the lag is visible without log-diving.

    This is the number that decides whether the drive view can be trusted, and an operator
    about to steer a robot should not have to exec into a container to find it.
    """
    params = source.get_params()
    assert set(params) >= {"fps", "resolution", "quality", "lag_s", "lag_peak_s"}
    assert params["fps"] == 15.0          # the inherited fields must survive the override
    assert params["quality"] == 75
