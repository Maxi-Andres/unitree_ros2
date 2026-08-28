"""JPEG framing in `HttpStreamSource._read_stream()` — the PC side of the MJPEG scanner.

Every test here names the defect it catches. Nothing touches DDS, ROS2, a socket or a robot:
`urlopen` is faked and the source is built without running `__init__` (which would start a
reader thread). Runs with the robot powered off, and with neither cv2 nor rclpy installed —
see `conftest.py`.

Why this file exists: the same ~20-line SOI/EOI scanner is duplicated in
`robot-ecosystem/robot-video-pipeline/robot/mjpeg_server.py::pump`, and **neither copy has a
buffer ceiling**. The duplication is deliberate — the two run on different machines and the
boundary forbids sharing a module — so the only way to keep them honest is a test on each
side. This is the PC side; the robot side has the twin of this file.

One test is marked `xfail(strict=True)`: it asserts the CORRECT behavior for a defect that is
still open. Strict means that when the defect is fixed the test starts passing and pytest
FAILS on the unexpected pass — telling you to delete the marker. Fix the code, delete the
marker; do not delete the test.
"""
from __future__ import annotations

import threading
import tracemalloc
import urllib.request

import camera_sources
import pytest

SOI = b"\xff\xd8"
EOI = b"\xff\xd9"


# --------------------------------------------------------------------------- #
# Harness
# --------------------------------------------------------------------------- #
class _FakeResponse:
    """Stands in for the object `urlopen` returns: a context manager with `.read(n)`.

    `_read_stream` treats an empty read as a dropped connection and raises `OSError`, which
    the caller (`_run`) turns into a backoff-and-retry. The fake reproduces that, and the
    tests catch it — reaching EOF is the normal way these runs end.
    """

    def __init__(self, chunks):
        self._chunks = list(chunks)

    def read(self, _size):
        return self._chunks.pop(0) if self._chunks else b""

    def __enter__(self):
        return self

    def __exit__(self, *_exc):
        return False


def _source(monkeypatch, chunks, on_frame, fps_gate=False):
    """An `HttpStreamSource` wired to `chunks`, built without starting its thread.

    `fps_gate=False` bypasses `_due`. Framing and rate-limiting are separate concerns, and
    `_due` is gated on wall-clock time, so it would drop the second frame of any two-frame
    test for reasons that have nothing to do with framing. The gate's own behavior is
    covered by `test_the_fps_gate_drops_frames`, the one caller that passes True.
    """
    src = object.__new__(camera_sources.HttpStreamSource)
    src._on_frame = on_frame
    src._url = "http://stub/stream"
    src._log = None
    src._stop = threading.Event()
    src._res = "native"   # with quality 0 this is the passthrough path: no cv2
    src._quality = 0
    src._fps = 1.0
    src._last = 0.0
    if not fps_gate:
        monkeypatch.setattr(src, "_due", lambda: True)
    monkeypatch.setattr(urllib.request, "urlopen",
                        lambda _req, timeout=None: _FakeResponse(chunks))
    return src


def _run(monkeypatch, chunks):
    """Drive `_read_stream` over `chunks` and return the frames it handed on, in order."""
    got = []
    src = _source(monkeypatch, chunks, got.append)
    with pytest.raises(OSError, match="stream closed by peer"):
        src._read_stream()
    return got


def _frame(payload: bytes = b"body") -> bytes:
    return SOI + payload + EOI


# --------------------------------------------------------------------------- #
# Correctness of the scanner
# --------------------------------------------------------------------------- #
def test_a_whole_frame_in_one_chunk_is_forwarded(monkeypatch):
    assert _run(monkeypatch, [_frame(b"one")]) == [_frame(b"one")]


def test_a_frame_split_across_chunks_is_reassembled(monkeypatch):
    """The defect: forwarding a truncated frame when a read lands mid-JPEG.

    Frames here are ~200 KB and reads are 8 KB, so this is the normal case, not the edge one.
    """
    whole = _frame(b"abcdefghij")
    assert _run(monkeypatch, [whole[:4], whole[4:9], whole[9:]]) == [whole]


def test_an_soi_marker_straddling_two_chunks_is_not_lost(monkeypatch):
    """The defect: dropping a frame whose 2-byte start marker was split by a read.

    This is what the one-byte `buf = buf[-1:]` tail retention exists for.
    """
    assert _run(monkeypatch, [b"junk\xff", b"\xd8payload" + EOI]) == [SOI + b"payload" + EOI]


def test_an_eoi_marker_straddling_two_chunks_is_not_lost(monkeypatch):
    assert _run(monkeypatch, [SOI + b"payload\xff", b"\xd9"]) == [SOI + b"payload" + EOI]


def test_multipart_headers_before_the_soi_are_discarded(monkeypatch):
    """The defect: handing multipart preamble bytes to the consumer as part of the JPEG.

    This is the whole reason the scanner looks for markers instead of parsing boundaries:
    the same code has to survive Frigate, mediamtx and go2_jpeg_stream's raw concatenation.
    """
    preamble = b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: 9\r\n\r\n"
    assert _run(monkeypatch, [preamble + _frame(b"x")]) == [_frame(b"x")]


def test_two_frames_in_one_chunk_are_both_forwarded(monkeypatch):
    """The defect: an inner loop that stops after one frame, halving the frame rate."""
    a, b = _frame(b"first"), _frame(b"second")
    assert _run(monkeypatch, [a + b]) == [a, b]


def test_bytes_between_two_frames_are_discarded(monkeypatch):
    a, b = _frame(b"first"), _frame(b"second")
    assert _run(monkeypatch, [a + b"\r\n--frame\r\n" + b]) == [a, b]


def test_a_chunk_with_no_markers_forwards_nothing(monkeypatch):
    assert _run(monkeypatch, [b"no markers here", b"still none"]) == []


def test_a_trailing_partial_frame_is_not_forwarded(monkeypatch):
    """The defect: flushing the buffer when the connection drops.

    A JPEG with no EOI is a truncated image; forwarding it is worse than dropping it.
    """
    assert _run(monkeypatch, [_frame(b"good"), SOI + b"truncated"]) == [_frame(b"good")]


def test_an_empty_read_is_reported_as_a_dropped_connection(monkeypatch):
    """The defect: treating EOF as end-of-stream and returning quietly.

    The raise is load-bearing: `_run` catches it and retries with backoff, which is how the
    bridge recovers when Frigate restarts. A quiet return would end the reader thread and
    the video would never come back without restarting the bridge.
    """
    src = _source(monkeypatch, [], lambda _f: None)
    with pytest.raises(OSError, match="stream closed by peer"):
        src._read_stream()


def test_the_fps_gate_drops_frames(monkeypatch):
    """The fps cap is real: at 1 fps, two frames arriving together become one.

    Guards the invariant the other tests bypass — if `_due` ever stopped gating, every test
    above would still pass while the bridge silently forwarded at the source's full rate.
    """
    got = []
    src = _source(monkeypatch, [_frame(b"a") + _frame(b"b")], got.append, fps_gate=True)
    with pytest.raises(OSError):
        src._read_stream()
    assert len(got) == 1, "at 1 fps the second frame of the same millisecond must be dropped"


# --------------------------------------------------------------------------- #
# The missing buffer ceiling — open defect
# --------------------------------------------------------------------------- #
@pytest.mark.xfail(
    strict=True,
    reason="no buffer ceiling: on an SOI with no EOI, `buf = buf[start:]` accumulates every "
    "subsequent chunk without bound. A camera service that dies mid-frame leaves exactly "
    "that on the wire.",
)
def test_an_soi_with_no_eoi_does_not_grow_the_buffer_without_bound(monkeypatch):
    """The defect: unbounded memory growth on a stream that starts a frame and never ends it.

    Measured with `tracemalloc` rather than by reaching into the method's locals: feed 5 MB
    after an unterminated SOI and assert the peak stays small. With a ceiling the buffer is
    bounded; without one the peak tracks the whole 5 MB.
    """
    chunk = b"\x00" * 65536
    chunks = [SOI] + [chunk] * 80  # 5 MB, none of it a valid frame

    tracemalloc.start()
    try:
        _run(monkeypatch, chunks)
        _current, peak = tracemalloc.get_traced_memory()
    finally:
        tracemalloc.stop()

    assert peak < 2 * 1024 * 1024, (
        f"the scanner held {peak / 1024 / 1024:.1f} MB of a 5 MB unterminated frame; it "
        "needs a ceiling that abandons the frame and resyncs on the next SOI"
    )
