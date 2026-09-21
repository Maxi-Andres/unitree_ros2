"""`h264_relay.AuReader` — the HQ half of the drive branch's framing.

SECOND COPY BY NECESSITY. The robot has the same parser (`mjpeg_server.AuReader`, guarded by
`robot-video-pipeline/tests/test_h264_branch.py`); the network boundary forbids sharing a
module between the two machines, so each side carries its own and a test on each keeps them
honest. **If you fix one, check the other.**

WHAT BREAKS SILENTLY HERE, and why each test exists:

* An access unit handed to a decoder with one byte missing is not a worse picture, it is a
  stream the decoder refuses — and the error points nowhere near the framing.
* `X-Capture` is the robot's clock when the camera produced the frame. Parse it wrongly and
  every latency number built on this path is wrong while looking plausible, which is worse
  than not having it.

Nothing here opens a socket.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import h264_relay

B = b"--ThisRandomString"
AU = bytes(range(256)) * 4          # every byte value, so any truncation shows


def part(payload=AU, capture=b"1790001782.123456", ctype=b"video/x-h264"):
    head = B + b"\r\nContent-Type: " + ctype + b"\r\n"
    if capture is not None:
        head += b"X-Capture: " + capture + b"\r\n"
    return head + b"Content-Length: " + str(len(payload)).encode() + b"\r\n\r\n" + payload \
        + b"\r\n"


def test_a_whole_part_yields_its_bytes_and_its_capture_time():
    r = h264_relay.AuReader()
    (capture, au), = r.feed(part())
    assert au == AU
    assert capture == 1790001782.123456


def test_a_part_split_across_chunks_is_reassembled():
    """The body arrives in pieces off a socket; returning early hands the decoder a truncated
    access unit, which it rejects outright."""
    r = h264_relay.AuReader()
    blob = part()
    assert r.feed(blob[:30]) == []
    assert r.feed(blob[30:200]) == []
    (capture, au), = r.feed(blob[200:])
    assert au == AU and capture == 1790001782.123456


def test_two_parts_in_one_chunk_come_out_in_order():
    """The relay forwards only the LAST of a batch, so the order is what decides which frame
    the operator sees."""
    r = h264_relay.AuReader()
    got = r.feed(part(capture=b"1.0") + part(capture=b"2.0"))
    assert [c for c, _ in got] == [1.0, 2.0]


def test_a_missing_capture_header_is_zero_and_not_a_crash():
    """An un-instrumented robot (STAMP off, or an older build) must still drive. Losing the
    latency measurement is acceptable; losing the picture is not."""
    r = h264_relay.AuReader()
    (capture, au), = r.feed(part(capture=None))
    assert capture == 0.0
    assert au == AU


def test_a_malformed_capture_header_is_zero_and_not_a_crash():
    r = h264_relay.AuReader()
    (capture, au), = r.feed(part(capture=b"not-a-number"))
    assert capture == 0.0
    assert au == AU


def test_the_capture_time_of_one_part_does_not_leak_into_the_next():
    """Each part carries its own. Reading the newest header in the buffer rather than the one
    belonging to THIS part would pair every frame with the wrong instant — a latency graph
    that looks fine and is wrong."""
    r = h264_relay.AuReader()
    got = r.feed(part(capture=b"10.5") + part(capture=None) + part(capture=b"30.25"))
    assert [c for c, _ in got] == [10.5, 0.0, 30.25]


def test_a_payload_that_never_arrives_does_not_grow_the_buffer_without_bound():
    cap = h264_relay.AuReader._MAX_PART
    r = h264_relay.AuReader()
    assert r.feed(B + b"\r\nContent-Length: 999999999\r\n\r\n") == []
    for _ in range(40):
        assert r.feed(b"x" * 100_000) == []
    assert len(r._buf) <= cap + 100_000, f"buffer grew to {len(r._buf)} with a cap of {cap}"


def test_garbage_with_no_header_is_dropped_and_the_stream_recovers():
    r = h264_relay.AuReader()
    for _ in range(60):
        r.feed(b"\x00\xff" * 100_000)
    assert len(r._buf) <= h264_relay.AuReader._MAX_PART
    (_, au), = r.feed(part())
    assert au == AU
