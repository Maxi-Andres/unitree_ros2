"""`h264_relay.UdpReassembler` / `UdpWatchdog` — the HQ half of the drive branch over UDP.

WHY THE BRANCH EXISTS: over TCP one lost packet froze the whole drive view until it was
retransmitted — measured over Starlink 2026-09-23, 10 stalls of 250-916 ms in 150 s, all on
the link. See the block above `UdpReassembler` in h264_relay.py.

WHAT BREAKS SILENTLY HERE, and why each test exists:

* The datagram layout is a CONTRACT with `mjpeg_server.udp_packets` on the robot, which this
  tier cannot import (network boundary). `GOLDEN` is the same byte vector the robot's test
  produces — `robot-video-pipeline/tests/test_h264_udp.py`. Keep the two identical.
* A recovered fragment that is one byte off is a corrupt access unit, and the browser's
  decoder refuses the stream with an error that points nowhere near here.
* A frame that completes after a newer one was shown would step the picture BACKWARDS on the
  view the operator steers by.
* This is fed by the network: garbage, strays from an old lease and floods of half frames
  must neither raise nor grow memory.
* The watchdog decides between "UDP is blocked, use TCP" and "the link blinked, reconnect".
  Mixing them up either leaves the operator with no picture or throws UDP away after every
  outage.

Nothing here opens a socket.
"""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import h264_relay

# session 0x01020304, frame 7, b"ABCDEFGHIJ", capture 1790001782.5, payload 4, group 2.
# SAME VECTOR as the robot side's test — keep the two identical.
GOLDEN = [bytes.fromhex(h) for h in (
    "41560100020004000403020107000000000003000a0000000000a09d50acda4141424344",
    "41560100020004000403020107000000010003000a0000000000a09d50acda4145464748",
    "41560100020004000403020107000000020003000a0000000000a09d50acda41494a",
    "41560101020004000403020107000000000003000a0000000000a09d50acda410404040c",
    "41560101020004000403020107000000010003000a0000000000a09d50acda41494a",
)]
SESSION = 0x01020304
DATA, PARITY = GOLDEN[:3], GOLDEN[3:]


def packets(frame, au, session=SESSION, capture=1.0, payload=1200, group=8):
    """A test-side sender, written from the documented format, NOT copied from the robot:
    the golden vector is what ties it to the real one."""
    h = h264_relay.UDP_HEADER
    frags = [au[i:i + payload] for i in range(0, len(au), payload)]

    def head(kind, index):
        return h.pack(b"AV", 1, kind, group, payload, session, frame, index, len(frags),
                      len(au), capture)
    data = [head(0, i) + f for i, f in enumerate(frags)]
    parity = []
    for g in range(0, len(frags), group):
        members = frags[g:g + group]
        parity.append(head(1, g // group) + h264_relay._xor(members, max(map(len, members))))
    return data, parity


def feed_all(r, datagrams, now=0.0):
    out = []
    for d in datagrams:
        out.extend(r.feed(d, now))
    return out


def test_the_contract_vector_decodes_to_the_exact_access_unit_and_capture_time():
    r = h264_relay.UdpReassembler(SESSION)
    assert feed_all(r, GOLDEN) == [(1790001782.5, b"ABCDEFGHIJ")]


def test_a_frame_is_delivered_on_its_last_data_fragment_without_waiting_for_parity():
    r = h264_relay.UdpReassembler(SESSION)
    assert feed_all(r, DATA) == [(1790001782.5, b"ABCDEFGHIJ")]
    assert feed_all(r, PARITY) == []          # already shown: parity must not re-deliver


@pytest.mark.parametrize("lost", [0, 1, 2])
def test_any_one_lost_fragment_is_rebuilt_exactly_from_parity(lost):
    """Index 2 is the SHORT last fragment, alone in its group: the trim is what is tested."""
    r = h264_relay.UdpReassembler(SESSION)
    kept = [d for i, d in enumerate(DATA) if i != lost]
    assert feed_all(r, kept + PARITY) == [(1790001782.5, b"ABCDEFGHIJ")]
    assert r.recovered == 1


@pytest.mark.parametrize("size", [1201, 7614, 22124])
def test_real_frame_sizes_survive_one_loss_per_group_in_any_order(size):
    """1201: one byte past a payload. 7614 and 22124: the QP40 and QP36 frames of 2026-09-23."""
    au = bytes((i * 131 + size) & 0xFF for i in range(size))
    data, parity = packets(1, au)
    groups = range(0, len(data), 8)
    kept = [d for i, d in enumerate(data) if i not in groups]   # lose the first of each group
    r = h264_relay.UdpReassembler(SESSION)
    assert feed_all(r, parity + kept[::-1]) == [(1.0, au)]      # parity first, data reversed


@pytest.mark.parametrize("size", [1201, 7614, 22124])
def test_losing_the_short_last_fragment_rebuilds_it_at_its_own_length(size):
    """Its group's parity is padded to a FULL payload; without the trim the frame comes back
    with trailing zeros — a corrupt access unit. The golden vector cannot catch this: there
    the short fragment is alone in its group, so its parity is exactly its length."""
    au = bytes((i * 131 + size) & 0xFF for i in range(size))
    data, parity = packets(1, au)
    r = h264_relay.UdpReassembler(SESSION)
    assert feed_all(r, data[:-1] + parity) == [(1.0, au)]


def test_two_losses_in_one_group_lose_that_frame_and_the_next_one_still_shows():
    r = h264_relay.UdpReassembler(SESSION)
    d1, _ = packets(1, b"a" * 5000)
    d2, p2 = packets(2, b"b" * 5000)
    d3, _ = packets(3, b"c" * 5000)
    assert feed_all(r, d1) == [(1.0, b"a" * 5000)]
    assert feed_all(r, d2[2:] + p2) == []
    assert feed_all(r, d3) == [(1.0, b"c" * 5000)]
    assert r.skipped == 1                      # counted, so a lossy link shows up in numbers


def test_a_frame_completing_after_a_newer_one_is_never_shown():
    r = h264_relay.UdpReassembler(SESSION)
    d1, _ = packets(1, b"a" * 3000)
    d2, _ = packets(2, b"b" * 3000)
    assert feed_all(r, d1[:-1]) == []
    assert feed_all(r, d2) == [(1.0, b"b" * 3000)]
    assert feed_all(r, d1[-1:]) == []          # frame 1 finally whole: too late


def test_frame_ids_wrap_without_freezing_the_view():
    """2**32 frames is ~9.5 years at 14 fps, but the robot's counter starts wherever it likes;
    a plain `>` would refuse every frame after the wrap."""
    r = h264_relay.UdpReassembler(SESSION)
    for frame in (2**32 - 2, 2**32 - 1, 0, 1):
        d, _ = packets(frame, b"x" * 100)
        assert feed_all(r, d) == [(1.0, b"x" * 100)]


@pytest.mark.parametrize("mutate", [
    lambda d: d[:10],                                      # truncated header
    lambda d: b"XX" + d[2:],                               # wrong magic
    lambda d: d[:2] + b"\x02" + d[3:],                     # unknown version
    lambda d: d[:3] + b"\x07" + d[4:],                     # unknown kind
    lambda d: d[:8] + b"\x00\x00\x00\x00" + d[12:],        # another lease's session
    lambda d: d[:18] + b"\x00\x00" + d[20:],               # zero fragments
    lambda d: d[:20] + b"\xff\xff\xff\x7f" + d[24:],       # au_len beyond count * payload
    lambda d: d[:-1],                                      # body shorter than declared
])
def test_malformed_or_foreign_datagrams_are_ignored_not_raised(mutate):
    r = h264_relay.UdpReassembler(SESSION)
    assert r.feed(mutate(DATA[0]), 0.0) == []
    assert r.bad == 1
    assert feed_all(r, DATA) == [(1790001782.5, b"ABCDEFGHIJ")]   # still works afterwards


def test_a_flood_of_half_frames_cannot_grow_memory():
    r = h264_relay.UdpReassembler(SESSION)
    for frame in range(1, 5000):
        d, _ = packets(frame, b"z" * 5000)
        r.feed(d[0], 0.0)
    assert len(r._pending) <= r._MAX_PENDING


def test_an_incomplete_frame_expires_instead_of_waiting_for_ever():
    r = h264_relay.UdpReassembler(SESSION)
    d, _ = packets(1, b"q" * 3000)
    r.feed(d[0], 0.0)
    r.feed(d[1], 0.0)
    assert r.feed(d[2], r._MAX_AGE_S + 0.1) == []  # the first two were expired meanwhile


# --- watchdog -----------------------------------------------------------------------------

def test_robot_sending_and_nothing_ever_arriving_means_fall_back_to_tcp():
    w = h264_relay.UdpWatchdog(0.0)
    assert w.verdict(1.0, 0) is None
    assert w.verdict(1.5, 19) is None
    assert w.verdict(2.0, 20) == "fallback"


def test_an_idle_encoder_is_not_a_blocked_path():
    """Count flat: the robot has nothing to send (h264 off, camera stalled). No verdict."""
    w = h264_relay.UdpWatchdog(0.0)
    assert w.verdict(60.0, 0) is None
    assert w.verdict(120.0, 3) is None


def test_silence_after_udp_worked_reconnects_and_never_falls_back():
    """What a long link outage looks like from HQ. Falling back here would give up UDP after
    every Starlink blackout."""
    w = h264_relay.UdpWatchdog(0.0)
    w.on_datagram(1.0, 10)
    assert w.verdict(3.0, 40) is None               # robot ahead, but only 2 s of silence
    assert w.verdict(6.5, 80) == "reconnect"


def test_datagrams_arriving_keep_the_watchdog_quiet():
    w = h264_relay.UdpWatchdog(0.0)
    for t in range(1, 60):
        w.on_datagram(float(t), t * 14)
        assert w.verdict(float(t) + 0.5, t * 14 + 7) is None
