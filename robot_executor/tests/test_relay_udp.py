"""RelayTransport over UDP — the executor half of continuous teleop.

WHY: over Starlink every HTTP command opened a new TCP connection (178 ms median, a lost SYN
costs a full second), and the move loop WAITED for each one. See `_RelayUdp` and the UDP
block in robot-command-relay/relay_server.py.

WHAT EACH TEST CATCHES:

* The datagram bytes are a CONTRACT with `relay_server.UdpControl` on the robot. `GOLDEN_*`
  are the same bytes that repo's test decodes — keep them identical, or the robot refuses
  every UDP command and the operator only gets the dead-man stops.
* The move loop must not block on the network while UDP works, and must fall back to HTTP
  BEFORE the robot's 1 s dead-man fires when it does not.
* A stop goes out on both paths, UDP first, and the HTTP one carries `ts` so the relay can
  fence moves that are still in flight.
* `ts` must be strictly increasing even when the clock does not move — the relay refuses a
  repeat as a replay.
* A forged ack must not keep the transport on a UDP path that is down.

No robot, no DDS, no network: the UDP socket is replaced by a recorder.
"""
from __future__ import annotations

import hashlib
import hmac
import struct
import sys
import threading
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import robot_executor_service as svc

KEY = b"test-key"
T = 1790001782.5
# SAME BYTES as robot-command-relay/tests/test_relay_udp.py — keep them identical.
GOLDEN_MOVE = bytes.fromhex(
    "524301010000a09d50acda410000803e000000be0000003faacc43a3b69996a4bfaf308232e6f1a6")
GOLDEN_STOP = bytes.fromhex(
    "524301020000b09d50acda41000000000000000000000000d5f78883e27df1aca935ca4de3521738")
GOLDEN_ACK = bytes.fromhex("524101000000a09d50acda41283081628d0b7b1dd931b231d4c23212")


def bare_udp():
    u = svc._RelayUdp.__new__(svc._RelayUdp)
    u._key = KEY
    u._last_ack = 0.0
    u._sending_since = None
    u.sent = u.acked = 0
    u.rtt_ms = None
    return u


class FakeUdp:
    """Stands in for _RelayUdp: records what would have been sent. `proven` makes every send
    acknowledged, like a relay that answers."""
    KIND_MOVE, KIND_STOP = 1, 2

    def __init__(self, silent=False, proven=True):
        self.sent = []
        self._silent = silent
        self._proven = proven
        self.acked = 0

    def send(self, kind, ts, vx=0.0, vy=0.0, vyaw=0.0):
        self.sent.append((kind, ts))
        if self._proven:
            self.acked += 1

    def silent(self, now):
        return self._silent

    def idle(self):
        pass


def relay(udp):
    t = svc.RelayTransport("go2", "http://10.1.254.18:8092", "tok", dry_run=True)
    t._udp = udp
    posts = []

    def post(body):
        posts.append(body)
        return {"ok": True, "reply": "ok stub"}
    t._post_raw = t._post
    t._post = post
    return t, posts


@pytest.fixture(autouse=True)
def fast(monkeypatch):
    monkeypatch.setattr(svc.RelayTransport, "_REFRESH_S", 0.01)
    monkeypatch.setattr(svc.RelayTransport, "_UDP_REFRESH_S", 0.005)
    monkeypatch.setattr(svc.RelayTransport, "_UDP_STOP_GAP_S", 0.0)


# --- the contract --------------------------------------------------------------------------

def test_the_datagrams_match_the_relay_contract_byte_for_byte():
    u = bare_udp()
    assert u.encode(1, T, 0.25, -0.125, 0.5) == GOLDEN_MOVE
    assert u.encode(2, T + 0.25) == GOLDEN_STOP


def test_an_authentic_ack_counts_and_gives_the_round_trip():
    u = bare_udp()
    assert u.on_ack(GOLDEN_ACK, now=10.0, wall=T + 0.08) is True
    assert u.acked == 1 and u.rtt_ms == 80.0


@pytest.mark.parametrize("ack", [
    GOLDEN_ACK[:-1] + bytes([GOLDEN_ACK[-1] ^ 1]),    # forged / corrupted
    GOLDEN_ACK[:-1],                                   # truncated
    b"",
])
def test_a_forged_or_broken_ack_does_not_keep_udp_alive(ack):
    u = bare_udp()
    u._sending_since = 0.0
    assert u.on_ack(ack, now=1.0, wall=T) is False
    assert u.silent(1.0)                               # still silent: fall back as planned


def test_silence_is_measured_from_the_last_ack_not_from_the_first_send():
    u = bare_udp()
    u._sending_since = 0.0
    u.on_ack(GOLDEN_ACK, now=5.0, wall=T)
    assert not u.silent(5.0 + svc._RelayUdp.SILENT_S - 0.01)
    assert u.silent(5.0 + svc._RelayUdp.SILENT_S + 0.01)


def test_udp_gives_up_before_the_robot_dead_man_does():
    """Catches: a fall-back slower than the 1 s window, so the robot stops mid-walk before
    the transport even notices."""
    assert svc._RelayUdp.SILENT_S + svc.RelayTransport._REFRESH_S < 1.0


# --- the move loop -------------------------------------------------------------------------

def test_while_udp_works_the_move_loop_touches_http_only_to_prove_it():
    udp = FakeUdp()
    t, posts = relay(udp)
    t._start_move(0.3, 0.0, 0.0, duration=0.1, continuous=False)
    t._move_thread.join(timeout=2)
    moves = [k for k, _ in udp.sent if k == 1]
    assert len(moves) >= 5
    # The fake acknowledges on send, so the path is proven from the first datagram and no
    # HTTP move is needed: the only HTTP call is the deadline stop.
    assert [p["verb"] for p in posts] == ["stop_move"]


def test_an_unproven_udp_path_is_doubled_over_http_so_the_robot_is_never_unrefreshed():
    """Catches: UDP re-enabled after a fall-back while still blocked — without the HTTP copy
    the robot gets nothing for SILENT_S + a POST, ~0.9 s over Starlink, against a 1 s
    dead-man."""
    udp = FakeUdp(proven=False)
    t, posts = relay(udp)
    t._start_move(0.3, 0.0, 0.0, duration=0.05, continuous=False)
    t._move_thread.join(timeout=2)
    http_moves = [p for p in posts if p["verb"] == "move"]
    udp_moves = [k for k, _ in udp.sent if k == 1]
    assert http_moves and len(http_moves) == len(udp_moves)


def test_a_silent_udp_path_falls_back_to_http_moves():
    udp = FakeUdp(silent=True)
    t, posts = relay(udp)
    t._start_move(0.3, 0.0, 0.0, duration=0.1, continuous=False)
    t._move_thread.join(timeout=2)
    assert "move" in [p["verb"] for p in posts]
    assert t._udp_off_until > time.monotonic()           # and it stays on HTTP for a while


def test_a_stop_goes_out_on_udp_first_then_on_http():
    udp = FakeUdp()
    t, _ = relay(udp)
    order = []
    udp.send = lambda kind, ts, *a: order.append(("udp", kind))
    t._post = lambda body: order.append(("http", body["verb"])) or {"ok": True, "reply": "ok"}
    t.execute("stop", {})
    assert order == [("udp", 2)] * svc.RelayTransport._UDP_STOP_COPIES + [("http", "stop_move")]


def test_http_movement_commands_carry_an_increasing_ts_and_the_rest_do_not(monkeypatch):
    t = svc.RelayTransport("go2", "http://stub", "tok", dry_run=True)
    seen = []
    monkeypatch.setattr(svc.urllib.request, "urlopen", lambda *a, **k: seen.append(a))
    t._dry_run = False

    bodies = []
    real_dumps = svc.json.dumps
    monkeypatch.setattr(svc.json, "dumps", lambda b, *a, **k: bodies.append(b) or real_dumps(b))
    for body in ({"verb": "move", "vx": 1}, {"verb": "stop_move"}, {"verb": "sit"}):
        t._post(body)
    assert "ts" in bodies[0] and "ts" in bodies[1] and "ts" not in bodies[2]
    assert bodies[1]["ts"] > bodies[0]["ts"]


def test_ts_is_strictly_increasing_even_with_a_frozen_clock(monkeypatch):
    """The relay refuses a repeated ts as a replay; two commands in the same microsecond, or
    after NTP steps the clock back, must still both count."""
    monkeypatch.setattr(svc.time, "time", lambda: T)
    t = svc.RelayTransport("go2", "http://stub", "tok", dry_run=True)
    a, b, c = t._next_ts(), t._next_ts(), t._next_ts()
    assert a < b < c


def test_ts_is_strictly_increasing_across_threads():
    t = svc.RelayTransport("go2", "http://stub", "tok", dry_run=True)
    out = []
    threads = [threading.Thread(target=lambda: out.extend(t._next_ts() for _ in range(500)))
               for _ in range(4)]
    for th in threads:
        th.start()
    for th in threads:
        th.join()
    assert len(set(out)) == len(out)


# --- configuration -------------------------------------------------------------------------

@pytest.mark.parametrize("raw, port", [
    ("8097", 8097), ("", 0), ("0", 0), ("80", 0), ("70000", 0), ("80x", 0), ("-1", 0),
])
def test_the_udp_port_fails_safe_to_http(monkeypatch, raw, port):
    monkeypatch.setenv("RELAY_UDP_PORT", raw)
    assert svc._relay_udp_port() == port


def test_dry_run_opens_no_udp_socket():
    t = svc.RelayTransport("go2", "http://10.1.254.18:8092", "tok", dry_run=True, udp_port=8097)
    assert t._udp is None


def test_the_mac_is_hmac_sha256_truncated_as_documented():
    """Pinned to the primitive, so a 'harmless' switch to another hash breaks here and not
    on the robot, where it would look like every command being silently refused."""
    body = struct.pack("<2sBBdfff", b"RC", 1, 1, T, 0.25, -0.125, 0.5)
    assert GOLDEN_MOVE[24:] == hmac.new(KEY, body, hashlib.sha256).digest()[:16]
