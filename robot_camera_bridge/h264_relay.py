#!/usr/bin/env python3
"""Relay the robot's all-intra H.264 drive branch to the backend, untouched.

WHY THIS IS A SEPARATE PATH from the JPEG one in `camera_sources.py`, and not a source of it:
the JPEG stream feeds YOLO and the VLM, which decode JPEGs. These bytes are H.264 and nothing
in this tier decodes them — the robot encodes, the browser decodes with WebCodecs. Mixing the
two into one channel would break detection the moment the drive view changed transport.

WHAT IT BUYS, measured on the robot 2026-09-21: 4207 B per frame against ~9000 for the JPEG at
the same quality (PSNR 29.34 vs 29.30), at 14.5 fps against the MJPEG's 10 — more frames for a
third less bandwidth. Every frame is a keyframe, so a lost one costs ONE frame and never a
freeze until the next IDR, which is what lets this ride a link with no retransmission.

WIRE FORMAT to the backend: one binary message per frame, 8-byte little-endian double (the
robot's clock when the camera produced the frame) + the access unit. One message, so a frame
and its capture time cannot be paired wrongly under load.
"""
from __future__ import annotations

import select
import socket
import ssl
import struct
import threading
import time
import urllib.request

import websocket  # websocket-client


class AuReader:
    """Frames `multipartmux` output into access units, by `Content-Length`.

    SECOND COPY, ON PURPOSE. The first is `mjpeg_server.AuReader` on the robot; the network
    boundary forbids sharing a module between the two machines, so each side carries its own
    and a test on each keeps them honest (`tests/test_h264_relay.py` here,
    `tests/test_h264_branch.py` there). If you fix one, check the other.

    Boundary-agnostic: it scans for the length header, not for a separator string.
    """

    _MAX_PART = 4 << 20

    def __init__(self):
        self._buf = b""

    def feed(self, chunk):
        """Add bytes; return a list of (capture_time, access_unit)."""
        self._buf += chunk
        out = []
        while True:
            head = self._buf.find(b"Content-Length:")
            if head < 0:
                if len(self._buf) > self._MAX_PART:
                    self._buf = b""          # desynchronised: resync rather than grow
                break
            eol = self._buf.find(b"\r\n", head)
            if eol < 0:
                break
            try:
                n = int(self._buf[head + 15:eol])
            except ValueError:
                self._buf = self._buf[eol + 2:]
                continue
            body = self._buf.find(b"\r\n\r\n", eol)
            if body < 0:
                break
            start = body + 4
            if n > self._MAX_PART:
                self._buf = self._buf[start:]
                continue
            if len(self._buf) < start + n:
                break
            out.append((_capture_time(self._buf[:head]), self._buf[start:start + n]))
            self._buf = self._buf[start + n:]
        return out


# ---------------------------------------------------------------------------------------
# THE ROBOT -> HQ HOP OVER UDP.
#
# Over TCP one lost packet freezes the whole drive view until it is retransmitted. MEASURED
# 2026-09-23 over Starlink: 10 stalls of 250-916 ms in 150 s, every one on the link. With the
# same loss, UDP datagrams of this size lost only whole frames, in runs of 1-4 (~280 ms at
# worst), and one XOR parity per group made 94% of frames arrive usable. Every frame is a
# keyframe, so a missing one costs that frame and nothing after it.
#
# Only this hop changes. What goes to the backend — capture time + access unit, one message
# per frame — is byte-for-byte what the TCP path sends, so the backend and the browser do not
# know which transport carried the frame.
#
# SECOND COPY OF THE WIRE FORMAT, ON PURPOSE: the sender is `mjpeg_server.udp_packets` on the
# robot (robot-video-pipeline), and the full format and the lease protocol are documented
# there. Both sides' `tests/test_h264_udp.py` assert the SAME golden datagrams. If you change
# a field here, change it there.
# ---------------------------------------------------------------------------------------
UDP_MAGIC = b"AV"
UDP_VERSION = 1
UDP_HEADER = struct.Struct("<2sBBBxHIIHHId")
UDP_KIND_DATA = 0
UDP_KIND_PARITY = 1
_U32 = 1 << 32


def _xor(chunks, size):
    """XOR of `chunks`, each zero-padded to `size`. Mirror of the robot's `_xor`."""
    acc = 0
    for c in chunks:
        acc ^= int.from_bytes(c, "little")
    return acc.to_bytes(size, "little")


class _Frame:
    __slots__ = ("au_len", "capture", "count", "data", "first_seen", "group", "parity",
                 "payload")

    def __init__(self, count, au_len, payload, group, capture, now):
        self.count, self.au_len, self.payload = count, au_len, payload
        self.group, self.capture, self.first_seen = group, capture, now
        self.data = {}
        self.parity = {}

    def frag_len(self, index):
        return self.payload if index < self.count - 1 else \
            self.au_len - (self.count - 1) * self.payload

    def members(self, g):
        return range(g * self.group, min((g + 1) * self.group, self.count))


class UdpReassembler:
    """Datagrams in, whole access units out — the newest only, never a stale one.

    Bounded on every axis, because this is fed by the network: at most `_MAX_PENDING` frames
    in flight, each dropped after `_MAX_AGE_S`, and a header that does not describe a sane
    frame is counted in `bad` and ignored. It never raises on input.

    Delivery is immediate: a frame goes out the moment its last data fragment lands, or the
    moment one missing fragment becomes recoverable from its group's parity. A frame that
    completes AFTER a newer one was delivered is dropped — showing it would step the picture
    backwards in time on the view the operator steers by.
    """

    _MAX_PENDING = 8                # ~0.5 s of frames at the camera's 14.3 fps
    _MAX_AGE_S = 0.5                # an incomplete frame older than this is not coming back
    _MAX_AU = 4 << 20               # same ceiling as AuReader
    _MAX_PAYLOAD = 1472             # the largest UDP payload an Ethernet MTU carries

    def __init__(self, session):
        self._session = session
        self._last = None           # frame id last delivered
        self._pending = {}
        self.delivered = 0
        self.recovered = 0          # frames that needed their parity
        self.skipped = 0            # frame ids that never became deliverable
        self.bad = 0                # datagrams that failed validation

    def _newer(self, frame):
        """Frame ids wrap at 2**32; "newer" is within half the space ahead."""
        return self._last is None or 0 < (frame - self._last) % _U32 < _U32 // 2

    def feed(self, datagram, now):
        """Add one datagram; return [(capture, access_unit)] — empty, or the one it completed."""
        self._expire(now)
        if len(datagram) < UDP_HEADER.size:
            self.bad += 1
            return []
        (magic, version, kind, group, payload, session, frame, index, count, au_len,
         capture) = UDP_HEADER.unpack_from(datagram)
        body = datagram[UDP_HEADER.size:]
        if (magic != UDP_MAGIC or version != UDP_VERSION or session != self._session
                or kind not in (UDP_KIND_DATA, UDP_KIND_PARITY) or group < 1
                or not 1 <= payload <= self._MAX_PAYLOAD or count < 1
                or not (count - 1) * payload < au_len <= min(count * payload, self._MAX_AU)):
            self.bad += 1
            return []
        if not self._newer(frame):
            return []               # late: something newer is already on screen
        f = self._pending.get(frame)
        if f is None:
            if len(self._pending) >= self._MAX_PENDING:
                del self._pending[min(self._pending,
                                      key=lambda k: self._pending[k].first_seen)]
            f = self._pending[frame] = _Frame(count, au_len, payload, group, capture, now)
        elif (f.count, f.au_len, f.payload, f.group) != (count, au_len, payload, group):
            self.bad += 1           # two datagrams disagree about the same frame
            return []
        if kind == UDP_KIND_DATA:
            if index >= count or len(body) != f.frag_len(index):
                self.bad += 1
                return []
            f.data[index] = body
        else:
            if index * group >= count or len(body) != max(
                    f.frag_len(i) for i in f.members(index)):
                self.bad += 1
                return []
            f.parity[index] = body
        return self._try_deliver(frame, f)

    def _try_deliver(self, frame, f):
        if len(f.data) < f.count:
            for g, par in f.parity.items():
                missing = [i for i in f.members(g) if i not in f.data]
                if len(missing) == 1:
                    lost = missing[0]
                    others = [f.data[i] for i in f.members(g) if i != lost]
                    f.data[lost] = _xor([*others, par], len(par))[:f.frag_len(lost)]
                    self.recovered += 1
            if len(f.data) < f.count:
                return []
        au = b"".join(f.data[i] for i in range(f.count))
        if self._last is not None:
            self.skipped += (frame - self._last) % _U32 - 1
        self._last = frame
        self.delivered += 1
        for k in [k for k in self._pending if not self._newer(k)]:
            del self._pending[k]
        return [(f.capture, au)]

    def _expire(self, now):
        for k in [k for k, f in self._pending.items() if now - f.first_seen > self._MAX_AGE_S]:
            del self._pending[k]


class UdpWatchdog:
    """Tells a blocked UDP path from a quiet one, using the robot's own count of frames sent.

    The lease heartbeat carries that count. Two verdicts, deliberately different in cost:

    * `"fallback"` — the robot has sent `min_sent` frames on THIS lease and not one datagram
      has arrived. UDP does not get through (a firewall, a NAT with no mapping); go back to
      TCP for a while instead of showing nothing.
    * `"reconnect"` — datagrams did arrive earlier, then nothing for `window_s` while the
      robot kept sending. Open a fresh lease. It is cheap, and it is also what a long link
      outage looks like from here, which is why this verdict must not be the fallback.
    """

    def __init__(self, now, window_s=5.0, min_sent=20):
        self._window = window_s
        self._min_sent = min_sent
        self._any = False
        self._t_ref = now
        self._sent_ref = None

    def on_datagram(self, now, robot_sent):
        self._any = True
        self._t_ref = now
        self._sent_ref = robot_sent

    def verdict(self, now, robot_sent):
        """None, "fallback" or "reconnect". `robot_sent` is None until the first heartbeat."""
        if robot_sent is None:
            return None
        if self._sent_ref is None:
            self._sent_ref = robot_sent if self._any else 0
        if robot_sent - self._sent_ref < self._min_sent:
            return None
        if not self._any:
            return "fallback"
        if now - self._t_ref >= self._window:
            return "reconnect"
        return None


def _capture_time(headers: bytes) -> float:
    """The robot's clock when the camera produced this frame, from `X-Capture`.

    0.0 when absent, which is what an un-instrumented robot sends: the picture still works,
    only the latency measurement is unavailable. Never raises — a malformed header must not
    be able to stop the drive view.
    """
    at = headers.rfind(b"X-Capture:")
    if at < 0:
        return 0.0
    end = headers.find(b"\r\n", at)
    try:
        return float(headers[at + 10:end if end > 0 else len(headers)])
    except ValueError:
        return 0.0


class H264Relay:
    """Read the robot's `/h264` and push it at the backend, forever.

    Its own thread and its own socket: a stall on this path must not touch the JPEG one, and
    vice versa. Same reconnect discipline as the camera sources — a stream that ran for
    `_HEALTHY_S` counts as healthy, so a blip does not push the backoff up for the rest of the
    process's life.
    """

    _HEALTHY_S = 5.0
    _MAX_BACKOFF_S = 15.0
    # How long to stay on TCP after UDP was found blocked, before trying UDP again. Not for
    # ever: the verdict can be wrong (an outage right as the lease opened looks the same),
    # and a permanent fall-back would quietly give up the whole point of the UDP path.
    _TCP_FALLBACK_S = 60.0
    # A lease with no heartbeat for this long is dead even if the socket has not noticed.
    _LEASE_SILENT_S = 5.0

    def __init__(self, robot_url, backend_url, logger=None, udp_port=0):
        self._url = robot_url
        self._backend = backend_url
        self._log = logger
        self._udp_port = udp_port
        self._tcp_until = 0.0
        self._stop = threading.Event()
        self.frames = 0
        self.dropped = 0
        self.transport = "udp" if udp_port else "tcp"
        self.reassembler = None      # the current lease's UdpReassembler, for diagnosis
        self._ws = None
        threading.Thread(target=self._run, name="h264-relay", daemon=True).start()
        if logger:
            via = f"udp :{udp_port}" if udp_port else "tcp"
            logger.info(f"[h264] relaying {robot_url} ({via}) -> {backend_url}")

    def _run(self):
        backoff = 1.0
        while not self._stop.is_set():
            started = time.monotonic()
            try:
                if self._udp_port and time.monotonic() >= self._tcp_until:
                    self.transport = "udp"
                    self._pump_udp()
                else:
                    self.transport = "tcp"
                    self._pump(until=self._tcp_until if self._udp_port else None)
                    backoff = 1.0            # a TCP spell that ran out on schedule is healthy
            except _UdpBlocked as exc:
                self._tcp_until = time.monotonic() + self._TCP_FALLBACK_S
                if self._log:
                    self._log.warn(f"[h264] {exc}; back to TCP for "
                                   f"{self._TCP_FALLBACK_S:.0f}s")
                self._close()
            except Exception as exc:
                if time.monotonic() - started >= self._HEALTHY_S:
                    backoff = 1.0
                if self._log and not self._stop.is_set():
                    self._log.warn(f"[h264] relay {self._url} failed: {exc}; "
                                   f"retry in {backoff:.0f}s")
                self._close()
                self._stop.wait(backoff)
                backoff = min(backoff * 2, self._MAX_BACKOFF_S)

    def _connect_backend(self):
        # cert_reqs NONE: the backend serves the app's own self-signed certificate, and this
        # is a loopback hop on the same machine. Same choice the JPEG producer already makes.
        try:
            self._ws = websocket.create_connection(
                self._backend, sslopt={"cert_reqs": ssl.CERT_NONE}, timeout=5,
                enable_multithread=True)
        except Exception as exc:
            # Name the END that failed. This relay has two, and a message that blames the
            # robot for the backend's refusal sends the next person to the wrong machine —
            # which is exactly what happened the first time it ran: a 403 from a backend that
            # had not been restarted yet, reported as "relay http://<robot>/h264 failed".
            raise OSError(f"backend {self._backend}: {exc}") from exc
        self._ws.settimeout(None)
        # DRAIN THE SOCKET, or the server hangs up on us.
        #
        # This relay only ever PUSHES, but uvicorn sends keepalive PINGs and websocket-client
        # answers PONG only while something is blocked in recv(). Without this thread the
        # pings go unanswered and the connection is closed from the far end after ~20 s; the
        # next send then fails with "socket is already closed" and the relay reconnects in a
        # loop that looks like a network problem and is not. MEASURED 2026-09-21, and the JPEG
        # producer in robot_camera_bridge.py documents the very same trap — it was there to be
        # copied and was not.
        ws = self._ws
        threading.Thread(target=self._drain, args=(ws,), daemon=True).start()

    def _drain(self, ws):
        """Answer the server's keepalive pings by staying in recv(). Exits when the socket
        closes or is replaced; the send path notices and reconnects."""
        try:
            while True:
                ws.recv()
        except Exception:
            pass

    def _close(self):
        if self._ws is not None:
            try:
                self._ws.close()
            except Exception:
                pass
            self._ws = None

    def _pump(self, until=None):
        """The TCP path. `until` (monotonic) ends it cleanly so the UDP path can be retried."""
        reader = AuReader()
        self._connect_backend()
        req = urllib.request.Request(self._url, headers={"User-Agent": "ai-vl-h264-relay"})
        with urllib.request.urlopen(req, timeout=10) as resp:
            while not self._stop.is_set():
                if until is not None and time.monotonic() >= until:
                    self._close()
                    return
                # read1(), NOT read(): read(n) blocks until it has ALL n bytes, which on a
                # frame stream means holding each frame until the NEXT ones fill the buffer —
                # a whole frame period of latency on the path that exists to be fast. The
                # sibling scanners here and on the robot document the same trap.
                chunk = resp.read1(65536)
                if not chunk:
                    raise OSError("stream closed by peer")
                frames = reader.feed(chunk)
                # DRAIN TO THE NEWEST BEFORE SENDING ANYTHING.
                #
                # Reading one frame per pass and forwarding it is how a backlog becomes
                # permanent: the source produces at 14 fps and this loop consumes at 14 fps,
                # so whatever gap opens once is carried for ever. MEASURED 2026-09-21 — a
                # burst from an encoder rebuild left this path 288 ms behind (four frames) and
                # it was STILL 288 ms behind forty seconds later, delivering at a perfect
                # 72 ms cadence the whole time. A steady cadence is not proof of freshness.
                #
                # So empty the socket first and keep only the last frame in it. Anything
                # older is already worthless to someone steering, and this is the same
                # discipline `Latest` on the robot and `_put_latest` in the backend apply.
                while select.select([resp.fileno()], [], [], 0)[0]:
                    more = resp.read1(65536)
                    if not more:
                        raise OSError("stream closed by peer")
                    frames.extend(reader.feed(more))
                if not frames:
                    continue
                # Forward only the NEWEST of a batch. If several arrived while this thread was
                # busy, every one but the last is already worthless to someone steering, and
                # sending them makes the operator watch a queue drain — old pictures, in
                # order, at the wrong time. Counted, not hidden: `dropped` says how often it
                # happens, which is how a slow relay would be noticed.
                self.dropped += len(frames) - 1
                capture, au = frames[-1]
                self._ws.send_binary(struct.pack("<d", capture) + au)
                self.frames += 1

    def _pump_udp(self):
        """The UDP path: hold a lease on `/h264?udp=PORT` and relay what arrives on the port.

        One thread, one select() over both sockets. The lease body is the robot's heartbeat
        (frames sent so far, one line a second); the datagrams are the frames. Drained to the
        newest before anything is forwarded, same discipline as the TCP path below.
        """
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.bind(("0.0.0.0", self._udp_port))  # noqa: S104  # the robot's source address is NAT'd; the lease session is the filter
            sock.setblocking(False)
            self._connect_backend()
            sep = "&" if "?" in self._url else "?"
            req = urllib.request.Request(f"{self._url}{sep}udp={self._udp_port}",
                                         headers={"User-Agent": "ai-vl-h264-relay"})
            with urllib.request.urlopen(req, timeout=10) as resp:
                try:
                    session = int(resp.headers.get("X-Udp-Session", ""))
                except ValueError:
                    # An old robot ignores `?udp=` and answers with the TCP multipart stream.
                    # Not an error in the robot and not a blocked path: it simply cannot do
                    # UDP, so stay on TCP for a spell rather than reconnect in a loop.
                    raise _UdpBlocked("robot did not grant a UDP lease "
                                      "(no X-Udp-Session: older mjpeg_server?)") from None
                self._udp_lease(resp, sock, session)
        finally:
            sock.close()

    def _udp_lease(self, resp, sock, session):
        reasm = self.reassembler = UdpReassembler(session)
        watchdog = UdpWatchdog(time.monotonic())
        robot_sent = None
        beat_buf = b""
        last_beat = time.monotonic()
        announced = False
        lease_fd = resp.fileno()
        while not self._stop.is_set():
            ready = select.select([sock, lease_fd], [], [], 1.0)[0]
            now = time.monotonic()
            frames = []
            if sock in ready:
                while True:
                    try:
                        datagram, src = sock.recvfrom(2048)
                    except BlockingIOError:
                        break
                    bad = reasm.bad
                    got = reasm.feed(datagram, now)
                    if reasm.bad == bad:              # ours and well-formed: UDP gets through
                        watchdog.on_datagram(now, robot_sent)
                    if got and not announced:
                        announced = True
                        if self._log:
                            self._log.info(f"[h264] udp frames arriving from {src[0]}")
                    frames.extend(got)
            if lease_fd in ready:
                chunk = resp.read1(4096)
                if not chunk:
                    raise OSError("udp lease closed by robot")
                beat_buf += chunk
                *lines, beat_buf = beat_buf.split(b"\n")
                beat_buf = beat_buf[-64:]            # bounded: a heartbeat is a short number
                for line in lines:
                    if line.strip().isdigit():
                        robot_sent = int(line)
                        last_beat = now
            if frames:
                self.dropped += len(frames) - 1
                capture, au = frames[-1]
                self._ws.send_binary(struct.pack("<d", capture) + au)
                self.frames += 1
            if now - last_beat > self._LEASE_SILENT_S:
                raise OSError(f"udp lease silent for {now - last_beat:.0f}s")
            verdict = watchdog.verdict(now, robot_sent)
            if verdict == "fallback":
                raise _UdpBlocked(f"robot sent {robot_sent} frames on the lease and no "
                                  f"datagram reached :{self._udp_port}")
            if verdict == "reconnect":
                raise OSError("udp went silent while the robot kept sending; new lease")

    def close(self):
        self._stop.set()
        self._close()


class _UdpBlocked(Exception):
    """UDP does not get through on this lease; the relay falls back to TCP for a while."""
