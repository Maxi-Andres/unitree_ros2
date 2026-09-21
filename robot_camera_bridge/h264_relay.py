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

    def __init__(self, robot_url, backend_url, logger=None):
        self._url = robot_url
        self._backend = backend_url
        self._log = logger
        self._stop = threading.Event()
        self.frames = 0
        self.dropped = 0
        self._ws = None
        threading.Thread(target=self._run, name="h264-relay", daemon=True).start()
        if logger:
            logger.info(f"[h264] relaying {robot_url} -> {backend_url}")

    def _run(self):
        backoff = 1.0
        while not self._stop.is_set():
            started = time.monotonic()
            try:
                self._pump()
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

    def _pump(self):
        reader = AuReader()
        self._connect_backend()
        req = urllib.request.Request(self._url, headers={"User-Agent": "ai-vl-h264-relay"})
        with urllib.request.urlopen(req, timeout=10) as resp:
            while not self._stop.is_set():
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

    def close(self):
        self._stop.set()
        self._close()
