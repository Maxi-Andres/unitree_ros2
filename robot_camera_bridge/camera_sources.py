#!/usr/bin/env python3
"""
camera_sources.py — pluggable robot camera sources for the AI-VL camera bridge.

Each source subscribes to its robot's camera and calls `on_frame` with a ready
JPEG (bytes). Adding a robot = adding a CameraSource; the bridge stays the same.

Sources:
  - Go2VideoApiSource   : Go2 front camera via the video API (GetImageSample) — the
    robot returns a ready JPEG, forwarded as-is (no decode). Reliable + low latency.
  - G1ImageTopicSource  : the G1 (humanoid) has NO Unitree video API; its head
    camera is a depth cam (RealSense) that publishes a ROS2 image topic. This source
    AUTO-DISCOVERS that topic (prefers a CompressedImage/JPEG topic to spare DDS
    bandwidth, else a raw sensor_msgs/Image which it decodes) and forwards JPEG.
  - TestPatternSource   : a synthetic moving frame — verifies the pipeline WITHOUT
    a robot (also used to smoke-test runtime source switching).

All sources share fps/resolution/quality params (set at build, live-tunable via the
bridge's /config), and a close() so the bridge can switch source robot at runtime.
"""
import asyncio
import ipaddress
import os
import ssl
import threading
import time
import urllib.error
import urllib.parse
import urllib.request

import cv2
import numpy as np
from rclpy.qos import qos_profile_sensor_data

# Target longer-vertical-side heights for the resolution presets ("native" = the
# source's own size / JPEG, forwarded without a resize).
_RES_HEIGHTS = {"native": None, "720p": 720, "480p": 480, "360p": 360}


def _encode_jpeg(bgr, quality):
    ok, buf = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
    return buf.tobytes() if ok else None


def _image_msg_to_bgr(msg):
    """sensor_msgs/Image -> BGR ndarray (no cv_bridge dependency)."""
    h, w = msg.height, msg.width
    enc = (msg.encoding or "").lower()
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if enc in ("rgb8", "bgr8"):
        img = buf.reshape(h, w, 3)
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if enc == "rgb8" else img
    if enc == "mono8":
        return cv2.cvtColor(buf.reshape(h, w), cv2.COLOR_GRAY2BGR)
    return buf.reshape(h, w, 3)  # best-effort fallback


def _is_whep_url(url):
    """A WHEP endpoint is an HTTP(S) URL whose path ends in /whep — that is how the spec
    addresses it and how mediamtx publishes it (`https://host:8889/<path>/whep`).

    Matched on the path and not on a scheme of our own invention, so the URL in .env is the
    one the browser and `curl` take too. The MJPEG reader is the fallback for every other
    http(s) URL, and being wrong here is not subtle: the MJPEG parser would scan an SDP
    document for JPEG markers forever.
    """
    if not url.startswith(("http://", "https://")):
        return False
    return urllib.parse.urlsplit(url).path.rstrip("/").endswith("/whep")


def _tls_policy(url, ca_file):
    """How to verify the WHEP endpoint: "plain" | "verify" | "pin" | "loopback-insecure".

    VERIFICATION IS THE DEFAULT, and the only way out of it is narrow and deliberate.
    mediamtx serves WHEP over HTTPS (`webrtcEncryption: yes`) with the self-signed
    `auto.crt` it generates for itself, which no store can verify. The standard's answer to
    a self-signed lab certificate is to PIN A CA, not to switch verification off — that is
    `ca_file` (STREAM_TLS_CA), and it is what a mediamtx on another machine must use; note
    it wins even on loopback, so pinning is never silently ignored.

    The single exception is a LOOPBACK host with no CA pinned: there is no network between
    us and the server, so there is nothing for a certificate to protect against. Every other
    host is verified, which is what makes a wrong hostname fail loudly instead of quietly
    accepting whatever answers.

    Split out from the context it builds so it can be tested without a certificate on disk.
    """
    if not url.startswith("https://"):
        return "plain"
    if ca_file:
        return "pin"
    if _is_loopback(urllib.parse.urlsplit(url).hostname):
        return "loopback-insecure"
    return "verify"


def _is_loopback(host):
    """Is `host` this same machine, with no network in between?

    Decides whether a self-signed certificate is acceptable (see
    `WhepStreamSource._tls_context`), so it must not be fooled by a name: only the literal
    loopback addresses and `localhost` count. Anything it cannot parse is treated as remote,
    which is the safe direction.
    """
    if not host:
        return False
    if host == "localhost":
        return True
    try:
        return ipaddress.ip_address(host).is_loopback
    except ValueError:
        return False


def _reprocess(jpeg=None, bgr=None, resolution="native", quality=0):
    """Apply resolution/quality and return JPEG bytes. Given a JPEG at 'native' +
    quality 0, it's returned untouched (no decode — the fast path)."""
    h = _RES_HEIGHTS.get(resolution)
    if bgr is None:
        if h is None and not quality:
            return jpeg
        bgr = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        if bgr is None:
            return jpeg
    if h is not None and bgr.shape[0] > h:
        w = max(1, int(bgr.shape[1] * h / bgr.shape[0]))
        bgr = cv2.resize(bgr, (w, h), interpolation=cv2.INTER_AREA)
    return _encode_jpeg(bgr, quality or 80)


class _ParamSource:
    """Shared fps/resolution/quality state + an fps time-gate. fps is enforced by
    time (never by touching ROS timers off-thread), so set_params is safe to call
    from the HTTP control thread."""

    _POLL_HZ = 30.0
    # How early a frame may arrive and still count. Real frames come off a polling loop
    # over DDS and wobble by a few ms; without slack, a source running just UNDER the cap
    # has half its frames land a hair early and be thrown away. Measured with the four
    # scenarios in the tests: 10% slack delivers 100% of a 14.8 fps source under a 15 fps
    # cap and still halves a 30 fps one, which is exactly the job.
    _JITTER_TOLERANCE = 0.90

    def __init__(self, fps, resolution, quality):
        self._fps = max(1.0, min(self._POLL_HZ, float(fps)))
        self._res = resolution if resolution in _RES_HEIGHTS else "native"
        self._quality = int(quality or 0)
        self._passed = 0.0   # monotonic time of the last frame the gate let through

    def _due(self):
        """Rate gate: cap the frame rate without destroying a source running near it.

        The old version reset the clock to the arrival time of each accepted frame:

            if now - self._last < 1.0 / self._fps: return False
            self._last = now

        which beats badly when the source runs just UNDER the cap. A frame landing a hair
        early is dropped, and the next one is then a whole extra period away, so the output
        collapses to roughly half the source rate. Measured against the robot 2026-09-11:
        source 14.8 fps, cap 15 fps, delivered 8.3 fps — 43% of the video thrown away by
        the thing meant to be letting it through. Raising the cap to 30 restored 13.7 fps
        immediately, which is what identified this.

        The fix is a small tolerance on the period rather than a stricter clock: a frame
        may arrive up to `_JITTER_TOLERANCE` of a period early and still count. That
        absorbs the source's wobble without letting a genuinely faster source through —
        measured, a 30 fps source under a 15 fps cap is still halved — and it releases no
        catch-up burst after a stall, which is how latency would otherwise arrive all at
        once on the view you steer by.
        """
        now = time.monotonic()
        period = 1.0 / self._fps   # _fps is clamped to >= 1.0 in __init__ and set_params
        if now - self._passed < period * self._JITTER_TOLERANCE:
            return False
        self._passed = now
        return True

    def set_params(self, fps=None, resolution=None, quality=None):
        if fps is not None:
            try:
                self._fps = max(1.0, min(self._POLL_HZ, float(fps)))
            except (TypeError, ValueError):
                pass
        if resolution is not None and resolution in _RES_HEIGHTS:
            self._res = resolution
        if quality is not None:
            try:
                self._quality = int(quality)
            except (TypeError, ValueError):
                pass

    def get_params(self):
        return {"fps": self._fps, "resolution": self._res, "quality": self._quality}

    def close(self):  # overridden to tear down ROS entities
        pass


class Go2VideoApiSource(_ParamSource):
    """Go2 front camera via the video API (GetImageSample, api_id 1001)."""

    GET_IMAGE_SAMPLE_API_ID = 1001

    def __init__(self, node, on_frame, fps=15, resolution="native", quality=0,
                 logger=None):
        super().__init__(fps, resolution, quality)
        from unitree_api.msg import Request, Response
        self._node = node
        self._on_frame = on_frame
        self._log = logger
        self._Request = Request
        self._pub = node.create_publisher(Request, "/api/videohub/request", 10)
        self._sub = node.create_subscription(
            Response, "/api/videohub/response", self._on_response, 10)
        self._timer = node.create_timer(1.0 / self._POLL_HZ, self._tick)

    def _tick(self):
        if not self._due():
            return
        req = self._Request()
        req.header.identity.api_id = self.GET_IMAGE_SAMPLE_API_ID
        self._pub.publish(req)

    def _on_response(self, msg):
        data = bytes(msg.binary)
        if not (len(data) >= 2 and data[0] == 0xFF and data[1] == 0xD8):
            return
        jpg = _reprocess(jpeg=data, resolution=self._res, quality=self._quality)
        if jpg:
            self._on_frame(jpg)

    def close(self):
        for destroy, ent in ((self._node.destroy_timer, self._timer),
                             (self._node.destroy_subscription, self._sub),
                             (self._node.destroy_publisher, self._pub)):
            try:
                destroy(ent)
            except Exception:
                pass


class G1ImageTopicSource(_ParamSource):
    """G1 head camera (RealSense) published as a ROS2 image topic. Auto-discovers
    the topic when none is given: prefers a CompressedImage (JPEG, light on DDS),
    else a raw sensor_msgs/Image (decoded here). Skips depth/infrared topics."""

    def __init__(self, node, on_frame, topic="", fps=12, resolution="native",
                 quality=70, logger=None):
        super().__init__(fps, resolution, quality)
        self._node = node
        self._on_frame = on_frame
        self._log = logger
        self._sub = None
        self._disc_timer = None
        if topic:
            self._subscribe(topic)
        else:
            # Retry discovery until the camera node appears on the graph.
            self._disc_timer = node.create_timer(1.0, self._discover)

    def _discover(self):
        try:
            names = self._node.get_topic_names_and_types()
        except Exception:
            return
        best = None  # (rank, name, is_compressed); lower rank = better
        for name, types in names:
            low = name.lower()
            if any(k in low for k in ("depth", "infra", "aligned", "disparity")):
                continue
            is_comp = "sensor_msgs/msg/CompressedImage" in types
            is_img = "sensor_msgs/msg/Image" in types
            if not (is_comp or is_img):
                continue
            color = "color" in low or "rgb" in low
            # Prefer: compressed+color(0) > compressed(1) > color raw(2) > any(3)
            rank = (0 if (is_comp and color) else 1 if is_comp
                    else 2 if color else 3)
            if best is None or rank < best[0]:
                best = (rank, name, is_comp)
        if best:
            _, name, is_comp = best
            if self._log:
                self._log.info(f"G1 camera: using {'compressed' if is_comp else 'raw'} "
                               f"image topic {name}")
            self._subscribe(name, compressed=is_comp)
            if self._disc_timer is not None:
                try:
                    self._node.destroy_timer(self._disc_timer)
                except Exception:
                    pass
                self._disc_timer = None

    def _subscribe(self, topic, compressed=None):
        # If not told, guess by suffix (…/compressed => CompressedImage).
        if compressed is None:
            compressed = topic.rstrip("/").endswith("compressed")
        if compressed:
            from sensor_msgs.msg import CompressedImage
            self._sub = self._node.create_subscription(
                CompressedImage, topic, self._cb_compressed, qos_profile_sensor_data)
        else:
            from sensor_msgs.msg import Image
            self._sub = self._node.create_subscription(
                Image, topic, self._cb_raw, qos_profile_sensor_data)

    def _cb_raw(self, msg):
        if not self._due():
            return
        try:
            jpg = _reprocess(bgr=_image_msg_to_bgr(msg),
                             resolution=self._res, quality=self._quality or 80)
            if jpg:
                self._on_frame(jpg)
        except Exception as e:
            if self._log:
                self._log.warn(f"G1 raw image convert error: {e}")

    def _cb_compressed(self, msg):
        if not self._due():
            return
        try:
            data = bytes(msg.data)
            fmt = (msg.format or "").lower()
            if "jpeg" in fmt or "jpg" in fmt or (
                    len(data) >= 2 and data[0] == 0xFF and data[1] == 0xD8):
                jpg = _reprocess(jpeg=data, resolution=self._res, quality=self._quality)
            else:  # e.g. png — decode then re-encode to JPEG
                bgr = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
                jpg = _reprocess(bgr=bgr, resolution=self._res,
                                 quality=self._quality or 80) if bgr is not None else None
            if jpg:
                self._on_frame(jpg)
        except Exception as e:
            if self._log:
                self._log.warn(f"G1 compressed image error: {e}")

    def close(self):
        for destroy, ent in ((self._node.destroy_subscription, self._sub),
                             (self._node.destroy_timer, self._disc_timer)):
            if ent is not None:
                try:
                    destroy(ent)
                except Exception:
                    pass
        self._sub = self._disc_timer = None


class HttpStreamSource(_ParamSource):
    """Read an MJPEG stream over HTTP instead of the robot's DDS.

    THE POINT: this source needs no ROS entities and no DDS at all, so the bridge keeps
    working when the robot is NOT on this machine's subnet — which is the normal case once
    the robot is itinerant (field, Starlink, LTE). DDS cannot cross a subnet boundary on
    these robots: measured 122 topics from the robot's own subnet, 2 from another one, 3
    even with explicit unicast peers. See robot-splunk-docs/RED-Y-DDS.md.

    The video already leaves the robot as H.264 (encoded in hardware on its Jetson, pushed
    over RTMP to mediamtx) and Frigate re-serves it as multipart/x-mixed-replace MJPEG. The
    frames arriving here are therefore ALREADY JPEG, so with the default quality=0 they are
    forwarded untouched — no decode, no re-encode, cheaper than the DDS path.

    Runs its own reader thread: an HTTP body read blocks, which must never happen on the ROS
    executor thread.
    """

    def __init__(self, node, on_frame, url, fps=15, resolution="native", quality=0,
                 logger=None):
        super().__init__(fps, resolution, quality)
        self._on_frame = on_frame
        self._url = url
        self._log = logger
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="http-stream",
                                        daemon=True)
        self._thread.start()
        if logger:
            logger.info(f"[camera] HTTP stream source: {url}")

    # A stream that delivered for this long counts as healthy: whatever ended it was a
    # blip, not a broken configuration, so the next reconnect starts from the short delay.
    _HEALTHY_S = 5.0

    def _run(self):
        backoff = 1.0
        while not self._stop.is_set():
            started = time.monotonic()
            # A reconnect starts a new stream with a new timebase, so the lag measured
            # against the old one means nothing. The PEAK is deliberately not reset: it is
            # the only record that the last stream went bad, and clearing it on reconnect
            # would hide exactly the failure this is here to catch.
            self._lag = 0.0
            self._last_lag_warn = 0.0
            try:
                self._read_stream()
            except Exception as exc:
                # Reset the backoff on a stream that ACTUALLY WORKED for a while.
                #
                # The reset used to sit right after `self._read_stream()`, which never
                # returns during normal operation — it only returns once a stop has been
                # requested — so the reset was unreachable and the backoff only ever grew:
                # 1, 2, 4, 8, 15, 15, 15… for the rest of the process's life. Measured on
                # the robot 2026-09-10: the retries in the log were pinned at 15 s, so every
                # network blip cost fifteen seconds of black screen while driving.
                if time.monotonic() - started >= self._HEALTHY_S:
                    backoff = 1.0
                if self._log and not self._stop.is_set():
                    self._log.warn(f"[camera] stream {self._url} failed: {exc}; "
                                   f"retry in {backoff:.0f}s")
                self._stop.wait(backoff)
                backoff = min(backoff * 2, 15.0)

    def _read_stream(self):
        # Scan for JPEG SOI/EOI markers rather than parsing multipart boundaries: it is
        # boundary-name agnostic, so the same code handles Frigate, mediamtx and any
        # generic MJPEG endpoint.
        req = urllib.request.Request(self._url, headers={"User-Agent": "ai-vl-bridge"})
        with urllib.request.urlopen(req, timeout=10) as resp:
            buf = b""
            while not self._stop.is_set():
                # read1(), NOT read(): read(n) on a buffered HTTP body blocks until it has
                # ALL n bytes, so the tail of one frame sits waiting for the head of the
                # NEXT one to fill the buffer — a full frame period of latency added to
                # every single frame, on the path you steer by.
                #
                # Measured 2026-09-10 against the robot, same stream, same 15 s, the only
                # difference being this call: read(8192) delivered frames 227.5 ms p50
                # after the robot published them; read1(8192) delivered them in 14.2 ms.
                # 213 ms, for one word.
                #
                # mjpeg_server.pump() on the robot documents this exact trap and avoids it.
                # This is its sibling copy of the same SOI/EOI scanner (see the module
                # docstring) and it had drifted. If you touch one, check the other.
                chunk = resp.read1(8192)
                if not chunk:
                    raise OSError("stream closed by peer")
                buf += chunk
                # Scan out every COMPLETE frame this read made available, then forward
                # only the NEWEST of them.
                #
                # WHY, and this is the whole "slow motion" symptom: TCP stalls (the tunnel
                # reorders ~2% of segments, measured 2026-09-11) and then delivers a
                # backlog all at once. Forwarding that backlog frame by frame means the
                # operator watches the queue drain — old pictures, in order, at the wrong
                # time. For a view someone steers by, every frame but the last one is
                # already worthless the moment a newer one exists.
                #
                # mjpeg_server's `Latest` on the robot says the same thing in its
                # docstring: "deliberately not a queue: a queue is how latency
                # accumulates". This is the sibling copy of that discipline, and it was
                # missing here.
                newest = None
                while True:
                    start = buf.find(b"\xff\xd8")
                    if start < 0:
                        # Keep the tail: a marker can straddle two chunks.
                        buf = buf[-1:]
                        break
                    end = buf.find(b"\xff\xd9", start + 2)
                    if end < 0:
                        if start > 0:
                            buf = buf[start:]
                        break
                    newest = buf[start:end + 2]
                    buf = buf[end + 2:]
                if newest is not None and self._due():
                    jpg = _reprocess(jpeg=newest, resolution=self._res,
                                     quality=self._quality)
                    if jpg:
                        self._on_frame(jpg)

    def close(self):
        self._stop.set()


class TestPatternSource(_ParamSource):
    """Synthetic moving frame (no robot) — verifies the pipeline end to end."""

    def __init__(self, node, on_frame, fps=15, resolution="native", quality=70,
                 logger=None):
        super().__init__(fps, resolution, quality)
        self._node = node
        self._on_frame = on_frame
        self._x = 0
        self._timer = node.create_timer(1.0 / self._POLL_HZ, self._tick)

    def _tick(self):
        if not self._due():
            return
        img = np.full((360, 640, 3), 32, np.uint8)
        self._x = (self._x + 12) % 640
        cv2.rectangle(img, (self._x, 150), (self._x + 80, 230), (0, 180, 255), -1)
        cv2.putText(img, "ROBOT CAM TEST", (18, 48),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
        jpg = _reprocess(bgr=img, resolution=self._res, quality=self._quality or 70)
        if jpg:
            self._on_frame(jpg)

    def close(self):
        try:
            self._node.destroy_timer(self._timer)
        except Exception:
            pass


class _LagWatchdog:
    """Is THIS reader the one adding latency? Shared by both H.264 sources.

    A *pull* reader slower than its source does not settle at a lower frame rate: the
    backlog upstream grows without bound and the latency climbs forever. That failure is
    invisible for the first seconds and obvious only after minutes, which is exactly how it
    shipped once and took the drive view to ~8 s.

    The measure needs no synchronised clock, only two elapsed times compared:

        lag = (our monotonic seconds since the first frame)
            - (the stream's own presentation seconds over the same frames)

    Both start at the same frame, so the unknown offset between the two clocks cancels and
    only their RATES are compared. Keeping up holds this at ~0; falling behind grows it by
    exactly the latency being accumulated. Absolute glass-to-glass is a different question
    and this does not answer it — this answers "am I the one adding to it".

    Lives here rather than in either source because both readers decode a stream they pull
    (RTSP through OpenCV, WHEP through aiortc) and the question is identical for both; at
    the second copy, factor it.
    """

    # Lag past which the reader is told it is losing the race. One second is already far
    # more than the whole rest of the path costs, so anything over it is a defect and not
    # jitter; the log is throttled to this many seconds so a bad night is not a log flood.
    _LAG_WARN_S = 1.0
    _LAG_WARN_EVERY_S = 30.0
    # Named in the warning, so a log line says WHICH reader fell behind when both exist.
    _LAG_LABEL = "reader"

    def _init_lag(self):
        self._lag = 0.0          # seconds this reader is behind the stream's own timebase
        self._lag_peak = 0.0     # worst since the process started, for after-the-fact triage
        self._last_lag_warn = 0.0

    def _reset_lag(self):
        """A reconnect starts a new stream with a new timebase, so the lag measured against
        the old one means nothing. The PEAK is deliberately NOT reset: it is the only record
        that the last stream went bad, and clearing it on reconnect would hide exactly the
        failure this is here to catch."""
        self._lag = 0.0
        self._last_lag_warn = 0.0

    def _track_lag(self, elapsed_s, stream_s):
        """Fold one frame into the measure above and warn, throttled, once it is a defect."""
        self._lag = elapsed_s - stream_s
        self._lag_peak = max(self._lag_peak, self._lag)
        if self._lag < self._LAG_WARN_S or not self._log:
            return
        now = time.monotonic()
        if now - self._last_lag_warn < self._LAG_WARN_EVERY_S:
            return
        self._last_lag_warn = now
        self._log.warn(
            f"[camera] {self._LAG_LABEL} is {self._lag:.1f}s behind its source and cannot "
            f"catch up on its own (peak {self._lag_peak:.1f}s). Frames are queueing "
            f"upstream, not being dropped, so this latency is permanent until the stream "
            f"is reopened.")

    def get_params(self):
        # Surfaced through the bridge's /status so the lag is visible WITHOUT reading logs —
        # this is the number that decides whether the drive view can be trusted.
        return {**super().get_params(),
                "lag_s": round(self._lag, 2), "lag_peak_s": round(self._lag_peak, 2)}


class RtspStreamSource(_LagWatchdog, _ParamSource):
    """Read the robot's H.264 from mediamtx, so the robot sends ONE stream and not two.

    WHY THIS EXISTS. The robot already encodes H.264 in hardware and pushes it to mediamtx.
    Pulling MJPEG off the robot as well means the same picture leaves it twice, and the MJPEG
    copy is the expensive one: measured 2026-09-11, 8.9 Mbps PER VIEWER against 1.4 Mbps for
    the H.264, because the robot serves a full copy to every HTTP viewer while mediamtx fans
    the H.264 out here at HQ. Taking detection off the robot's MJPEG is what lets that second
    stream be switched off entirely.

    NOT via Frigate. Frigate re-serves the same H.264 as MJPEG and would need no decoder
    here, but it is an NVR and buffers on purpose — ~7 s, measured. Boxes drawn from a
    7-second-old frame over a 200 ms live view are worse than no boxes. mediamtx's RTSP is
    the short path.

    The trade is a decode: frames arrive as H.264, so unlike the MJPEG sources there is no
    pass-through and every frame is decoded and re-encoded to JPEG for the backend. That
    costs a few ms per frame on a workstation, and it happens at HQ rather than on the
    robot's Jetson, which is the machine that has no headroom.

    ⚠️ DO NOT POINT THE DRIVE VIEW AT THIS. Reading mediamtx with OpenCV costs 2455 ms of
    FIXED delay — measured 2026-09-15 two independent ways, constant from the first frame,
    not accumulating, and unmoved by every FFmpeg option tried (probesize, analyzeduration,
    max_delay, reorder_queue_size, threads, TCP vs UDP) or by mediamtx's writeQueueSize. It
    is the CLIENT, not the stream: three consumers of the same path at the same time gave
    WebRTC/WHEP 200 ms, Frigate's own ffmpeg 1475 ms, this reader 2455 ms. Root cause still
    unknown. `WhepStreamSource` below is the reader for anything a human steers by; this one
    stays for recording-shaped consumers and as the fallback when aiortc is not installed.
    """

    # Matches HttpStreamSource: a stream that ran this long was working, so whatever ended
    # it was a blip and the next reconnect starts from the short delay.
    _HEALTHY_S = 5.0
    # FFmpeg options, passed the only way OpenCV accepts them. tcp because the tunnel to HQ
    # reorders; nobuffer/low_delay because the default is tuned for playback smoothness, not
    # for a view someone steers by; stimeout so a dead link raises instead of blocking the
    # reader thread forever.
    _FFMPEG_OPTS = "rtsp_transport;tcp|fflags;nobuffer|flags;low_delay|stimeout;5000000"
    _LAG_LABEL = "rtsp reader"

    def __init__(self, node, on_frame, url, fps=15, resolution="native", quality=0,
                 logger=None):
        # quality 0 means "forward the source JPEG untouched" everywhere else; here there is
        # no source JPEG, so it would mean "encode at the _reprocess default". Pin it to
        # something explicit instead of inheriting that surprise.
        super().__init__(fps, resolution, quality or 75)
        self._on_frame = on_frame
        self._url = url
        self._log = logger
        self._stop = threading.Event()
        self._init_lag()
        self._thread = threading.Thread(target=self._run, name="rtsp-stream", daemon=True)
        self._thread.start()
        if logger:
            logger.info(f"[camera] RTSP/H.264 source: {url}")

    def _run(self):
        backoff = 1.0
        while not self._stop.is_set():
            started = time.monotonic()
            self._reset_lag()
            try:
                self._read_stream()
            except Exception as exc:
                if time.monotonic() - started >= self._HEALTHY_S:
                    backoff = 1.0
                if self._log and not self._stop.is_set():
                    self._log.warn(f"[camera] rtsp {self._url} failed: {exc}; "
                                   f"retry in {backoff:.0f}s")
                self._stop.wait(backoff)
                backoff = min(backoff * 2, 15.0)

    def _read_stream(self):
        os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = self._FFMPEG_OPTS
        cap = cv2.VideoCapture(self._url, cv2.CAP_FFMPEG)
        # One frame of decoder buffer. Anything deeper is latency that arrives as a burst of
        # stale pictures after a stall — the "slow motion" symptom the MJPEG source documents.
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        try:
            if not cap.isOpened():
                raise OSError("could not open stream")
            # grab() first, retrieve() only for the frames actually forwarded.
            #
            # What this saves, precisely, because it was once believed to save more. In
            # OpenCV's FFmpeg backend grab() still DECODES the frame; retrieve() is only the
            # YUV->BGR conversion. So this removes the conversion and the JPEG encode from
            # every dropped frame — real work, and it took the encode rate from 12.91 to
            # 4.28 fps at a 5 fps gate — but it does NOT speed up consumption, and measured
            # it did not move the consumed rate at all (13.7 fps either way).
            #
            # It is therefore NOT what keeps this reader from falling behind. What does is
            # that the source is LOCAL: mediamtx runs on this same machine (the devcontainer
            # is on host networking), so this hop never crosses the robot link, and a bare
            # decode of 720p H.264 here drains a backlog several times faster than real time.
            # Measured 2026-09-14 with tests/reader_bench.py, 180 s against the live stream:
            # drift +0.0 ms/s, and a deliberately injected 12 s stall recovered in under 2 s.
            # The watchdog below is there to notice the day that stops being true.
            t0 = pts0 = None
            while not self._stop.is_set():
                if not cap.grab():
                    raise OSError("stream ended")
                # POS_MSEC is 0/garbage until the first frame is fully decoded, and a
                # non-advancing timestamp would fake a perfect 1:1 climb, so wait for a
                # positive one before anchoring.
                pts_ms = cap.get(cv2.CAP_PROP_POS_MSEC)
                if pts_ms > 0:
                    now = time.monotonic()
                    if t0 is None:
                        t0, pts0 = now, pts_ms
                    else:
                        self._track_lag(now - t0, (pts_ms - pts0) / 1000.0)
                if not self._due():
                    continue                      # drained, not decoded, not encoded
                ok, bgr = cap.retrieve()
                if not ok:
                    continue
                self._on_frame(_reprocess(bgr=bgr, resolution=self._res,
                                          quality=self._quality))
        finally:
            cap.release()

    def close(self):
        self._stop.set()


class WhepStreamSource(_LagWatchdog, _ParamSource):
    """Read the robot's H.264 from mediamtx over WebRTC/WHEP — the only client of that
    stream that is measured in milliseconds.

    WHY THIS EXISTS, and it is worth stating plainly because it undoes a decision made twice.
    The drive view, YOLO and the VLM all eat from this bridge, and the bridge read the
    robot's own MJPEG over HTTP — a SECOND copy of the same picture, over TCP, across the
    field link. Measured over LTE on 2026-09-16:

        H.264 over SRT   11.0 -> 14.2 fps      the transport that discards and moves on
        MJPEG over TCP    4.5 fps              one lost packet blocks everything behind it

    So detection ran at a third of the frame rate of the picture the operator was watching,
    and it cost 0.170 Mbps of a ~0.93 Mbps uplink (measured on the socket, same day) to do
    it. Reading the H.264 that already arrived costs the robot NOTHING: mediamtx is on this
    machine and fans it out over loopback.

    WHY NOT RTSP, which is the same stream from the same server: 2455 ms of fixed delay, see
    the warning on `RtspStreamSource`. WHEP is the path the browser already uses at ~200 ms,
    and using it here makes the bridge see exactly what the operator sees.

    WHAT IT COSTS. H.264 in, JPEG out, so every frame is decoded (aiortc/PyAV) and
    re-encoded. That is real CPU — but it is spent on the workstation, which has headroom,
    instead of on the robot's Jetson, which does not.

    THE PICTURE IS 1080p HERE. The MJPEG branch was pre-shrunk on the robot (MJPEG_WIDTH)
    because it crossed the link; this one does not cross anything, so set STREAM_RESOLUTION
    for what the consumers want rather than for what the uplink survives.
    """

    # Matches the other sources: a stream that ran this long was working, so whatever ended
    # it was a blip and the next reconnect starts from the short delay.
    _HEALTHY_S = 5.0
    _LAG_LABEL = "whep reader"
    # WebRTC has no connection to break: a dead uplink just stops delivering frames, with no
    # error anywhere. Without a deadline the reader would wait forever and the drive view
    # would sit frozen on its last picture. 5 s is ~70 frames at the robot's ~14 fps, so it
    # cannot fire on jitter.
    _RECV_TIMEOUT_S = 5.0
    # How often that wait wakes up. It is also what bounds close(): a source switch must not
    # sit behind a five-second read, and frames from a closed source must not reach the
    # backend after the new one starts.
    _STOP_POLL_S = 1.0
    # The SDP exchange is two small HTTP requests on loopback; anything slower is broken.
    _SIGNALING_TIMEOUT_S = 10.0
    # An SDP answer is a few kB. Bound the read so a misconfigured URL pointing at something
    # that is not a WHEP endpoint (an HTML error page, a file server) cannot be read into
    # memory unbounded.
    _MAX_SDP_BYTES = 256 * 1024
    # A missing dependency does not fix itself, so back off far further than a network blip.
    _MISSING_DEP_RETRY_S = 60.0

    def __init__(self, node, on_frame, url, fps=15, resolution="native", quality=0,
                 logger=None, ca_file=None):
        # quality 0 means "forward the source JPEG untouched" everywhere else; here there is
        # no source JPEG, so it would mean "encode at the _reprocess default". Pin it to
        # something explicit instead of inheriting that surprise.
        super().__init__(fps, resolution, quality or 75)
        self._on_frame = on_frame
        self._url = url
        self._ca_file = ca_file or None
        self._log = logger
        self._stop = threading.Event()
        self._init_lag()
        self._thread = threading.Thread(target=self._run, name="whep-stream", daemon=True)
        self._thread.start()
        if logger:
            logger.info(f"[camera] WHEP/H.264 source: {url}")

    def _run(self):
        backoff = 1.0
        while not self._stop.is_set():
            started = time.monotonic()
            self._reset_lag()
            try:
                asyncio.run(self._session())
            except ImportError as exc:
                # Retrying in a second cannot install a package, and at the short backoff
                # this would print the same line every second for as long as the bridge
                # runs. Say what to do, then wait long enough to be readable.
                if self._log and not self._stop.is_set():
                    self._log.warn(f"[camera] the WHEP source needs aiortc ({exc}); "
                                   f"pip3 install -r robot_camera_bridge/requirements.txt, "
                                   f"or point STREAM_URL at an MJPEG or RTSP source")
                self._stop.wait(self._MISSING_DEP_RETRY_S)
            except Exception as exc:
                if time.monotonic() - started >= self._HEALTHY_S:
                    backoff = 1.0
                if self._log and not self._stop.is_set():
                    self._log.warn(f"[camera] whep {self._url} failed: {exc}; "
                                   f"retry in {backoff:.0f}s")
                self._stop.wait(backoff)
                backoff = min(backoff * 2, 15.0)

    async def _session(self):
        # Imported HERE and not at module level on purpose. The test suite and CI run with
        # only ruff and pytest installed (see tests/conftest.py), and a module-level aiortc
        # would make every source in this file unimportable there — including the ones that
        # have nothing to do with WebRTC. It also keeps the dependency optional for a
        # deployment that never selects this source.
        from aiortc import RTCPeerConnection, RTCSessionDescription

        pc = RTCPeerConnection()
        tracks = asyncio.Queue()
        resource = None

        @pc.on("track")
        def _on_track(track):
            tracks.put_nowait(track)

        try:
            # recvonly, and video only: we are a viewer, and the robot publishes no audio.
            pc.addTransceiver("video", direction="recvonly")
            await pc.setLocalDescription(await pc.createOffer())
            # Non-trickle: setLocalDescription has already gathered the candidates, so the
            # offer we post is complete and there is no second signalling channel to keep
            # open. urllib blocks, so it runs off the event loop.
            answer, resource = await asyncio.get_running_loop().run_in_executor(
                None, self._exchange_sdp, pc.localDescription.sdp)
            await pc.setRemoteDescription(RTCSessionDescription(sdp=answer, type="answer"))
            track = await asyncio.wait_for(tracks.get(), self._SIGNALING_TIMEOUT_S)
            await self._pump(track)
        except asyncio.TimeoutError as exc:
            raise OSError("the server answered but sent no video track") from exc
        finally:
            await pc.close()
            self._release(resource)

    def _exchange_sdp(self, offer):
        """POST the offer, return (answer SDP, resource URL) — WHEP's whole handshake.

        Checked rather than trusted, because the failure this catches is silent: a URL that
        is not a WHEP endpoint answers 200 with HTML, which `setRemoteDescription` would
        then reject with a parser error that says nothing about the real mistake.
        """
        req = urllib.request.Request(
            self._url, data=offer.encode(), method="POST",
            headers={"Content-Type": "application/sdp", "User-Agent": "ai-vl-bridge"})
        with urllib.request.urlopen(req, timeout=self._SIGNALING_TIMEOUT_S,
                                    context=self._tls_context()) as resp:
            if resp.status != 201:
                raise OSError(f"WHEP endpoint answered {resp.status}, expected 201 Created")
            body = resp.read(self._MAX_SDP_BYTES + 1)
            if len(body) > self._MAX_SDP_BYTES:
                raise OSError(f"WHEP answer exceeds {self._MAX_SDP_BYTES} bytes; "
                              f"this is not an SDP document")
            sdp = body.decode("utf-8", "replace")
            if not sdp.startswith("v="):
                raise OSError(f"WHEP answer is not SDP (starts with {sdp[:20]!r})")
            return sdp, resp.headers.get("Location")

    def _release(self, resource):
        """DELETE the session resource — WHEP's teardown, and not optional here.

        Without it mediamtx keeps the reader until ICE times out, so a reconnect loop stacks
        sessions on top of each other and every one of them is still being sent video over
        loopback. Best-effort: a teardown that fails must never stop the reconnect.
        """
        if not resource:
            return
        req = urllib.request.Request(urllib.parse.urljoin(self._url, resource),
                                     method="DELETE")
        try:
            with urllib.request.urlopen(req, timeout=self._SIGNALING_TIMEOUT_S,
                                        context=self._tls_context()):
                pass
        except (urllib.error.URLError, OSError) as exc:
            if self._log:
                self._log.warn(f"[camera] whep session not released: {exc}")

    def _tls_context(self):
        """Turn `_tls_policy` into the context urlopen wants (None = its own default)."""
        policy = _tls_policy(self._url, self._ca_file)
        if policy == "plain" or policy == "verify":
            return None                      # http, or https with default verification
        if policy == "pin":
            return ssl.create_default_context(cafile=self._ca_file)
        ctx = ssl.create_default_context()
        ctx.check_hostname = False
        ctx.verify_mode = ssl.CERT_NONE
        return ctx

    async def _pump(self, track):
        """Decode every frame, forward the ones the rate gate lets through.

        EVERY frame is received, always — the gate drops AFTER the decode, never before.
        aiortc hands frames over an unbounded queue, so a reader that skipped receiving
        would not slow down, it would fall behind forever; that is the failure `_track_lag`
        watches for. What the gate saves is the colour conversion and the JPEG encode, which
        is the expensive half.
        """
        t0 = pts0 = None
        while not self._stop.is_set():
            frame = await self._recv(track)
            if frame is None:
                return                       # a stop was asked for, not a failure
            now = time.monotonic()
            if frame.pts is not None and frame.time_base:
                stream_s = float(frame.pts * frame.time_base)
                if t0 is None:
                    t0, pts0 = now, stream_s
                else:
                    self._track_lag(now - t0, stream_s - pts0)
            if not self._due():
                continue
            jpg = _reprocess(bgr=frame.to_ndarray(format="bgr24"),
                             resolution=self._res, quality=self._quality)
            # Checked again after the encode: close() can land while a frame is in flight,
            # and a switched-away source must not push pictures into the new source's view.
            if jpg and not self._stop.is_set():
                self._on_frame(jpg)

    async def _recv(self, track):
        """One frame, or None if a stop was requested. Raises once the stream is dead.

        Waits in short slices instead of one long one so that close() takes effect in
        `_STOP_POLL_S` rather than in `_RECV_TIMEOUT_S`, while a genuinely dead stream still
        raises after the full deadline.
        """
        waited = 0.0
        while not self._stop.is_set():
            try:
                return await asyncio.wait_for(track.recv(), self._STOP_POLL_S)
            except asyncio.TimeoutError:
                waited += self._STOP_POLL_S
                if waited >= self._RECV_TIMEOUT_S:
                    raise OSError(f"no frame for {self._RECV_TIMEOUT_S:.0f}s; "
                                  f"the stream is gone") from None
        return None

    def close(self):
        self._stop.set()



def build_source(robot, node, on_frame, cfg, logger=None):
    """Factory: pick the CameraSource for `robot` using cfg (env-derived)."""
    if robot == "go2":
        # Default quality 0 = forward the robot's own JPEG untouched (decode-free).
        return Go2VideoApiSource(
            node, on_frame,
            fps=float(cfg.get("GO2_VIDEO_FPS", 15)),
            resolution=cfg.get("GO2_RESOLUTION", "native"),
            quality=0, logger=logger)
    if robot == "g1":
        # This G1 firmware exposes the Unitree videohub (ready JPEGs, efficient) just
        # like the Go2, and its RealSense raw topic (/camera/color/image_raw) is
        # often idle (0 Hz) -> a "frozen" frame. Default to the videohub; opt into
        # the ROS2 image topic with G1_CAMERA_SOURCE=topic (or by setting
        # G1_IMAGE_TOPIC), e.g. once the RealSense node is actually streaming.
        use_topic = (cfg.get("G1_CAMERA_SOURCE", "").lower() == "topic"
                     or bool(cfg.get("G1_IMAGE_TOPIC")))
        if use_topic:
            return G1ImageTopicSource(
                node, on_frame,
                topic=cfg.get("G1_IMAGE_TOPIC", ""),  # "" => auto-discover
                fps=float(cfg.get("G1_VIDEO_FPS", 12)),
                resolution=cfg.get("G1_RESOLUTION", "native"),
                quality=int(cfg.get("JPEG_QUALITY", 70) or 70), logger=logger)
        return Go2VideoApiSource(
            node, on_frame,
            fps=float(cfg.get("G1_VIDEO_FPS", 12)),
            resolution=cfg.get("G1_RESOLUTION", "native"),
            quality=0, logger=logger)
    if robot == "stream":
        # No robot subnet required: reads the video that already left the robot. The URL
        # picks the reader, so moving between transports is one line in .env — no new mode
        # to remember, and the existing go2|g1|stream|test switch keeps working untouched.
        #
        # WHICH URL TO USE, measured and not a matter of taste:
        #   .../whep  -> WebRTC, ~200 ms. The drive view, YOLO and the VLM all read this.
        #   rtsp://   -> the same stream from the same server, 2455 ms FIXED. Recording only.
        #   http://   -> MJPEG. Costs the robot a second copy of the picture over the field
        #               link; it is what WHEP replaced. Kept for Frigate and for the robot's
        #               own :8093 when there is no mediamtx.
        url = cfg.get("STREAM_URL", "http://127.0.0.1:5000/api/robot")
        kwargs = {}
        if url.startswith(("rtsp://", "rtsps://")):
            source = RtspStreamSource
        elif _is_whep_url(url):
            source = WhepStreamSource
            kwargs["ca_file"] = cfg.get("STREAM_TLS_CA", "")
        else:
            source = HttpStreamSource
        return source(
            node, on_frame,
            url=url,
            fps=float(cfg.get("STREAM_FPS", 15)),
            resolution=cfg.get("STREAM_RESOLUTION", "native"),
            quality=int(cfg.get("STREAM_QUALITY", 0) or 0), logger=logger, **kwargs)
    if robot == "test":
        return TestPatternSource(
            node, on_frame, fps=float(cfg.get("TEST_FPS", 15)),
            quality=int(cfg.get("JPEG_QUALITY", 70) or 70), logger=logger)
    raise ValueError(f"unknown robot camera source '{robot}' (go2|g1|stream|test)")
