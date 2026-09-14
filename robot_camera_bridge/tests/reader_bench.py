#!/usr/bin/env python3
"""
reader_bench.py — score a camera-source reader AGAINST ITS SOURCE, before wiring it in.

WHY THIS EXISTS. A *pull* reader slower than its source does not settle at a lower frame
rate: the backlog upstream grows without bound and the latency climbs forever. That is not
a theory — with `RtspStreamSource` reading 12.91 fps from a ~14 fps stream, the drive view
reached ~8 SECONDS before it was reverted. The failure is invisible in the first seconds and
obvious only after minutes, which is exactly why it shipped.

WHAT IT MEASURES, and why fps is not enough. Consumed fps alone cannot tell "the source is
slow" from "I am falling behind" — both read low. The decisive number is the DRIFT:

    drift(t) = wall_clock_now - stream_presentation_timestamp

The absolute value of that difference is meaningless (the two clocks share no origin, and
the stream's timebase starts wherever it starts). Its SLOPE is not: for a reader keeping up
it is flat, and for one falling behind it climbs by exactly the seconds of latency it is
accumulating per second. So this needs NO clock synchronisation with the robot, the encoder
or mediamtx — only our own monotonic clock against the stream's own timebase.

    slope ~ 0        keeping up; latency is bounded
    slope > 0        falling behind; every minute adds `slope * 60` s of latency

Reported as ms of accumulated latency per second of runtime, plus the total over the run.

USAGE (inside the devcontainer, which is where the bridge runs):
    python3 tests/reader_bench.py --url rtsp://127.0.0.1:8554/robot --seconds 60 read
    python3 tests/reader_bench.py --url ... --fps 5 grab
    python3 tests/reader_bench.py --url ... gst
    ... or `all` to score every strategy that this build can run, in sequence.
"""
import argparse
import sys
import threading
import time

import cv2
import numpy as np

# Same FFmpeg options the source uses, so the bench measures the reader and not a
# different transport.
_FFMPEG_OPTS = "rtsp_transport;tcp|fflags;nobuffer|flags;low_delay|stimeout;5000000"


class Drift:
    """Accumulates (wall - pts) samples and fits a slope by least squares."""

    def __init__(self):
        self.t = []      # seconds of runtime
        self.d = []      # wall - pts, seconds

    def add(self, runtime_s, pts_ms):
        # POS_MSEC is 0/garbage on some builds until the first frame is fully decoded;
        # a non-advancing pts would fake a perfect 1:1 climb, so drop non-positive ones.
        if pts_ms is None or pts_ms <= 0:
            return
        self.t.append(runtime_s)
        self.d.append(runtime_s - pts_ms / 1000.0)

    def slope_ms_per_s(self):
        """ms of latency accumulated per second of runtime. None if not enough samples."""
        if len(self.t) < 10:
            return None
        t = np.asarray(self.t)
        d = np.asarray(self.d)
        # Ignore the first 10% of the run: the decoder's start-up transient is not drift.
        keep = t >= t[-1] * 0.10
        if keep.sum() < 10:
            return None
        slope = np.polyfit(t[keep], d[keep], 1)[0]
        return slope * 1000.0

    def profile(self, bucket_s=10.0):
        """Median drift per time bucket. A slope fit smears a STEP into a gentle ramp, and
        the step is the interesting shape: a reader that merely keeps up cannot recover
        from one, so every stall it survives is latency it carries for the rest of the run."""
        if not self.t:
            return []
        out = []
        end = self.t[-1]
        b = 0.0
        while b < end:
            vals = [d for tt, d in zip(self.t, self.d, strict=True) if b <= tt < b + bucket_s]
            if vals:
                out.append((b, float(np.median(vals)) * 1000.0))
            b += bucket_s
        return out

    def total_ms(self):
        if len(self.d) < 2:
            return None
        head = float(np.median(self.d[: max(2, len(self.d) // 10)]))
        tail = float(np.median(self.d[-max(2, len(self.d) // 10):]))
        return (tail - head) * 1000.0


class Stall:
    """Injects ONE pause into the consumer, to reproduce what a real stall does.

    Real stalls are not hypothetical: iacore's first /detect call after a model load took
    3043 ms measured here, and while the backend is busy with it the backend stops reading
    the WS, so the bridge's blocking send_binary() stops the READER for as long as it
    lasts. This makes that event reproducible without waiting for one."""

    def __init__(self, at_s=0.0, for_s=0.0):
        self.at, self.dur, self.done = at_s, for_s, False

    def maybe(self, runtime_s):
        if self.done or self.dur <= 0 or runtime_s < self.at:
            return
        self.done = True
        time.sleep(self.dur)


def _gate(fps):
    """The source's rate gate, copied so the bench exercises the same shape."""
    state = {"passed": 0.0}
    period = 1.0 / max(1.0, fps)

    def due():
        now = time.monotonic()
        if now - state["passed"] < period * 0.90:
            return False
        state["passed"] = now
        return True
    return due


def _encode(bgr, quality=75):
    ok, buf = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, quality])
    return buf.tobytes() if ok else None


# --------------------------------------------------------------------------- strategies
# Each returns (consumed, forwarded, Drift). `consumed` is frames taken off the stream;
# `forwarded` is frames that reached the JPEG encode, i.e. what the backend would see.

def run_read(url, seconds, fps):
    """The ORIGINAL source: read() every iteration, gate afterwards. The one that reached
    ~8 s of latency. Kept so the bench can show the regression it is guarding against."""
    cap = _open(url)
    due, drift = _gate(fps), Drift()
    consumed = forwarded = 0
    t0 = time.monotonic()
    try:
        while time.monotonic() - t0 < seconds:
            ok, bgr = cap.read()
            if not ok:
                break
            consumed += 1
            drift.add(time.monotonic() - t0, cap.get(cv2.CAP_PROP_POS_MSEC))
            if due():
                _encode(bgr)
                forwarded += 1
    finally:
        cap.release()
    return consumed, forwarded, drift


def run_grab(url, seconds, fps, stall=None):
    """grab() always, retrieve()+encode only when the gate passes.

    NOTE what this does and does not save. In OpenCV's FFmpeg backend grab() still DECODES
    the frame; retrieve() is only the YUV->BGR conversion. So this removes the conversion
    and the JPEG encode from the dropped frames -- real work -- but NOT the decode, which is
    why it did not move the consumed rate at all (12.91 fps both before and after)."""
    cap = _open(url)
    due, drift = _gate(fps), Drift()
    consumed = forwarded = 0
    t0 = time.monotonic()
    try:
        while time.monotonic() - t0 < seconds:
            if not cap.grab():
                break
            consumed += 1
            drift.add(time.monotonic() - t0, cap.get(cv2.CAP_PROP_POS_MSEC))
            if stall:
                stall.maybe(time.monotonic() - t0)
            if not due():
                continue
            ok, bgr = cap.retrieve()
            if ok:
                _encode(bgr)
                forwarded += 1
    finally:
        cap.release()
    return consumed, forwarded, drift


def run_thread(url, seconds, fps):
    """A drain thread that ONLY grabs, and a consumer that retrieves the latest frame.

    This decouples the drain rate from the encode cost the way appsink does, WITHOUT any
    GStreamer plugin. Its ceiling is whatever a bare grab() loop can sustain -- if the
    decode alone is slower than the source, no amount of threading saves it, and the bench
    will say so in the slope."""
    cap = _open(url)
    due, drift = _gate(fps), Drift()
    stop = threading.Event()
    stats = {"consumed": 0, "forwarded": 0}
    lock = threading.Lock()
    t0 = time.monotonic()

    def drain():
        while not stop.is_set():
            if not cap.grab():
                stop.set()
                return
            frame = None
            with lock:
                stats["consumed"] += 1
                drift.add(time.monotonic() - t0, cap.get(cv2.CAP_PROP_POS_MSEC))
                if due():
                    ok, bgr = cap.retrieve()
                    if ok:
                        stats["forwarded"] += 1
                        frame = bgr
            if frame is not None:
                _encode(frame)          # outside the lock: the drain must not wait on it

    th = threading.Thread(target=drain, daemon=True)
    th.start()
    try:
        while time.monotonic() - t0 < seconds and not stop.is_set():
            time.sleep(0.1)
    finally:
        stop.set()
        th.join(timeout=3.0)
        cap.release()
    return stats["consumed"], stats["forwarded"], drift


_GST_PIPELINE = (
    "rtspsrc location={url} protocols=tcp latency=0 drop-on-latency=true "
    "! rtph264depay ! h264parse ! avdec_h264 ! videoconvert "
    "! appsink drop=true max-buffers=1 sync=false"
)


def run_gst(url, seconds, fps):
    """GStreamer with `appsink drop=true max-buffers=1`.

    The structural fix: GStreamer's own thread pulls the stream at ITS pace and the sink
    keeps exactly one frame, throwing away anything the consumer did not take. A reader
    that discards by design cannot accumulate -- falling behind costs freshness, never
    latency. Needs gst-plugins-{base,good,bad,libav} in the image."""
    pipeline = _GST_PIPELINE.format(url=url)
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        raise OSError("GStreamer pipeline would not open (plugins missing?)")
    due, drift = _gate(fps), Drift()
    consumed = forwarded = 0
    t0 = time.monotonic()
    try:
        while time.monotonic() - t0 < seconds:
            ok, bgr = cap.read()
            if not ok:
                break
            consumed += 1
            drift.add(time.monotonic() - t0, cap.get(cv2.CAP_PROP_POS_MSEC))
            if due():
                _encode(bgr)
                forwarded += 1
    finally:
        cap.release()
    return consumed, forwarded, drift


def run_ws(url, seconds, fps, ws_url=None, stall=None):
    """grab/retrieve/encode AND the real blocking send to the backend, on the reader thread.

    This is the shape the bridge actually runs: `CameraBridge.on_frame` calls
    `websocket.send_binary()` synchronously, under a lock, on whatever thread produced the
    frame -- for RtspStreamSource that is the reader thread itself. So a backend that is
    slow, a congested socket, or one TCP retransmit does not cost a frame, it STOPS THE
    READER, and the stream backs up behind it. Isolated readers cannot show this; only
    measuring with the send in the loop can.

    Also reports the worst single send, because the mean is not what breaks a reader: one
    500 ms stall per second is a 50% duty cycle no matter how good the average looks."""
    import ssl

    import websocket
    # Open the capture FIRST: the RTSP handshake takes ~2.4 s, and a socket opened before
    # it just sits there idle through the negotiation.
    cap = _open(url)
    ws = websocket.create_connection(
        ws_url, sslopt={"cert_reqs": ssl.CERT_NONE}, timeout=5, enable_multithread=True)
    ws.settimeout(None)

    def _drain_pings():
        """The bridge runs this and so must the bench. We only PUSH frames, but the server
        sends keepalive PINGs and websocket-client answers them from inside recv(); with
        nobody in recv() the connection is dropped ~40 s in. Found the hard way here: a
        30 s run passed and a 90 s run died with "socket is already closed"."""
        try:
            while True:
                ws.recv()
        except Exception:
            pass

    threading.Thread(target=_drain_pings, daemon=True).start()
    due, drift = _gate(fps), Drift()
    consumed = forwarded = 0
    send_times = []
    t0 = time.monotonic()
    try:
        while time.monotonic() - t0 < seconds:
            if not cap.grab():
                break
            consumed += 1
            drift.add(time.monotonic() - t0, cap.get(cv2.CAP_PROP_POS_MSEC))
            if stall:
                stall.maybe(time.monotonic() - t0)
            if not due():
                continue
            ok, bgr = cap.retrieve()
            if not ok:
                continue
            jpeg = _encode(bgr)
            if jpeg is None:
                continue
            t_send = time.monotonic()
            ws.send_binary(jpeg)
            send_times.append((time.monotonic() - t_send) * 1000.0)
            forwarded += 1
    finally:
        cap.release()
        try:
            ws.close()
        except Exception:
            pass
    if send_times:
        st = np.asarray(send_times)
        print(f"         send_binary: mean {st.mean():.2f} ms   "
              f"p95 {np.percentile(st, 95):.2f} ms   max {st.max():.1f} ms   "
              f"({len(st)} sends, {st.sum() / 1000.0:.1f}s of the {seconds:.0f}s run)")
    return consumed, forwarded, drift


STRATEGIES = {"read": run_read, "grab": run_grab, "thread": run_thread, "gst": run_gst,
              "ws": run_ws}


def _open(url):
    import os
    os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = _FFMPEG_OPTS
    cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not cap.isOpened():
        raise OSError(f"could not open {url}")
    return cap


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("strategy", nargs="+", choices=[*STRATEGIES, "all"])
    ap.add_argument("--url", default="rtsp://127.0.0.1:8554/robot")
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--fps", type=float, default=15.0, help="rate gate, as the bridge sets it")
    ap.add_argument("--ws-url", default="wss://localhost:8443/ws/robot-cam",
                    help="backend WS, for the `ws` strategy (the full bridge path)")
    ap.add_argument("--stall-at", type=float, default=0.0,
                    help="inject one pause this many seconds in (grab/ws only)")
    ap.add_argument("--stall-for", type=float, default=0.0, help="how long that pause lasts")
    ap.add_argument("--profile", action="store_true",
                    help="median drift per bucket -- shows STEPS, which a slope hides")
    ap.add_argument("--bucket", type=float, default=10.0,
                    help="profile bucket in seconds; make it small to time a recovery")
    args = ap.parse_args()

    # `all` deliberately excludes `ws`: it pushes real frames at the live backend, so it
    # is opt-in by name rather than something a broad sweep does behind your back.
    names = [n for n in STRATEGIES if n != "ws"] if "all" in args.strategy else args.strategy
    print(f"source: {args.url}    {args.seconds:.0f}s per strategy    gate {args.fps:g} fps\n")
    print(f"{'strategy':>8} {'consumed':>9} {'forwarded':>10} {'drift':>12} {'total':>10}")
    print(f"{'':>8} {'fps':>9} {'fps':>10} {'ms/s':>12} {'ms':>10}")
    print("-" * 54)
    for name in names:
        try:
            kw = {"ws_url": args.ws_url} if name == "ws" else {}
            if name in ("grab", "ws") and args.stall_for > 0:
                kw["stall"] = Stall(args.stall_at, args.stall_for)
            consumed, forwarded, drift = STRATEGIES[name](
                args.url, args.seconds, args.fps, **kw)
        except Exception as exc:
            print(f"{name:>8} {'-':>9} {'-':>10}   FAILED: {exc}")
            continue
        slope = drift.slope_ms_per_s()
        total = drift.total_ms()
        print(f"{name:>8} {consumed / args.seconds:>9.2f} {forwarded / args.seconds:>10.2f} "
              f"{(f'{slope:+.1f}' if slope is not None else 'n/a'):>12} "
              f"{(f'{total:+.0f}' if total is not None else 'n/a'):>10}")
        if args.profile:
            prof = drift.profile(args.bucket)
            base = prof[0][1] if prof else 0.0
            for b, val in prof:
                print(f"{'':>8}   t={b:5.0f}s   drift {val - base:+8.0f} ms (relative)")
    print("\ndrift ms/s is the verdict: ~0 keeps up, positive accumulates without bound.")
    print("A reader at +100 ms/s adds 6 s of latency per minute of driving.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
