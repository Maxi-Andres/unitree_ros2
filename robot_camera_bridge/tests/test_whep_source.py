"""`WhepStreamSource` — the reader that took the drive view off the robot's second stream.

WHAT THESE COVER, and each one is a defect that already has a name here:

  * THE URL DECIDES THE READER. `.../whep` handed to the MJPEG reader does not fail: it
    scans an SDP document for JPEG markers and finds none, forever, with a black view and
    nothing in the log. The dispatch is one `if` and it is the only thing between a working
    drive view and that.
  * TLS VERIFICATION IS THE DEFAULT. mediamtx's WHEP is HTTPS with a self-signed cert, and
    the tempting fix is to switch verification off everywhere. The exception must stay
    pinned to loopback, where there is no network to intercept.
  * A DEAD UPLINK MUST RAISE. WebRTC has no connection to break — frames simply stop — so
    without a deadline the reader waits forever and the view sits frozen on its last
    picture. This is the failure that a `while True: recv()` would hide.
  * THE HANDSHAKE IS CHECKED, NOT TRUSTED. A URL that is not a WHEP endpoint answers 200
    with HTML; unchecked, that surfaces as an SDP parser error that says nothing about the
    real mistake.

Nothing here opens a socket or needs aiortc: the source is built without running `__init__`
(which would start a reader thread), and the aiortc import lives inside `_session`, which
these tests never reach. See `conftest.py`.
"""
from __future__ import annotations

import asyncio

import camera_sources
import pytest

# --------------------------------------------------------------------------- dispatch

@pytest.mark.parametrize("url, expected", [
    ("https://127.0.0.1:8889/robot/whep", True),
    ("http://127.0.0.1:8889/robot/whep", True),
    ("https://hq.example:8889/robot/whep/", True),      # a trailing slash is still WHEP
    ("https://127.0.0.1:8889/robot/whip", False),       # publishing, not playing
    ("http://10.1.254.18:8093/stream", False),          # the robot's MJPEG
    ("http://127.0.0.1:5000/api/robot", False),         # Frigate
    ("rtsp://127.0.0.1:8554/robot", False),
    ("https://127.0.0.1:8889/whepsomething", False),    # not a /whep path segment
])
def test_whep_urls_are_recognised(url, expected):
    assert camera_sources._is_whep_url(url) is expected


def test_build_source_picks_the_reader_from_the_url(monkeypatch):
    """The three stream readers are not interchangeable — picking the wrong one is silent.

    RTSP costs 2455 ms of fixed delay, MJPEG costs the robot a second copy over the field
    link, and WHEP is the one that is neither. `build_source` is where that choice is made.
    """
    built = {}

    def _fake(kind):
        def make(node, on_frame, url, **kw):
            built["kind"], built["url"], built["kw"] = kind, url, kw
            return object()
        return make

    monkeypatch.setattr(camera_sources, "WhepStreamSource", _fake("whep"))
    monkeypatch.setattr(camera_sources, "RtspStreamSource", _fake("rtsp"))
    monkeypatch.setattr(camera_sources, "HttpStreamSource", _fake("http"))

    for url, kind in (("https://127.0.0.1:8889/robot/whep", "whep"),
                      ("rtsp://127.0.0.1:8554/robot", "rtsp"),
                      ("http://10.1.254.18:8093/stream", "http")):
        camera_sources.build_source("stream", None, None, {"STREAM_URL": url})
        assert built["kind"] == kind, f"{url} was read by the {built['kind']} reader"
        assert built["url"] == url


def test_only_the_whep_reader_is_given_the_ca(monkeypatch):
    """STREAM_TLS_CA is a WHEP setting; handing it to a reader that takes no such argument
    is a TypeError at construction, i.e. no video at all."""
    seen = {}
    monkeypatch.setattr(camera_sources, "WhepStreamSource",
                        lambda node, on_frame, url, **kw: seen.update(whep=kw) or object())
    monkeypatch.setattr(camera_sources, "HttpStreamSource",
                        lambda node, on_frame, url, **kw: seen.update(http=kw) or object())

    cfg = {"STREAM_URL": "https://hq.example:8889/robot/whep", "STREAM_TLS_CA": "/etc/ca.pem"}
    camera_sources.build_source("stream", None, None, cfg)
    assert seen["whep"]["ca_file"] == "/etc/ca.pem"

    cfg["STREAM_URL"] = "http://10.1.254.18:8093/stream"
    camera_sources.build_source("stream", None, None, cfg)
    assert "ca_file" not in seen["http"]


# ------------------------------------------------------------------------------- TLS

@pytest.mark.parametrize("url, ca, expected", [
    ("https://127.0.0.1:8889/robot/whep", "", "loopback-insecure"),
    ("https://localhost:8889/robot/whep", "", "loopback-insecure"),
    ("https://[::1]:8889/robot/whep", "", "loopback-insecure"),
    # The one that matters: a remote mediamtx must NOT inherit the loopback exception.
    ("https://192.168.20.99:8889/robot/whep", "", "verify"),
    ("https://hq.example:8889/robot/whep", "", "verify"),
    # A pinned CA wins everywhere, loopback included — pinning is never silently ignored.
    ("https://127.0.0.1:8889/robot/whep", "/etc/ca.pem", "pin"),
    ("https://hq.example:8889/robot/whep", "/etc/ca.pem", "pin"),
    ("http://127.0.0.1:8889/robot/whep", "", "plain"),
])
def test_tls_policy(url, ca, expected):
    assert camera_sources._tls_policy(url, ca) == expected


@pytest.mark.parametrize("host", ["10.1.254.18", "192.168.20.99", "127.0.0.1.evil.com",
                                  "notanaddress", ""])
def test_hosts_that_are_not_loopback(host):
    """Anything unparseable counts as remote: the safe direction is "verify"."""
    assert camera_sources._is_loopback(host) is False


# ------------------------------------------------------------------------- handshake

class _Resp:
    """The shape urlopen returns: a context manager with a status, a bounded read and
    headers. Written here because the assertions are about what `_exchange_sdp` REJECTS."""

    def __init__(self, status=201, body=b"v=0\r\n", location="/robot/whep/abc"):
        self.status, self._body, self._loc = status, body, location
        self.headers = {"Location": location}

    def read(self, n=-1):
        return self._body if n < 0 else self._body[:n]

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False


@pytest.fixture
def source():
    """A source with no session behind it and no reader thread running."""
    src = object.__new__(camera_sources.WhepStreamSource)
    src._url = "https://127.0.0.1:8889/robot/whep"
    src._ca_file = None
    src._log = None
    src._stop = camera_sources.threading.Event()
    return src


def _answer(monkeypatch, resp):
    monkeypatch.setattr(camera_sources.urllib.request, "urlopen",
                        lambda req, **kw: resp)


def test_a_good_answer_returns_the_sdp_and_the_resource(source, monkeypatch):
    """The resource URL is what `_release` DELETEs; losing it leaks a mediamtx reader per
    reconnect, each one still being sent video."""
    _answer(monkeypatch, _Resp(body=b"v=0\r\no=- 1 1 IN IP4 0.0.0.0\r\n"))
    sdp, resource = source._exchange_sdp("v=0\r\n")
    assert sdp.startswith("v=0")
    assert resource == "/robot/whep/abc"


def test_a_200_is_not_a_whep_answer(source, monkeypatch):
    """WHEP answers 201 Created. A 200 is some other endpoint being agreeable."""
    _answer(monkeypatch, _Resp(status=200))
    with pytest.raises(OSError, match="expected 201"):
        source._exchange_sdp("v=0\r\n")


def test_html_is_refused_before_the_sdp_parser_sees_it(source, monkeypatch):
    """The real mistake is the URL; an SDP parser error would never say so."""
    _answer(monkeypatch, _Resp(body=b"<!DOCTYPE html><title>404</title>"))
    with pytest.raises(OSError, match="not SDP"):
        source._exchange_sdp("v=0\r\n")


def test_the_answer_is_bounded(source, monkeypatch):
    """An SDP is a few kB. Without a cap, a URL pointing at a file server reads the file
    into memory."""
    _answer(monkeypatch, _Resp(body=b"v=0" + b"a" * camera_sources.WhepStreamSource._MAX_SDP_BYTES))
    with pytest.raises(OSError, match="exceeds"):
        source._exchange_sdp("v=0\r\n")


# ------------------------------------------------------------------------ dead stream

class _SilentTrack:
    """A track that never delivers — an uplink that went away mid-drive."""

    async def recv(self):
        await asyncio.Event().wait()          # forever


class _OneFrameTrack:
    def __init__(self, frame):
        self._frame = frame

    async def recv(self):
        return self._frame


def test_a_stream_that_stops_delivering_raises(source, monkeypatch):
    """WebRTC has no connection to break: frames just stop. Without this deadline the reader
    thread waits forever, the reconnect never happens, and the drive view stays frozen on
    its last picture with nothing in the log."""
    monkeypatch.setattr(camera_sources.WhepStreamSource, "_STOP_POLL_S", 0.01)
    monkeypatch.setattr(camera_sources.WhepStreamSource, "_RECV_TIMEOUT_S", 0.05)
    with pytest.raises(OSError, match="no frame"):
        asyncio.run(source._recv(_SilentTrack()))


def test_close_is_not_held_up_by_a_silent_stream(source, monkeypatch):
    """close() must take effect while a read is in flight — a source switch that waits for
    the full deadline forwards frames from the OLD robot into the new source's view."""
    monkeypatch.setattr(camera_sources.WhepStreamSource, "_STOP_POLL_S", 0.01)
    monkeypatch.setattr(camera_sources.WhepStreamSource, "_RECV_TIMEOUT_S", 10.0)
    source._stop.set()
    assert asyncio.run(source._recv(_SilentTrack())) is None


def test_a_frame_that_arrives_is_returned(source):
    """The guard above must not eat the normal case."""
    assert asyncio.run(source._recv(_OneFrameTrack("frame"))) == "frame"
