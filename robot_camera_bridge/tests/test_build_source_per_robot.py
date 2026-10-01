"""`build_source` with per-robot stream URLs (GO2_STREAM_URL / G1_STREAM_URL).

The rule: a robot with its own URL is read over the network; a robot without one stays on DDS,
exactly as before the URLs existed. Broken either way, the failure is silent — a black camera,
or a robot on its subnet suddenly reading a URL nobody configured — so both sides are pinned.
Sources are not constructed for real here: they would open sockets and ROS entities.
"""
from __future__ import annotations

import camera_sources as cs


def pick(robot, cfg, monkeypatch):
    seen = {}
    monkeypatch.setattr(cs, "_stream_source",
                        lambda url, *a, **k: seen.setdefault("stream", url))
    monkeypatch.setattr(cs, "Go2VideoApiSource",
                        lambda *a, **k: seen.setdefault("dds", True))
    cs.build_source(robot, node=None, on_frame=None, cfg=cfg)
    return seen


def test_g1_with_its_url_reads_the_network(monkeypatch):
    assert pick("g1", {"G1_STREAM_URL": "http://g1:8093/stream"}, monkeypatch) == \
        {"stream": "http://g1:8093/stream"}


def test_go2_with_its_url_reads_the_network(monkeypatch):
    assert pick("go2", {"GO2_STREAM_URL": "http://go2:8093/stream"}, monkeypatch) == \
        {"stream": "http://go2:8093/stream"}


def test_a_robot_never_takes_the_other_robots_url(monkeypatch):
    assert pick("g1", {"GO2_STREAM_URL": "http://go2:8093/stream"}, monkeypatch) == {"dds": True}


def test_without_a_url_both_stay_on_dds(monkeypatch):
    assert pick("go2", {}, monkeypatch) == {"dds": True}
    assert pick("g1", {}, monkeypatch) == {"dds": True}


def test_stream_mode_still_uses_stream_url(monkeypatch):
    assert pick("stream", {"STREAM_URL": "http://x/s"}, monkeypatch) == {"stream": "http://x/s"}
