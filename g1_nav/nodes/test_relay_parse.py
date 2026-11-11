"""relay_parse: the relay's odometry and obstacle lines into what relay_bridge publishes."""
import json
import math

import pytest
import relay_parse as rp


def odom_line(**kw):
    d = {"t": 100.0, "age_ms": 5, "x": 1.0, "y": -2.0, "z": 0.9, "yaw": 0.5,
         "vx": 0.3, "vy": 0.0, "wz": 0.1, "fsm": 500}
    d.update(kw)
    return json.dumps(d)


def test_odom_is_planar_and_keeps_the_twist():
    o = rp.parse_odom(odom_line())
    assert o == {"t": 100.0, "x": 1.0, "y": -2.0, "yaw": 0.5, "vx": 0.3, "vy": 0.0, "wz": 0.1}
    assert "z" not in o                          # Unitree's z drifts; Nav2 is 2D


@pytest.mark.parametrize("bad", [odom_line(age_ms=900), odom_line(age_ms=-1), "not json", "{}",
                                 odom_line(x=float("nan")), odom_line(yaw="a")])
def test_odom_lines_that_cannot_be_used(bad):
    assert rp.parse_odom(bad) is None


def lidar_line(**kw):
    d = {"t": 50.0, "age_ms": 40, "odom": 1, "res": 0.1, "pose": [0.5, -1.0, 0.25],
         "cells": [[10, -3], [0, 0]]}
    d.update(kw)
    return json.dumps(d)


def test_obstacle_cells_become_points_in_the_odometry_frame():
    t, pts, pose = rp.parse_obstacles(lidar_line())
    assert t == 50.0 and pose == (0.5, -1.0, 0.25)
    assert pts[0] == pytest.approx((1.0, -0.3, rp.OBSTACLE_Z))
    assert len(pts) == 2


@pytest.mark.parametrize("bad", [lidar_line(odom=0), lidar_line(age_ms=800), lidar_line(age_ms=-1),
                                 "{", json.dumps({"age_ms": 1, "odom": 1}),
                                 lidar_line(pose=[1, 2])])
def test_obstacle_lines_that_cannot_be_used(bad):
    assert rp.parse_obstacles(bad) is None


def test_an_empty_map_is_still_a_map():
    assert rp.parse_obstacles(lidar_line(cells=[]))[1] == []   # clears Nav2's costmap


def test_yaw_quaternion():
    q = rp.yaw_to_quat(math.pi / 2)
    assert q == pytest.approx((0, 0, math.sqrt(0.5), math.sqrt(0.5)))


def test_a_fresh_velocity_is_sent_as_a_move():
    body, stopped = rp.next_command((0.2, 0.0, -0.31415), 10.0, 10.1, stopped=True)
    assert body == {"verb": "move", "vx": 0.2, "vy": 0.0, "vyaw": -0.314}
    assert stopped is False


def test_zero_or_stale_sends_one_stop_then_nothing():
    for cmd, t in (((0.0, 0.0, 0.0), 10.0), ((0.3, 0.0, 0.0), 9.0), (None, 0.0)):
        body, stopped = rp.next_command(cmd, t, 10.0, stopped=False)
        assert body == {"verb": "stop_move"} and stopped
        assert rp.next_command(cmd, t, 10.0, stopped=True) == (None, True)   # not again


def test_moving_again_after_a_stop():
    _, stopped = rp.next_command(None, 0.0, 10.0, stopped=False)
    body, stopped = rp.next_command((0.0, 0.0, 0.4), 10.2, 10.25, stopped)
    assert body["verb"] == "move" and not stopped


def test_scan_beams_hit_the_nearest_obstacle_in_the_robot_frame():
    # Robot at (1, 1) facing +y (yaw 90 deg); an obstacle 2 m in front of it, one 1 m to its left.
    pts = [(1.0, 3.0, 0.5), (0.0, 1.0, 0.5), (1.0, 4.0, 0.5)]
    r = rp.obstacles_to_scan(pts, (1.0, 1.0, math.pi / 2))
    assert r[0] == pytest.approx(2.0)                 # straight ahead: the nearer of the two
    assert r[90] == pytest.approx(1.0)                # left = +90 deg, counter-clockwise
    assert math.isinf(r[180]) and math.isinf(r[270])  # nothing behind or right: clear


def test_scan_ignores_its_own_body_and_far_things():
    r = rp.obstacles_to_scan([(0.1, 0.0, 0.5), (9.0, 0.0, 0.5)], (0.0, 0.0, 0.0))
    assert all(math.isinf(v) for v in r)


def test_it_asks_for_the_way_when_waiting_and_not_on_every_retry():
    waiting = [("ClearLocalCostmap", "SUCCESS"), ("Wait", "RUNNING")]
    assert rp.should_say(waiting, 100.0, None)
    assert not rp.should_say(waiting, 110.0, 100.0)                    # 10 s later: quiet
    assert rp.should_say(waiting, 100.0 + rp.SAY_EVERY_S, 100.0)        # then again
    assert not rp.should_say([("Wait", "SUCCESS"), ("FollowPath", "RUNNING")], 100.0, None)


def test_phrases_never_repeat_back_to_back_and_all_get_said():
    seen, last = set(), None
    for k in range(200):
        i = rp.pick_phrase(5, last, rand=lambda k=k: (k * 0.37) % 1.0)
        assert 0 <= i < 5 and i != last
        seen.add(i)
        last = i
    assert seen == {0, 1, 2, 3, 4}
    assert rp.pick_phrase(1, 0) == 0


def test_a_relative_goal_lands_in_odom():
    x, y, yaw = rp.goal_from_relative((1.0, 2.0, math.pi / 2), 1.0, 0.5, math.pi)
    assert (x, y) == pytest.approx((0.5, 3.0))          # facing +y: forward is +y, left is -x
    assert yaw == pytest.approx(-math.pi / 2)            # wrapped


def test_nav_state_words():
    assert [rp.nav_state(c) for c in (4, 5, 6, 99)] == ["succeeded", "canceled", "aborted",
                                                         "aborted"]


def test_paths_are_thinned_for_the_page_keeping_both_ends():
    pts = [(i * 0.05, 0.0) for i in range(500)]
    out = rp.downsample_path(pts)
    assert len(out) == rp.PATH_MAX_POINTS
    assert out[0] == [0.0, 0.0] and out[-1] == [24.95, 0.0]
    assert rp.downsample_path([(0, 0), (1, 1)]) == [[0, 0], [1, 1]]


def test_blocked_follows_the_tree():
    assert rp.blocked_after([("ClearLocalCostmap", "RUNNING"), ("Wait", "RUNNING")], False)
    assert not rp.blocked_after([("Wait", "SUCCESS"), ("FollowPath", "RUNNING")], True)
    assert rp.blocked_after([("ComputePathToPose", "SUCCESS")], True)       # unchanged
