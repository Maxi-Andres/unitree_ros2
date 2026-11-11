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
    d = {"t": 50.0, "age_ms": 40, "odom": 1, "res": 0.1, "pose": [0, 0, 0],
         "cells": [[10, -3], [0, 0]]}
    d.update(kw)
    return json.dumps(d)


def test_obstacle_cells_become_points_in_the_odometry_frame():
    t, pts = rp.parse_obstacles(lidar_line())
    assert t == 50.0
    assert pts[0] == pytest.approx((1.0, -0.3, rp.OBSTACLE_Z))
    assert len(pts) == 2


@pytest.mark.parametrize("bad", [lidar_line(odom=0), lidar_line(age_ms=800), lidar_line(age_ms=-1),
                                 "{", json.dumps({"age_ms": 1, "odom": 1})])
def test_obstacle_lines_that_cannot_be_used(bad):
    assert rp.parse_obstacles(bad) is None


def test_an_empty_map_is_still_a_map():
    assert rp.parse_obstacles(lidar_line(cells=[])) == (50.0, [])   # clears Nav2's costmap


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
