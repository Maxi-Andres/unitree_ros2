#!/usr/bin/env python3
"""
go2_commands.py — pure skill -> Unitree Go2 sport-command mapping (no ROS deps).

Turns one interpreter skill (`{skill, params}`, the JSON that AI-VL's /command
produces) into a Go2 SportClient intent: the `/api/sport/request` api_id + parameter,
or a velocity Move. Kept dependency-free (no rclpy) so the mapping can be unit-tested
on any machine; the actual publishing lives in robot_executor_service.py.

api_ids and parameter shapes are taken verbatim from unitree_ros2's
example/src/include/common/ros2_sport_client.h + ros2_sport_client.cpp.
"""

# Categorical speed -> Move velocities (m/s, rad/s). Matches AI-VL's GO2_SPEED_PRESETS.
GO2_SPEED_PRESETS = {
    "slow":   {"vx": 0.3, "vyaw": 0.5},
    "normal": {"vx": 0.6, "vyaw": 1.0},
    "fast":   {"vx": 1.2, "vyaw": 2.0},
}
DEFAULT_SPEED = "slow"

# Safety clamps for direct velocity control (the `move` skill from the drive pad).
MAX_VX = 1.2    # m/s forward/back
MAX_VY = 0.8    # m/s strafe left/right
MAX_VYAW = 2.0  # rad/s yaw

MOVE_API_ID = 1008
STOPMOVE_API_ID = 1003


def _clamp(value, limit):
    try:
        v = float(value)
    except (TypeError, ValueError):
        return 0.0
    return max(-limit, min(limit, v))

# Skills that are a single no-parameter sport command.
GO2_SIMPLE_API = {
    "stop": STOPMOVE_API_ID,
    "stand_up": 1004,
    "balance_stand": 1002,
    "stand_down": 1005,
    "sit": 1009,
    "rise_sit": 1010,
    "recovery_stand": 1006,
    "damp": 1001,
    "hello": 1016,
    "stretch": 1017,
    "scrape": 1029,
    "heart": 1036,
    "dance1": 1022,
    "dance2": 1023,
    "front_jump": 1031,
    "front_pounce": 1032,
    "front_flip": 1030,
    "back_flip": 2043,
    "left_flip": 2041,
}

# Skills that take a boolean flag as {"data": bool} (params["on"], default True).
GO2_FLAG_API = {
    "handstand": 2044,
    "walk_upright": 2050,
    "pose": 1028,
}

# set_gait value -> (api_id, needs_flag). Some gait switches take {"data": true}.
GO2_GAIT_API = {
    "classic": (2049, True),
    "free_walk": (2045, False),
    "trot_run": (1062, False),
    "static_walk": (1061, False),
    "economic": (1063, False),
    "cross_step": (2051, True),
}

# Acrobatics that lift the body / risk a fall — blocked while SAFE_MODE is on.
DANGEROUS_SKILLS = {
    "front_flip", "back_flip", "left_flip", "front_jump", "front_pounce",
    "handstand", "walk_upright",
}


def _velocity_for(skill, params):
    """(vx, vy, vyaw) for a walk/turn skill from its direction + speed preset."""
    preset = GO2_SPEED_PRESETS.get(params.get("speed") or DEFAULT_SPEED,
                                   GO2_SPEED_PRESETS[DEFAULT_SPEED])
    if skill == "walk":
        direction = params.get("direction") or "forward"
        v = preset["vx"]
        return {"forward": (v, 0.0, 0.0), "backward": (-v, 0.0, 0.0),
                "left": (0.0, v, 0.0), "right": (0.0, -v, 0.0)}.get(
                    direction, (v, 0.0, 0.0))
    # turn
    direction = params.get("direction") or "left"
    w = preset["vyaw"]
    return (0.0, 0.0, w if direction == "left" else -w)


def resolve(skill, params):
    """Map a skill+params to a Go2 command intent.

    Returns one of:
      {"kind": "move", "vx","vy","vyaw", "duration": float|None, "continuous": bool}
      {"kind": "stop"}
      {"kind": "single", "api_id": int, "parameter": dict|None}
      {"kind": "unsupported", "reason": str}
    """
    params = params or {}

    # Direct velocity control (the drive-pad joysticks / WASD): raw vx/vy/vyaw,
    # continuous by default, clamped to the safety limits above.
    if skill == "move":
        duration = params.get("duration_s")
        return {"kind": "move",
                "vx": _clamp(params.get("vx"), MAX_VX),
                "vy": _clamp(params.get("vy"), MAX_VY),
                "vyaw": _clamp(params.get("vyaw"), MAX_VYAW),
                "duration": duration if isinstance(duration, (int, float)) else None,
                "continuous": bool(params.get("continuous", True))}

    if skill in ("walk", "turn"):
        vx, vy, vyaw = _velocity_for(skill, params)
        duration = params.get("duration_s")
        return {"kind": "move", "vx": vx, "vy": vy, "vyaw": vyaw,
                "duration": duration if isinstance(duration, (int, float)) else None,
                "continuous": bool(params.get("continuous", False))}

    if skill == "stop":
        return {"kind": "stop"}

    if skill in GO2_SIMPLE_API:
        return {"kind": "single", "api_id": GO2_SIMPLE_API[skill], "parameter": None}

    if skill in GO2_FLAG_API:
        on = params.get("on", True)
        return {"kind": "single", "api_id": GO2_FLAG_API[skill],
                "parameter": {"data": bool(on)}}

    if skill == "set_gait":
        gait = params.get("gait") or "classic"
        if gait not in GO2_GAIT_API:
            return {"kind": "unsupported", "reason": f"unknown gait '{gait}'"}
        api_id, needs_flag = GO2_GAIT_API[gait]
        return {"kind": "single", "api_id": api_id,
                "parameter": {"data": True} if needs_flag else None}

    return {"kind": "unsupported", "reason": f"skill '{skill}' not mapped for Go2"}
