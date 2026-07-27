#!/usr/bin/env python3
"""
g1_commands.py — pure skill -> Unitree G1 command mapping (no ROS deps).

Turns one interpreter skill (`{skill, params}`) into a G1 intent the executor
publishes as a `unitree_api/msg/Request`. Unlike the Go2 (one sport topic), the G1
spans THREE request topics, so each intent names which publisher to use:
  - "sport" -> /api/sport/request : locomotion (SetVelocity) + FSM postures + the
               loco gestures WaveHand/ShakeHand.
  - "arm"   -> /api/arm/request   : preset arm actions (ExecuteAction by id).
  - "voice" -> /api/voice/request : audio / TTS.

Kept dependency-free (no rclpy) so the mapping is unit-testable anywhere. api_ids,
topics and parameter shapes are taken verbatim from unitree_ros2
example/src/include/g1/* and unitree_sdk2 include/unitree/robot/g1/* (loco/arm/audio).
"""

# --- Publisher names (the transport maps these to real topics) -------------- #
SPORT = "sport"
ARM = "arm"
VOICE = "voice"
SPORT_TOPIC = "/api/sport/request"
ARM_TOPIC = "/api/arm/request"
VOICE_TOPIC = "/api/voice/request"
TOPICS = {SPORT: SPORT_TOPIC, ARM: ARM_TOPIC, VOICE: VOICE_TOPIC}

# --- api_ids ---------------------------------------------------------------- #
SET_VELOCITY_API_ID = 7105     # loco: {"velocity":[vx,vy,vyaw],"duration":D}
SET_FSM_ID_API_ID = 7101       # loco: {"data":<fsm id>}
SET_BALANCE_MODE_API_ID = 7102  # loco: {"data":<mode>}
SET_STAND_HEIGHT_API_ID = 7104  # loco: {"data":<height>}
SET_ARM_TASK_API_ID = 7106      # loco gestures on the SPORT topic: {"data":<task>}
ARM_EXECUTE_ACTION_API_ID = 7106  # arm actions on the ARM topic: {"data":<action id>}
AUDIO_TTS_API_ID = 1001         # voice: {"index":n,"text":str,"speaker_id":0}
AUDIO_SET_VOLUME_API_ID = 1006  # voice: {"volume":0-100}

# SetFsmId ids (LocoClient) — the posture/FSM skills map straight to these.
FSM_IDS = {
    "zero_torque": 0,
    "damp": 1,
    "squat": 2,
    "sit": 3,
    "stand_up": 4,
    "start": 500,  # main operation control ("ready" state)
}
HIGH_STAND = 4294967295.0  # (float)UINT32_MAX sentinel -> tallest stand
LOW_STAND = 0.0            # UINT32_MIN sentinel -> lowest stand
# Loco gesture task ids (SET_ARM_TASK on the sport topic).
WAVE_TASK = 0
WAVE_TURN_TASK = 1
SHAKE_START_TASK = 2

# Arm preset actions -> ExecuteAction ids (matches command_common.ARM_ACTION_IDS).
ARM_ACTION_IDS = {
    "release_arm": 99,
    "two_hand_kiss": 11,
    "left_kiss": 12,
    "right_kiss": 12,
    "hands_up": 15,
    "clap": 17,
    "high_five": 18,
    "hug": 19,
    "heart": 20,
    "right_heart": 21,
    "reject": 22,
    "right_hand_up": 23,
    "x_ray": 24,
    "face_wave": 25,
    "high_wave": 26,
    "shake_hand": 27,
}

# Categorical speed -> Move velocities (m/s, rad/s). Conservative — the humanoid can
# fall. Matches AI-VL's G1_SPEED_PRESETS in command_common.
G1_SPEED_PRESETS = {
    "slow":   {"vx": 0.2, "vyaw": 0.3},
    "normal": {"vx": 0.4, "vyaw": 0.6},
    "fast":   {"vx": 0.7, "vyaw": 1.0},
}
DEFAULT_SPEED = "slow"

# Safety clamps for direct velocity control (the drive pad's `move` skill).
MAX_VX = 0.8
MAX_VY = 0.5
MAX_VYAW = 1.2

# Skills blocked while SAFE_MODE is on: zero-torque cuts all motor torque, so a
# standing robot collapses — only safe when it is secured/held.
DANGEROUS_SKILLS = {"zero_torque"}


def _clamp(value, limit):
    try:
        v = float(value)
    except (TypeError, ValueError):
        return 0.0
    return max(-limit, min(limit, v))


def _velocity_for(skill, params):
    """(vx, vy, vyaw) for a walk/turn skill from its direction + speed preset."""
    preset = G1_SPEED_PRESETS.get(params.get("speed") or DEFAULT_SPEED,
                                  G1_SPEED_PRESETS[DEFAULT_SPEED])
    if skill == "walk":
        direction = params.get("direction") or "forward"
        v = preset["vx"]
        return {"forward": (v, 0.0, 0.0), "backward": (-v, 0.0, 0.0),
                "left": (0.0, v, 0.0), "right": (0.0, -v, 0.0)}.get(
                    direction, (v, 0.0, 0.0))
    direction = params.get("direction") or "left"
    w = preset["vyaw"]
    return (0.0, 0.0, w if direction == "left" else -w)


def resolve(skill, params):
    """Map a skill+params to a G1 command intent.

    Returns one of:
      {"kind": "move", "vx","vy","vyaw", "duration": float|None, "continuous": bool}
      {"kind": "stop"}
      {"kind": "single", "pub": str, "api_id": int, "parameter": dict|None}
      {"kind": "say", "text": str}
      {"kind": "unsupported", "reason": str}
    """
    params = params or {}

    # Direct velocity control (drive-pad joysticks / WASD): raw vx/vy/vyaw.
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

    # --- Posture / FSM (SetFsmId on the sport topic) ----------------------- #
    if skill in FSM_IDS:
        return {"kind": "single", "pub": SPORT, "api_id": SET_FSM_ID_API_ID,
                "parameter": {"data": FSM_IDS[skill]}}
    if skill == "balance_stand":
        return {"kind": "single", "pub": SPORT, "api_id": SET_BALANCE_MODE_API_ID,
                "parameter": {"data": 0}}
    if skill == "high_stand":
        return {"kind": "single", "pub": SPORT, "api_id": SET_STAND_HEIGHT_API_ID,
                "parameter": {"data": HIGH_STAND}}
    if skill == "low_stand":
        return {"kind": "single", "pub": SPORT, "api_id": SET_STAND_HEIGHT_API_ID,
                "parameter": {"data": LOW_STAND}}

    # --- Loco gestures (SET_ARM_TASK on the sport topic) ------------------- #
    if skill == "wave_hand":
        task = WAVE_TURN_TASK if params.get("turn") else WAVE_TASK
        return {"kind": "single", "pub": SPORT, "api_id": SET_ARM_TASK_API_ID,
                "parameter": {"data": task}}
    if skill == "shake_hand":
        return {"kind": "single", "pub": SPORT, "api_id": SET_ARM_TASK_API_ID,
                "parameter": {"data": SHAKE_START_TASK}}

    # --- Preset arm actions (ExecuteAction on the arm topic) --------------- #
    if skill == "arm_action":
        action = params.get("action") or "release_arm"
        if action not in ARM_ACTION_IDS:
            return {"kind": "unsupported", "reason": f"unknown arm action '{action}'"}
        return {"kind": "single", "pub": ARM, "api_id": ARM_EXECUTE_ACTION_API_ID,
                "parameter": {"data": ARM_ACTION_IDS[action]}}

    # --- Audio / TTS (voice topic) ----------------------------------------- #
    if skill in ("say", "speak", "tts"):
        text = (params.get("text") or "").strip()
        if not text:
            return {"kind": "unsupported", "reason": "say needs a 'text' param"}
        return {"kind": "say", "text": text}
    if skill == "set_volume":
        try:
            vol = max(0, min(100, int(params.get("volume", 80))))
        except (TypeError, ValueError):
            vol = 80
        return {"kind": "single", "pub": VOICE, "api_id": AUDIO_SET_VOLUME_API_ID,
                "parameter": {"volume": vol}}

    return {"kind": "unsupported", "reason": f"skill '{skill}' not mapped for G1"}
