#!/usr/bin/env python3
"""
g1_commands.py — pure skill -> Unitree G1 command mapping (no ROS deps).

Turns one interpreter skill (`{skill, params}`) into a G1 intent the executor
publishes as a `unitree_api/msg/Request`. Unlike the Go2 (one sport topic), the G1
spans FOUR request topics, so each intent names which publisher to use:
  - "sport"  -> /api/sport/request : locomotion (SetVelocity) + FSM postures + the
                loco gestures WaveHand/ShakeHand.
  - "arm"    -> /api/arm/request   : preset arm actions (ExecuteAction by id).
  - "voice"  -> /api/voice/request : audio / TTS.
  - "motion" -> /api/motion_switcher/request : swap the whole motion controller
                (SelectMode by name) — how the app reaches its extra locomotion modes.

Kept dependency-free (no rclpy) so the mapping is unit-testable anywhere. api_ids,
topics and parameter shapes are taken verbatim from unitree_ros2
example/src/include/g1/* and unitree_sdk2 include/unitree/robot/g1/* (loco/arm/audio).
"""

# --- Publisher names (the transport maps these to real topics) -------------- #
SPORT = "sport"
ARM = "arm"
VOICE = "voice"
MOTION = "motion"
SPORT_TOPIC = "/api/sport/request"
ARM_TOPIC = "/api/arm/request"
VOICE_TOPIC = "/api/voice/request"
MOTION_TOPIC = "/api/motion_switcher/request"
TOPICS = {SPORT: SPORT_TOPIC, ARM: ARM_TOPIC, VOICE: VOICE_TOPIC,
          MOTION: MOTION_TOPIC}
# The robot answers each request on the matching response topic, carrying the request's
# identity.id and a status code. Reading these is the only way to learn WHY a command
# did nothing (see ERROR_HINTS below).
RESPONSE_TOPICS = {name: topic.replace("/request", "/response")
                   for name, topic in TOPICS.items()}

# --- api_ids ---------------------------------------------------------------- #
GET_FSM_ID_API_ID = 7001       # loco: -> {"data": <current fsm id>}
GET_FSM_MODE_API_ID = 7002     # loco: -> {"data": <current fsm mode>}
GET_BALANCE_MODE_API_ID = 7003  # loco: -> {"data": <balance mode>}
GET_STAND_HEIGHT_API_ID = 7005  # loco: -> {"data": <stand height>}
ARM_GET_ACTION_LIST_API_ID = 7107  # arm: -> the robot's OWN preset action list
SET_VELOCITY_API_ID = 7105     # loco: {"velocity":[vx,vy,vyaw],"duration":D}
SET_FSM_ID_API_ID = 7101       # loco: {"data":<fsm id>}
SET_BALANCE_MODE_API_ID = 7102  # loco: {"data":<mode>}
SET_STAND_HEIGHT_API_ID = 7104  # loco: {"data":<height>}
SET_ARM_TASK_API_ID = 7106      # loco gestures on the SPORT topic: {"data":<task>}
SET_SPEED_MODE_API_ID = 7107    # loco: {"data":<mode>} (values undocumented)
ARM_EXECUTE_ACTION_API_ID = 7106  # arm actions on the ARM topic: {"data":<action id>}
AUDIO_TTS_API_ID = 1001         # voice: {"index":n,"text":str,"speaker_id":0}
AUDIO_SET_VOLUME_API_ID = 1006  # voice: {"volume":0-100}
MOTION_SELECT_MODE_API_ID = 1002  # motion switcher: {"name":"<mode or alias>"}
ARM_EXECUTE_CUSTOM_API_ID = 7108  # arm: {"action_name":"<name>"} — the named routines
ARM_STOP_CUSTOM_API_ID = 7113     # arm: cut a running named routine short

# SetFsmId ids (LocoClient) — the posture/FSM skills map straight to these.
#
# WHERE EACH ID COMES FROM (tags used below):
#   [robot] observed live on OUR robot with g1_fsm_watch.py — the firmware reported it.
#   [sdk]   hardcoded in the unitree_sdk2 / unitree_ros2 headers on this machine.
#   [web]   documented outside the SDK (CMU Robotics Knowledgebase, QUADRUPED G1 docs).
#
# UNLIKE the arm actions, there is NO "list" api for the locomotion FSM: the robot will
# tell you the id it is IN (GetFsmId), never the set of ids it has. So this table can
# only grow one observation at a time — put the robot in a mode from the app, read the
# number. That is what g1_fsm_watch.py is for.
#
# The 500/501 and 801/802 pairs line up as walk/run for the two waist variants:
#   walk -> 500 (1-DoF waist) | 501 (3-DoF waist)
#   run  -> 801 (1-DoF waist) | 802 (3-DoF waist)
# Our robot answers 501 and 802, i.e. it is the 3-DoF-waist G1.
#
# The operating sequence the robot actually needs, hanging on the gantry:
#   damp (1) -> stand_up / "Preparation" (4) -> a walk or run mode
#
# Still missing: the app's Climb and Lie up. On older firmware "start locomotion" is
# reported as 200 rather than 500; if 500 is rejected, try 200 via `set_fsm_id`.
FSM_IDS = {
    "zero_torque": 0,   # [sdk]
    "damp": 1,          # [sdk] [robot] observed
    "squat": 706,       # [robot] CONFIRMED off the wire: this is what the app's Squat
                        # AND Squat up both send (api 7101, {"data":706}) — one toggle.
                        # From standing it goes down and parks damped; from down it gets
                        # up and restores the locomotion mode it had.
    "squat_sdk": 2,     # [sdk] the SDK's Squat2StandUp state. On THIS robot it
                        # half-falls, so it is hidden from the interpreter and gated
                        # behind safe mode — kept only because it is the documented id.
    "sit": 3,           # [sdk]
    "stand_up": 4,      # [sdk] [robot] observed — the app's Ready/Preparation (L1+UP)
    "start": 500,       # [sdk] [web]   walk/main operation (R1+X), 1-DoF-waist variant
    "walk_waist": 501,  # [web] [robot] observed — walk on the 3-DoF-waist variant
    "run": 801,         # [web]         run (R2+X), 1-DoF-waist variant
    "run_waist": 802,   # [robot] observed — CONFIRMED: the app's Run on this robot
    "climb": 812,       # [robot] observed — CONFIRMED: the app's Climb on this robot
                        # (3-DoF waist). By the pair pattern 811 is presumably the
                        # 1-DoF-waist counterpart, but that one is NOT verified, so it
                        # is deliberately absent rather than guessed.
    "lie_up": 702,      # [robot] observed — CONFIRMED: the app's Lie up. Entered from
                        # damp and it walks its own fsm_mode 1 -> 2 -> 0 while the robot
                        # gets up, ending in a locomotion mode.
}

# HOW TO IDENTIFY ANY APP BUTTON EXACTLY — the phone app is itself a DDS participant on
# this bus (it polls GetFsmId here every ~500 ms), so its commands can simply be read:
#
#     source /workspace/setup.sh
#     ros2 topic echo /api/sport/request     # then press the button in the app
#
# Filter out the api_id 7001/7002 polling noise and what is left is the real command,
# api_id + parameter. That is how `squat` above was settled: both the app's Squat and its
# Squat up publish api 7101 {"data":706}, i.e. ONE toggle — no inference needed. Use the
# same trick for anything still unknown (e.g. whatever the app labels "Welcome"); the
# arm topic /api/arm/request works the same way.
HIGH_STAND = 4294967295.0  # (float)UINT32_MAX sentinel -> tallest stand
LOW_STAND = 0.0            # UINT32_MIN sentinel -> lowest stand
# FSM ids that accept gestures and arm actions. From g1_arm_action_error.hpp:
# "The actions are only supported in fsm id {500, 501, 801}". Proof that ids beyond
# the six the SDK wraps exist — the app's Run / Walk (waist control) / Climb / Lie up
# modes live among them, but Unitree does not publish their numbers. Read the current
# one with GetFsmId (api 7001) or off rt/sportmodestate.
GESTURE_FSM_IDS = (500, 501, 801)

# Loco gesture task ids (SET_ARM_TASK on the sport topic). ShakeHand is TWO stages:
# task 2 offers the hand, task 3 ends it and brings the arm back. Sending only 2
# (what this file did before) leaves the G1 standing there with its arm out.
WAVE_TASK = 0
WAVE_TURN_TASK = 1
SHAKE_START_TASK = 2
SHAKE_END_TASK = 3

# Response codes the robot answers with (unitree_sdk2 g1_loco_error.hpp and
# g1_arm_action_error.hpp) + what an operator should DO. Without reading the response
# these failures are invisible: the command looks accepted and the robot does nothing.
ERROR_HINTS = {
    7301: "LocoState not available — the loco service is not running on the robot.",
    7302: "Invalid fsm id for this robot.",
    7303: "Invalid task id — this gesture is not available in the current state; "
          "run 'Preparation' first.",
    7400: "The rt/armsdk topic is occupied — another program is driving the arms.",
    7401: "The arm is holding its last action; send 'release_arm' (or repeat the same "
          "action) to let go.",
    7402: "Invalid arm action id.",
    7404: f"Arm actions only work in fsm id {GESTURE_FSM_IDS} — run 'Preparation' first.",
}

# Skills that need the robot to already be in an operation state (GESTURE_FSM_IDS).
# We deliberately do NOT auto-switch: forcing FSM 500 would make a sitting or limp
# humanoid stand up on its own, which is exactly the kind of surprise motion nobody
# asked for. Instead the executor reports the state error with a clear hint.
NEEDS_OPERATION_STATE = {"wave_hand", "shake_hand", "arm_action"}


def error_hint(code):
    """Human-readable meaning for a robot response code (or a generic fallback)."""
    return ERROR_HINTS.get(code, f"robot returned error code {code}")


# Read-only queries the robot answers on the response topics. These are the ONLY
# authoritative source for the app modes Unitree does not publish: switch the robot
# into "Run" / "Walk (waist control)" / "Climb" / "Lie up" from the phone app, read
# `fsm_id` here, and that number can then be added above as a named skill.
# `action_list` is the robot's OWN preset arm-action list — the SDK's local table has
# 16 entries and its own header warns that the app and the firmware differ, so this
# is how "welcome" and "dance" get real ids instead of guesses.
STATE_QUERIES = {
    "fsm_id": (SPORT, GET_FSM_ID_API_ID),
    "fsm_mode": (SPORT, GET_FSM_MODE_API_ID),
    "balance_mode": (SPORT, GET_BALANCE_MODE_API_ID),
    "stand_height": (SPORT, GET_STAND_HEIGHT_API_ID),
    "action_list": (ARM, ARM_GET_ACTION_LIST_API_ID),
}

# Arm preset actions -> ExecuteAction ids. VERIFIED against the robot's own list
# (GetActionList, arm api 7107): it declares 23, not the 16 the SDK header hardcodes,
# and the SDK repeats id 12 for both kisses — the robot says right kiss is 13.
# Matches command_common.ARM_ACTION_IDS in AI-VL-core.
ARM_ACTION_IDS = {
    "release_arm": 99,
    "turn_back_wave": 1,
    "two_hand_kiss": 11,
    "left_kiss": 12,
    "right_kiss": 13,
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
    "box_win_left": 28,
    "box_win_right": 29,
    "box_win_both": 30,
    "hand_on_heart": 33,
    "hands_up_right": 34,
    "forward_push": 36,
}

# Named "teach" routines (the app's dances). Different api: ExecuteAction by NAME.
CUSTOM_ACTIONS = ("Waist_Drum_Dance", "Scratch_head", "Spin_discs", "Throw_money")

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

# Skills blocked while SAFE_MODE is on. Criterion: anything that can make the robot
# LOSE ITS SUPPORT (fall / go limp) or change its control mode unpredictably.
# Controlled posture changes that stay balanced the whole way down (sit, squat, stand
# heights) are NOT blocked — they are the normal way to park a humanoid.
# Mirrors command_common.G1_DANGEROUS in AI-VL-core (the UI reads that one).
DANGEROUS_SKILLS = {
    "zero_torque",      # no motor torque at all -> a standing robot collapses
    "damp",             # limp/compliant -> collapses from any standing posture
    "set_fsm_id",       # raw state jump: can land the robot in a mode that drops it
    "set_speed_mode",   # raw, undocumented speed mode (the app's Run lives here)
    "switch_mode",      # swaps the whole motion controller out from under it
    "dance",            # multi-second whole-body routine: needs clear space
    "squat_sdk",        # the SDK's squat (fsm 2): observed half-falling on this robot
}


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
        # Two stages, like LocoClient::ShakeHand: `on` true offers the hand (task 2),
        # false ends the handshake and brings the arm back (task 3). Defaults to
        # offering, so a bare "shake hands" still starts one.
        end = params.get("on") is False
        task = SHAKE_END_TASK if end else SHAKE_START_TASK
        return {"kind": "single", "pub": SPORT, "api_id": SET_ARM_TASK_API_ID,
                "parameter": {"data": task}}

    # --- Preset arm actions (ExecuteAction on the arm topic) --------------- #
    if skill == "arm_action":
        action = params.get("action") or "release_arm"
        if action not in ARM_ACTION_IDS:
            return {"kind": "unsupported", "reason": f"unknown arm action '{action}'"}
        return {"kind": "single", "pub": ARM, "api_id": ARM_EXECUTE_ACTION_API_ID,
                "parameter": {"data": ARM_ACTION_IDS[action]}}

    # --- Named "teach" routines (the app's dances) -------------------------- #
    # A DIFFERENT api from the numbered actions: indexed by name, not id.
    if skill == "dance":
        name = (params.get("name") or "").strip()
        if not name:
            return {"kind": "unsupported", "reason": "dance needs a routine 'name'"}
        return {"kind": "single", "pub": ARM, "api_id": ARM_EXECUTE_CUSTOM_API_ID,
                "parameter": {"action_name": name}}
    if skill == "stop_dance":
        return {"kind": "single", "pub": ARM, "api_id": ARM_STOP_CUSTOM_API_ID,
                "parameter": None}

    # --- Raw mode control (operator-only; see command_common's hidden skills) --- #
    # The escape hatches for the app modes Unitree does not document (Run, Walk with
    # waist control, Climb, Lie up). Once GetFsmId reveals an id, promote it to a
    # named skill above instead of leaving operators to type numbers.
    if skill == "set_fsm_id":
        try:
            fsm_id = int(params.get("fsm_id"))
        except (TypeError, ValueError):
            return {"kind": "unsupported", "reason": "set_fsm_id needs an integer 'fsm_id'"}
        return {"kind": "single", "pub": SPORT, "api_id": SET_FSM_ID_API_ID,
                "parameter": {"data": fsm_id}}
    if skill == "set_speed_mode":
        try:
            mode = int(params.get("mode"))
        except (TypeError, ValueError):
            return {"kind": "unsupported", "reason": "set_speed_mode needs an integer 'mode'"}
        return {"kind": "single", "pub": SPORT, "api_id": SET_SPEED_MODE_API_ID,
                "parameter": {"data": mode}}
    if skill == "switch_mode":
        name = (params.get("name") or "").strip()
        if not name:
            return {"kind": "unsupported", "reason": "switch_mode needs a 'name'"}
        return {"kind": "single", "pub": MOTION, "api_id": MOTION_SELECT_MODE_API_ID,
                "parameter": {"name": name}}

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
