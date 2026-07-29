#!/usr/bin/env python3
"""
g1_fsm_watch.py — read the G1's state ids straight off the robot, live.

WHY THIS EXISTS: the Unitree phone app offers locomotion modes whose FSM ids Unitree
does not publish anywhere (Climb, Lie up; and the app shows arm actions like Welcome
and Dance that are missing from the SDK's 16-entry table). The robot knows them. So:

    1. Run this in the devcontainer, with the robot hanging on its gantry.
    2. Press every option in the app, one at a time.
    3. Each new state prints a line here. That number IS the id.
    4. Add it to FSM_IDS in g1_commands.py as a named skill — done.

It is READ-ONLY: it publishes only Get* api_ids (7001/7002/7003/7005 on the loco
topic, 7107 on the arm topic), so it cannot move the robot. Safe to leave running.

Run it through the wrapper, which sources the ROS2 env for you (without that,
`unitree_api` is not importable):
    bash /workspace/robot_executor/run_fsm_watch.sh

    --interval S   seconds between polls (default 0.3)
    --all          also print the full state + the robot's own action list, once
"""
import argparse
import json
import signal
import sys
import time

import g1_commands
import robot_executor_service as svc

# Names we already know, so the output reads as "801 (run)" instead of a bare number
# and a NEW id is obvious at a glance.
KNOWN = {v: k for k, v in g1_commands.FSM_IDS.items()}


def describe(fsm_id):
    name = KNOWN.get(fsm_id)
    return f"{fsm_id} ({name})" if name else f"{fsm_id}  <-- NEW: not in FSM_IDS yet"


def value_of(result):
    """The queries answer {"data": <value>}; anything else is passed through."""
    if isinstance(result, dict):
        if "error" in result:
            return None
        return result.get("data", result)
    return result


def main():
    ap = argparse.ArgumentParser(description="Watch the G1's FSM id change live.")
    ap.add_argument("--interval", type=float, default=0.3)
    ap.add_argument("--all", action="store_true",
                    help="also dump the full state + the robot's own action list once")
    args = ap.parse_args()

    # Treat SIGTERM like Ctrl-C, so `timeout N`, `docker stop` or a killed terminal end
    # the run through the normal cleanup instead of leaving rclpy half shut down.
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))

    try:
        transport = svc.G1Ros2Transport(dry_run=False)
    except ModuleNotFoundError as exc:
        # robot_executor_service imports rclpy/unitree_api lazily, so a missing ROS2
        # environment only blows up HERE, when the transport is built.
        sys.exit(f"[fsm-watch] '{exc.name}' is not importable: the ROS2 environment is "
                 "not sourced.\n           Run it through the wrapper instead:\n"
                 "               bash /workspace/robot_executor/run_fsm_watch.sh")

    print("watching the G1 state — press the app's options one at a time "
          "(Ctrl-C to stop)\n", flush=True)

    if args.all:
        print("full state + the robot's OWN preset action list:")
        print(json.dumps(transport.query_state(), indent=2), flush=True)
        print()

    last = None
    silent_since = None
    try:
        while True:
            state = transport.query_state(["fsm_id", "fsm_mode"])
            fsm_id = value_of(state.get("fsm_id"))
            fsm_mode = value_of(state.get("fsm_mode"))
            if fsm_id is None:
                # No answer: say it once, not every poll.
                if silent_since is None:
                    silent_since = time.time()
                    err = (state.get("fsm_id") or {}).get("error", "no answer")
                    print(f"[{time.strftime('%H:%M:%S')}] {err} — is the robot on and "
                          f"on the same DDS network?", flush=True)
            else:
                silent_since = None
                current = (fsm_id, fsm_mode)
                if current != last:
                    print(f"[{time.strftime('%H:%M:%S')}] fsm_id = {describe(fsm_id)}"
                          f"   fsm_mode = {fsm_mode}", flush=True)
                    last = current
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\nstopped.")
    finally:
        # stop_first=False: not even a zero-velocity command, so this stays strictly
        # read-only as advertised.
        transport.shutdown(stop_first=False)
        import rclpy
        if rclpy.ok(context=svc._context):
            rclpy.shutdown(context=svc._context)


if __name__ == "__main__":
    sys.exit(main())
