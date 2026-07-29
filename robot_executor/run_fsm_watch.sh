#!/usr/bin/env bash
# Watch the G1's FSM ids live, INSIDE the unitree_ros2 devcontainer.
# Sources the ROS2 environment first (without it `unitree_api` is not importable —
# that is the ModuleNotFoundError you get from running the .py directly).
#
#     bash /workspace/robot_executor/run_fsm_watch.sh --all
#
# Read-only: it publishes only Get* api_ids, so it cannot move the robot. Press each
# option in the Unitree app one at a time; every new state prints a line, and an id we
# do not have yet is flagged. See g1_fsm_watch.py for the details.
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"   # .../robot_executor
WS="$(dirname "$HERE")"                                # workspace root (/workspace)

# shellcheck source=/dev/null
source "$WS/setup.sh"

exec python3 "$HERE/g1_fsm_watch.py" "$@"
