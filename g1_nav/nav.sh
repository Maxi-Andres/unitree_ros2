#!/usr/bin/env bash
# Start Nav2 on PC2 for the G1, version 1: no map, Unitree's leg odometry, the relay's obstacle
# map (config/nav2_g1.yaml says why). Until Ctrl-C.
#   ./nav.sh
# THIS MOVES THE ROBOT once a goal is sent (/navigate_to_pose): keep the remote in hand. Nav2's
# /cmd_vel reaches the robot only through the relay's `move` (relay_bridge), with the relay's
# clamp and dead-man; any stop or drive-pad command from the operator wins.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"

set +u
# shellcheck disable=SC1091
source /opt/ros/foxy/setup.bash
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"     # never Unitree's domain 0
export ROS_LOCALHOST_ONLY=1
export OPENBLAS_NUM_THREADS=1                   # numpy on the Jetson otherwise spins ~4 cores

PARAMS="$HERE/config/nav2_g1.yaml"
pids=()
cleanup() {
  kill "${pids[@]}" 2>/dev/null || true
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

python3 "$HERE/nodes/relay_bridge.py" &
pids+=($!)
# Each node named as config/nav2_g1.yaml and the lifecycle manager expect: Foxy's planner calls
# itself `nav2_planner`, so without the rename its parameters never apply and the lifecycle
# manager waits for a `planner_server` that is not there (seen on PC2, 2026-11-11).
for exe in "nav2_controller controller_server" "nav2_planner planner_server" \
           "nav2_recoveries recoveries_server" "nav2_bt_navigator bt_navigator"; do
  # shellcheck disable=SC2086
  set -- $exe
  nice -n 5 ros2 run "$1" "$2" --ros-args -r __node:="$2" --params-file "$PARAMS" &
  pids+=($!)
done
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -r __node:=lifecycle_manager_navigation \
  -p use_sim_time:=false -p autostart:=true \
  -p "node_names:=[controller_server, planner_server, recoveries_server, bt_navigator]" &
pids+=($!)
wait
