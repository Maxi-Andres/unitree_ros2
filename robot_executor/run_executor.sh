#!/usr/bin/env bash
# Launch the AI-VL robot executor INSIDE the unitree_ros2 devcontainer.
# Sources the ROS2 environment (DDS interface + unitree_api messages) and then
# runs the executor service. Run it from a container shell:
#     bash /workspace/robot_executor/run_executor.sh
# Optional overrides: DRY_RUN=true / SAFE_MODE=true bash run_executor.sh
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"   # .../robot_executor
WS="$(dirname "$HERE")"                                # workspace root (/workspace)

# First run: seed .env from the example.
if [ ! -f "$HERE/.env" ]; then
  cp "$HERE/.env.example" "$HERE/.env"
  echo "[executor] created .env from .env.example"
fi

# ROS2 env: sets RMW=cyclonedds + the DDS network interface (e.g. enp4s0) so the
# node can reach the robot, and puts unitree_api/unitree_go on the Python path.
# shellcheck source=/dev/null
source "$WS/setup.sh"

echo "[executor] ROS2 env sourced; starting robot_executor_service.py …"
exec python3 "$HERE/robot_executor_service.py"
