#!/usr/bin/env bash
# Launch the AI-VL robot camera bridge INSIDE the unitree_ros2 devcontainer.
# Sources the ROS2 environment (so it can read the robot camera topics) and runs
# the bridge. Run it from a container shell:
#     bash /workspace/robot_camera_bridge/run_camera_bridge.sh
# Optional: CAMERA_ROBOT=test / START_STREAMING=true bash run_camera_bridge.sh
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"   # .../robot_camera_bridge
WS="$(dirname "$HERE")"                                # workspace root (/workspace)

if [ ! -f "$HERE/.env" ]; then
  cp "$HERE/.env.example" "$HERE/.env"
  echo "[camera] created .env from .env.example"
fi

# shellcheck source=/dev/null
source "$WS/setup.sh"
echo "[camera] ROS2 env sourced; starting robot_camera_bridge.py …"
exec python3 "$HERE/robot_camera_bridge.py"
