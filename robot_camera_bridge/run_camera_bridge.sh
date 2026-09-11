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

# Supervise, do not exec.
#
# WHY: this is the only piece of the live-video path with no supervisor. The three robot
# services all have `Restart=always` in systemd; the bridge runs in the devcontainer on the
# workstation and had nothing, so when it died the drive view stayed dark until someone
# noticed and restarted it by hand. That happened on 2026-09-10 — rclpy raised
# ExternalShutdownException and the video was simply gone.
#
# Deliberately NOT `set -e`: a supervisor that exits when its child fails is not a
# supervisor. This is the exception the engineering standard §6 names, and the same shape
# robot-video-pipeline/robot/run-video.sh already uses on the robot.
#
# The delay BACKS OFF, and that is not decoration. The first version restarted every 2 s
# unconditionally; when a second copy of this script was already running and holding the
# control port, the loser hot-looped forever — measured 2026-09-11: three supervisors
# alive at once, 619 restarts, ~1 per second. Every failed start opens a fresh full-rate
# stream from the robot before dying, so a loop like that is not idle, it is load.
running=1
delay=2
trap 'running=0' INT TERM
while [ "$running" = 1 ]; do
  started=$SECONDS
  python3 "$HERE/robot_camera_bridge.py"
  rc=$?
  [ "$running" = 1 ] || break
  # Exit code 3 means another instance already owns the control port. Restarting cannot
  # help — that instance IS the service — so step aside instead of fighting it.
  if [ "$rc" = 3 ]; then
    echo "[camera] another bridge already owns the control port; leaving it alone" >&2
    exit 0
  fi
  # A run that lasted a while was healthy: whatever ended it was a blip, so retry fast.
  if [ $((SECONDS - started)) -ge 30 ]; then delay=2; fi
  echo "[camera] bridge exited (rc=$rc); restarting in ${delay}s" >&2
  sleep "$delay"
  delay=$(( delay < 30 ? delay * 2 : 30 ))
done
