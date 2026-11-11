#!/usr/bin/env bash
# Start the G1's lidar odometry on PC2: livox_ros_driver2 (listening only) and FAST-LIO2.
#   ./run.sh            both, until Ctrl-C
#   ./run.sh driver     the driver alone
# Read-only towards the robot: ROS2 stays on this machine (ROS_LOCALHOST_ONLY, its own domain),
# and the lidar keeps sending to the subnet broadcast that Unitree's PC1 and the relay share
# (LIVOX_KEEP_HOST_CFG, see README.md). It checks that before and after.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
NAV_WS="${NAV_WS:-$HOME/nav_ws}"
RELAY="${RELAY_DIR:-$HOME/robot-command-relay}"
WHAT="${1:-all}"

set +u
# shellcheck disable=SC1091
source /opt/ros/foxy/setup.bash
# shellcheck disable=SC1091
source "$NAV_WS/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"     # never Unitree's domain 0
export ROS_LOCALHOST_ONLY=1
export LIVOX_KEEP_HOST_CFG=1
export LIVOX_LIDAR_CLOCK=1      # the lidar's own clock for the points and IMU (README)
export LD_LIBRARY_PATH="$NAV_WS/livox_sdk/lib:${LD_LIBRARY_PATH:-}"   # ours, not /usr/local's

lidar_host() { python3 "$RELAY/tools/livox_set_host.py" show | grep -E "0x000[67]"; }
check_broadcast() {   # both keys (points 0x0006, IMU 0x0007) on the broadcast, or say so
  if [ "$(lidar_host | grep -c "192.168.123.255" || true)" -ne 2 ]; then
    echo "!! the lidar no longer sends to the broadcast:"; lidar_host
    echo "!! put it back: python3 $RELAY/tools/livox_set_host.py set 192.168.123.255"
    return 1
  fi
}

check_broadcast
pids=()
cleanup() {
  kill "${pids[@]}" 2>/dev/null || true
  wait 2>/dev/null || true
  check_broadcast && echo "lidar still on the broadcast"
}
trap cleanup EXIT INT TERM

nice -n 10 ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args \
  -p xfer_format:=1 -p multi_topic:=0 -p data_src:=0 -p publish_freq:=10.0 \
  -p output_data_type:=0 -p frame_id:=livox_frame \
  -p user_config_path:="$NAV_WS/config/g1_mid360s.json" &
pids+=($!)
if [ "$WHAT" != "driver" ]; then
  sleep 3
  nice -n 10 ros2 run fast_lio fastlio_mapping --ros-args \
    --params-file "$NAV_WS/config/fastlio_g1.yaml" &
  pids+=($!)
  # FAST-LIO2 moves `body` (the lidar's IMU) inside `camera_init` (where that IMU was at the
  # start), and the lidar is UPSIDE DOWN on the head, 0.416 m above the torso (URDF mid360_joint;
  # the roll of 180° verified 2026-11-05). Two fixed transforms give FAST-LIO2's estimate of the
  # robot as fastlio_odom -> fastlio_base_link, x forward and z up (checked on the robot
  # 2026-11-11). Its own names, not odom/base_link: those belong to Nav2's odometry
  # (nav.sh, Unitree's), and base_link with two parents would break the tf tree.
  # (static_transform_publisher: x y z yaw pitch roll.)
  ros2 run tf2_ros static_transform_publisher 0.0003 0 0.416 0 0 3.14159265 fastlio_odom camera_init &
  pids+=($!)
  ros2 run tf2_ros static_transform_publisher -0.0003 0 0.416 0 0 3.14159265 body fastlio_base_link &
  pids+=($!)
fi
wait
