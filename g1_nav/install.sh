#!/usr/bin/env bash
# Build the G1's lidar odometry stack on PC2: Livox-SDK2 (private copy), livox_ros_driver2 and
# FAST-LIO2, each at a pinned commit with the patches in ./patches. Run ON PC2 (Orin NX, Ubuntu
# 20.04, ROS2 Foxy already installed by Unitree). Touches nothing outside $NAV_WS:
#   - Unitree's own Livox SDK in /usr/local is NOT replaced (it is a 2023 build that does not know
#     the G1's Mid-360s, dev_type 35); ours is installed to $NAV_WS/livox_sdk.
#   - nothing is installed with apt or sudo.
# Re-running it rebuilds from scratch. README.md says why each patch exists.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
NAV_WS="${NAV_WS:-$HOME/nav_ws}"
JOBS="${JOBS:-3}"                 # leave cores to the relay, the video and Unitree's own software

SDK_REPO=https://github.com/Livox-SDK/Livox-SDK2.git
SDK_REV=c0796f04c143143899c87a773d9f6b7136453c0b
DRIVER_REPO=https://github.com/Livox-SDK/livox_ros_driver2.git
DRIVER_REV=21445540f0d100dc86a7e6df312dd70bbdb4afdf
FASTLIO_REPO=https://github.com/hku-mars/FAST_LIO.git
FASTLIO_REV=cc3c9e8b1469b33b8072764d6db2348ed6dc597f       # branch ROS2

checkout() {   # repo dir rev [--recursive]
  local repo=$1 dir=$2 rev=$3
  if [ ! -d "$dir/.git" ]; then
    git clone -q "$repo" "$dir"
  fi
  git -C "$dir" fetch -q origin
  git -C "$dir" checkout -q -f "$rev"
  git -C "$dir" clean -q -fdx
  git -C "$dir" submodule -q update --init --recursive
}

mkdir -p "$NAV_WS/src" "$NAV_WS/ref" "$NAV_WS/config"

echo "== Livox-SDK2 -> $NAV_WS/livox_sdk"
checkout "$SDK_REPO" "$NAV_WS/ref/Livox-SDK2" "$SDK_REV"
git -C "$NAV_WS/ref/Livox-SDK2" apply "$HERE/patches/livox-sdk2.patch"
mkdir -p "$NAV_WS/ref/Livox-SDK2/build"
(cd "$NAV_WS/ref/Livox-SDK2/build" &&
  nice -n 19 cmake .. -DCMAKE_INSTALL_PREFIX="$NAV_WS/livox_sdk" -DCMAKE_BUILD_TYPE=Release >/dev/null &&
  nice -n 19 make -j"$JOBS" >/dev/null && make install >/dev/null)

echo "== livox_ros_driver2 + FAST_LIO"
checkout "$DRIVER_REPO" "$NAV_WS/src/livox_ros_driver2" "$DRIVER_REV"
git -C "$NAV_WS/src/livox_ros_driver2" apply "$HERE/patches/livox_ros_driver2.patch"
# What the driver's own build.sh does for ROS2, without its `colcon build` of the whole workspace.
cp -f "$NAV_WS/src/livox_ros_driver2/package_ROS2.xml" "$NAV_WS/src/livox_ros_driver2/package.xml"
cp -rf "$NAV_WS/src/livox_ros_driver2/launch_ROS2/" "$NAV_WS/src/livox_ros_driver2/launch/"
checkout "$FASTLIO_REPO" "$NAV_WS/src/FAST_LIO" "$FASTLIO_REV"
git -C "$NAV_WS/src/FAST_LIO" apply "$HERE/patches/fast_lio.patch"

cp -f "$HERE/config/"* "$NAV_WS/config/"

set +u
# shellcheck disable=SC1091
source /opt/ros/foxy/setup.bash
set -u
cd "$NAV_WS"
rm -rf build install log
MAKEFLAGS=-j"$JOBS" nice -n 19 colcon build --packages-select livox_ros_driver2 --cmake-args \
  -DROS_EDITION=ROS2 -DDISTRO_ROS= \
  -DLIVOX_LIDAR_SDK_LIBRARY="$NAV_WS/livox_sdk/lib/liblivox_lidar_sdk_shared.so" \
  -DLIVOX_LIDAR_SDK_INCLUDE_DIR="$NAV_WS/livox_sdk/include"
set +u
# shellcheck disable=SC1091
source "$NAV_WS/install/setup.bash"
set -u
MAKEFLAGS=-j"$JOBS" nice -n 19 colcon build --packages-select fast_lio
echo "built: $NAV_WS/install (run.sh starts it)"
