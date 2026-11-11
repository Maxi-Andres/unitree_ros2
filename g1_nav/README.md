# g1_nav — lidar odometry and mapping for the G1, on PC2

FAST-LIO2 on the G1's Mid-360s, through `livox_ros_driver2`, on the robot's PC2 (Orin NX, Ubuntu
20.04, the ROS2 Foxy that Unitree installed). This is the first step of the Nav2 + SLAM trial
(`~/Desktop/.claude/roadmap/G1.md` §6.8.1). Off-the-shelf tools, with the smallest patches
that make them coexist with what already runs on the robot.

Why: the relay's own lidar bridge places each 300 ms of points with the pose of that moment, so
while the robot turns, walls smear 15-20 cm and show as hundreds of false obstacles a map
(measured 2026-11-11). FAST-LIO2 corrects every point by its own timestamp with the lidar's IMU
(*deskew*), and gives lidar odometry and a map.

## The constraint everything here respects

The Mid-360s (`192.168.123.120`) sends its points and IMU to the subnet **broadcast**
(`192.168.123.255`), so that Unitree's PC1 and PC2 (the relay's `g1_lidar_bridge`) both get them
(`robot-command-relay/tools/livox_set_host.py`). A stock Livox SDK, on connecting, **rewrites
the lidar's host config to point at itself** — that happened once on 2026-11-11 and took the
lidar from PC1 for about a minute. Hence the patches:

| Patch | What | Why |
|---|---|---|
| `livox-sdk2.patch` | `LIVOX_KEEP_HOST_CFG=1` skips the host-config update and reports the lidar as if it succeeded | never rewrite where the lidar sends its data |
| `livox-sdk2.patch` | data sockets bind any address | a socket bound to PC2's address never sees the broadcast; shares the ports with the relay's bridge (`SO_REUSEADDR` on both) |
| `livox_ros_driver2.patch` | a lidar not listed in `lidar_configs` is published too | listing ours would make the driver write its install attitude to it |
| `fast_lio.patch` | no `pcl_ros` (not in Foxy; unused by the code), PCL `filters` and `tf2_ros` linked explicitly, Foxy's service callback signature | builds on Foxy without apt |

Also: **our own Livox-SDK2** is built into `~/nav_ws/livox_sdk` — Unitree's in `/usr/local` is a
2023 build that does not know the Mid-360s (`dev_type 35`) and is left alone. The slave mode of
the SDK (`"master_sdk": false`) was tried first: it writes nothing, but it learns of lidars only
from their status messages, which go to PC1, so it never publishes.

## Install (on PC2, as `unitree`)

```bash
cd ~/unitree_ros2 && git pull
./g1_nav/install.sh            # ~5 min; clones pinned commits into ~/nav_ws, patches, builds
```

## Run

```bash
./g1_nav/run.sh                # driver + FAST-LIO2, Ctrl-C to stop
./g1_nav/run.sh driver         # the driver alone
```

`run.sh` checks the lidar is still on the broadcast before and after. ROS2 stays on PC2:
`ROS_LOCALHOST_ONLY=1`, `ROS_DOMAIN_ID=77` (Unitree's bus is domain 0). To look at it from
another shell on PC2, source the same environment:

```bash
source /opt/ros/foxy/setup.bash && source ~/nav_ws/install/setup.bash
export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1
ros2 topic hz /Odometry        # FAST-LIO2: also /path, /tf, /cloud_registered
ros2 topic echo /livox/lidar --no-arr
```

If the lidar ever stops sending to the broadcast:
`python3 ~/robot-command-relay/tools/livox_set_host.py set 192.168.123.255`.

## Measured (2026-11-11, first light)

- Driver: `/livox/lidar` (CustomMsg, per-point time, ~17 800 points a message at 10 Hz) and
  `/livox/imu`; ~25 % of one core. The relay's bridge kept its 200 000 points/s meanwhile.
- FAST-LIO2: IMU initialised, map built, `/Odometry` published; standing still, the position
  stayed within ~2 cm of zero.
- Not yet: walking, drift against tape, CPU under load, Nav2.
