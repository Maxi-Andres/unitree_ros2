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
| `livox_ros_driver2.patch` | `LIVOX_LIDAR_CLOCK=1`: an unsynced lidar's points and IMU keep the lidar's own clock, put on the host's by the smallest delay seen | upstream stamps each packet with when the host processed it, so per-point times carry the receive jitter. **Tried 2026-11-11 against the turn-in-place error: no change** — kept because it is the more correct stamp |
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

## Nav2 (`nav.sh`) and the app

`./g1_nav/nav.sh` starts Nav2 with Unitree's leg odometry and the relay's obstacle map
(`nodes/relay_bridge.py`, `config/nav2_g1.yaml`, `config/navigate_wait_and_retry.xml`). The app
drives it **without any change**: with `WALK_NAV2_URL=http://127.0.0.1:8095` in the relay's
`relay.env`, a goto from the Navigation page or the agent's `walk` is handed to Nav2 through the
bridge's small HTTP API (127.0.0.1 only, the relay's token), and Nav2's velocity comes back to the
relay on `POST /walk/move`, which it refuses unless such a walk runs. The operator still wins:
any command or stop cancels the walk and Nav2's goal. Blocked by someone, it stops, asks for
the way out loud ("Disculpe.", "Permiso."…, `assets/`), waits 3 s and tries again, up to ~1.5
min. One speed, 0.3 m/s (the app's Slow): below it the G1 marks time. Walk or Run is the
operator's (the page's choice is applied before the walk is handed over).

At boot it is started by `systemd/g1-nav.service` (install steps in the file). Without it,
nothing listens on 8095 and the relay falls back to its own walk.

## Measured (2026-11-11, first light)

- Driver: `/livox/lidar` (CustomMsg, per-point time, ~17 800 points a message at 10 Hz) and
  `/livox/imu`; ~25 % of one core. The relay's bridge kept its 200 000 points/s meanwhile.
- FAST-LIO2: IMU initialised, map built, `/Odometry` published; standing still, the position
  stayed within ~2 cm of zero.
- Against Unitree's leg odometry, same runs (2026-11-11, driven with the remote):

  | Ground truth | FAST-LIO2 | Unitree |
  |---|---|---|
  | 2 m straight, tape | 1.93 m | 1.62 m |
  | half turn in place and back, put back on the start (≤ 2 cm) | 4 cm off | 33 cm off |

  `config/fastlio_g1.yaml` is the one that did it: `blind` 1.4 m (the head lidar sees its own
  legs and feet ~1.25 m away; with 0.5 it got lost turning in place), `extrinsic_est_en` false
  (the IMU is inside the lidar), map/scan filters 0.3 m for indoors. CPU ~19 % of a core each
  for the driver and FAST-LIO2.
- Open: **while turned round**, FAST-LIO2 and Unitree differ by ~0.6 m (a rigid 0.31 m lever
  arm fits it to 3 cm) and nobody measured which is right there; back at the start heading
  FAST-LIO2 is right. Settled the same night, crossing the keyboard commands (relay audit
  log) with all three: **turning in place, FAST-LIO2 overstates the displacement** (~0.9 m
  for a 90° turn, given back when turning back) while Unitree's ~0.2-0.3 m is what the user
  saw. Forward motion and turn angles agree in all three. **Ruled out** the same night: the
  packet timestamps (`LIVOX_LIDAR_CLOCK`, no change: 0.99 / 0.59 / 0.80 m for turns of
  170 / 92 / 79 degrees, Unitree 0.06 / 0.05 / 0.17) and the IMU's g units (FAST-LIO2 scales
  them). Still suspected: the IMU-to-lidar extrinsic (copied from the Mid-360, ours is a
  Mid-360s; LI-Init calibrates it), and a floor-heavy view from the upside-down head.
- `run.sh` publishes `odom -> camera_init` and `body -> base_link` (the lidar upside down,
  0.416 m above the torso), so `odom -> base_link` is the robot, x forward and z up. Checked
  on the robot against the keyboard commands: forward is forward, a left turn is positive.
