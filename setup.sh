#!/bin/bash
echo "Setup unitree ros2 environment"

# 1. Cargamos Humble (el contenedor usa Humble, no Foxy)
source /opt/ros/humble/setup.bash

# 2. Usamos la ruta /workspace que es donde el contenedor monta el proyecto
source /workspace/cyclonedds_ws/install/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# --------------------------------------------------------------------------- #
# 3. DDS transport, driven by dds.env instead of being hardcoded here.
#
# Two knobs (see dds.env.example):
#   CYCLONEDDS_IFACE   host interface CycloneDDS binds to. This stays the WIRED one
#                      even when the robot moves to WiFi, as long as this machine
#                      reaches the robot's network by routing.
#   ROBOT_DDS_PEERS    comma-separated robot IPs for UNICAST discovery. This is the
#                      knob that matters off-subnet: DDS discovery is multicast by
#                      default and multicast is LINK-LOCAL, so it never crosses a
#                      VLAN/subnet boundary. Naming the robot by IP here is what makes
#                      discovery work over inter-VLAN routing (or through a WiFi AP
#                      that filters multicast). Leave empty on a flat wired LAN.
#
# Edit dds.env by hand, or set it from the AI-VL page — the executor's POST /dds
# writes this file and restarts itself through run_executor.sh, which re-sources this
# script, so the new transport takes effect without touching the container.
# --------------------------------------------------------------------------- #
# dds.env is PARSED, never sourced: the AI-VL page writes it through an HTTP endpoint,
# and sourcing would make anything in it executable. Parsing also tolerates spaces in a
# hand-edited value ("a, b"), which `source` would try to run as a command.
_DDS_ENV="$(dirname "${BASH_SOURCE[0]}")/dds.env"
if [ -f "$_DDS_ENV" ]; then
  while IFS='=' read -r _k _v; do
    _k="$(echo "$_k" | tr -d '[:space:]')"
    case "$_k" in ''|'#'*) continue ;; esac
    _v="${_v%\"}"; _v="${_v#\"}"; _v="${_v%\'}"; _v="${_v#\'}"
    _v="$(echo "$_v" | tr -d '[:space:]')"
    case "$_k" in
      CYCLONEDDS_IFACE) CYCLONEDDS_IFACE="$_v" ;;
      ROBOT_DDS_PEERS)  ROBOT_DDS_PEERS="$_v" ;;
    esac
  done < "$_DDS_ENV"
fi
CYCLONEDDS_IFACE="${CYCLONEDDS_IFACE:-enp4s0}"

_peers_xml=""
if [ -n "${ROBOT_DDS_PEERS// /}" ]; then
  _IFS_BAK="$IFS"; IFS=','
  for _p in $ROBOT_DDS_PEERS; do
    _p="$(echo "$_p" | tr -d '[:space:]')"
    [ -n "$_p" ] && _peers_xml="${_peers_xml}<Peer address=\"${_p}\"/>"
  done
  IFS="$_IFS_BAK"
fi

if [ -n "$_peers_xml" ]; then
  # CRITICAL: specifying <Peers> REPLACES CycloneDDS's default peer list, and that
  # default is the SPDP multicast address — so a bare peer list silently turns local
  # multicast discovery OFF. Our own services then stop finding each other, and a robot
  # back on the local subnet becomes invisible. Keep multicast by listing it first.
  # ParticipantIndex=auto is what CycloneDDS recommends alongside unicast peers.
  _discovery_xml="<Discovery><ParticipantIndex>auto</ParticipantIndex><Peers><Peer address=\"239.255.0.1\"/>${_peers_xml}</Peers></Discovery>"
  echo "  DDS: iface=${CYCLONEDDS_IFACE}  multicast + unicast peers=${ROBOT_DDS_PEERS}"
else
  _discovery_xml=""
  echo "  DDS: iface=${CYCLONEDDS_IFACE}  (multicast discovery — same subnet only)"
fi

export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${CYCLONEDDS_IFACE}\" priority=\"default\" multicast=\"default\"/></Interfaces></General>${_discovery_xml}</Domain></CycloneDDS>"
unset _DDS_ENV _peers_xml _discovery_xml _p _IFS_BAK _k _v