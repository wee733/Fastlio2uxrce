#!/usr/bin/env bash
set -eo pipefail
profile="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../config" && pwd)/localhost_udp.xml"
[[ -r "$profile" ]] || { echo "Missing FAST-LIO DDS profile: $profile" >&2; exit 1; }
read -r rmem_max < /proc/sys/net/core/rmem_max
if (( rmem_max < 4194304 )); then
  echo "FAST-LIO localhost_udp requires net.core.rmem_max >= 4194304 bytes (current $rmem_max). Set the host socket receive limit before starting FAST-LIO." >&2
  exit 1
fi
unset ROS_LOCALHOST_ONLY ROS_STATIC_PEERS ROS_DISCOVERY_SERVER
export ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$profile" FASTDDS_DEFAULT_PROFILES_FILE="$profile"
# Jazzy sets data sharing OFF and reallocating histories for variable-size Livox clouds.
# Keep its endpoint defaults while the participant loads the loopback transport from XML.
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4 RMW_FASTRTPS_USE_QOS_FROM_XML=0
exec "$@"
