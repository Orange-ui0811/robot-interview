#!/usr/bin/env bash
set -eo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$root"
python3 -m pytest -s -rs -q tests
if [[ "${1:-}" == "--unit" ]]; then
  exit 0
fi
distro="${GO1_ROS_DISTRO:-humble}"
underlay="/opt/ros/$distro/setup.bash"
if [[ ! -r "$underlay" || ! -r go1_ros2_ws/install/setup.bash ]]; then
  printf 'ROS integration requires a built workspace. See scripts/build.sh.\n' >&2
  exit 1
fi
source "$underlay"
source go1_ros2_ws/install/setup.bash
export ROS_DOMAIN_ID="${GO1_TEST_DOMAIN_ID:-$((60 + $$ % 40))}"
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE CYCLONEDDS_URI
if [[ "$distro" == jazzy ]]; then
  # Let the loopback profile configure both discovery and the data transport.
  unset ROS_LOCALHOST_ONLY ROS_STATIC_PEERS ROS_DISCOVERY_SERVER RMW_FASTRTPS_USE_QOS_FROM_XML
  export ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export FASTRTPS_DEFAULT_PROFILES_FILE="$root/scripts/fastdds_loopback.xml"
  if [[ ! -r "$FASTRTPS_DEFAULT_PROFILES_FILE" ]]; then
    printf 'Missing smoke test DDS profile: %s\n' "$FASTRTPS_DEFAULT_PROFILES_FILE" >&2
    exit 1
  fi
  printf 'ROS smoke DDS: Fast DDS, loopback UDPv4, domain %s\n' "$ROS_DOMAIN_ID"
else
  export ROS_LOCALHOST_ONLY=1
fi
ros2 interface show go1_interfaces/action/MoveFor
exec python3 scripts/ros_smoke.py
