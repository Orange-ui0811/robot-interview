#!/usr/bin/env bash
set -eo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
distro="${GO1_ROS_DISTRO:-humble}"
underlay="/opt/ros/$distro/setup.bash"
overlay="$root/go1_ros2_ws/install/setup.bash"
if [[ ! -r "$underlay" || ! -r "$overlay" ]]; then
  printf 'Missing ROS setup or workspace overlay. Run scripts/build.sh first.\n' >&2
  exit 1
fi
source "$underlay"
source "$overlay"
exec ros2 launch go1_bringup go1_demo.launch.py "$@"
