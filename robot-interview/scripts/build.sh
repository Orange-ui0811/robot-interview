#!/usr/bin/env bash
set -eo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
distro="${GO1_ROS_DISTRO:-humble}"
setup="/opt/ros/$distro/setup.bash"
if [[ ! -r "$setup" ]]; then
  printf 'ROS environment is missing: %s\n' "$setup" >&2
  exit 1
fi
source "$setup"
command -v colcon >/dev/null || { printf 'Missing colcon\n' >&2; exit 1; }
cd "$root/go1_ros2_ws"
exec colcon build --symlink-install "$@"
