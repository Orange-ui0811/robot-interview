#!/usr/bin/env bash
set -eo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
distro="${GO1_ROS_DISTRO:-humble}"
source "/opt/ros/$distro/setup.bash"
source "$root/go1_ros2_ws/install/setup.bash"
mkdir -p -- "$root/bags"
out="${1:-$root/bags/go1-$(date +%Y%m%d-%H%M%S)}"
exec ros2 bag record -o "$out" --qos-profile-overrides-path "$root/scripts/record_qos.yaml" \
  /go1/cmd_vel /go1/cmd_vel_stamped /go1/command_events /go1/state /joint_states /odom /imu \
  /battery_state /tf /tf_static /robot_description /go1/path \
  /go1/move_for/_action/feedback /go1/move_for/_action/status \
  --include-hidden-topics
