#!/usr/bin/env bash
set -eo pipefail

printf 'Kernel: %s\n' "$(uname -sr)"
if [[ -r /etc/os-release ]]; then
  (. /etc/os-release; printf 'OS: %s\n' "$PRETTY_NAME")
fi
printf 'WSL distribution: %s\n' "${WSL_DISTRO_NAME:--}"
printf 'GUI: DISPLAY=%s WAYLAND_DISPLAY=%s\n' "${DISPLAY:-unset}" "${WAYLAND_DISPLAY:-unset}"
printf 'ROS Domain: %s\n' "${ROS_DOMAIN_ID:-0}"
distro="${GO1_ROS_DISTRO:-humble}"
setup="/opt/ros/$distro/setup.bash"
if [[ -r "$setup" ]]; then
  source "$setup"
fi
missing=0
for tool in python3 git ros2 colcon rviz2; do
  if command -v "$tool" >/dev/null; then
    printf '%s: %s\n' "$tool" "$(command -v "$tool")"
  else
    printf '%s: missing\n' "$tool"
    missing=1
  fi
done
if command -v python3 >/dev/null; then
  python3 -V
  python3 -m pytest --version || missing=1
  python3 -c 'import rclpy; print("rclpy:", rclpy.__file__)' || missing=1
fi
if command -v ros2 >/dev/null; then
  for pkg in robot_state_publisher rviz2 rosbag2; do
    ros2 pkg prefix "$pkg" || missing=1
  done
fi
if (( missing )); then
  printf 'See README_zh.md for Ubuntu setup.\n' >&2
fi
exit "$missing"
