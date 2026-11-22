# Go1 MCP Driver

This driver connects Phanthy Motus tools to the Go1 ROS 2 simulator. Build it as
part of the sibling `go1_ros2_ws` using `go1_bringup`, then launch it with
`go1_demo.launch.py` or run it independently:

```bash
source /opt/ros/humble/setup.bash
source ~/robot-interview/go1_ros2_ws/install/setup.bash
ros2 run go1_bringup go1_mcp_node --port 15795
```

The same ROS domain must contain `/go1/move_for`, `/go1/stop` and the
simulated sensor publishers. To register with a reachable local Agent Core, set
`AGENT_CORE_URL` or pass `--core-url`. Use the Core's DDS domain and transport
profile so it can discover the JSON output topics.
