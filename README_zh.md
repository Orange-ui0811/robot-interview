# Go1 ROS 2、RViz2 与 Phanthy Motus MCP 模拟驱动

这套工程用上游 Go1 URDF、单一运动学模拟器和 ROS 2 原生接口演示机器狗驱动开发。RViz2 可配置为显示机身、四腿、坐标树、里程计与轨迹。用户可通过 ROS 2 CLI 或 MCP 向同一个模拟器下发指令。

## 交付结构

```text
robot-interview/
├── go1_ros2_ws/src/
│   ├── go1_description/          原始 URDF、状态发布 Launch、RViz2 配置
│   ├── go1_interfaces/           MoveFor.action 及 rosidl 构建
│   ├── go1_mock/                 MockGo1SDK、ROS 2 消息和控制接口
│   └── go1_bringup/              一键 Launch、参数、RViz2、安装 MCP 桥
├── phanthymotus-driver/
│   └── interview/go1_mcp/        独立 MCP 驱动和平台清单
├── scripts/                      环境检查、构建、演示、验收、录包
├── tests/                        模拟核心、HTTP 请求处理和注册生命周期测试
└── docs/                         源码审计、架构、验收报告
```

交付中的驱动子目录是放入上游克隆的新增文件。独立演示用到的 URDF 已随工程分发。Agent Core 可选，安装及配置细节见下文。

## 1. 环境与安装

首选 Ubuntu 22.04 + ROS 2 Humble Desktop + 系统 Python 3.10。Windows 可使用带 WSLg 的 WSL2。从 Windows 先确认实际发行版。

```powershell
wsl --list --verbose
```

从 Ubuntu 终端执行后续命令。Ubuntu 24.04 + ROS 2 Jazzy 已通过本项目的无界面集成测试。使用该环境时，构建和测试脚本需设置 `GO1_ROS_DISTRO=jazzy`，图形演示见下方 Jazzy 启动步骤。

先按 [ROS 2 Humble 官方安装说明](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) 配置 ROS APT 源，然后在获得本机安装权限时运行下面的命令。已经有 ROS Desktop 的环境可直接执行 `scripts/check_env.sh` 核查缺项。

```bash
sudo apt update
sudo apt install ros-humble-desktop ros-humble-rosidl-default-generators \
  ros-humble-robot-state-publisher ros-humble-rosbag2 python3-colcon-common-extensions \
  python3-pytest python3-rosdep git curl unzip
```

ROS Humble 的 `rclpy` 使用系统 Python 3.10。若用虚拟环境装 pytest，使用系统解释器创建 `python3 -m venv --system-site-packages .venv`，并确认 `import rclpy` 成功。

### 放到 WSL 主目录

在 Ubuntu 终端把项目放到 `~/robot-interview`。已有 ZIP 时，将下面的路径替换为实际下载位置。通过 Git 克隆后可直接进入已经检出的目录。

```bash
unzip -n /path/to/go1-robot-interview.zip -d "$HOME"
cd ~/robot-interview
bash scripts/check_env.sh
```

如果需要完整上游仓库供面试对照或运行 Core，先在一个新目录中按固定版本克隆。以下命令在已经存在目录时只显示该目录的 Git 状态，不会切换分支。建议在解压前完成克隆，随后用 `unzip -n` 增加模拟驱动目录。已有交付树时可在另一新目录执行同一流程。

```bash
mkdir -p ~/robot-interview
cd ~/robot-interview
if [ ! -e phanthymotus-driver ]; then
  git clone https://github.com/4paradigm/phanthymotus-driver.git &&
    git -C phanthymotus-driver checkout --detach 25c56b0f4fdb5616f41a9a7dc315824e8d34f5c9
else
  git -C phanthymotus-driver status --short --branch
  git -C phanthymotus-driver rev-parse HEAD
fi
if [ ! -e phanthymotus ]; then
  git clone https://github.com/4paradigm/phanthymotus.git &&
    git -C phanthymotus checkout --detach a642f8a8579876f5a8939718b1bff8eb9d3d59a8
else
  git -C phanthymotus status --short --branch
  git -C phanthymotus rev-parse HEAD
fi
```

## 2. 编译与启动

```bash
cd ~/robot-interview
export ROS_DOMAIN_ID=43 ROS_LOCALHOST_ONLY=1
bash scripts/build.sh
bash scripts/run_demo.sh
```

或者直接从工作空间运行。

```bash
cd ~/robot-interview/go1_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch go1_bringup go1_demo.launch.py
```

在其它终端使用相同 `ROS_DOMAIN_ID` 和 `ROS_LOCALHOST_ONLY`，并加载同一 ROS underlay 和 overlay。若图形窗口不可用，启动加 `rviz:=false`。用 `mcp:=false` 单独启动 ROS2，通过 `ros2 run go1_bringup go1_mcp_node` 另启 MCP 桥。

RViz2 的 Fixed Frame 为 `odom`；默认启用 Go1 RobotModel、TF、Odometry、Path 和 Grid。先看静止模型是否有完整四腿，再发一条 MoveFor，检查机身位移与腿部关节。若想看清腿部，展开 TF 显示项并暂时隐藏坐标轴。系统默认 50 Hz 状态更新。

### 在 WSL Jazzy 中验收 RViz2

打开两个 Ubuntu 24.04 终端。在两个终端各执行下面这段，共用通过无界面测试的 Fast DDS 回环配置。已完成构建时可以直接使用现有安装目录。

```bash
cd ~/robot-interview
source /opt/ros/jazzy/setup.bash
source go1_ros2_ws/install/setup.bash
unset ROS_LOCALHOST_ONLY ROS_STATIC_PEERS ROS_DISCOVERY_SERVER RMW_FASTRTPS_USE_QOS_FROM_XML
unset FASTDDS_DEFAULT_PROFILES_FILE CYCLONEDDS_URI
export ROS_DOMAIN_ID=97 ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/scripts/fastdds_loopback.xml"
```

在第一个终端启动演示，并保持窗口运行。

```bash
ros2 launch go1_bringup go1_demo.launch.py rviz:=true core_url:=disabled
```

看到 Go1 四腿站姿后，在第二个终端发送运动指令。

```bash
ros2 action send_goal /go1/move_for go1_interfaces/action/MoveFor \
  '{vx: 0.2, vy: 0.0, wz: 0.0, duration_s: 3.0}' --feedback
```

验收时观察四腿随运动摆动、机身前进约 0.6 m 后停住，以及 `Path` 留下轨迹。终端应返回 `SUCCEEDED`。如果 RViz2 只显示网格，在 Displays 中检查 Fixed Frame 为 `odom`，并展开 `Go1` 查看 `/robot_description` 与 TF 状态。

## 3. ROS 2 操作

从新终端配置环境后执行。

```bash
export ROS_DOMAIN_ID=43 ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash
source ~/robot-interview/go1_ros2_ws/install/setup.bash

ros2 topic list
ros2 topic echo /joint_states --once
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo odom FR_foot
ros2 service call /go1/status std_srvs/srv/Trigger '{}'
ros2 action info /go1/move_for
```

每条 Topic 指令在 0.5 秒后过期。用持续 10 Hz 发布演示。将 `linear.x` 设为正或负可前进或后退，`linear.y` 控制横移，`angular.z` 使用 rad/s。

```bash
ros2 topic pub -r 10 /go1/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2}}'
# 在另一个终端调用停止。先停掉持续发速度的进程，避免新的实时命令重新开始运动。
ros2 service call /go1/stop std_srvs/srv/Trigger '{}'
```

使用 Action 一次执行 3 秒并查看反馈。Ctrl+C 可让支持取消的 ROS 2 CLI 向 Goal 发送取消请求。也可以在另一个终端调用 Stop Service，中断运行中的 Action。

```bash
ros2 action send_goal /go1/move_for go1_interfaces/action/MoveFor \
  '{vx: 0.2, vy: 0.0, wz: 0.0, duration_s: 3.0}' --feedback
```

## 4. MCP 操作与 Core 接入

默认 HTTP 端点为 `http://127.0.0.1:15795/mcp`。用于 Phanthy Motus 的 JSON-RPC 支持 `initialize`、`tools/list`、`tools/call`。模型也提供 `resources/list` 和 `resources/read`。

```bash
mcp() { curl -fsS -H 'Content-Type: application/json' -d "$1" http://127.0.0.1:15795/mcp; printf '\n'; }

mcp '{"jsonrpc":"2.0","id":1,"method":"initialize","params":{"protocolVersion":"2024-11-05","capabilities":{},"clientInfo":{"name":"cli","version":"1"}}}'
mcp '{"jsonrpc":"2.0","method":"notifications/initialized"}'
mcp '{"jsonrpc":"2.0","id":2,"method":"tools/list"}'
mcp '{"jsonrpc":"2.0","id":3,"method":"tools/call","params":{"name":"loco","arguments":{"action":"move","vx":0.2,"vyaw":30,"duration":3}}}'
mcp '{"jsonrpc":"2.0","id":4,"method":"tools/call","params":{"name":"loco_state","arguments":{"action":"read"}}}'
mcp '{"jsonrpc":"2.0","id":5,"method":"tools/call","params":{"name":"loco","arguments":{"action":"stop"}}}'
```

`loco.move` 返回已接受的 ROS Goal ID，`loco_state` 中可读到执行状态。按上游 Go1 的接口，MCP 的 `vyaw` 输入是 °/s；其它运动接口使用 SI。其它工具为 `battery`、`imu`、`joints` 和 `model`。

接入 Core 时，按固定版本的官方部署流程启动 Agent Core。确认它能从同一 Linux 网络命名空间访问 MCP loopback 地址。使用 Core 部署实际加载的 Fast DDS 配置，模拟器、MCP 桥、状态发布和 RViz 均需加载同一套参数。如下使用了官方 Compose 默认的 Domain 42 与 RMW。自签的 Core 端点需要为 Python 提供信任链，`SSL_CERT_FILE` 可指向已确认的 CA 文件。

```bash
cd ~/robot-interview
unset ROS_LOCALHOST_ONLY CYCLONEDDS_URI
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/phanthymotus/agent-core/deploy/dds-local.xml"
bash scripts/run_demo.sh core_url:=https://localhost:15678
```

Core 收到注册后会读取六个工具。可在 Dashboard 查看 `Go1 Kinematic Simulator`，确认 `joints` 宣告 `sensor/skeleton`，`model` 能读取 URDF，调用 `loco` 时 `/odom` 同步变化。Core 和 Dashboard 的验收状态见测试报告。

## 5. 测试与录制

```bash
cd ~/robot-interview
bash scripts/smoke_test.sh --unit
bash scripts/smoke_test.sh
```

全量脚本起独立 DDS Domain 上的 headless Launch。测试 17 条 TF 父子边、标准状态消息、0.6 m 三秒任务、反馈/取消/停止、真实 HTTP 调用与 ROS 状态一致，以及进程关闭。测试会输出位移和 launch 日志路径。

演示运行时，从相同 Domain 的另一终端开始录制并用 Ctrl+C 结束。

```bash
bash scripts/record_demo.sh
ros2 bag info bags/go1-YYYYMMDD-HHMMSS
```

录包包含标准消息、TF、命令 Topic、Action 反馈和状态，以及 `/go1/command_events` 中的接受、取消和停止事件。回放状态用于看图时，先关闭模拟器与 MCP 避免主题冲突；可以用 `ros2 bag play --topics` 只回放状态、TF 和轨迹。

## 6. 排查

| 现象 | 检查 |
| --- | --- |
| 找不到 `rclpy` 或 MoveFor | 检查系统 Python 版本，以及 ROS underlay 和 workspace overlay 均已 source。 |
| `go1_mcp_node` 找不到或无执行权限 | 确认上游同级目录中有 `interview/go1_mcp`。Linux 中可执行 `chmod +x phanthymotus-driver/interview/go1_mcp/go1_mcp_node` 后重新编译。 |
| 没有图形窗口 | 从 WSL 检查 `DISPLAY`、`WAYLAND_DISPLAY` 和 WSLg；使用 `rviz:=false` 运行 headless 测试。 |
| RViz 只显示网格 | Fixed Frame 应为 `odom`，RobotModel Description Topic 应为 `/robot_description`。检查 DDS Domain、`/joint_states` 与 `odom → trunk`。 |
| 单条 Twist 只走一小段 | 检查 Watchdog 是否超时，提高指令频率或通过 MoveFor 指定持续时间。 |
| Action Goal 被拒绝 | 查询 `/go1/status` 的 owner、task 和 stop_reason，检查是否有别的任务、是否仍在 0.15 s 停止保持期。 |
| MCP 超时 | 检查端口和本地 TCP，确认 MCP 与模拟器 DDS 参数一致；若仅 Core 注册失败，核对 loopback 路由及 TLS。 |
| 真机与模拟 Topic 重名 | 独立演示使用隔离的 ROS Domain。Core 联调使用仅含本项目的模拟环境。 |

[源码审计](docs/SOURCE_AUDIT.md) · [架构与真机 SDK 替换边界](docs/ARCHITECTURE.md) · [测试报告](docs/TEST_REPORT.md)

仿真中展示的是平面位姿积分、站姿与程序化步态。IMU 姿态、角速度和电量来自模拟模型，足端接触、力矩和真实动力学不在本次实现范围内。
