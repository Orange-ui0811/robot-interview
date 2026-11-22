# 验收报告

日期为 2026-11-22。Windows 检查使用 Python 3.13.8 与 pytest 8.3.3。ROS 2 集成测试在 WSL Ubuntu 24.04 中运行。

## 本机执行结果

| 验证 | 结果 | 边界与证据 |
| --- | --- | --- |
| `python -m pytest -q -s -rs tests` | **27 通过，4 跳过** | 通过项包含 URDF 字节一致性、关节与 TF 拓扑、站姿和步态、位姿积分、命令限幅、无效值拒绝、Watchdog、并发所有权、Action 核心状态机、HTTP 解析及注册协议。 |
| 3 s 运动学场景 | **通过** | 注入单调时钟。初始 (0, 0)，vx=0.2 m/s，实测模拟 x=0.600000000 m、y=0.000000000 m，绝对误差 1.110e−16 m，结束后保持。此结果来自 SDK 层。 |
| URDF Git blob | **通过** | `git hash-object --no-filters` 返回 `6de9c5c2e594d440dae7fbc823fb270743f51b9f`，与上游固定提交的文件一致。 |
| Python 源码编译 | **通过** | `python -m compileall -q go1_ros2_ws/src phanthymotus-driver/interview scripts tests` 返回 0。 |
| 包清单、配置与入口解析 | **通过** | 解析 4 份 ROS package.xml、6 份 YAML/JSON/RViz 配置和无扩展名 MCP Python 入口。 |
| TCP HTTP transport | **跳过 4 项** | 可绑定 loopback socket，受限执行进程连接时返回 `WinError 10013`。HTTP handler 的独立用例读写真实格式的 HTTP 字节，传输在内存中。注册报文与定时刷新使用测试接收端替身。 |
| Headless ROS 2 启动及集成 | **被环境阻塞** | 直接执行 `scripts/ros_smoke.py` 缺 `action_msgs`，MCP 入口缺 `ament_index_python`。此处未加载真实 ROS 消息、DDS 连接、Action 服务端或 RViz2。 |
| 上游本地克隆 | **被网络阻塞** | 两次 `git clone --no-checkout` 均无法连接本机配置的 Git 代理。通过 GitHub 读取固定提交的源码和模型，已在 [源码审计](SOURCE_AUDIT.md) 记录。 |

## Ubuntu 24.04 · ROS 2 Jazzy 复测

从用户的 Windows PowerShell 进入 WSL 后，`colcon build --symlink-install` 编译 4 个包成功，总耗时 15.2 s。WSL 中 pytest **31 项通过**，包括真实回环 TCP 用例。`ros2 interface show go1_interfaces/action/MoveFor` 成功显示 Goal、Result 与 Feedback 字段。

20:46 的测试通过遥测、TF 和状态服务。MCP 桥在匹配订阅者后累计有 82 次骨架帧发布调用正常返回，测试订阅者和独立 CLI 均超时。20:50 复测沿用 ROS Domain 97，将测试用 Fast DDS 回环 UDP 配置的 `maxMessageSize` 限制为 1200 字节后，两帧连续骨架数据送达，无界面集成测试通过。对照结果表明报文配置影响了交付，具体丢包位置未经抓包确认。

复测命令为 `GO1_ROS_DISTRO=jazzy GO1_TEST_DOMAIN_ID=97 bash scripts/smoke_test.sh`。三秒 ROS Action 实测模拟里程计为 x=0.600000009 m、y=0.000000000 m，Topic、Trigger、Action 和 MCP 共享运动状态的检查通过，三个节点均干净退出。完整启动日志位于 WSL 的 `go1_ros2_ws/log/go1-ros-smoke-35rxz1mg/launch.log`。

## 最终 A–E 验收状态

| 项 | 当前结果 | Linux/WSL 验收 |
| --- | --- | --- |
| A · RViz2 | URDF、站姿、动静态 TF 的静态配置齐备。图形启动及运动渲染未验证。 | `bash scripts/run_demo.sh` 后查看 RViz2，执行 README 的 MoveFor 和旋转指令。 |
| B · ROS 2 | WSL Jazzy 的无界面测试通过。覆盖标准消息、TF、Topic、状态和停止 Service、Action 反馈与取消、独立里程计订阅和节点退出。 | 使用上文复测命令。 |
| C · MCP | 本地 HTTP 工具调用、连续骨架帧、Goal ID 与 ROS 共享运动状态已验证。 | Core 自动注册和 Dashboard 仍需按 README 实测。 |
| D · 工程测试 | Windows 静态检查和 WSL colcon、rosidl、31 项 pytest 及全量无界面测试通过。 | rosbag2 录制与回放待验证。 |
| E · 可复现性 | 固定版本、原始模型、依赖、命令和测试已记录。尚无另一台 Linux 主机复现结果。 | 在新目录解压，执行 README 的检查、构建、演示与验收命令。 |

这次没有连接或控制真机。报告没有把模拟步态视为动力学验证。
