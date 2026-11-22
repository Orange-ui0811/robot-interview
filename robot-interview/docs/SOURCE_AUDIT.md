# 上游源码审计

| 仓库 | 审计提交 | 本地获取情况 |
| --- | --- | --- |
| [phanthymotus-driver](https://github.com/4paradigm/phanthymotus-driver/commit/25c56b0f4fdb5616f41a9a7dc315824e8d34f5c9) | `25c56b0f4fdb5616f41a9a7dc315824e8d34f5c9` | 通过 GitHub 读取指定提交下的源码和 URDF。当前交付的同名目录包含新增驱动文件。 |
| [phanthymotus](https://github.com/4paradigm/phanthymotus/commit/a642f8a8579876f5a8939718b1bff8eb9d3d59a8) | `a642f8a8579876f5a8939718b1bff8eb9d3d59a8` | 通过 GitHub 读取注册、MCP 客户端和部署配置。 |

本机 Git 连接上游失败。两个提交是 GitHub 所返回的提交，当前工作区中没有两份完整 Git 克隆。按 README 中的顺序可先克隆固定版本，再把新增文件放到同一目录结构中。

## URDF 与关节

模型从 Driver 固定提交逐字节复制到 `go1_description/urdf/go1_model.urdf`，Git blob 为 `6de9c5c2e594d440dae7fbc823fb270743f51b9f`。模型共 17 个 Link 和 16 个 Joint，其中 12 个 revolute，4 个足端 fixed。视觉部件为盒、圆柱和球体，无外部 mesh。根 Link 为 `trunk`，每条腿按 `trunk → <leg>_hip → <leg>_thigh → <leg>_calf → <leg>_foot` 相连。每条腿的 thigh 与 calf 长 0.213 m，足端球半径 0.020 m。[固定版本 URDF](https://github.com/4paradigm/phanthymotus-driver/blob/25c56b0f4fdb5616f41a9a7dc315824e8d34f5c9/unitree/go1/resource/go1_model.urdf)

| 腿 | 关节顺序，从 0 开始 |
| --- | --- |
| FR | 0 `FR_hip_joint`、1 `FR_thigh_joint`、2 `FR_calf_joint` |
| FL | 3 `FL_hip_joint`、4 `FL_thigh_joint`、5 `FL_calf_joint` |
| RR | 6 `RR_hip_joint`、7 `RR_thigh_joint`、8 `RR_calf_joint` |
| RL | 9 `RL_hip_joint`、10 `RL_thigh_joint`、11 `RL_calf_joint` |

| 每腿关节 | 类型 | 轴 XYZ | URDF 位置限位 rad | URDF 速度限位 rad/s | URDF effort 限位 N·m | 默认站立 rad |
| --- | --- | --- | --- | --- | --- | --- |
| hip | revolute | `1 0 0` | −1.047 至 1.047 | 30.1 | 23.7 | 0.00 |
| thigh | revolute | `0 1 0` | −0.663 至 2.966 | 30.1 | 23.7 | 0.75 |
| calf | revolute | `0 1 0` | −2.721 至 −0.837 | 20.1 | 35.5 | −1.50 |
| foot | fixed | — | — | — | — | — |

## 原始实现与复用关系

| 模块 | 上游行为 | 本工程处理 |
| --- | --- | --- |
| [Go1 main](https://github.com/4paradigm/phanthymotus-driver/blob/25c56b0f4fdb5616f41a9a7dc315824e8d34f5c9/unitree/go1/main.py) | `Go1Bundle` 按配置组装工厂创建的插件，统一 `start_all/stop_all`，合并 `get_tool(s)`，按名称及 `action` 分发。HTTP `/mcp` 提供初始化、工具列表、工具调用。 | 沿用工具描述和 JSON-RPC 包装，独立实现 ROS 客户端及生命周期。 |
| [Go1 SDK Client](https://github.com/4paradigm/phanthymotus-driver/blob/25c56b0f4fdb5616f41a9a7dc315824e8d34f5c9/unitree/go1/go1_sdk_client.py) | 后台收发 HighState 与 HighCmd。收包解析后在锁内更新 `_snapshot`，`snapshot()` 返回字典副本。字段含 `fresh, control_level, mode, mode_name, gait_type, gait_name, progress, body_height, foot_raise_height, yaw_speed, velocity, position, imu, joints, foot_force, foot_pos, foot_speed, range_obstacle, wireless_remote, battery`。缺样本时 `fresh=False`。 | 模拟端内存中只有一份权威状态。新建符合运动学约束的快照，并从它生成标准 ROS 消息。 |
| [Go1 SDK Proxy](https://github.com/4paradigm/phanthymotus-driver/blob/25c56b0f4fdb5616f41a9a7dc315824e8d34f5c9/unitree/go1/sdk_proxy.py) | `SdkProxy` 用子进程隔离 SDK，进程间队列传递 `snapshot`、诊断和运动请求，在子进程调用真机 SDK。 | 模拟器无真机进程、UDP 连接和厂商 SDK 依赖。 |
| [Go1 Sensors](https://github.com/4paradigm/phanthymotus-driver/blob/25c56b0f4fdb5616f41a9a7dc315824e8d34f5c9/unitree/go1/sensors.py) | 传感卡从同一快照取数据，经 `String` JSON 发布，`info` 也可轮询。骨架卡含 `idx/name/q/q_rad/dq` 等字段及 `imu_quat`（wxyz），模型卡返回 `urdf`。 | 用标准 ROS 消息作源，按关节名恢复上游骨架顺序。模拟样本显式带 `simulated`，缺失的力矩、温度或同帧姿态用 `null`。 |
| [Go1 Controllers](https://github.com/4paradigm/phanthymotus-driver/blob/25c56b0f4fdb5616f41a9a7dc315824e8d34f5c9/unitree/go1/controllers.py) | `loco.move` 接受 `vx,vy,vyaw,duration`，在插件边界把 °/s 转为 rad/s，再调用 SDK。真机 SDK 有有效期内移动 mode=2 和 idle mode=0；定时运动可续发，高层目标到期归零。 | 命令字段沿用 Go1 插件，`vyaw` 保留度每秒兼容。ROS 内由 MoveFor 持有定时运动，`loco.stop` 调 Trigger。 |
| [Go1 config](https://github.com/4paradigm/phanthymotus-driver/blob/25c56b0f4fdb5616f41a9a7dc315824e8d34f5c9/unitree/go1/config.yaml)、[driver.yaml](https://github.com/4paradigm/phanthymotus-driver/blob/25c56b0f4fdb5616f41a9a7dc315824e8d34f5c9/unitree/go1/driver.yaml)、[CONTRIBUTING](https://github.com/4paradigm/phanthymotus-driver/blob/25c56b0f4fdb5616f41a9a7dc315824e8d34f5c9/unitree/go1/CONTRIBUTING.md) | 原驱动端口 15715，`plugins.*.enabled` 启用状态、控制、视觉和外设卡，描述文件列出可发布卡片。 | 新驱动使用 15795、独立 ID 和六张卡，交付只含模拟所需模块。 |
| [Generic Backend](https://github.com/4paradigm/phanthymotus-driver/blob/25c56b0f4fdb5616f41a9a7dc315824e8d34f5c9/simulator/generic/backend/__init__.py)、[World](https://github.com/4paradigm/phanthymotus-driver/blob/25c56b0f4fdb5616f41a9a7dc315824e8d34f5c9/simulator/generic/world.py) | `LocalBackend` 建立独轮车底盘、关节插值和栅格传感。在 `World` 中集中推进时钟、锁定状态与任务。 | 采用注入时钟、世界唯一状态源、卡片只读的结构；新写含横向速度的平面积分与四足关节动画。 |
| [vendor_runtime.py](https://github.com/4paradigm/phanthymotus-driver/blob/25c56b0f4fdb5616f41a9a7dc315824e8d34f5c9/common/vendor_runtime.py)、[README_dev.md](https://github.com/4paradigm/phanthymotus-driver/blob/25c56b0f4fdb5616f41a9a7dc315824e8d34f5c9/README_dev.md) | 抽象插件包、HTTP 处理、定期注册。硬件 DDS 默认机器人 Domain 0、Core Domain 42，使用各自 Context/Executor，CycloneDDS 可绑指定网卡。 | 复用协议形状，保持独立实现。模拟、MCP 桥和 RViz 共用一个本地 ROS Domain，不需要连接硬件网卡。 |
| [Agent Core 注册与工具发现](https://github.com/4paradigm/phanthymotus/blob/a642f8a8579876f5a8939718b1bff8eb9d3d59a8/agent-core/src/api/mcp_manage.py)、[MCP Client](https://github.com/4paradigm/phanthymotus/blob/a642f8a8579876f5a8939718b1bff8eb9d3d59a8/agent-core/src/mcp_client.py) | `POST /api/mcp` 注册，Core 发送 `initialize (2024-11-05)` 和 `tools/list`，读取 `type/inputSchema/topic_out`；`tools/call` 用于状态和执行。 | 支持相同 HTTP 请求。配置 Core URL 后向 `/api/mcp` 周期发送带 `id/name/url/transport/category` 的注册。 |
| [Agent Core compose](https://github.com/4paradigm/phanthymotus/blob/a642f8a8579876f5a8939718b1bff8eb9d3d59a8/agent-core/deploy/docker-compose.yml)、[dds-local.xml](https://github.com/4paradigm/phanthymotus/blob/a642f8a8579876f5a8939718b1bff8eb9d3d59a8/agent-core/deploy/dds-local.xml) | Linux host networking，Fast DDS Domain 42，Fast DDS profile 将 UDP 约束至 loopback。 | 如果同主机运行 Core，模拟启动时使用同一个 Domain、RMW 与 profile。独立演示无需 Core。 |

## 许可证

上游 Apache-2.0 许可证随工程分发。Go1 URDF 原始字节及头部注释保留。其余驱动与模拟代码均位于新增目录。
