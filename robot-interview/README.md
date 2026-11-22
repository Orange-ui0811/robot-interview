# Go1 ROS 2 模拟驱动

用 Unitree Go1 URDF 和运动学模拟器展示 ROS 2 Topic、Service、Action、TF、RViz2 及 Phanthy Motus MCP 接入。ROS 2 和 MCP 共用同一个模拟器。

Ubuntu 24.04 + ROS 2 Jazzy 已通过无界面集成测试。三秒 `MoveFor` 指令以 0.2 m/s 前进约 0.6 m，同时通过连续 MCP 骨架流、停止与取消、节点退出等检查。

- [安装、构建与 RViz2 演示](README_zh.md)
- [架构与接口](docs/ARCHITECTURE.md)
- [验收报告](docs/TEST_REPORT.md)
- [上游源码与许可证](docs/SOURCE_AUDIT.md)
