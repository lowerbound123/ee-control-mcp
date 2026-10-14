# ROS 连接异常排查（2026-10-10）

用户报告：运行 `./scripts/start_mcp.sh --build-messages --readonly`，尚未调用控制工具，机械臂抖动且多数 topic 不可用。日志包含 `GET /health`，机器人端确认为 ROS 2 Humble。停止 MCP 后是否恢复尚未确认。

已确认的配置问题：旧启动脚本加载 Jazzy，并在 Jazzy 下编译消息包；参考 `deploydev` 使用 Humble。必须先纠正 ROS 运行时与消息构建环境，但这些事实尚不能单独证明抖动的完整因果链。

| 项目 | deploydev | 修正前的 MCP |
|---|---|---|
| ROS 环境 | `.env.humble.bash` 加载 `/opt/ros/humble_origin` 和现场 X5_ws | 加载 `/opt/ros/jazzy`，重新构建本地 x5_ros2 消息包 |
| 初始化 | `rclpy.init()`，默认 Context | `Context()` + `rclpy.init(context=...)` |
| 节点 | `robot_operator` | `ee_control_mcp`，不启用参数服务 |
| 执行线程 | 后台 `rclpy.spin(node)` | 后台 `SingleThreadedExecutor.spin_once(0.1)` |
| 反馈订阅 | 相同左右臂 `RobotStatus` topic，深度 2 的默认 QoS | 相同 topic，sensor-data QoS：Best Effort、深度 5 |
| 相机订阅 | 相同三个 `CompressedImage` topic，默认 QoS 深度 2 | 相同 topic，sensor-data QoS |
| 控制发布 | 普通 direct 模式创建 `RobotStatus` 发布器，发布到 `/arm_master_l_status` / `/arm_master_r_status` | 控制模式使用用户指定 `RobotCmd` 和 `/arm_master_l_cmd` / `/arm_master_r_cmd`；只读不创建这些发布器 |
| 启动动作 | connect 不执行机械臂 reset；runner 可随后 reset。左相机无帧时 connect 会请求相机硬件 reset | 无机械臂 reset、无相机 reset，队列为空时执行器等待 |
| HTTP 连接 | 无 MCP HTTP | 应用启动阶段连接 ROS；`GET /health`、MCP 初始化/工具发现仅返回状态或元数据 |
| Domain | 启动环境继承现场配置，示例脚本为 112/148 | 按用户先前要求固定 60；需与机器人端一致 |
| RMW | 文件中未明确设置 RMW 实现 | 文件中未明确设置 RMW 实现，继承启动环境/使用 ROS 默认实现 |

代码依据：

- `/home/zhouyuxiang/coding/postrain4internvla/.env.humble.bash`
- `/home/zhouyuxiang/coding/postrain4internvla/deploydev/robots/acone.py`：connect、read_state、相机恢复。
- `/home/zhouyuxiang/coding/postrain4internvla/deploydev/robots/acone_driver/ros2_operator.py`：订阅、发布器、follow_arm_publish。
- 本项目 `backends.py`、`runtime.py`、`server.py`、`scripts/start_mcp.sh`。

消息源的 `RobotStatus` 定义与另一个已有 ROS overlay 相同：Header、6 维 float64 end_pos、各 7 维 float64 joint_pos/joint_vel/joint_cur。未发现这两份源文件的字段差异；这不代表远端加载的实际二进制已比对。`RobotCmd` 与 `RobotStatus` 的控制路径是不同接口，不应直接互换 topic 或类型。

ROS 官方建议分布式系统使用一致的 ROS 版本和 RMW 实现：[官方文档源文件](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Concepts/Intermediate/About-Different-Middleware-Vendors.rst)。仅相同 Domain、相同 topic 名、相同 `.msg` 文本不能替代运行时兼容性核对。日志中的 `ROS_LOCALHOST_ONLY` 弃用警告说明配置处理方式，不能作为本次抖动的根因证据。

本次修改：默认预期版本改为 Humble；启动脚本选择匹配版本、不回退 Jazzy；脚本在消息构建前及 overlay 加载后检查版本，后端在创建 ROS Context/participant 前再次检查；按 ROS 版本分开消息 build/install/log 目录。QoS、执行线程和控制协议保留，以免同时改变多个调查变量。

验证仅使用替身 ROS 和真实本机 HTTP/MCP 客户端。覆盖只读启动、`/health`、MCP 初始化/工具发现、props/state、只读拒绝 home：没有创建机械臂命令发布器、没有命令入队。版本不匹配和 overlay 切回 Jazzy 均在节点创建前被拒绝。

当前本机仅安装 Jazzy，原 `.venv` 使用 Python 3.12。本次没有安装或启动 Humble、没有重新连接真实机器人、没有更改机器人端环境或删除原 Jazzy 构建结果。应在匹配的 Humble/Python 环境运行服务并构建消息包。实际抖动是否消除仍需现场确认；如在匹配环境下仍异常，需要机器人端 DDS/驱动/CAN 日志、topic 不可用的具体表现、停止 MCP 后的恢复情况及资源使用数据，继续区分发现/传输故障和驱动调度故障。

## Robotics52 部署后续排查

在 `pjlab@10.6.3.69` 的 Humble / Domain 60 环境中，只读查询 ROS 图发现：`/arm_master_l` 和 `/arm_master_r` 分别订阅 `/arm_master_l_cmd`、`/arm_master_r_cmd`（`RobotCmd`），并发布 `/arm_master_l_status`、`/arm_master_r_status`（`RobotStatus`）。原 MCP 订阅的 `/arm_slave_l_status`、`/arm_slave_r_status` 没有发布者。因此相机虽有新帧，双臂状态始终为空，`state_ready=false`。临时只读订阅实际 master status topic，已收到两臂各 6 维 end_pos、各 7 维 joint_pos/joint_vel/joint_cur；没有发布控制消息。

此外，虽然服务监听 `0.0.0.0`，`http://10.6.3.69:3000/health` 返回 200，MCP 初始化仍因 SDK 的 Host 校验返回 421（`Invalid Host header`）。监听地址不会自动扩展 Host/Origin 白名单，健康接口成功也不能证明 MCP 初始化成功。

修正采用独立 `config/robotics52.yaml`：订阅 master status topic，监听 `0.0.0.0`，在 Host/Origin 白名单加入 `10.6.3.69`；`scripts/start_remote_mcp.sh` 默认加载此配置。通用配置仍保留原始方案的 slave status topic，可供其他明确发布这些反馈的部署使用。这些修正针对已证实的反馈缺失与 HTTP 421；不能据此宣称原机械臂抖动已解决。

用户关闭 MCP 后进行配置修改。回归验证使用替身 ROS 和本机环回模拟 HTTP/MCP 服务；远端仅执行 `--check-config` 导入与配置检查，不启动 ROS 节点、不发布动作。实际修正后的 ROS 状态和 IP 直连需要用户重启服务后再验证。
