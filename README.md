# 双臂 ROS HTTP MCP

通过官方 MCP Python SDK 的 Streamable HTTP，把 ROS 2 `RobotCmd` 双臂控制暴露为 MCP 工具。默认端点为 `http://127.0.0.1:3000/mcp`，所有客户端共用一个串行执行队列。服务启动只建立 ROS 连接和订阅，不自动发送动作或归零。

字符串命令参考 `show_harness_mcp`；状态布局和三个相机 topic 参考 `postrain4internvla/deploydev`。控制消息使用 `RobotCmd`，EEF 为 `mode=4`，关节为 `mode=5`。`home` 固定为六个关节归零、夹爪 `0`。没有仿真专用的 `finish_episode`。

## 安装与启动

需要 Python 3.10 或更高版本。模拟模式不需要 ROS；真机模式还需要可导入的 `rclpy`、`arx5_arm_msg.msg.RobotCmd` / `RobotStatus` 和 `sensor_msgs.msg.CompressedImage`。Python 必须与 ROS 编译时的版本匹配。

当前机器人运行 **ROS 2 Humble**，默认 `ros.expected_distro: humble`。真机服务必须在匹配的 Humble 环境中运行，使用对应的 Python 和消息包；仅本机安装了 Jazzy 不代表它适合连接该机器人。下面创建虚拟环境时，系统 Python 也必须属于所选 ROS 的运行环境。

```bash
cd /home/zhouyuxiang/coding/ee-control-mcp
/usr/bin/python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install -e '.[dev]'

# 检查配置，不连接 ROS、不启动 HTTP。
ee-control-mcp --config config/default.yaml --check-config

# 模拟状态、夹爪和三个标注相机，可测试全部 MCP 工具。
ee-control-mcp --backend sim --config config/default.yaml
```

在另一个终端读取状态和相机：

```bash
source .venv/bin/activate
python examples/client.py --images --cameras head left_wrist
# 默认只读；下面的参数才执行用户原始示例的 EEF 目标、等待 3 秒、home。
python examples/client.py --execute-demo
```

真实 ROS 使用 [一键启动脚本](scripts/start_mcp.sh)，先准备与机器人一致的 Humble 环境，再在 Bash 或 Zsh 中执行相同命令。当前这台电脑只有 `/opt/ros/jazzy`，脚本会退出；需要在 Humble 主机或 Humble 容器中准备依赖后运行。

```bash
cd /home/zhouyuxiang/coding/ee-control-mcp

# 首次：如果消息工作空间尚未编译，先编译 arx5_arm_msg，再以只读模式启动。
./scripts/start_mcp.sh --build-messages --readonly

# 后续启动默认只读，也可显式加 --readonly。
./scripts/start_mcp.sh

# 启用机械臂控制工具。
./scripts/start_mcp.sh --control
```

脚本根据调用它的 Bash/Zsh 自动选择 `setup.bash` / `setup.zsh`，按 YAML 的 `ros.expected_distro` 选择 `/opt/ros/humble` 或 `/opt/ros/humble_origin`，加载消息工作空间和项目 `.venv`，然后以前台进程启动 MCP。加载 ROS 后、加载消息 overlay 后都会检查版本；后端还会在 `rclpy.init()` 前独立检查，不匹配时不创建 DDS participant。通信参数固定为 `ROS_DOMAIN_ID=60`、`ROS_LOCALHOST_ONLY=0`。机械臂驱动需要在相同 ROS Domain 中运行；启动脚本不启动驱动。按 Ctrl+C 退出服务后返回原终端。

默认消息工作空间为 `/home/zhouyuxiang/coding/acone_control_debug/code/x5_ros2`，默认虚拟环境和配置文件分别为项目内 `.venv`、`config/default.yaml`。脚本从自身位置定位项目，可以从其他目录调用。路径不同或需要修改端口时：

```bash
./scripts/start_mcp.sh --workspace /path/to/x5_ros2 --readonly --port 3010
./scripts/start_mcp.sh --workspace /path/to/x5_ros2 --control
./scripts/start_mcp.sh --help
```

还支持 `--ros-dir`、`--venv`、`--conda-env`、`--conda-exe`、`--config`、`--host`、`--log-level` 和 `--check-config`。`--conda-env` 接受环境名或路径，自动激活独立 Conda 环境；不能与 `--venv` 同时使用，也不能选择 `base`。未在 PATH 中找到 Conda 时，用 `--conda-exe` 指定可执行文件。脚本在子进程中清理调用者继承的 `PYTHONPATH` / `PYTHONHOME`，避免 Conda 自身或 MCP 误加载其他 Python 版本的库；Conda 先激活，再由指定 ROS 和消息工作空间重新建立搜索路径，原终端不受影响。

`--build-messages` 只编译 `arx5_arm_msg`，使用指定环境的 Python；构建失败或依赖缺失时不会启动服务。构建目录按版本隔离，例如 `build-mcp-humble`、`install-mcp-humble`、`log-mcp-humble`，不复用之前 Jazzy 的 `build/install`。启动优先加载 `install-mcp-humble`；不存在时可加载已有 `install`，但仍检查其 ROS 版本。Python 环境须预先创建并安装依赖，Python 版本须与 ROS 匹配。

启动脚本直接执行，不使用 `source`。如果只想在当前终端配置 ROS 环境，仍可使用 `source scripts/setup_ros_env.sh`；它现在默认 Humble，也可用 `EE_CONTROL_MCP_ROS_DIR` 指定安装位置，`EE_CONTROL_MCP_ROS_DISTRO` 设置预期版本。只读启动会传入 `--no-actuation`，不创建机械臂控制发布器，控制工具在入队前返回错误；`--control` 和 `--readonly` 不能同时使用。两种模式启动时均不自动发送动作或归零。

只读仍会加入 ROS/DDS 网络并订阅机械臂、相机；禁止动作发布不等于不影响通信资源。2026-10-10 的连接异常对比、版本修正与尚未确定的原因记录在 [ROS 连接排查](docs/ros-connection-investigation.md)。机器人版本未变更时，不应修改 `expected_distro` 来绕过检查。

### pjlab@10.6.3.69 的 Conda 部署

远端项目目录为 `/home/pjlab/zhouyuxiang/ee-control-mcp`，使用该目录下的独立 Conda 环境 `.conda`，Python 3.10。依赖安装在这个环境中，不安装到 `base`。环境创建与安装命令为：

```bash
cd /home/pjlab/zhouyuxiang/ee-control-mcp
/home/pjlab/miniconda3/bin/conda create --prefix "$PWD/.conda" \
  --override-channels -c conda-forge python=3.10 pip -y
source /home/pjlab/miniconda3/etc/profile.d/conda.sh
conda activate "$PWD/.conda"
python -m pip install -c config/constraints-humble.txt -e '.[dev]'
```

部署入口固定加载 `/opt/ros/humble_origin` 和现有 `/home/arx/LIFT/ARX_X5/ROS2/X5_ws` 消息工作空间，无须再次编译消息包，并默认使用 [Robotics52 配置](config/robotics52.yaml)。该配置监听 `0.0.0.0:3000`，允许通过 `10.6.3.69` 直接连接，反馈订阅使用现场驱动实际发布的 `/arm_master_l_status` / `/arm_master_r_status`。环境准备后，直接执行：

```bash
cd /home/pjlab/zhouyuxiang/ee-control-mcp
./scripts/start_remote_mcp.sh --check-config  # 导入检查，不创建 ROS 节点
./scripts/start_remote_mcp.sh --readonly      # 默认只读，端点 10.6.3.69:3000/mcp
./scripts/start_remote_mcp.sh --control       # 显式启用控制
```

脚本会自动激活项目 `.conda`；运行前无需手动 `conda activate`。远端仍使用 `ROS_DOMAIN_ID=60`，需要与实际驱动的 Domain 一致；参考项目环境文件中的 `148` 不会自动覆盖此设置。

在客户端把 MCP URL 设置为 `http://10.6.3.69:3000/mcp`；机器本地的 `http://127.0.0.1:3000/mcp` 也可以使用。此机器的 `/arm_slave_*_status` 没有发布者，通用 `config/default.yaml` 的反馈 topic 不适用于这里；直接调用通用脚本时须加 `--config config/robotics52.yaml`。`--config` 仍可覆盖远端入口选择的配置。

现场两臂的 `arm_end_type=2`，`RobotCmd.gripper` 使用新版夹爪原始电机单位，负值张开：此配置的 `open=-3.4`、`close=0`。空爪关闭的实际反馈约为左 `-0.068`、右 `-0.058`，全开约为左 `-3.317`、右 `-3.296`。全开最大偏差约 `0.104`，加少量反馈余量后，夹爪到位容差设为 `0.12`，反馈原样返回。字符串开合使用不带数值的 `left:open` / `right:close`；需要部分张开时，可用 `left:open:-1.7` 或在 `move_eef` / `move_joints` 的 `gripper` 字段设置原始值。此型号的开合范围为 `-3.4..0`，不应使用通用配置中的正值 `open=1`。

模拟后端直接跟随目标并生成 JPEG，**没有动力学或 FK/IK**；关节归零不会计算新的 EEF 位姿。不要用模拟结果推断真机可达性。真实 ROS 依赖缺失时启动报错，不会静默切换成模拟模式。

## MCP 客户端配置

本项目的 Codex 配置位于 `.codex/config.toml`，连接现有远端 `http://10.6.3.69:3000/mcp`。项目级配置仅在项目被明确标记为可信后加载，见 [官方 MCP 配置说明](https://learn.chatgpt.com/docs/extend/mcp?surface=cli)。启用后刷新客户端的 MCP 连接即可，机械臂服务继续使用原地址和端口。

支持 `mcpServers` JSON 配置的客户端可参考 [mcp.json](mcp.json)。Codex 的 TOML 配置示例，格式见 [官方连接说明](https://developers.openai.com/learn/docs-mcp)：

```toml
[mcp_servers.ee-control-mcp]
url = "http://127.0.0.1:3000/mcp"
# 设置了服务端 Token 时启用下一行：
# bearer_token_env_var = "EE_CONTROL_MCP_TOKEN"
```

服务端和示例客户端都从 `EE_CONTROL_MCP_TOKEN` 读取可选 Bearer Token。配置文件只保存变量名；环境变量未设置或为空时不启用鉴权。启用后 `/mcp` 和 `/health` 都要求 `Authorization: Bearer <token>`。

可用 `--host`、`--port` 或 YAML 修改监听地址。远程访问还要在 `http.allowed_hosts` 中加入服务端实际 IP/域名，例如 `192.168.1.20:*`，浏览器客户端的 Origin 加入 `allowed_origins`。这两个设置控制 SDK 的 Host/Origin 校验，监听 `0.0.0.0` 本身不会放行任意 Host。跨网络的加密连接可由 HTTPS 反向代理提供。

## 工具与调用参数

| 工具 | 用途 |
|---|---|
| `props` | topic、原始状态字段、方向映射、档位和控制参数 |
| `state` | 最新反馈、每臂目标与误差、队列进度，可附加 JPEG |
| `send_command` | 验证整批字符串命令，再按顺序入队 |
| `move_eef` | 左/右臂分别设置绝对 `[x,y,z,roll,pitch,yaw]` 和可选夹爪、档位 |
| `move_joints` | 左/右臂分别设置六维绝对关节角和可选夹爪、档位 |

三个控制工具共用参数：`wait=false`、`timeout_s=30`、`clear_queue=false`、`include_images=false`、`cameras=null`、`image_quality=null`。`timeout_s` 范围为 `0..120` 秒，仅限制调用等待。`cameras=null` 在请求图像时表示三个相机；也可指定 `head`、`left_wrist`、`right_wrist` 的非空、无重复列表。

`state` 还支持 `wait_for_new=false`、`since_seq=null`、`timeout_s=3`：等待比给定 `seq` 更新的机械臂反馈，省略 `since_seq` 时等待本次调用之后的更新。读取不会消费消息；图像更新不会触发机械臂反馈等待。

绝对 EEF 示例，与用户原始代码的最终目标相同：

```json
{
  "targets": {
    "left": {"end_pos": [0.1, 0.03, 0.05, 0, 0, 0], "gripper": 1, "speed": "slow"},
    "right": {"end_pos": [0.05, 0, 0.1, 0, 0, 0], "gripper": 1, "speed": "medium"}
  },
  "wait": true
}
```

`move_joints` 使用相同 `targets` 结构，把 `end_pos` 换成 `joint_pos`，例如 `[0,0,0,0,0,0]`。每臂可以独立指定档位；未写夹爪时保持已有控制值，首次取经过配置校准的当前夹爪反馈。左右臂各自创建 ROS 消息。

## 字符串命令

```text
<left|right|both>:<motion>[&motion][:value][@slow|medium|fast]
```

| 动作 | 默认增量 |
|---|---|
| `forward` / `backward` | 基座坐标 `+x` / `-x` |
| `left` / `right` | 基座坐标 `+y` / `-y` |
| `up` / `down` | 基座坐标 `+z` / `-z` |
| `twist-clockwise` / `twist-counterclockwise` | 末端局部旋转向量 `+rx` / `-rx` |
| `tilt-down-toward-table` / `tilt-up-away-from-table` | 末端局部旋转向量 `+ry` / `-ry` |
| `turn-left-across-table` / `turn-right-across-table` | 末端局部旋转向量 `+rz` / `-rz` |
| `open` / `close` | 配置的夹爪值，默认 `1` / `0` |
| `hold` | 保持该臂目标，须与另一个臂的有效动作配合 |
| `home` | `mode=5`、六个关节 `0`、夹爪 `0` |

平移值单位为米，旋转值单位为弧度。省略 `:value` 时默认平移 `0.05 m`、旋转 `0.1 rad`。平移与旋转数值须有限且为正，方向由动作名确定；`open` / `close` 的显式值使用原始 `RobotCmd` 夹爪单位，须有限，可以为负，不做 `0..1` 归一化或范围裁剪。可用范围取决于实际夹爪型号；省略数值时采用配置的开合值。`&` 组合中的显式值应用于每个动作，也应用于其中的夹爪词，例如 `left:up&close:0.02` 表示上移 2 cm 并设置夹爪为 `0.02`。对于 Robotics52 新版夹爪，上移并关闭可用不带数值的 `left:up&close`；独立控制位移和夹爪原始值时使用 `move_eef`。

`home`、`hold` 不能与其他动作组成 `&`，不能带数值；纯夹爪动作和 `hold` 不能加档位。并行步骤中一个臂只能出现一次，`both` 已包含两个臂。

```json
{"commands": "left:up:0.02@slow | right:down:0.01@fast", "wait": true}
```

```json
{
  "commands": [
    ["left:forward&up:0.01@medium", "right:open"],
    "left:left:0.03 | right:hold",
    "both:home@slow"
  ],
  "wait": true,
  "include_images": true,
  "cameras": ["head"]
}
```

`|` 或内层列表构成同一步双臂动作，外层列表构成顺序动作。内层列表的每个字符串只能是一个臂子句，不能再包含 `|`。相对动作从上次目标累加；首次控制或关节模式切回 EEF 时，从新鲜实际反馈建立参考目标。方向可以在 YAML 的 `control.motions` 中用六维增量覆盖，实际映射以 `props` 为准。

## 均匀插值与执行结果

默认固定发布周期为 `1/30 s`，默认档位 `slow`。每段均匀前进，没有逐渐放大步长或减速曲线。

| 档位 | 平移最大步长 | 姿态/每个关节最大角步长 |
|---|---:|---:|
| `slow` | 1 mm | 0.005 rad |
| `medium` | 2 mm | 0.010 rad |
| `fast` | 5 mm | 0.025 rad |

EEF 步数取 `ceil(平移欧氏距离 / 平移步长)` 与 `ceil(姿态最短旋转角 / 角步长)` 的最大值；关节步数取六轴 `ceil(绝对差 / 角步长)` 的最大值。最少一步。位置和关节线性等分，姿态最短弧 SLERP 等分后转回 ROS 的 XYZ RPY，每段包含精确最终目标。步数为整数，实际均匀步长可能小于配置的最大粒度。

双臂同一步在同一发布周期开始，各自按档位插值；先完成的一臂重复保持终点。最终目标发出后，默认要求连续两次新反馈满足位置 `5 mm`、姿态 `0.05 rad` 或关节 `0.02 rad` 容差，并检查夹爪容差。读取同一缓存反馈不能累加到位次数。夹爪直接设置目标值，不做轨迹插值。Python/ROS 调度不是实时控制，周期超限时不突发补发轨迹点。

到位检查逐条检查收到的反馈，不能跳过中间的超差样本。内部为每臂保留最近 256 条反馈；若执行器来不及读取导致历史溢出，则本次执行失败并清空待执行队列。每臂在自己的终点发布后建立计数边界，发布期间并发收到的反馈保守地不参与到位计数。

返回包含 `batch_id`、`accepted`、`status`、`total_steps`、`completed_steps`、`failed_steps`、`cancelled_steps`；等待调用还包含 `timed_out`。状态为 `queued`、`running`、`completed`、`failed` 或 `cancelled`。`state.recent_batches` 保留最近八批结果，`execution` 展示当前插值与到位进度。

整批语法或调用选项无效时不入队。`clear_queue=true` 只取消排队项，保留正在执行的步骤，再提交新批次。`wait` 等待自己的整批，不等待其他客户端之后提交的命令。等待超时返回当前进度，运动继续；此时可用 `state` 查看结果。单步骤执行的独立限制由 `step_timeout_s` 控制。

发布失败、驱动订阅者断开、反馈超过有效期或执行到位超时，均使当前执行失败并取消全部待执行步骤。服务停止继续发布并清除内部参考目标；之后新命令从反馈建立参考。驱动是否继续追踪最后目标取决于驱动本身，这不是硬件急停。

## ROS 状态与相机

控制 topic 为 `/arm_master_l_cmd`、`/arm_master_r_cmd`；反馈 topic 为 `/arm_slave_l_status`、`/arm_slave_r_status`。`RobotStatus` 需要 `end_pos[6]`、`joint_pos[7]`、`joint_vel[7]`、`joint_cur[7]`。

状态保持 deploydev 的原始布局，左臂在前：

| 字段 | 内容 |
|---|---|
| `qpos` | 左臂七维 `joint_pos` + 右臂七维 `joint_pos` |
| `qvel` | 左右臂各七维 `joint_vel` |
| `effort` | 左右臂各七维 `joint_cur`，单位保持驱动原样 |
| `eef` | 每臂 `[x,y,z,roll,pitch,yaw,raw_gripper]`，共 14 维 |

任一臂尚无反馈时，聚合字段为 `null`；过期反馈仍可读取，`state_ready` / `arms.*.ready` 为 `false`，控制工具会拒绝使用。每臂还包含实际状态、`commanded_target`、`target_error`、ROS `timestamp`、接收年龄 `age_s` 和 `seq`。ROS 原始时间戳保留，有效期用本机单调时钟接收时间计算，不受 ROS 仿真时钟或系统时间跳变影响。

夹爪反馈**不归一化**。控制内部如需从反馈换算为 `RobotCmd.gripper` 单位，可配置 `gripper_feedback_scale` 和 `gripper_feedback_offset`；默认恒等映射。真实驱动若反馈的是夹爪角度而命令使用 `0..1`，应先校准这两个参数，否则夹爪到位检测可能超时。

| 相机 | `CompressedImage` topic |
|---|---|
| `head` | `/camera/camera_h/color/image_rect_raw/compressed` |
| `left_wrist` | `/camera/camera_l/color/image_rect_raw/compressed` |
| `right_wrist` | `/camera/camera_r/color/image_rect_raw/compressed` |

按需解码为 RGB 再编码 JPEG，保持长宽比，默认最长宽/高不超过 `1024×768`。默认反馈有效期 `0.5 s`、图像 `1 s`。每个成功图像对应一个 MCP `ImageContent` 块，JSON 的 `images` 按同一顺序提供相机名、时间、年龄、序号和尺寸。`cameras_missing`、`cameras_stale`、`camera_errors` 分别报告缺失、过期或解码失败，不影响其他相机或状态返回。机械臂与各相机是各自最新缓存，不保证硬件时间同步。

## 观测自动落盘

`state` 和三个控制工具返回状态后，会自动保存本次读取到的新观测。保存位置固定为服务进程启动时的工作目录下 `logs/system-YYYYMMDD-HHMMSS-ffffff/`，首次需要保存时创建目录，每次运行使用独立目录。通过启动脚本运行时，工作目录为项目根目录。`props`、`/health` 和后台 ROS 订阅不会触发保存。

```text
logs/system-20261010-203000-123456/
  state.csv
  left-20261010-203005-123456.jpg
  head-20261010-203005-123456.jpg
  right-20261010-203005-123456.jpg
```

状态按反馈 `seq` 去重，追加写入 `state.csv`，表头只写一次。CSV 包含读取时间 `timestamp`、落盘时间 `logged_at`、左右臂反馈序号与 ROS 时间、`state_ready`，以及按现有左臂在前布局展开的 `qpos_0..13`、`qvel_0..13`、`effort_0..13`、`eef_0..13` 数值列。`state_json` 保存完整返回状态，包含每臂详细反馈、目标、误差、队列进度及本次图像元数据。任一臂缺失时，聚合数值列留空，完整状态中仍保留 `null` 和缺失标记。

图像按各相机帧的 `seq` 独立去重，仅保存本次请求实际返回的 JPEG 字节，编码质量与尺寸和客户端收到的图像一致。`left_wrist` / `head` / `right_wrist` 分别使用 `left` / `head` / `right` 文件名前缀，文件时间为落盘时间，精确到微秒。未请求、缺失、过期或解码失败的图像不写入。重复读取同一缓存不会重复写入，即使重新选择相机、改变 JPEG 质量或读取年龄发生变化；收到新消息时，即使数据或 ROS 时间戳相同，也会保存。

所有客户端和工具共享去重记录，写盘使用线程锁串行处理，并在工作线程中执行。写盘失败会输出服务端日志，并在原返回结果中增加 `observation_log_errors`；原状态、图像及控制执行结果继续返回。失败的状态或图像不会标记为已保存，下次读取会重试。

## 代码结构与验证

```text
src/ee_control_mcp/
  server.py          MCP 工具、HTTP 鉴权和生命周期
  commands.py        整批语法解析与绝对目标验证
  controller.py      公共串行队列、模式切换与到位检测
  interpolation.py   线性/SLERP 均匀轨迹
  backends.py        ROS 通信与无 ROS 模拟后端
  observations.py    原始状态/图像缓存
  recording.py       请求观测去重、CSV 追加与 JPEG 落盘
  config.py          YAML 默认值与校验
  runtime.py         后端、执行器与状态组合
  cli.py             启动入口
```

```bash
python -m pytest -q
ruff check src tests examples
ruff format --check src tests examples
```

自动测试只使用模拟后端或 ROS 消息边界替身，覆盖原始双臂 EEF 目标、`home`、三档步长/SLERP、双臂不同档位、模式切换、组合命令、队列和取消、无效整批拒绝、过期反馈、等待超时、状态维度、图像颜色与故障。HTTP 集成测试绑定本机临时端口，通过官方 `ClientSession` 验证初始化、工具发现、控制与图像，并验证 Token 和 Host 检查。

启动脚本测试在真实 Bash/Zsh 中使用替身 ROS 环境和消息包，验证自动选择 Shell、环境加载、带空格的路径、只读/控制切换、可选消息构建、错误处理及服务退出码，不连接真实 ROS 或发布机械臂命令。

真机验收单独进行：先用 `--no-actuation` 核对反馈与三个相机、确认夹爪单位和方向映射，再在可控工作空间使用 `slow` 的小位移核对实际到位和模式切换，最后验证上述原始 EEF 目标与关节归零。自动测试不会执行这些真机动作。
