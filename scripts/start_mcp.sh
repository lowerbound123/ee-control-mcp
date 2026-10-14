#!/usr/bin/env bash
# 直接执行此脚本；ROS/虚拟环境只配置给服务进程，Ctrl+C 后返回原终端。

if [[ -n "${ZSH_VERSION:-}" ]]; then
    if [[ "${ZSH_EVAL_CONTEXT:-}" == *:file ]]; then
        printf '请直接执行 scripts/start_mcp.sh，不要 source 启动脚本。\n' >&2
        return 2
    fi
    mcp_start_script=${(%):-%x}
    mcp_shell=zsh
elif [[ -n "${BASH_VERSION:-}" ]]; then
    if [[ "${BASH_SOURCE[0]}" != "$0" ]]; then
        printf '请直接执行 scripts/start_mcp.sh，不要 source 启动脚本。\n' >&2
        return 2
    fi
    mcp_start_script=${BASH_SOURCE[0]}
    # 可执行文件的 shebang 先进入 Bash；从 Zsh 终端调用时切回 Zsh。
    mcp_parent_shell=$(ps -p "$PPID" -o comm= 2>/dev/null)
    case "${mcp_parent_shell#-}" in
        zsh) exec zsh -f "$mcp_start_script" "$@" ;;
    esac
    mcp_shell=bash
else
    printf '启动脚本需要 Bash 或 Zsh。\n' >&2
    exit 1
fi

mcp_project_root=$(cd -- "$(dirname -- "$mcp_start_script")/.." && pwd -P) || exit 1
mcp_ros_dir=
mcp_workspace="$mcp_project_root/../acone_control_debug/code/x5_ros2"
mcp_venv="$mcp_project_root/.venv"
mcp_venv_requested=0
mcp_conda_env=
mcp_conda_exe=${CONDA_EXE:-}
mcp_conda_exe_requested=0
mcp_config="$mcp_project_root/config/default.yaml"
mcp_mode=readonly
mcp_requested_mode=
mcp_build_messages=0
mcp_forward=()

mcp_usage() {
    cat <<'USAGE'
用法：scripts/start_mcp.sh [选项]
  --readonly           只读取真实 ROS 状态与相机（默认）
  --control            启用真实 ROS 控制发布器；启动不发送动作
  --workspace DIR      arx5_arm_msg 工作空间根目录
  --build-messages     启动前在工作空间中编译 arx5_arm_msg
  --ros-dir DIR        ROS 安装目录（默认匹配 ros.expected_distro）
  --venv DIR           Python 虚拟环境（默认 项目/.venv）
  --conda-env ENV       独立 Conda 环境名或绝对路径（不能使用 base）
  --conda-exe FILE      Conda 可执行文件（可自动发现，须配合 --conda-env）
  --config FILE        MCP YAML（默认 项目/config/default.yaml）
  --host ADDRESS       HTTP 监听地址
  --port PORT          HTTP 监听端口
  --log-level LEVEL    debug/info/warning/error
  --check-config       加载环境并检查配置，然后退出
  -h, --help           显示帮助
USAGE
}

mcp_require_value() {
    if [[ $# -lt 2 || -z "${2:-}" || "$2" == --* ]]; then
        printf '%s 需要一个参数值。\n' "$1" >&2
        exit 2
    fi
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --readonly|--control)
            if [[ -n "$mcp_requested_mode" && "$mcp_requested_mode" != "$1" ]]; then
                printf '%s\n' '--readonly 与 --control 不能同时使用。' >&2
                exit 2
            fi
            mcp_requested_mode=$1
            mcp_mode=${1#--}
            shift
            ;;
        --workspace|--ros-dir|--venv|--conda-env|--conda-exe|--config)
            mcp_require_value "$@"
            case "$1" in
                --workspace) mcp_workspace=$2 ;;
                --ros-dir) mcp_ros_dir=$2 ;;
                --venv) mcp_venv=$2; mcp_venv_requested=1 ;;
                --conda-env) mcp_conda_env=$2 ;;
                --conda-exe) mcp_conda_exe=$2; mcp_conda_exe_requested=1 ;;
                --config) mcp_config=$2 ;;
            esac
            shift 2
            ;;
        --host|--port|--log-level)
            mcp_require_value "$@"
            mcp_forward+=("$1" "$2")
            shift 2
            ;;
        --build-messages) mcp_build_messages=1; shift ;;
        --check-config) mcp_forward+=("$1"); shift ;;
        -h|--help) mcp_usage; exit 0 ;;
        *) printf '未知选项：%s\n' "$1" >&2; mcp_usage >&2; exit 2 ;;
    esac
done

# 先解析所有选项和路径，避免在无效调用中加载环境或编译包。
if [[ -n "$mcp_conda_env" && "$mcp_venv_requested" == 1 ]]; then
    printf '%s\n' '--conda-env 与 --venv 不能同时使用。' >&2
    exit 2
fi
if [[ "$mcp_conda_exe_requested" == 1 && -z "$mcp_conda_env" ]]; then
    printf '%s\n' '--conda-exe 需要配合 --conda-env。' >&2
    exit 2
fi
if [[ ! -d "$mcp_workspace" ]]; then
    printf '消息工作空间不存在：%s；请使用 --workspace 指定。\n' "$mcp_workspace" >&2
    exit 1
fi
mcp_workspace=$(cd -- "$mcp_workspace" && pwd -P) || exit 1
if [[ "$mcp_config" != /* ]]; then
    mcp_config="$PWD/$mcp_config"
fi
if [[ ! -r "$mcp_config" ]]; then
    printf '无法读取配置：%s\n' "$mcp_config" >&2
    exit 1
fi
if [[ -n "$mcp_conda_env" ]]; then
    # 调用者的 ROS/Conda 配置可能把其他 Python 版本的包加入搜索路径。
    # 先隔离 Conda 自身及所选环境；ROS 路径随后由指定 setup 重新建立。
    unset PYTHONPATH PYTHONHOME
    if [[ -z "$mcp_conda_exe" ]]; then
        mcp_conda_exe=$(command -v conda 2>/dev/null)
    fi
    if [[ ! -x "$mcp_conda_exe" ]]; then
        printf '未找到 Conda 可执行文件；请用 --conda-exe 指定。\n' >&2
        exit 1
    fi
    mcp_conda_base=$("$mcp_conda_exe" info --base) || exit 1
    mcp_conda_base=$(cd -- "$mcp_conda_base" && pwd -P) || exit 1
    if [[ -d "$mcp_conda_env" ]]; then
        mcp_conda_env=$(cd -- "$mcp_conda_env" && pwd -P) || exit 1
    fi
    if [[ "$mcp_conda_env" == base || "$mcp_conda_env" == "$mcp_conda_base" ]]; then
        printf '不能使用 Conda base；请选择独立环境。\n' >&2
        exit 2
    fi
    # Conda 先激活，再加载 ROS，避免 Conda 的 PATH 调整覆盖 ROS 环境。
    if [[ ! -r "$mcp_conda_base/etc/profile.d/conda.sh" ]] || \
            ! source "$mcp_conda_base/etc/profile.d/conda.sh" || \
            ! conda activate "$mcp_conda_env"; then
        printf '无法激活 Conda 环境：%s\n' "$mcp_conda_env" >&2
        exit 1
    fi
    mcp_conda_prefix=$(cd -- "${CONDA_PREFIX:-}" && pwd -P) || exit 1
    if [[ "$mcp_conda_prefix" == "$mcp_conda_base" ]]; then
        printf '不能使用 Conda base；请选择独立环境。\n' >&2
        exit 2
    fi
    unset VIRTUAL_ENV
    mcp_python="$mcp_conda_prefix/bin/python"
    mcp_environment="Conda:$mcp_conda_prefix"
else
    if [[ ! -r "$mcp_venv/bin/activate" || ! -x "$mcp_venv/bin/python" ]]; then
        printf '虚拟环境不完整：%s；请先创建并安装项目依赖，或使用 --venv/--conda-env。\n' \
            "$mcp_venv" >&2
        exit 1
    fi
    mcp_venv=$(cd -- "$mcp_venv" && pwd -P) || exit 1
    mcp_python="$mcp_venv/bin/python"
    mcp_environment="venv:$mcp_venv"
fi
if [[ ! -x "$mcp_python" ]]; then
    printf '环境中没有可执行的 Python：%s\n' "$mcp_python" >&2
    exit 1
fi
mcp_expected_distro=$("$mcp_python" - "$mcp_config" <<'PY'
import sys
from ee_control_mcp.config import load_config

try:
    print(load_config(sys.argv[1]).ros.expected_distro)
except (ValueError, OSError) as exc:
    print(f"配置检查失败：{exc}", file=sys.stderr)
    sys.exit(1)
PY
) || exit 1
if [[ -z "$mcp_ros_dir" ]]; then
    for mcp_candidate in "/opt/ros/$mcp_expected_distro" "/opt/ros/${mcp_expected_distro}_origin"; do
        if [[ -r "$mcp_candidate/setup.$mcp_shell" ]]; then
            mcp_ros_dir=$mcp_candidate
            break
        fi
    done
    if [[ -z "$mcp_ros_dir" ]]; then
        printf '未找到与机器人匹配的 ROS %s；不会回退到其他版本。\n' "$mcp_expected_distro" >&2
        printf '请在匹配的 ROS/Python 环境运行，或用 --ros-dir 指定安装目录。MCP 未连接 ROS。\n' >&2
        exit 1
    fi
fi
mcp_verify_distro() {
    if [[ "${ROS_DISTRO:-}" != "$mcp_expected_distro" ]]; then
        printf 'ROS 版本不匹配：机器人要求 %s，当前环境为 %s；MCP 未连接 ROS。\n' \
            "$mcp_expected_distro" "${ROS_DISTRO:-<unset>}" >&2
        printf '请使用匹配的 ROS、Python 和消息工作空间，不要混用 Jazzy/Humble 构建结果。\n' >&2
        exit 1
    fi
}
mcp_ros_setup="$mcp_ros_dir/setup.$mcp_shell"
mcp_install_base="install-mcp-$mcp_expected_distro"
mcp_workspace_setup="$mcp_workspace/$mcp_install_base/setup.$mcp_shell"
if [[ "$mcp_build_messages" == 0 && ! -r "$mcp_workspace_setup" ]]; then
    # An existing matching robot workspace may already have a normal install.
    # Its chained ROS version is checked again immediately after sourcing.
    mcp_workspace_setup="$mcp_workspace/install/setup.$mcp_shell"
fi
if [[ ! -r "$mcp_ros_setup" ]]; then
    printf '无法读取 ROS 环境：%s\n' "$mcp_ros_setup" >&2
    exit 1
fi
if [[ "$mcp_build_messages" == 0 && ! -r "$mcp_workspace_setup" ]]; then
    printf '工作空间尚未编译：%s；可增加 --build-messages 首次编译。\n' \
        "$mcp_workspace_setup" >&2
    exit 1
fi

if ! source "$mcp_ros_setup"; then
    printf '加载 ROS 环境失败：%s\n' "$mcp_ros_setup" >&2
    exit 1
fi
if [[ -z "$mcp_conda_env" ]] && ! source "$mcp_venv/bin/activate"; then
    printf '激活虚拟环境失败：%s\n' "$mcp_venv" >&2
    exit 1
fi
mcp_verify_distro

if [[ "$mcp_build_messages" == 1 ]]; then
    if ! command -v colcon >/dev/null 2>&1; then
        printf '未找到 colcon，请先安装 ROS 构建工具。\n' >&2
        exit 1
    fi
    printf '编译消息包 arx5_arm_msg：%s\n' "$mcp_workspace"
    if ! (cd -- "$mcp_workspace" && colcon --log-base "log-mcp-$mcp_expected_distro" \
            build --build-base "build-mcp-$mcp_expected_distro" \
            --install-base "$mcp_install_base" --packages-select arx5_arm_msg \
            --cmake-args "-DPython3_EXECUTABLE=$mcp_python"); then
        printf '消息包编译失败，MCP 未启动。\n' >&2
        exit 1
    fi
fi

if [[ ! -r "$mcp_workspace_setup" ]] || ! source "$mcp_workspace_setup"; then
    printf '加载消息工作空间失败：%s\n' "$mcp_workspace_setup" >&2
    exit 1
fi
mcp_verify_distro
# 在加载工作空间后设置，避免 overlay 覆盖通信参数。
export ROS_DOMAIN_ID=60
export ROS_LOCALHOST_ONLY=0

if ! "$mcp_python" - "$mcp_mode" <<'PY'
import sys

try:
    import rclpy
    from arx5_arm_msg.msg import RobotStatus
    from sensor_msgs.msg import CompressedImage
    if sys.argv[1] == "control":
        from arx5_arm_msg.msg import RobotCmd
    import ee_control_mcp.server
except Exception as exc:
    print(f"Python/ROS 依赖检查失败（{sys.executable}）：{exc}", file=sys.stderr)
    sys.exit(1)
PY
then
    printf '请确认消息包、ROS 与虚拟环境的 Python 版本匹配，并已安装 MCP 项目。\n' >&2
    exit 1
fi

mcp_args=(--backend ros --config "$mcp_config" "${mcp_forward[@]}")
if [[ "$mcp_mode" == readonly ]]; then
    mcp_args+=(--no-actuation)
fi
printf '启动 MCP：Shell=%s，模式=%s，ROS_DISTRO=%s，ROS_DOMAIN_ID=%s，工作空间=%s，Python=%s\n' \
    "$mcp_shell" "$mcp_mode" "$ROS_DISTRO" "$ROS_DOMAIN_ID" "$mcp_workspace" "$mcp_environment"
cd -- "$mcp_project_root" || exit 1
exec "$mcp_python" -m ee_control_mcp "${mcp_args[@]}"
