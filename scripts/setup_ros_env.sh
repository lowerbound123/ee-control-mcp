#!/usr/bin/env bash
# 在当前 Bash 或 Zsh 中使用：source scripts/setup_ros_env.sh

if [[ -n "${ZSH_VERSION:-}" ]]; then
    if [[ "${ZSH_EVAL_CONTEXT:-}" != *:file ]]; then
        printf '请使用 source scripts/setup_ros_env.sh，让环境变量在当前终端中生效。\n' >&2
        exit 1
    fi
    _ee_control_mcp_ros_suffix=zsh
elif [[ -n "${BASH_VERSION:-}" ]]; then
    if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
        printf '请使用 source scripts/setup_ros_env.sh，让环境变量在当前终端中生效。\n' >&2
        exit 1
    fi
    _ee_control_mcp_ros_suffix=bash
else
    printf '当前 Shell 不受支持，请在 Bash 或 Zsh 中 source 此脚本。\n' >&2
    return 1 2>/dev/null || exit 1
fi

_ee_control_mcp_expected_distro=${EE_CONTROL_MCP_ROS_DISTRO:-humble}
_ee_control_mcp_ros_dir=${EE_CONTROL_MCP_ROS_DIR:-/opt/ros/$_ee_control_mcp_expected_distro}
if [[ -z "${EE_CONTROL_MCP_ROS_DIR:-}" \
      && ! -r "$_ee_control_mcp_ros_dir/setup.$_ee_control_mcp_ros_suffix" \
      && -r "/opt/ros/${_ee_control_mcp_expected_distro}_origin/setup.$_ee_control_mcp_ros_suffix" ]]; then
    _ee_control_mcp_ros_dir="/opt/ros/${_ee_control_mcp_expected_distro}_origin"
fi
_ee_control_mcp_ros_setup="$_ee_control_mcp_ros_dir/setup.$_ee_control_mcp_ros_suffix"
unset _ee_control_mcp_ros_dir _ee_control_mcp_ros_suffix
if [[ ! -r "$_ee_control_mcp_ros_setup" ]]; then
    printf '无法读取 %s；需要与机器人一致的 ROS %s，不会回退到 Jazzy。\n' \
        "$_ee_control_mcp_ros_setup" "$_ee_control_mcp_expected_distro" >&2
    unset _ee_control_mcp_ros_setup _ee_control_mcp_expected_distro
    return 1
fi

if ! source "$_ee_control_mcp_ros_setup"; then
    unset _ee_control_mcp_ros_setup _ee_control_mcp_expected_distro
    return 1
fi
if [[ "${ROS_DISTRO:-}" != "$_ee_control_mcp_expected_distro" ]]; then
    printf 'ROS 版本不匹配：机器人要求 %s，当前环境为 %s。\n' \
        "$_ee_control_mcp_expected_distro" "${ROS_DISTRO:-<unset>}" >&2
    unset _ee_control_mcp_ros_setup _ee_control_mcp_expected_distro
    return 1
fi

export ROS_DOMAIN_ID=60
export ROS_LOCALHOST_ONLY=0

printf 'ROS %s 已加载（%s）：ROS_DOMAIN_ID=%s，ROS_LOCALHOST_ONLY=%s\n' \
    "$ROS_DISTRO" "$_ee_control_mcp_ros_setup" "$ROS_DOMAIN_ID" "$ROS_LOCALHOST_ONLY"
unset _ee_control_mcp_ros_setup _ee_control_mcp_expected_distro
