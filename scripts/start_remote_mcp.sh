#!/usr/bin/env bash
# pjlab@10.6.3.69 部署入口；由通用脚本选择 Bash/Zsh，默认只读。
mcp_remote_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P) || exit 1
export CONDA_NO_PLUGINS=true
exec "$mcp_remote_root/scripts/start_mcp.sh" \
    --config "$mcp_remote_root/config/robotics52.yaml" \
    --conda-env "$mcp_remote_root/.conda" \
    --conda-exe /home/pjlab/miniconda3/bin/conda \
    --ros-dir /opt/ros/humble_origin \
    --workspace /home/arx/LIFT/ARX_X5/ROS2/X5_ws \
    "$@"
