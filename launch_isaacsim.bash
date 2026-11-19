#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
isaacsim_dir="${ISAACSIM_PATH:-$HOME/sim/IsaacSim-6.0.1}"
isaacsim_python="${ISAACSIM_PYTHON:-$isaacsim_dir/python.sh}"
entrypoint=drone_camera_lidar_isaac_sim.py
if [[ "${1:-}" == --follow-camera ]]; then
    entrypoint=drone_camera_lidar_isaac_sim_following.py
    shift
fi
if [[ ! -x "$isaacsim_python" ]]; then
    echo "Isaac Sim python.sh not found: $isaacsim_python. Set ISAACSIM_PATH or ISAACSIM_PYTHON." >&2
    exit 1
fi
if [[ -z "${ROS_DISTRO:-}" ]]; then
    # Source ROS in the launcher, not inside the simulator's Python process.
    if [[ -f /opt/ros/jazzy/setup.bash ]]; then
        set +u
        source /opt/ros/jazzy/setup.bash
        set -u
    else
        echo 'Source your ROS 2 environment before launching with ROS publishers.' >&2
    fi
fi
exec "$isaacsim_python" "$project_dir/$entrypoint" "$@"
