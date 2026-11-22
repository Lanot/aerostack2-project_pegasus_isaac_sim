#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
isaacsim_dir="${ISAACSIM_PATH:-$HOME/sim/IsaacSim-6.0.1}"
isaacsim_python="${ISAACSIM_PYTHON:-$isaacsim_dir/python.sh}"
entrypoint="scripts/isaac_sim_run.py"

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

# This is a startup reminder, not a live DDS connectivity check.
cat >&2 <<'AS2_NOTICE'
[AS2 reminder] Autonomous takeoff requires a running Micro XRCE-DDS Agent and live PX4 odometry.
[AS2 reminder] Start the agent once: MicroXRCEAgent udp4 -p 8888 (or launch_as2.bash -x).
[AS2 reminder] AS2 arm/offboard service success does not confirm PX4 accepted the commands.
[AS2 reminder] If takeoff waits, check PX4 DDS odometry, AS2 sensor_measurements/odom, and the odom TF.
[AS2 reminder] Match the PX4 DDS namespace and AS2 fmu_prefix (configured model_name).
AS2_NOTICE

exec "$isaacsim_python" "$project_dir/$entrypoint" "$@"

