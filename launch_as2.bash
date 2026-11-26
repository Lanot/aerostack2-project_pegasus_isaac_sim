#!/usr/bin/env bash
set -euo pipefail

usage() {
    echo "  options:"
    echo "      --follow-camera / --no-follow-camera: override world camera setting"
    echo "      --follow-drone NAME: override world camera target"
    echo "      -m, --swarm: use config/world_swarm.yaml. Default: config/world.yaml"
    echo "      -w, --config FILE: use a custom world description file"
    echo "      -n, --namespace NAMES: comma-separated drone names. Default: all configured drones"
    echo "      --video-relay: start ROS UDP relays for configured cameras (default: false)"
    echo "      --video-relay-host IP: QGC destination (default: 127.0.0.1)"
    echo "      --headless: launch Isaac Sim without its GUI"
    echo "      --px4-env-file FILE: load PX4 tuning environment in the simulator pane"
    echo "      --physics-device cpu|cuda, --physics-hz HZ, --physics-threads N: override simulation YAML"
    echo "      --profile / --no-profile, --render-quality current|fast, --direct-tensors / --no-direct-tensors"
    echo "      -s, --no-simulation: do not launch Isaac Sim"
    echo "      -x: launch one Micro XRCE-DDS Agent for all drones"
    echo "      -c CONTROLLER: select pid or df. Default: pid"
    echo "      -r, --record: record rosbag"
    echo "      -g: use gnome-terminal instead of tmux"
    echo "      --no-attach: stream simulator logs here instead of attaching to tmux"
    echo "      --no-log-follow: return to the shell after launching (logs are still saved)"
    echo "      -h, --help: show this help"
}

# Resolve paths relative to this project.
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$project_dir"

# Initialize variables with default values.
swarm="false"
simulation_config=""
drones_namespace_comma=""
launch_simulation="true"
headless="false"
video_relay="false"
video_relay_host="127.0.0.1"
micro_xrce_agent="false"
motion_controller="pid"
rosbag="false"
use_gnome="false"
attach_session="true"
follow_logs="true"
follow_camera=""
follow_drone=""
simulation_overrides=()
px4_env_file=""

# Parse command-line arguments.
while (($#)); do
    case "$1" in
        -m|--swarm)
            swarm="true"
            simulation_config=""
            shift
            ;;
        -w|--config)
            simulation_config="${2:?Missing config path}"
            shift 2
            ;;
        -n|--namespace)
            drones_namespace_comma="${2:?Missing drone names}"
            shift 2
            ;;
        --video-relay)
            video_relay="true"
            shift
            ;;
        --video-relay-host)
            video_relay_host="${2:?Missing QGC host}"
            shift 2
            ;;
        --px4-env-file|--px4_env_file)
            px4_env_file="${2:?Missing PX4 environment file}"
            shift 2
            ;;
        --px4-env-file=*|--px4_env_file=*)
            px4_env_file="${1#*=}"
            if [[ -z "$px4_env_file" ]]; then
                echo "Missing PX4 environment file" >&2
                exit 2
            fi
            shift
            ;;
        --headless)
            headless="true"
            shift
            ;;
        --physics-device|--physics_device|--physics-hz|--physics_hz|--physics-threads|--physics_threads|--render-quality|--render_quality)
            simulation_overrides+=("$1" "${2:?Missing simulation option value}")
            shift 2
            ;;
        --physics-device=*|--physics_device=*|--physics-hz=*|--physics_hz=*|--physics-threads=*|--physics_threads=*|--render-quality=*|--render_quality=*)
            simulation_overrides+=("$1")
            shift
            ;;
        --profile|--no-profile|--direct-tensors|--direct_tensors|--no-direct-tensors|--no-direct_tensors)
            simulation_overrides+=("$1")
            shift
            ;;
        -s|--no-simulation)
            launch_simulation="false"
            shift
            ;;
        -x)
            micro_xrce_agent="true"
            shift
            ;;
        -c)
            motion_controller="${2:?Missing controller}"
            shift 2
            ;;
        -r|--record)
            rosbag="true"
            shift
            ;;
        -g)
            use_gnome="true"
            shift
            ;;
        --follow-camera)
            follow_camera="true"
            shift
            ;;
        --no-follow-camera)
            follow_camera="false"
            shift
            ;;
        --follow-drone)
            follow_drone="${2:?Missing camera target}"
            shift 2
            ;;
        --no-log-follow)
            follow_logs="false"
            shift
            ;;
        --no-attach)
            attach_session="false"
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown argument: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

# Validate before launching processes. Source inside the simulator pane so an
# existing tmux server's environment cannot discard PX4 parameter overrides.
if [[ -n "$px4_env_file" ]]; then
    if [[ "$launch_simulation" != "true" ]]; then
        echo "--px4-env-file requires launching the simulator; omit -s/--no-simulation." >&2
        exit 2
    fi
    if [[ ! -f "$px4_env_file" || ! -r "$px4_env_file" ]]; then
        echo "PX4 environment file is not a readable file: $px4_env_file" >&2
        exit 2
    fi
    px4_env_file="$(realpath -- "$px4_env_file")"
    bash -n -- "$px4_env_file"
    echo "PX4 environment file: $px4_env_file"
fi

# Set the simulation world description config file.
if [[ -z "$simulation_config" ]]; then
    if [[ "$swarm" == "true" ]]; then
        simulation_config="config/world_swarm.yaml"
    else
        simulation_config="config/world.yaml"
    fi
fi
simulation_config="$(realpath "$simulation_config")"
run_dir="$(mktemp -d /tmp/as2-pegasus-launch.XXXXXX)"
if ((${#simulation_overrides[@]})); then
    python3 scripts/world_config.py --config "$simulation_config" \
        "${simulation_overrides[@]}" --effective-world --output "$run_dir/world.yaml"
    simulation_config="$run_dir/world.yaml"
fi

# Get configured drone names unless a subset was explicitly selected.
configured_namespaces=$(python3 scripts/world_config.py --config "$simulation_config" --names)
drones_namespace_comma="${drones_namespace_comma:-$configured_namespaces}"
IFS=',' read -r -a drone_namespaces <<< "$drones_namespace_comma"

# Select the motion-controller plugin.
case "$motion_controller" in
    pid)
        motion_controller_plugin="pid_speed_controller"
        ;;
    df)
        motion_controller_plugin="differential_flatness_controller"
        ;;
    *)
        echo "Invalid controller: $motion_controller. Use -c pid or -c df." >&2
        exit 2
        ;;
esac

# ROS relays run through the package installed in this sourced workspace.
if [[ "$video_relay" == "true" ]]; then
    if ! command -v ros2 >/dev/null || ! ros2 pkg prefix project_pegasus_isaac_sim >/dev/null 2>&1; then
        echo "[ERROR] [video-relay] Build project_pegasus_isaac_sim with colcon and source install/setup.bash first." >&2
        exit 2
    fi
fi

# Generate each selected drone's PX4 routing before launching any process.
simulation_log="$run_dir/isaacsim.log"
if [[ "$launch_simulation" == "true" ]]; then
    touch "$simulation_log"
    echo "Isaac Sim / Pegasus log: $simulation_log"
fi
for namespace in "${drone_namespaces[@]}"; do
    case ",$configured_namespaces," in
        *",$namespace,"*)
            ;;
        *)
            echo "Drone '$namespace' is not in $simulation_config" >&2
            exit 2
            ;;
    esac

    python3 scripts/world_config.py \
        --config "$simulation_config" \
        --platform-config "$namespace" \
        --base-config config/config.yaml \
        --output "$run_dir/$namespace.yaml"

    if [[ "$video_relay" == "true" ]]; then
        python3 scripts/video_relay_config.py --config "$simulation_config" \
            --drone "$namespace" --host "$video_relay_host" \
            --output "$run_dir/$namespace.video.json"
    fi
done

# Select between tmux and gnome-terminal.
tmuxinator_mode="start"
if [[ "$use_gnome" == "true" ]]; then
    tmuxinator_mode="debug"
elif [[ -n "${TMUX:-}" && "$attach_session" == "true" ]]; then
    echo "Already inside tmux: detach first, or use --no-attach." >&2
    exit 2
fi

# Launch Aerostack2 for each drone namespace.
for index in "${!drone_namespaces[@]}"; do
    namespace="${drone_namespaces[$index]}"
    base_launch="false"
    launch_agent="false"
    if [[ "$index" == "0" ]]; then
        base_launch="$launch_simulation"
        launch_agent="$micro_xrce_agent"
    fi

    launch_settings=(
        "drone_namespace=$namespace"
        "simulation_config_file=$simulation_config"
        "simulation_log_file=$simulation_log"
        "px4_env_file=$px4_env_file"
        "platform_config_file=$run_dir/$namespace.yaml"
        "base_launch=$base_launch"
        "headless=$headless"
        "video_relay=$video_relay"
        "video_relay_config_file=$run_dir/$namespace.video.json"
        "micro_xrce_agent=$launch_agent"
        "motion_controller_plugin=$motion_controller_plugin"
        "rosbag=$rosbag"
        "follow_camera=$follow_camera"
        "follow_drone=$follow_drone"
    )

    if [[ "$use_gnome" == "true" ]]; then
        commands_file="$run_dir/$namespace.commands"
        tmuxinator "$tmuxinator_mode" -n "$namespace" -p tmuxinator/aerostack2.yaml \
            "${launch_settings[@]}" > "$commands_file"
        python3 utils/tmuxinator_to_genome.py -p "$commands_file"
    else
        tmuxinator "$tmuxinator_mode" -n "$namespace" -p tmuxinator/aerostack2.yaml \
            "${launch_settings[@]}"
    fi
done

# Attach to the first drone's tmux session.
if [[ "$attach_session" == "true" && "$use_gnome" == "false" ]]; then
    tmux attach-session -t "${drone_namespaces[0]}"
fi

# The simulator stays in tmux; this terminal only reads its combined output.
if [[ "$attach_session" == "false" && "$use_gnome" == "false" &&
      "$launch_simulation" == "true" && "$follow_logs" == "true" ]]; then
    echo "Streaming Isaac Sim / Pegasus logs. Ctrl+C stops viewing; tmux sessions keep running."
    exec tail -n +1 -f "$simulation_log"
fi
