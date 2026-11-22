#!/usr/bin/env bash
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$project_dir"
config="$project_dir/config/world.yaml"
namespace_list=""
keyboard=false
rviz=false
rosbag=false
mocap=false
use_gnome=false
attach=true
while (($#)); do
    case "$1" in
        -m|--swarm) config="$project_dir/config/world_swarm.yaml"; shift ;;
        -w|--config) config="${2:?Missing config path}"; shift 2 ;;
        -n|--namespace) namespace_list="${2:?Missing drone names}"; shift 2 ;;
        -t) keyboard=true; shift ;;
        -v) rviz=true; shift ;;
        -r) rosbag=true; shift ;;
        --mocap) mocap=true; shift ;;
        -g) use_gnome=true; shift ;;
        --no-attach) attach=false; shift ;;
        -h|--help) echo 'Usage: launch_ground_station.bash [-m] [--config FILE] [-n drone0,drone1] [-t] [-v] [-r] [--mocap] [-g] [--no-attach]'; exit 0 ;;
        *) echo "Unknown argument: $1" >&2; exit 2 ;;
    esac
done
all_names="$(python3 scripts/world_config.py --config "$config" --names)"
namespace_list="${namespace_list:-$all_names}"
IFS=',' read -r -a names <<< "$namespace_list"
for name in "${names[@]}"; do
    case ",$all_names," in *",$name,"*) ;; *) echo "Drone '$name' is not in $config" >&2; exit 2 ;; esac
done
settings=("drone_namespace=$namespace_list" "keyboard_teleop=$keyboard" "rviz=$rviz" "rosbag=$rosbag" "mocap4ros2=$mocap")
if [[ "$use_gnome" == true ]]; then
    commands="$(mktemp /tmp/as2-ground-station.XXXXXX)"
    tmuxinator debug -n ground_station -p tmuxinator/ground_station.yaml "${settings[@]}" > "$commands"
    python3 utils/tmuxinator_to_genome.py -p "$commands"
    rm "$commands"
else
    tmuxinator start -n ground_station -p tmuxinator/ground_station.yaml "${settings[@]}"
    if [[ "$attach" == true ]]; then
        if [[ -n "${TMUX:-}" ]]; then tmux switch-client -t ground_station; else tmux attach-session -t ground_station; fi
    fi
fi
