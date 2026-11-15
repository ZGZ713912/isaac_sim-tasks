#!/usr/bin/env bash
# Runs the configured local PipeFeedback graph through the normal executor.
set -euo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
task_rmcs="${task_root%/*}/RMCS"
task_config="rmcs_ws/src/rmcs_rl/config/simulation/wheel-leg-pipe.yaml"
task_container="${RMCS_SIM_CONTAINER:-rmcs_devcontainer-rmcs-develop-1}"
while (($#)); do
    case "$1" in
        --rmcs-root) task_rmcs="$2"; shift 2 ;;
        --config) task_config="$2"; shift 2 ;;
        *) echo "Unknown pipe argument: $1" >&2; exit 2 ;;
    esac
done
task_rmcs="$(realpath -- "$task_rmcs")"
if [[ "$task_config" != /* ]]; then task_config="$task_rmcs/$task_config"; fi
task_relative="$(realpath --relative-to="$task_rmcs" -- "$task_config")"
if [[ "$task_relative" == ../* ]]; then
    echo "Pipe configuration must be inside the RMCS workspace." >&2
    exit 2
fi
export RCUTILS_LOGGING_USE_STDOUT=0
if [[ -f /opt/ros/jazzy/setup.bash && -f "$task_rmcs/rmcs_ws/install/setup.bash" ]]; then
    set +u
    source /opt/ros/jazzy/setup.bash
    source "$task_rmcs/rmcs_ws/install/setup.bash"
    set -u
    exec ros2 run rmcs_executor rmcs_executor --ros-args --params-file "$task_config"
fi
task_destination="$(docker inspect "$task_container" --format '{{range .Mounts}}{{println .Source .Destination}}{{end}}' | awk -v src="$task_rmcs" '$1==src {print $2; exit}')"
if [[ -z "$task_destination" ]]; then
    echo "No ROS workspace/bind mount for $task_rmcs; set RMCS_SIM_CONTAINER if needed." >&2
    exit 1
fi
exec docker exec -i -e RCUTILS_LOGGING_USE_STDOUT=0 "$task_container" bash -c \
    'source /opt/ros/jazzy/setup.bash && source "$1/rmcs_ws/install/setup.bash" && exec ros2 run rmcs_executor rmcs_executor --ros-args --params-file "$1/$2"' \
    simulation-pipe "$task_destination" "$task_relative"
