#!/usr/bin/env bash
# Run a ROS master pane and three independent ROS nodes in tmux.

set -eo pipefail

SESSION=isaac_datacollection_sim
WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
SCRIPT="$WORKSPACE/isaac_datacollection_start.sh"

error() {
    printf 'ERROR: %s\n' "$*" >&2
    return 1
}

master_responds() {
    timeout 2s rosparam list >/dev/null 2>&1
}

run_core() {
    if master_responds; then
        printf 'Using existing ROS master at %s.\n' "$ROS_MASTER_URI"
        touch "$log_dir/core-ready"
        return
    fi
    local uri="${ROS_MASTER_URI%/}"
    roscore -p "${uri##*:}" &
    local core_pid=$! deadline=$((SECONDS + startup_timeout))
    trap 'kill -TERM "$core_pid" 2>/dev/null || true; wait "$core_pid" 2>/dev/null || true; return 130' INT TERM
    until master_responds; do
        if ! kill -0 "$core_pid" 2>/dev/null || (( SECONDS >= deadline )); then
            kill -TERM "$core_pid" 2>/dev/null || true
            wait "$core_pid" || true
            error "ROS master did not start at $ROS_MASTER_URI."
            return 1
        fi
        sleep 0.25
    done
    if ! kill -0 "$core_pid" 2>/dev/null; then
        error "ROS master exited before it became ready."
        return 1
    fi
    touch "$log_dir/core-ready"
    wait "$core_pid"
}

run_node() {
    local deadline=$((SECONDS + startup_timeout))
    until [[ -e "$log_dir/core-ready" ]] && master_responds; do
        if [[ -e "$log_dir/core-exit" ]] || (( SECONDS >= deadline )); then
            error "ROS master did not become ready at $ROS_MASTER_URI."
            return 1
        fi
        sleep 0.25
    done
    local arms=false grippers=false recording=false
    case "$role" in
        arms) arms=true ;;
        grippers) grippers=true ;;
        recording) recording=true ;;
    esac
    # The reference gives the arm process a head start on the shared HID device.
    if [[ "$role" != arms ]]; then
        sleep 2
    fi
    export ROSCONSOLE_FORMAT='${message}'
    roslaunch --wait exosuit_teleop teleop.launch \
        "start_arms:=$arms" "start_grippers:=$grippers" "start_recording:=$recording" "$@"
}

pane_command() {
    if [[ "$role" == core ]]; then
        run_core
    else
        run_node "$@"
    fi
}

if [[ "${1:-}" == --pane ]]; then
    role=$2 log_dir=$3 startup_timeout=$4
    shift 4
    trap 'printf "%s\n" "$?" > "$log_dir/$role-exit"' EXIT
    export PYTHONUNBUFFERED=1
    pane_command "$@" 2>&1 | tee -ia "$log_dir/$role.log"
    exit $?
fi

detach=false
launch_args=()
for arg in "$@"; do
    case "$arg" in
        --detach) detach=true ;;
        --help)
            cat <<'USAGE'
Usage: ./isaac_datacollection_start.sh [--detach] [name:=value ...]
Launch arguments: trajectories_file, transition_duration_s, left_serial, right_serial.
Defaults: ROS Master http://192.168.1.112:11311,
          simulation_data_collection/devel/setup.bash, 30 s startup timeout.
ROS_MASTER_URI, ROS_HOSTNAME and ROS_IP are honored when set.
Overrides: ISAAC_ROS_MASTER_URI, ISAAC_ROS_IP, ISAAC_CATKIN_SETUP,
           ISAAC_ROS_START_TIMEOUT, ISAAC_LOG_ROOT (default ~/.ros/exosuit-isaac).
USAGE
            exit 0
            ;;
        trajectories_file:=*|transition_duration_s:=*|left_serial:=*|right_serial:=*)
            launch_args+=("$arg") ;;
        *) error "Unknown argument '$arg'. Use --help." ;;
    esac
done

command -v tmux >/dev/null || error "tmux is required."
if tmux has-session -t "=$SESSION" 2>/dev/null; then
    printf "Using existing tmux session '%s'.\n" "$SESSION"
else
    setup="${ISAAC_CATKIN_SETUP:-$WORKSPACE/devel/setup.bash}"
    [[ "$setup" == /* ]] || setup="$PWD/$setup"
    [[ -r "$setup" ]] || error "Catkin setup is unreadable at $setup. Build the workspace or set ISAAC_CATKIN_SETUP."
    master_uri="http://192.168.1.112:11311"
    startup_timeout="${ISAAC_ROS_START_TIMEOUT:-30}"
    if [[ ! "$master_uri" =~ ^http://[^/]+:([1-9][0-9]{0,4})/?$ ]] ||
       (( BASH_REMATCH[1] > 65535 )); then
        error "Use an HTTP ROS master URI with an explicit port."
    fi
    ros_address=("ROS_IP=192.168.1.112")
    if [[ ! "$startup_timeout" =~ ^[1-9][0-9]{0,3}$ ]]; then
        error "ISAAC_ROS_START_TIMEOUT must be a positive integer (seconds, up to 9999)."
    fi

    log_root="${ISAAC_LOG_ROOT:-$HOME/.ros/exosuit-isaac}"
    mkdir -p -- "$log_root"
    log_root="$(cd -- "$log_root" && pwd -P)"
    log_dir="$(mktemp -d "$log_root/run-$(date +%Y%m%d-%H%M%S).XXXXXX")"

    # Each pane owns a normal Bash; send the startup commands as in the reference.
    if ! window="$(tmux new-session -d -x 120 -y 40 -P -F '#{window_id}' \
                       -s "$SESSION" -n ros_exosuit /bin/bash -i)"; then
        tmux has-session -t "=$SESSION" 2>/dev/null || exit 1
    else
        trap 'tmux kill-session -t "=$SESSION" 2>/dev/null || true' EXIT
        tmux set-option -w -t "$window" pane-base-index 0
        tmux split-window -dh -t "$window" /bin/bash -i
        tmux split-window -dv -t "$window.0" /bin/bash -i
        tmux split-window -dv -t "$window.1" /bin/bash -i
        tmux select-layout -t "$window" tiled >/dev/null
        tmux set-option -w -t "$window" pane-border-status top
        tmux set-option -w -t "$window" pane-border-format \
            '#{pane_index} #{?#{==:#{pane_index},0},ROS Core,#{?#{==:#{pane_index},1},Arms,#{?#{==:#{pane_index},2},Grippers,Recording}}}'
        tmux select-pane -t "$window.0"

        start_pane() {
            local pane=$1 role=$2 command prefix
            shift 2
            # Source the overlay in the interactive Bash so it remains usable at the prompt.
            printf -v prefix 'cd -- %q && source %q && unset ROS_HOSTNAME ROS_IP && export ROS_MASTER_URI=%q ROS_LOG_DIR=%q' \
                "$WORKSPACE" "$setup" "$master_uri" "$log_dir/ros"
            printf -v command ' %q' bash "$SCRIPT" --pane "$role" "$log_dir" "$startup_timeout" "$@"
            local address=""
            if (( ${#ros_address[@]} )); then
                printf -v address ' %q' "${ros_address[0]}"
            fi
            tmux send-keys -t "$window.$pane" -l "$prefix$address && export ROS_IP &&$command"
            tmux send-keys -t "$window.$pane" C-m
        }

        start_pane 0 core
        start_pane 1 arms "${launch_args[@]}"
        start_pane 2 grippers
        start_pane 3 recording
        trap - EXIT
        printf 'Session logs: %s\n' "$log_dir"
    fi
fi

if ! "$detach"; then
    if [[ -n "${TMUX:-}" ]]; then
        tmux switch-client -t "=$SESSION"
    else
        tmux attach-session -t "=$SESSION"
    fi
fi
