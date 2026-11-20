#!/usr/bin/env bash
# Run the local master and three independent ROS nodes in tmux.

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
        error "Refusing to reuse a ROS master at $ROS_MASTER_URI."
        return 1
    fi
    roscore -p "${ROS_MASTER_URI##*:}" &
    local core_pid=$! deadline=$((SECONDS + startup_timeout))
    trap 'kill -TERM "$core_pid" 2>/dev/null || true; wait "$core_pid" 2>/dev/null || true; return 130' INT TERM
    until master_responds; do
        if ! kill -0 "$core_pid" 2>/dev/null || (( SECONDS >= deadline )); then
            kill -TERM "$core_pid" 2>/dev/null || true
            wait "$core_pid" || true
            error "Dedicated ROS master did not start at $ROS_MASTER_URI."
            return 1
        fi
        sleep 0.25
    done
    if ! kill -0 "$core_pid" 2>/dev/null; then
        error "Dedicated ROS master exited before it became ready."
        return 1
    fi
    touch "$log_dir/core-ready"
    wait "$core_pid"
}

run_node() {
    local deadline=$((SECONDS + startup_timeout))
    until [[ -e "$log_dir/core-ready" ]] && master_responds; do
        if [[ -e "$log_dir/core-exit" ]] || (( SECONDS >= deadline )); then
            error "Dedicated ROS master did not become ready at $ROS_MASTER_URI."
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
        "sim_master_uri:=$ROS_MASTER_URI" simulation_only:=true \
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
simulation_only=false
launch_args=()
for arg in "$@"; do
    case "$arg" in
        --detach) detach=true ;;
        --help)
            cat <<'USAGE'
Usage: ./isaac_datacollection_start.sh simulation_only:=true [--detach] [name:=value ...]
Launch arguments: safety_file, trajectories_file, model_urdf, left_serial, right_serial.
Defaults: ROS Master http://127.0.0.1:11321, ROS_IP 127.0.0.1,
          simulation_data_collection/devel/setup.bash, 30 s startup timeout.
Overrides: ISAAC_ROS_MASTER_URI, ISAAC_ROS_IP, ISAAC_CATKIN_SETUP,
           ISAAC_ROS_START_TIMEOUT, ISAAC_LOG_ROOT (default ~/.ros/exosuit-isaac).
USAGE
            exit 0
            ;;
        simulation_only:=true) simulation_only=true ;;
        safety_file:=*|trajectories_file:=*|model_urdf:=*|left_serial:=*|right_serial:=*)
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
    "$simulation_only" || error "Confirm the isolated simulation network with simulation_only:=true."
    master_uri="${ISAAC_ROS_MASTER_URI:-http://127.0.0.1:11321}"
    ros_ip="${ISAAC_ROS_IP:-127.0.0.1}"
    startup_timeout="${ISAAC_ROS_START_TIMEOUT:-30}"
    if [[ ! "$master_uri" =~ ^http://([0-9.]+):([1-9][0-9]{0,4})$ ||
          "${BASH_REMATCH[1]}" != "$ros_ip" ]] ||
       (( BASH_REMATCH[2] > 65535 || BASH_REMATCH[2] == 11311 )); then
        error "Use http://ROS_IP:PORT with a local IPv4 address and a dedicated simulation port."
    fi
    python3 -c 'import ipaddress, socket, sys
host = ipaddress.IPv4Address(sys.argv[1])
if host.is_unspecified or host.is_multicast:
    sys.exit(1)
with socket.socket() as probe:
    probe.bind((str(host), 0))' "$ros_ip" || error "ISAAC_ROS_IP must be a local IPv4 unicast address."
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
            printf -v prefix 'cd -- %q && source %q && export ROS_MASTER_URI=%q ROS_IP=%q ROS_LOG_DIR=%q && unset ROS_HOSTNAME &&' \
                "$WORKSPACE" "$setup" "$master_uri" "$ros_ip" "$log_dir/ros"
            printf -v command ' %q' bash "$SCRIPT" --pane "$role" "$log_dir" "$startup_timeout" "$@"
            tmux send-keys -t "$window.$pane" -l "$prefix$command"
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
