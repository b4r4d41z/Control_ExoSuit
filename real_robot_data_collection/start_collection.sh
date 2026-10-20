#!/bin/bash
set -euo pipefail

SESSION="data_collection"

#http://192.168.1.106:11311
ROS_MASTER_URI="http://192.168.3.3:11311"
ROS_IP="192.168.3.52"

PROJECT_DIR="/home/Synthetic_data_collection/real_robot_data_collection"
ROS_SETUP="/opt/ros/noetic/setup.bash"
WS_SETUP="${PROJECT_DIR}/devel/setup.bash"

tmux kill-session -t "$SESSION" 2>/dev/null || true

tmux new-session -d -s "$SESSION" -n main
tmux split-window -h -t "$SESSION":0
tmux split-window -v -t "$SESSION":0.0
tmux split-window -v -t "$SESSION":0.1
tmux select-layout -t "$SESSION":0 tiled

COMMON="cd \"$PROJECT_DIR\"; source \"$ROS_SETUP\"; source \"$WS_SETUP\"; export ROS_MASTER_URI=\"$ROS_MASTER_URI\"; export ROS_IP=\"$ROS_IP\""

tmux send-keys -t "$SESSION:0.0" "$COMMON; rostopic list" C-m
sleep 2
tmux send-keys -t "$SESSION:0.1" "$COMMON; roslaunch data_bag_recorder record.launch" C-m
tmux send-keys -t "$SESSION:0.2" "$COMMON; roslaunch grip_state_publisher grip_state.launch" C-m
tmux send-keys -t "$SESSION:0.3" "$COMMON; roslaunch servo_angle_publisher servo_angle.launch" C-m

tmux attach -t "$SESSION"
