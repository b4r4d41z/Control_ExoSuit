#!/bin/bash
# --- Automated startup script for real_robot_data_collection ---
# Creates all panes first, then launches each process in order.

SESSION="fix_bugs"

#=======LOCAL TESTS INSIDE DOCKER========
ROS_MASTER_URI="http://172.17.0.1:11311"
ROS_IP="172.17.0.1"
#========================================

PROJECT_DIR="$HOME/ExoSuit/real_robot_data_collection"

# Kill previous session if exists
tmux kill-session -t $SESSION 2>/dev/null

# --- STEP 1: Create empty panes first ---
tmux new-session -d -s $SESSION -n main       # Pane 0
tmux split-window -h -t $SESSION              # Pane 1 (right)
tmux split-window -v -t $SESSION:0.0          # Pane 2 (bottom left)
tmux split-window -v -t $SESSION:0.1          # Pane 3 (bottom right)
tmux select-layout -t $SESSION tiled          # arrange all panes nicely

# --- STEP 2: Start processes in correct order ---

# Pane 0: roscore
tmux send-keys -t $SESSION:0.0 "source /opt/ros/noetic/setup.bash" C-m
#tmux send-keys -t $SESSION:0.0 "source devel/setup.bash" C-m
tmux send-keys -t $SESSION:0.0 "export ROS_MASTER_URI=$ROS_MASTER_URI" C-m
tmux send-keys -t $SESSION:0.0 "export ROS_IP=$ROS_IP" C-m
tmux send-keys -t $SESSION:0.0 "roscore" C-m

# Pause to allow roscore to initialize
sleep 5

# Pane 1: data_bag_recorder
tmux send-keys -t $SESSION:0.1 "cd $PROJECT_DIR" C-m
tmux send-keys -t $SESSION:0.1 "source devel/setup.bash" C-m
tmux send-keys -t $SESSION:0.1 "export ROS_MASTER_URI=$ROS_MASTER_URI" C-m
tmux send-keys -t $SESSION:0.1 "export ROS_IP=$ROS_IP" C-m
tmux send-keys -t $SESSION:0.1 "roslaunch data_bag_recorder record.launch" C-m
 
# Pane 2: grip_state_publisher
tmux send-keys -t $SESSION:0.2 "cd $PROJECT_DIR" C-m
tmux send-keys -t $SESSION:0.2 "source devel/setup.bash" C-m
tmux send-keys -t $SESSION:0.2 "export ROS_MASTER_URI=$ROS_MASTER_URI" C-m
tmux send-keys -t $SESSION:0.2 "export ROS_IP=$ROS_IP" C-m
tmux send-keys -t $SESSION:0.2 "roslaunch grip_state_publisher grip_state.launch" C-m

# Pane 3: servo_angle_publisher
tmux send-keys -t $SESSION:0.3 "cd $PROJECT_DIR" C-m
tmux send-keys -t $SESSION:0.3 "source devel/setup.bash" C-m
tmux send-keys -t $SESSION:0.3 "export ROS_MASTER_URI=$ROS_MASTER_URI" C-m
tmux send-keys -t $SESSION:0.3 "export ROS_IP=$ROS_IP" C-m
tmux send-keys -t $SESSION:0.3 "roslaunch servo_angle_publisher servo_angle.launch" C-m

# --- STEP 3: Attach session ---
tmux attach -t $SESSION

