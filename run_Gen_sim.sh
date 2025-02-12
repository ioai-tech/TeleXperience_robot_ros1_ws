#!/bin/bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash;
export ROS_IP=192.168.123.24
export ROS_MASTER_URI=http://192.168.123.24:11311
export ROS_DOMAIN_ID=0
ROBOT_NAME=OpenLoong

# Create new tmux session and name it io_teleop_v3_ros1_ws
tmux new-session -d -s io_teleop_v3_ros1_ws
tmux send-keys -t io_teleop_v3_ros1_ws "source /opt/ros/noetic/setup.bash; source devel/setup.bash; roscore" C-m

# wait for roscore started
sleep 1

tmux select-pane -t io_teleop_v3_ros1_ws:0.0
tmux split-window -v -t io_teleop_v3_ros1_ws
tmux send-keys -t io_teleop_v3_ros1_ws "source /opt/ros/noetic/setup.bash; source devel/setup.bash; python3 src/io_teleop_robot_control_node/scripts/general_sim_robot_control_node.py --robot_name $ROBOT_NAME" C-m


# Set the session window title
tmux rename-window -t io_teleop_v3_ros1_ws:0 'io_teleop_v3_ros1_ws'

# Attach to the tmux session
tmux attach -t io_teleop_v3_ros1_ws
