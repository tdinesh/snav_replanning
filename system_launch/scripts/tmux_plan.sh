#!/bin/bash

SESSION_NAME=tmux_plan

if [ -z ${TMUX} ];
then
  TMUX= tmux new-session -s $SESSION_NAME -d
  echo "Starting new session."
else
  echo "Already in tmux, leave it first."
  exit
fi

ln -sf /opt/ptmx /dev/ptmx

SETUP_ROS_STRING="source ~/home_linaro/.bashrc_voxl"

tmux setw -g mouse on

tmux new-window -t $SESSION_NAME -n "Exp"
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 4; roslaunch system_launch 2d_exp.launch mav_name:=${MAV_NAME}" Enter
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 5; roslaunch system_launch snav_traj_replanning.launch mav_name:=${MAV_NAME}" Enter
tmux select-layout -t $SESSION_NAME even-horizontal

tmux new-window -t $SESSION_NAME -n "In"
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; rosservice call ${MAV_NAME}/StartExploration"
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; roscd snavquad_interface/scripts/capture/; ./record.sh ${MAV_ID}"

tmux new-window -t $SESSION_NAME -n "Plan"
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 4; roslaunch snavquad_interface octomap_mapping.launch mav_name:=${MAV_NAME}" Enter
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 5; roslaunch snavquad_interface depth_pcl.launch mav_name:=${MAV_NAME}" Enter
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 5; roslaunch system_launch jps2d.launch mav_name:=${MAV_NAME}" Enter
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 5; roslaunch system_launch jps3d.launch mav_name:=${MAV_NAME}" Enter
tmux select-layout -t $SESSION_NAME tiled

tmux new-window -t $SESSION_NAME -n "Kill"
tmux send-keys -t $SESSION_NAME "tmux kill-session -t tmux_plan"

tmux select-window -t $SESSION_NAME:0
tmux -2 attach-session -t $SESSION_NAME

clear