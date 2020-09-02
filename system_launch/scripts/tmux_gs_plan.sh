#!/bin/bash

if [ $# -eq 0 ]; then
  echo "Input mav number as argument"
  exit 1
fi

MAV_ID=$1
if echo $MAV_ID | grep -Eq '^[+-]?[0-9]+$'
then
  echo "Running Ground Station for MAV $MAV_ID"
else
  echo "Input mav number(integer) as first argument"
  exit 1
fi

MAV_NAME=dragonfly${MAV_ID}
MAV_IP=192.168.131.${MAV_ID}
MAV_ADDRESS=${MAV_NAME}

if [ $# -eq 2 ]; then
  echo "Overriding mav IP $2"
  MAV_IP=$2
  MAV_ADDRESS=MAV_IP
fi

#NET_IP=$(ifconfig | sed -En 's/127.0.0.1//;s/.*inet (addr:)?(([0-9]*\.){3}[0-9]*).*/\2/p')

NET_IP=$(ip route get 8.8.8.8 | head -1 | awk '{print $7}')
if [ -z "$NET_IP" ]; then
  NET_IP=$(ip -o -4 addr list wlan0 | awk '{print $4}' | cut -d/ -f1)
fi

MASTER_URI=http://${MAV_ADDRESS}:11311

SETUP_ROS_STRING="export ROS_IP=${NET_IP}; export ROS_MASTER_URI=${MASTER_URI}"

SESSION_NAME=tmux_plan

if [ -z ${TMUX} ];
then
  TMUX= tmux new-session -s $SESSION_NAME -d
  echo "Starting new session."
else
  echo "Already in tmux, leave it first."
  exit
fi

#Generate rviz config file for specific mav from default one
RVIZ_CONFIG_FILE="$HOME/.ros/$MAV_NAME.rviz"
LAUNCH_PATH=$(rospack find system_launch)
cp $LAUNCH_PATH/config/rviz/exploration.rviz ~/.ros/$MAV_NAME.rviz
sed -i "s/dragonfly32/$MAV_NAME/g" ~/.ros/$MAV_NAME.rviz

tmux rename-window -t $SESSION_NAME "Main"
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; rosparam set robot_name $MAV_NAME; rosrun rqt_mav_manager rqt_mav_manager" Enter
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; roslaunch system_launch snav_rviz.launch mav_name:=${MAV_NAME} rviz_config:=$RVIZ_CONFIG_FILE" Enter
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; rosrun rqt_image_view rqt_image_view image:=/$MAV_NAME/image_overlay/compressed" Enter
tmux select-layout -t $SESSION_NAME tiled

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