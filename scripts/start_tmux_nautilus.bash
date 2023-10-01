#!/bin/bash

DAY=$(date "+%Y-%m-%d")
NOW=$(date "+%Y-%m-%dT%H.%M.%S.%N")
LOGDIR="/home/field/project11/logs"
mkdir -p "$LOGDIR"
LOG_FILE="${LOGDIR}/autostart_nautilus_${NOW}.txt"

{

printenv
set -x

echo ""
echo "#############################################"
echo "Starting nautilus"
date
echo "#############################################"
echo ""

while ! ping -c 1 -W 1 robobox; do
	sleep 1
done

source /home/field/.ros_project11.bash

export ROS_MASTER_URI=http://robobox:11311
export ROS_IP=$ROBOBOX_ROS_IP

/usr/bin/tmux new -d -s nautilus
/usr/bin/tmux send-keys -t nautilus "source /home/field/.bashrc" C-m
/usr/bin/tmux send-keys -t nautilus "source /home/field/project11/catkin_ws/src/drix_project11/scripts/robobox_as_core.bash" C-m
/usr/bin/tmux send-keys -t nautilus "rosrun rosmon rosmon --name=rosmon_p11_nautilus nautilus_project11 nautilus.launch logDirectory:=${LOGDIR}" C-m

set +x

} >> "${LOG_FILE}" 2>&1

