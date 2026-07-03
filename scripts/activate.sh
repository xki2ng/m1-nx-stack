#!/bin/bash
# Nav2 lifecycle activation - runs directly on NX
LOG=/home/robot/fastlio/logs/activate.log
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp ROS_DOMAIN_ID=66

exec > $LOG 2>&1
set -x

echo "=== $(date) Activating Nav2 ==="

# controller_server is slowest, do it first
for attempt in 1 2 3; do
    echo "controller configure attempt $attempt"
    ros2 lifecycle set /controller_server configure && break
    sleep 3
done
sleep 12
ros2 lifecycle set /controller_server activate
echo "controller_server: $(ros2 lifecycle get /controller_server | head -1)"

# Others are fast
for n in planner_server behavior_server bt_navigator; do
    ros2 lifecycle set /$n configure
    sleep 2
    ros2 lifecycle set /$n activate
    echo "$n: $(ros2 lifecycle get /$n | head -1)"
done

echo "=== DONE ==="
ros2 action list | grep navigate
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator
