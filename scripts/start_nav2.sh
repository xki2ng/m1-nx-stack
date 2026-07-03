#!/bin/bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp ROS_DOMAIN_ID=66

ros2 launch /home/robot/fastlio/m1_nav2.launch.py > /home/robot/fastlio/logs/nav2.log 2>&1 &
sleep 18

ros2 lifecycle set /controller_server configure --spin-time 5
sleep 15
ros2 lifecycle set /controller_server activate --spin-time 5
echo "controller: $(ros2 lifecycle get /controller_server | head -1)"

for n in planner_server behavior_server bt_navigator; do
    ros2 lifecycle set /$n configure --spin-time 3
    sleep 3
    ros2 lifecycle set /$n activate --spin-time 3
    echo "$n: $(ros2 lifecycle get /$n | head -1)"
done

ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator
ros2 action list | grep navigate
echo "DONE"
