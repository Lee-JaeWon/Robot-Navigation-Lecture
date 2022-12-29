#!/bin/bash

# run
source /opt/ros/${ROS_DISTRO}/setup.bash
source /catkin_ws/devel/setup.bash

roslaunch --wait pkg f1tenth_gym_agent.launch

# basically if agents fail for any reason, we don't care
echo "Agent completed. Returning 0 for SUCCESS"
exit 0