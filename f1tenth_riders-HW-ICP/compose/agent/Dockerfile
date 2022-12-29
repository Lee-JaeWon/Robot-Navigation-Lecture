# Base image
ARG PARENT
FROM ${PARENT}

COPY ./compose/ros.asc /ros.asc
RUN sudo apt-key add /ros.asc

RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-ackermann-msgs python3-pip

# Creating a catkin workspace
RUN mkdir -p /catkin_ws/src

# Copying
COPY ./pkg /catkin_ws/src/pkg
RUN chmod +x /catkin_ws/src/pkg/nodes/f1tenth_ros_agent.py
RUN pip3 install -r /catkin_ws/src/pkg/requirements.txt

## Building your ROS packages
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash; cd catkin_ws; catkin_make; source devel/setup.bash"

COPY ./compose/agent/start.sh /start.sh
RUN chmod +x /start.sh

ENTRYPOINT [ "/start.sh" ]