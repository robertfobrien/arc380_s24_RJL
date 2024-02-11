FROM ros:noetic

# Install dependencies
RUN apt-get update
# Control
RUN apt-get install -y \
    ros-noetic-robot-state-publisher \
    ros-noetic-controller-manager \
    ros-noetic-joint-state-controller \
    ros-noetic-joint-state-publisher \
    ros-noetic-joint-state-publisher-gui
RUN apt-get update
RUN apt-get install -y \
    ros-noetic-joint-trajectory-controller
# rviz
RUN apt-get install -y \
    ros-noetic-rviz
# MoveIt
RUN apt-get install -y \
    ros-noetic-moveit
# Gazebo
RUN apt-get install -y \
    gazebo11 \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control
# rosbridge
RUN apt-get install -y \
    ros-noetic-rosbridge-server
# Python
RUN apt-get update
RUN apt-get install -y python3-pip
RUN pip3 install \
    roslibpy \
    rospkg
# Cleanup
RUN rm -rf /var/lib/apt/lists/*

# Set up noVNC
ENV DISPLAY=novnc:0.0

# Set up workspace
ENV CATKIN_WS=/root/catkin_ws
RUN mkdir -p $CATKIN_WS/src
WORKDIR $CATKIN_WS
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; catkin_make'

# Source workspace
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source $CATKIN_WS/devel/setup.bash" >> ~/.bashrc

# Expose ports
# 9090: rosbridge
EXPOSE 9090
# 11311: ROS Master
EXPOSE 11311
# 11345: Gazebo
EXPOSE 11345
