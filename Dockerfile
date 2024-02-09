FROM ros:noetic

# Install dependencies
RUN apt-get update
RUN apt-get install -y \
    # Control
    ros-noetic-robot-state-publisher \
    ros-noetic-controller-manager \
    ros-noetic-joint-state-controller \
    ros-noetic-joint-state-publisher \
    ros-noetic-joint-state-publisher-gui \
    ros-noetic-joint-trajectory-controller \
    # rviz
    ros-noetic-rviz \
    # MoveIt
    ros-noetic-moveit \
    # Gazebo
    gazebo11 \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    # Cleanup
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN apt-get update
RUN apt-get install -y python3-pip
RUN pip3 install \
    roslibpy \
    rospkg

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
# 11311: ROS Master
EXPOSE 11311
# 11345: Gazebo
EXPOSE 11345
