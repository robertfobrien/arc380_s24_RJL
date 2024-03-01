FROM gramaziokohler/ros-noetic-moveit:latest


# Install compas_rrc
WORKDIR /root/catkin_ws/src
RUN git clone https://github.com/compas-rrc/compas_rrc_ros.git
WORKDIR /root/catkin_ws
RUN /bin/bash -c 'catkin_make'

RUN /bin/bash -c '. '