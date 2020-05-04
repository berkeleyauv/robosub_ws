# Giant kudos to bpinaya for https://github.com/bpinaya/robond-docker
FROM dorowu/ubuntu-desktop-lxde-vnc

# Adding keys for ROS
RUN apt-get update && apt-get install dirmngr -y
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Installing ROS Melodic
RUN apt-get update && apt install ros-melodic-desktop-full wget git vim -y
#RUN rosdep init && rosdep update

# Install VNC and UUV Simulator packages
RUN apt update \
    && apt install ros-melodic-uuv-simulator python-catkin-tools vim -y 

# Create catkin workspace
RUN /bin/bash -c 'source /opt/ros/melodic/setup.bash \
    && mkdir -p "/root/catkin_ws/src" \
    && cd "/root/catkin_ws/src" \
    && catkin_init_workspace \
    && cd "/root/catkin_ws" \
    && catkin build'

# Install ROS package dependencies
RUN apt-get update \
    && apt-get install ros-melodic-image-pipeline ros-melodic-rqt -y # For image_view \
    && rosmake image_view rqt

# Add setup files to bashrc
RUN echo source /opt/ros/melodic/setup.bash >> /root/.bashrc \
    && echo source /root/catkin_ws/devel/setup.bash >> /root/.bashrc

# Set some environment variables
ENV DISPLAY :0.0
ENV GAZEBO_MODEL_PATH /root/catkin_ws/src/descriptions/vortex_descriptions/world_models:/root/catkin_ws/src/descriptions/vortex_descriptions/models

# "catkin build" needs to be run in /root/catkin_ws/src whenever a new package is added

