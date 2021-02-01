#!/usr/bin/env bash

# Setup language locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS2 apt key
sudo apt update && sudo apt install curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Install ROS2 Foxy
sudo apt update
sudo apt install -y ros-foxy-desktop

# Install ROS build tools
sudo apt install -y python3-argcomplete python3-colcon-common-extensions python3-rosdep python3-vcstool
sudo rosdep init
rosdep update

# Clone subrepos using VCS
cd src
vcs import < ../repos/master.repos
vcs import < ../repos/deps.repos
cd ..

# Install all ROS dependencies using rosdep
rosdep install --from-paths src --ignore-src -r -y

# Get the directory where robosub is located
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && cd ../../ && pwd )"

# Setup bashrc 
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc
echo "source $DIR/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
