#!/usr/bin/env bash

sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt update && sudo apt install curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

sudo apt update -y
sudo apt install -y ros-foxy-desktop
source /opt/ros/foxy/setup.bash
sudo apt install -y -qq python3-argcomplete
sudo apt install -y -qq python3-colcon-common-extensions
sudo apt-get -qq -y install python-rosdep
sudo rosdep init
rosdep update
sudo rosdep fix-permissions
