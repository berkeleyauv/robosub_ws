#!/usr/bin/env bash

sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt update && sudo apt install curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

sudo apt update -y
sudo apt install -y ros-eloquent-desktop ros-eloquent-launch-xml ros-eloquent-launch-yaml
echo "source /opt/ros/eloquent/setup.bash" >> ~/.bashrc
echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc
source ~/.bashrc
sudo apt install -y -qq python3-argcomplete python3-colcon-common-extensions python-rosdep python3-vcstool
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
