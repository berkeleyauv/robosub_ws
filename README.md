# robosub

This is the top level repository for the UR@B AUV for the Robosub competition.

***
# Getting Started
The recommended installation process is to go to the `robosub` directory and run

    bash scripts/install_scripts/install-ros-foxy.sh

which will install ROS2 Foxy, Gazebo 11, download the necessary repositories, install the requirements using `rosdep`, and setup your `~/.bashrc` for future use. Then, run 

    colcon build --symlink-install

which will build the necessary files.

# Running the sub

Make sure you have sourced the `foxy` setup file, `gazebo` setup file, and `robosub` setup file already (this is done for you in the install script). Then run each command in a different terminal. We recommend using [tmux](https://github.com/tmux/tmux/wiki) to run the commands in separate panes so you can see all the outputs.

## Teleop Mode

1. `ros2 launch simulator robosub.launch` will spawn Gazebo with the RoboSub 2019 world
2. `ros2 launch sub_descriptions upload_rexrov_default.launch.py x:=10 y:=-10 z:=-10` will spawn in our submarine into Gazebo.
3. `ros2 launch uuv_control_cascaded_pid key_board_velocity.launch model_name:=rexrov` will start the PID controllers for the sub and the thruster manager.
4. `ros2 run uuv_teleop vehicle_keyboard_teleop.py --ros-args -r output:=/rexrov/cmd_vel` to use a keyboard to send commands to the motor controllers.

## Position PID Mode

1. `ros2 launch simulator robosub.launch` will spawn Gazebo with the RoboSub 2019 world
2. `ros2 launch sub_descriptions upload_rexrov_default.launch.py x:=10 y:=-10 z:=-10` will spawn in our submarine into Gazebo.
3. `ros2 launch uuv_control_cascaded_pids position_hold.launch` will start the PID nodes and thruster manager.
4. `ros2 run controls pose_publisher.py` will ask for the `cmd_pose`.

# More Info

## VCS
***
UR@B takes advantage of `vcs` in order to pull and manage repositories and dependencies. 


> **Pro Tip**: To avoid having the keep entering your github username and password for pushing private repos you can run `git config --global credentials.helper store` to save your username and password to the machine. However, make sure you only do this on a computer that only you use since it stores your password in unencrypted plaintext.

### Repos Lists

Overlay repos are repositories directly managed by UR@B and that provide the core software for the platform.

- `master.repos` - pull the master head of all core UR@B packages


Underlay repos are third-party repositories but are
required as dependencies in a semi-managed state (i.e. `apt install` either do not exist or do not give the desired control over the software versioning, etc.).

- `deps.repos` - pulls all external dependencies at the head of their "development" branch.
Note the branch name may not be development but should be the branch the represents the most current
version of the software suitable for our platform (in many packages for example this may be the `ros2`
branch). This is used for nightly and development builds.

- `deps_release.repos` - pulls dependencies at a known commits compatible with the release software.
This is used for master builds.

Remember to add any new repos that are added in one of the `.repos` files to the `.gitignore`.

In order to import the repos, run `vcs import < {file}.repos` in the `src/` folder. Usually, you will want to do the following

    vcs import < ../repos/master.repos 
    vcs import < ../repos/deps.repos

>**Pro Tip**: If you want to update repositories (fetch remote) installed with vcs simply run `vcs custom --args remote update` in the directory with the repositories you wish to update.

***

## Dependencies

All dependencies will be automatically installed in the install script using `rosdep` with

    rosdep install --from-paths src --ignore-src -r -y

## Building

We use the ROS2 standard `colcon` build tool to build our packages. After running the install script, you should now be able to successfully run `colcon build --symlink-install` to build your workspace. Afterwards, do `source install/setup.bash` to make sure the packages are visible to ROS2.

## Contributing and Code Versioning
Development should proceed as follows.
- All new features require a new branch on the package it will be in. These branches should be made off the master branch.
- Once a stable fully tested version of the branch code is finished, it will be merged into the master branch.

