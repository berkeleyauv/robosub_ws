# Simulator

This will set up Underwater Robotics @ Berkeley's team simulator.

# VMWare Instructions

Install VMWare Workstation Pro/Fusion with Berkeley's license from here https://software.berkeley.edu/vmware. Then download a base Ubuntu 16 desktop image from https://releases.ubuntu.com/16.04/. Now create a new virtual machine and give the Ubuntu image as the disc file. Now you can run the VM and set it up with your username and password.

After you first log in, open up a terminal with Ctrl+Alt+T or search for it and then get our code with

    git clone https://github.com/berkeleyauv/robosub.git

Now you can follow the commands in the Dockerfile to install ROS, Gazebo, and the UUV simulator.

# Mac Instructions

## --Installation--

#### Install Docker Desktop
Install from [here](https://docs.docker.com/docker-for-mac/install/). This is for running Gazebo/the simulator in an Ubuntu environment even though you're using Mac. Verify installation with
```
docker --version
```

#### Pull this git repository
```
# cd to a folder where you want to keep this forever
git clone https://github.com/berkeleyauv/simulator.git
```

#### Build and start up the Docker image
Build the docker image in the repo and name it urab-sim. This can take up to 30 minutes without cache because it downloads all of Ubuntu desktop and ROS/Gazebo.
```
cd simulator
make build
```

## --Starting the simulator--

#### Open Docker Desktop
Search for the application on your computer and let it start up. The icon should appear in your toolbar on the top right.

#### Start the Docker process
Run the docker image to start the docker process while setting up port forwarding and folder sharing between the container and your computer.
```
make run
```

#### View the Ubuntu desktop from your browser
This will allow you to run commands in terminal as if you had Ubuntu, ROS, Gazebo, and all simulator dependencies installed.

* Open your preferred web browser
* Go to `localhost:6080` and wait a few seconds for the connection to happen

#### Start simulating
Open the command line

* Click on the bottom-left icon and open System Tools > LXTerminal

Build the custom ROS packages

```
cd ~/catkin_ws/src
catkin build
```

Use `roslaunch` to start simulating a world. The syntax is `roslaunch <package name> <launch_filename.launch>`. Here's an example that launches the full RoboSub world:

```
roslaunch vortex_descriptions robosub_world_sub.launch
```

All changes to files in /root/catkin_ws/src are synced to the folder where you cloned the repo on your computer. 

# Ubuntu Instructions

## --Installation--

#### Pull this git repository
```
# cd to a folder where you want to keep this forever
git clone https://github.com/berkeleyauv/simulator.git
```

#### Follow the setup instructions from the Dockerfile, but adjust accordingly
#### to Ubuntu
For the commands after the line with "FROM", run each command separately, using sudo.

Each command also needs to be run separately for the rest of the Dockerfile.

It may be necessary to set the GAZEBO_MODEL_PATH environment variable each time
you start your computer. Or add the whole export command to your \~/.bashrc file with `echo <the export command here> >> ~/.bashrc`


## --Starting the simulator--

Use `roslaunch` to start simulating a world. The syntax is `roslaunch <package name> <launch_filename.launch>`. Here's an example that launches the full RoboSub world:

```
roslaunch vortex_descriptions robosub_world.launch
```

# Debugging
See the "Simulator Progress" doc in the team Google Drive in the Software/Perception/ folder

# References
[Vortex NTNU's uuv-simulator](https://github.com/vortexntnu/uuv-simulator)

[UUVSimulator](https://uuvsimulator.github.io/)
```
@inproceedings{Manhaes_2016,
    doi = {10.1109/oceans.2016.7761080},
    url = {https://doi.org/10.1109%2Foceans.2016.7761080},
    year = 2016,
    month = {sep},
    publisher = {{IEEE}},
    author = {Musa Morena Marcusso Manh{\~{a}}es and Sebastian A. Scherer and Martin Voss and Luiz Ricardo Douat and Thomas Rauschenbach},
    title = {{UUV} Simulator: A Gazebo-based package for underwater intervention and multi-robot simulation},
    booktitle = {{OCEANS} 2016 {MTS}/{IEEE} Monterey}
}
```
