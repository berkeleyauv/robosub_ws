# robosub

Main top level repository for organizing urab robosub related code
***
## Getting Started
Start by making sure ROS2 is installed on Ubuntu 18.04 Follow [this](https://index.ros.org/doc/ros2/Installation/Eloquent/Linux-Install-Debians/) or run the `intsall-ros-eloquent.sh` script from `scripts/install_scripts` (this is the preferred method of installation).
***
Urab takes advantage of the [vcs tool](https://github.com/dirk-thomas/vcstool) in order to pull and
manage repositories and dependencies. A breakdown of the various `.repos` files is below:
#### Overlay Repos

Overlay repos are repositories directly managed by urab and that provide the core software for the platform.

`development.repos`  - pulls the development head of all core urab packages (used for development/nightly builds)

`master.repos` - pull the master head of all core urab packages (used for master builds)

**Pro Tip**: To avoid having the keep entering your github username and password for
pushing/pulling private repos you can run `git config --global credentials.helper store`
and then run a git command such as `git push` that requires authentication so you can enter and save your
username and password to the machine.

***
#### Underlay Repos

Underlay repos are repositories that are managed primarily by other sources including open source, but are
required as dependencies for the Overlay repos in a semi-managed state (i.e. `apt install` either do not exist or
do not give the desired control over the software versioning, etc.).

`deps.repos` - pulls all "underlay" external dependencies at the head of their "development" branch.
Note the branch name may not be development but should be the branch the represents the most current
version of the software suitable for our platform (in many packages for example this may be the `ros2`
branch). This is used for nightly and development builds.

`deps_release.repos` - pulls dependencies at a known commits compatible with the release software.
This is used for master builds.
***

### Building
In order to build the code make sure to run `vcs import < ***.repos` in the `ws/src` repository. This
should be done for both the desired underlay and overlay .repos file in order to ensure that all
dependencies are handled and that the code is at the correct version. For example if I want to build the
code that is in the master release, I would  run from within `ws/src` the following:
`vcs import < ../../repos/master.repos` and `vcs import < ../../repos/deps.repos`.

**Pro Tip**: If you want to update repositories (fetch remote) installed with vcs simply run `vcs custom --args remote update` in the
directory with the repositories you wish to update.

Once you have performed the above tasks you will want to finish up by running the rosdep command
to cover any minor deps managed entirely by the ROS2 open source community.
`rosdep install --from-paths src --ignore-src -r -y`
You should now be able to successfully run `colcon build --symlink-install` to compile your code. For release, make sure to build with the release compilation flag set using the following command `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release`.

## Contributing and Code Versioning
Development should proceed as follows.
- All new features require a new branch on the package it will be in. These branches should be made off the development branch.
- When done, new features will be merged into the development branch of each package for integration testing.
- Once a stable fully tested version of the development code is finished, it will be pushed to the master branch.

Remember to add any new repos that are added in one of the `.repos` files to the `.gitignore`.
