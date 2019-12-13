# Quad-rotor-Project

Simulation part.

---

## Dependencies

* hector_quadrotor packages
  * hector_quadrotor(https://github.com/tu-darmstadt-ros-pkg/hector_quadrotor)
  * hector_models(https://github.com/tu-darmstadt-ros-pkg/hector_models)
  * hector_slam(https://github.com/tu-darmstadt-ros-pkg/hector_slam)
  * hector_gazebo(https://github.com/tu-darmstadt-ros-pkg/hector_gazebo)
  * hector_localization(https://github.com/tu-darmstadt-ros-pkg/hector_localization)
  * (OPTIONAL)gazebo_ros_pkgs(https://github.com/ros-simulation/gazebo_ros_pkgs)
  * (OPTIONAL)uuid_msgs(https://github.com/ros-geographic-info/unique_identifier)
* (OPTIONAL)geographic_info ros package(https://github.com/ros-geographic-info/geographic_info)
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(linux)
  * Linux: make is installed by default on most Linux distros
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
* Gazebo >= 7
* Rviz

## Basic Build Instructions

1. Clone hector_* packages
2. Compile: `catkin_make`
3. Clone this repo
4. Compile: `catkin_make`

## How to tun:

1. `roslaunch quadrotor_demo spawn_single_quadrotor.launch`
2. `rosrun quadrotor_demo pose_pub`
