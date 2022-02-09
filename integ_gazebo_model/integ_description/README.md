# Integ_description

## Table of Contents
1. [General Info](#general-info)
2. [Installation](#installation)

### General Info
***
In this package you will find:
* A file "arm.urdf.xacro" which defines our RR robot and the onboard camera.
* A "target.urdf" file to describe the target model. 

The camera is placed on the end effector and only tracks targets in the XY plane. The simulation is launched on Rviz. For a simulation on Gazebo, see the launch files in the integ_gazebo folder.

## Installation
***
Download the projet to your ROS workspace using the following commands 
```
$ git clone https://github.com/Valentin-Molina/ecn-integ-project/src/integ_description.git
$ cd ../ros/src
$ catkin build integ_description
$ roslaunch integ_description arm.rviz
```
