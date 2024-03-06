# Projet Intégration ECN - ROS 2

# ecn-integ-project

## Table of Contents
1. [General Info](#general-info)
2. [Dependencies](#dependencies)
3. [Installation](#installation)

### General Info
***
As part of the INTEG module at Ecole Centrale de Nantes, this project aims to create an 2R robot in Gazebo capable of tracking a moving target.

## Dependencies
***
A list of technologies used within the project:
* [ROS 2](https://ros.org): Version Noetic
* [C++](https://cplusplus.com): Version 17
* [Gazebo](https://example.com): Fortress

## Installation
***
Download the projet to your ROS 2 workspace
Compile with `colcon`
```
$ cd ~
$ git clone https://github.com/CentraleNantesRobotics/integration_project_ecn -b ros2
$ cd integration_project_ecn
$ colcon build
$ source install/setup.bash
