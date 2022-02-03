# Projet Intégration ECN

# ecn-integ-project

## Table of Contents
1. [General Info](#general-info)
2. [Dependencies](#dependencies)
3. [Installation](#installation)

### General Info
***
As part of the INTEG module at Ecole Centrale de Nantes, this project aims to create an 2R robot in Gazebo capable of shooting at a moving target.

## Dependencies
***
A list of technologies used within the project:
* [ROS](https://ros.org): Version Noetic 
* [C++](https://cplusplus.com): Version 11.0
* [Gazebo](https://example.com): Version 9.0

## Installation
***
Download the projet to your user home using the following commands.
The repository will be a ros workspace.
```
$ cd ~
$ git clone https://https://github.com/CentraleNantesRobotics/integration_project_ecn
$ cd integration_project_ecn
$ catkin build
$ source devel/setup.bash