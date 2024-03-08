#!/usr/bin/bash

# run Ignition in background and wait a bit
screen -dmS ignition -m bash -c 'ros2 launch integ_description world_launch.py'

sleep 5

# spawn the models / robot state publisher with passed arguments
ros2 launch integ_description all_launch.py $@

# kill Ignition
screen -S ignition -X stuff '^C'
