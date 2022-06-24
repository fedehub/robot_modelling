#! /bin/bash

cd ./urdf
check_urdf simple_robot.urdf
urdf_to_graphiz simple_robot.urdf
evince simple_robot.pdf
roslaunch robot_modelling model_viewer.launch


