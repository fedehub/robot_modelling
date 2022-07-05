#! /bin/bash

clear
cd ./urdf
rosrun xacro xacro manipulator_arm.xacro --inorder > manipulator_arm.urdf
check_urdf manipulator_arm.urdf
urdf_to_graphiz manipulator_arm.urdf
evince manipulator_arm.pdf
roslaunch robot_modelling model_viewer_arm.launch


