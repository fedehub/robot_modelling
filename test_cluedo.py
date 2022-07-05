#! /bin/bash

clear
cd ./urdf
rosrun xacro xacro mobile_and_arm.xacro --inorder > mobile_and_arm.urdf
urdf_to_graphiz mobile_and_arm.urdf
evince mobile_and_arm.pdf
roslaunch robot_modelling model_viewer_cluedo.launch


