#! /bin/bash

clear
cd ./urdf
rosrun xacro xacro differential_wheeled_base.xacro --inorder > differential_wheeled_base.urdf
urdf_to_graphiz differential_wheeled_base.urdf
evince differential_wheeled_base.pdf
roslaunch robot_modelling model_viewer_mobile.launch


