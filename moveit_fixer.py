#! /bin/bash

apt-get -y remove ros-noetic-moveit*
apt-get -y remove ros-noetic-srdfdom

cd ./src
git clone https://github.com/ros-planning/moveit.git
cd moveit
git checkout 2b881e5e3c4fd900d4d4310f4b12f9c4a63eb5dd
cd ..
git clone https://github.com/ros-planning/moveit_msgs.git
cd moveit_msgs
git checkout 612d7d5bb1047f65acd495a7e632da78a621545d
cd ..
git clone https://github.com/ros-planning/moveit_resources.git
cd moveit_resources
git checkout f6a7d161e224b9909afaaf621822daddf61b6f52
cd ..
git clone https://github.com/ros-planning/srdfdom.git
cd srdfdom
git checkout b1d67a14e45133928f9793e9ee143998219760fd
cd ..
apt-get install ros-noetic-rosparam-shortcuts
cd ..
catkin_make 

