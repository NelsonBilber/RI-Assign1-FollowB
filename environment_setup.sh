#!/bin/bash

mkdir clean_workspace

cd clean workspace

mkdir src

cd src

mkdir followb_robot

cp -Rv ../../RI-Assign1-FollowB/* followb_robot

git clone https://github.com/stdr-simulator-ros-pkg/stdr_simulator.git

cd ..

rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO

cp -Rv src/followb_robot/stdr_aux_files/* src/stdr_simulator

catkin_make