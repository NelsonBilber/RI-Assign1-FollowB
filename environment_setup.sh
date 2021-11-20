#!/bin/bash

mkdir clean_workspace

cd clean_workspace

mkdir src

cd src

mkdir followb_robot

cp -Rv ../../source_code/* followb_robot

git clone https://github.com/stdr-simulator-ros-pkg/stdr_simulator.git

cd ..

rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO

cp -Rv src/followb_robot/stdr_aux_files/* src/stdr_simulator

catkin_make