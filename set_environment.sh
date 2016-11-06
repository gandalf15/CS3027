#!/bin/bash

gnome-terminal \
	--window-with-profile=hold --hide-menubar --geometry=80x10 -e htop \
	--window-with-profile=hold --hide-menubar --geometry=80x10 --working-directory=$HOME/Documents/Robotics-CS3027/ \
	--window-with-profile=hold --hide-menubar --geometry=80x10 --working-directory=$HOME/Documents/Robotics-CS3027/ \
	--window-with-profile=hold --hide-menubar --geometry=80x10 -e "roslaunch robot01 r.launch" \
	#--window-with-profile=hold --hide-menubar -e "rosrun stage_ros stageros /opt/ros/indigo/share/stage_ros/world/willow-erratic.world" \
	#--window-with-profile=hold --hide-menubar --geometry=80x10 -e roscore \