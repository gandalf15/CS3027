#!/bin/bash

gnome-terminal \
	--window-with-profile=hold --hide-menubar --geometry=80x15 -e htop \
	--window-with-profile=hold --hide-menubar --geometry=80x15 -e roscore \
	--window-with-profile=hold --hide-menubar --geometry=80x15 --working-directory=$HOME/Documents/Robotics-CS3027/ \
	--window-with-profile=hold --hide-menubar -e "rosrun stage_ros stageros /opt/ros/indigo/share/stage_ros/world/willow-erratic.world" \
	


