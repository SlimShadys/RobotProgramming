#!/bin/bash

# Colors
green=$(tput setaf 2)
normal=$(tput sgr0)

# Variables
dir=$(dirname "$0")

clear

cd "$dir/workspace"     # Let's enter the workspace
echo "${green}-------- RUNNING CATKIN_MAKE ---------${normal}"
catkin_make             # Build the project

echo "${green}-------- SOURCING THE WORKSPACE ---------${normal}"
source devel/setup.sh   # Source the workspace
echo "Workspace successfully sourced. Now you can run ROS nodes :)"
echo "${green}-----------------------------------------${normal}"
