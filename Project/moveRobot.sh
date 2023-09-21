#!/bin/bash


if [ "$#" -ne 4 ]; then
    echo "Wrong args | Usage: $0 <topic_name> <linear_x> <linear_y> <angular_x>"
fi

topic_name=$1
linear_x=$2
linear_y=$3
angular_x=$4

rostopic pub /"$topic_name"/cmd_vel geometry_msgs/Twist "linear:
  x: $linear_x
  y: $linear_y
  z: 0.0
angular:
  x: $angular_x
  y: 0.0
  z: 0.0"
