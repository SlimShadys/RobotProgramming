#!/bin/bash

if [ "$#" -ne 1 ]; then
    echo "Wrong args | Usage: $0 <topic_name>"
fi

topic_name=$1

rostopic echo /"$topic_name"/odom

