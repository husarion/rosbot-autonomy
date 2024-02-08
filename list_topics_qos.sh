#!/bin/bash

# Get list of all topics
topics=$(ros2 topic list)

# Loop through each topic and print its QoS settings
for topic in $topics
do
    echo "QoS for topic $topic:"
    ros2 topic info "$topic" -v
    echo ""
done
