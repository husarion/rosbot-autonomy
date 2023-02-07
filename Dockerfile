FROM husarnet/ros:humble-ros-core

RUN apt update && apt install -y \
      ros-$ROS_DISTRO-rosbridge-suite