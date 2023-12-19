#!/bin/bash

docker stop $(docker ps -q)

docker run --rm -it --privileged \
$(yq .services.rosbot.image $(dirname "$0")/compose.yaml) \
ros2 run rosbot_utils flash_firmware
