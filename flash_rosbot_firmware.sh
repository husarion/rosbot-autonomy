#!/bin/bash

get_arch() {
ARCH=$(uname -m)

case $ARCH in
    x86_64)
        echo "amd64"
        ;;
    aarch64)
        echo "arm64"
        ;;
    *)
        echo "Unknown architecture"
        ;;
esac
}

# Check if yq is installed
if ! command -v /usr/bin/yq &> /dev/null; then
    # Check if the user is root
    if [ "$EUID" -ne 0 ]; then
        echo "Please run as root to install yq (required for this script)"
        exit 1
    fi
    
    YQ_VERSION=v4.35.1
    curl -L https://github.com/mikefarah/yq/releases/download/${YQ_VERSION}/yq_linux_$(get_arch) -o /usr/bin/yq
    chmod +x /usr/bin/yq
    echo "done!"
    echo "Now rerun the script as a normal user"
    exit 0
fi

echo "Stopping all running containers"
docker ps -q | xargs -r docker stop

echo "Flashing the firmware for STM32 microcontroller in ROSbot"
docker run --rm -it --privileged \
$(yq .services.rosbot.image $(dirname "$0")/compose.yaml) \
ros2 run rosbot_utils flash_firmware
