set dotenv-load # to read ROBOT_NAMESPACE from .env file

[private]
alias husarnet := connect-husarnet
[private]
alias flash := flash-firmware
[private]
alias start := start-rosbot

[private]
default:
    @just --list --unsorted

[private]
pre-commit:
    #!/bin/bash
    if ! command -v pre-commit &> /dev/null; then
        pip install pre-commit
        pre-commit install
    fi
    pre-commit run -a

_install-rsync:
    #!/bin/bash
    if ! command -v rsync &> /dev/null || ! command -v sshpass &> /dev/null || ! command -v inotifywait &> /dev/null; then
        if [ "$EUID" -ne 0 ]; then
            echo -e "\e[1;33mPlease run as root to install dependencies\e[0m"
            exit
        fi
        sudo apt-get install -y rsync sshpass inotify-tools
    fi

_install-yq:
    #!/bin/bash
    if ! command -v /usr/bin/yq &> /dev/null; then
        if [ "$EUID" -ne 0 ]; then
            echo -e "\e[1;33mPlease run as root to install dependencies\e[0m"
            exit
        fi

        YQ_VERSION=v4.35.1
        ARCH=$(arch)

        if [ "$ARCH" = "x86_64" ]; then
            YQ_ARCH="amd64"
        elif [ "$ARCH" = "aarch64" ]; then
            YQ_ARCH="arm64"
        else
            YQ_ARCH="$ARCH"
        fi

        curl -L https://github.com/mikefarah/yq/releases/download/${YQ_VERSION}/yq_linux_${YQ_ARCH} -o /usr/bin/yq
        chmod +x /usr/bin/yq
        echo "yq installed successfully!"
    fi

# connect to Husarnet VPN network
connect-husarnet joincode hostname:
    #!/bin/bash
    if [ "$EUID" -ne 0 ]; then
        echo "Please run as root"
        exit
    fi
    if ! command -v husarnet > /dev/null; then
        echo "Husarnet is not installed. Installing now..."
        curl https://install.husarnet.com/install.sh | sudo bash
    fi
    husarnet join {{joincode}} {{hostname}}

# flash the proper firmware for STM32 microcontroller in ROSbot 2R / 2 PRO
flash-firmware: _install-yq
    #!/bin/bash
    echo "Stopping all running containers"
    docker ps -q | xargs -r docker stop

    echo "Flashing the firmware for STM32 microcontroller in ROSbot"
    docker run \
        --rm -it --privileged \
        $(yq .services.rosbot.image compose.yaml) \
        ros2 run rosbot_utils flash_firmware

# start ROSbot 2R / 2 PRO autonomy containers
start-rosbot:
    #!/bin/bash
    if grep -q "Intel(R) Atom(TM) x5-Z8350" /proc/cpuinfo && [[ "${CONTROLLER}" == "mppi" ]]; then
        echo -e "\e[1;33mMPPI controller is currently not compatible with ROSbot 2 PRO. Please use DWB or RPP controller\e[0m"
        exit
    fi

    mkdir -m 775 -p maps

    sudo snap install rosbot --channel=humble/stable
    sudo /var/snap/rosbot/common/post_install.sh
    sudo rosbot.flash
    sudo snap set rosbot ros.domain-id=77 transport=udp

    sudo snap install husarion-rplidar --channel=humble/stable
    sudo snap set husarion-rplidar configuration=a2m12
    sudo snap set husarion-rplidar ros.domain-id=77 transport=udp

    sudo snap install husarion-webui --channel=humble/stable
    sudo snap set husarion-webui ros.domain-id=77 transport=udp
    sudo cp config/foxglove-autonomy.json /var/snap/husarion-webui/common
    sudo snap set husarion-webui webui.layout=autonomy
    sudo husarion-webui.start

    grep -v '^unset ' "/var/snap/rosbot/common/ros.env" | sed 's/^export //g' > .env.dds

    docker compose down
    docker compose pull
    docker compose up
