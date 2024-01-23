set dotenv-load

[private]
alias husarnet := connect-husarnet
[private]
alias flash := flash-firmware
[private]
alias rosbot := start-rosbot
[private]
alias pc := start-pc
[private]
alias teleop := run-teleop
[private]
alias teleop-docker := run-teleop-docker
[private]
alias gazebo := start-gazebo-sim
[private]
alias webots := start-webots-sim

[private]
default:
  @just --list --unsorted

_install-rsync:
    #!/bin/bash
    if ! command -v rsync &> /dev/null; then
        if [ "$EUID" -ne 0 ]; then \
            echo "Please run as root to install dependencies"; \
            exit 1; \
        fi
        sudo apt-get install -y rsync sshpass inotify-tools
    fi

_install-yq:
    #!/bin/bash
    if ! command -v /usr/bin/yq &> /dev/null; then \
        if [ "$EUID" -ne 0 ]; then \
            echo -e "\e[1;33mPlease run as root to install dependencies\e[0m"
            exit 1; \
        fi

        YQ_VERSION=v4.35.1
        ARCH=$(arch)

        if [ "$ARCH" = "x86_64" ]; then \
            YQ_ARCH="amd64"; \
        elif [ "$ARCH" = "aarch64" ]; then \
            YQ_ARCH="arm64"; \
        else \
            YQ_ARCH="$ARCH"; \
        fi

        curl -L https://github.com/mikefarah/yq/releases/download/${YQ_VERSION}/yq_linux_${YQ_ARCH} -o /usr/bin/yq
        chmod +x /usr/bin/yq
        echo "yq installed successfully!"
    fi

# connect to Husarnet VPN network
connect-husarnet joincode hostname:
    #!/bin/bash
    if [ "$EUID" -ne 0 ]; then \
        echo "Please run as root"; \
        exit; \
    fi
    if ! command -v husarnet > /dev/null; then \
        echo "Husarnet is not installed. Installing now..."; \
        curl https://install.husarnet.com/install.sh | sudo bash; \
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
    mkdir -m 777 -p maps
    docker compose down
    docker compose pull
    docker compose up

# start RViz visualization on PC
start-pc:
    xhost +local:docker
    docker compose -f compose.pc.yaml down
    docker compose -f compose.pc.yaml pull
    docker compose -f compose.pc.yaml up

# start Gazebo simulator with autonomy
start-gazebo-sim:
    xhost +local:docker
    docker compose -f compose.sim.gazebo.yaml down
    docker compose -f compose.sim.gazebo.yaml pull
    docker compose -f compose.sim.gazebo.yaml up

# start Webots simulator with autonomy
start-webots-sim:
    xhost +local:docker
    docker compose -f compose.sim.webots.yaml down
    docker compose -f compose.sim.webots.yaml pull
    docker compose -f compose.sim.webots.yaml up

# run teleop_twist_keybaord (host)
run-teleop:
    #!/bin/bash
    export FASTRTPS_DEFAULT_PROFILES_FILE=$(pwd)/shm-only.xml
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r __ns:=/${ROBOT_NAMESPACE}

# run teleop_twist_keybaord (inside rviz2 container)
run-teleop-docker:
    docker compose -f compose.pc.yaml exec rviz /bin/bash -c "/ros_entrypoint.sh ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r __ns:=/${ROBOT_NAMESPACE}"

# copy repo content to remote host with 'rsync' and watch for changes
sync hostname password="husarion":  _install-rsync
    #!/bin/bash
    mkdir -m 777 -p maps
    sshpass -p "{{password}}" rsync -vRr --exclude='.git/' --exclude='maps/' --delete ./ husarion@{{hostname}}:/home/husarion/${PWD##*/}
    while inotifywait -r -e modify,create,delete,move ./ --exclude='.git/' --exclude='maps/' ; do
        sshpass -p "{{password}}" rsync -vRr --exclude='.git/' --exclude='maps/' --delete ./ husarion@{{hostname}}:/home/husarion/${PWD##*/}
    done

