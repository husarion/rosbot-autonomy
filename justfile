set dotenv-load # to read ROBOT_NAMESPACE from .env file

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
            exit 1
        fi
        sudo apt-get install -y rsync sshpass inotify-tools
    fi

_install-yq:
    #!/bin/bash
    if ! command -v /usr/bin/yq &> /dev/null; then
        if [ "$EUID" -ne 0 ]; then
            echo -e "\e[1;33mPlease run as root to install dependencies\e[0m"
            exit 1
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
        echo -e "\e[93mPlease run as root to install Husarnet\e[0m"; \
        exit
    fi
    if ! command -v husarnet > /dev/null; then
        echo "Husarnet is not installed. Installing now..."
        curl https://install.husarnet.com/install.sh | sudo bash
    fi
    husarnet join {{joincode}} {{hostname}}

# flash the proper firmware for STM32 microcontroller in ROSbot 2R / 2 PRO
flash-firmware hostname="${ROBOT_NAMESPACE}" password="husarion": _install-rsync _install-yq
    #!/bin/bash
    if ping -c 1 -W 3 {{hostname}} > /dev/null; then
        image=$(yq .services.rosbot.image compose.yaml)
        RUN_COMMAND="docker ps -q | xargs -r docker stop && docker run --rm -it --privileged $image ros2 run rosbot_utils flash_firmware"

        echo -e "\033[32mFlashing the firmware for STM32 microcontroller in ROSbot\033[0m"
        sshpass -p {{password}} ssh husarion@{{hostname}} -t "$RUN_COMMAND"

    else
        echo -e "\e[93mUnable to reach the device or encountering a network issue. Verify the availability of your device in the Husarnet Network at https://app.husarnet.com/.\e[0m"; \
    fi

# start ROSbot 2R / 2 PRO autonomy containers
start-rosbot:
    #!/bin/bash
    if [[ $USER == "husarion" ]]; then \
        trap 'docker compose down' SIGINT # Remove containers after CTRL+C
        mkdir -m 775 -p maps
        docker compose pull; \
        docker compose up; \
    else \
        echo "This command can be run only on ROSbot 2R / 2 PRO."; \
    fi

# start RViz visualization on PC
start-pc:
    #!/bin/bash
    xhost +local:docker
    trap 'docker compose -f compose.pc.yaml down' SIGINT
    docker compose -f compose.pc.yaml pull
    docker compose -f compose.pc.yaml up

# restart the navigation stack (and SLAM)
restart-nav2:
    #!/bin/bash
    docker compose down navigation
    docker compose up -d navigation

# start Gazebo simulator with autonomy
start-gazebo-sim:
    #!/bin/bash
    xhost +local:docker
    trap 'docker compose  -f compose.sim.gazebo.yaml down' SIGINT
    docker compose -f compose.sim.gazebo.yaml pull
    docker compose -f compose.sim.gazebo.yaml up

# start Webots simulator with autonomy
start-webots-sim:
    #!/bin/bash
    xhost +local:docker
    trap 'docker compose  -f compose.sim.webots.yaml down' SIGINT
    docker compose -f compose.sim.webots.yaml pull
    docker compose -f compose.sim.webots.yaml up

# run teleop_twist_keybaord (host)
run-teleop:
    #!/bin/bash
    export FASTRTPS_DEFAULT_PROFILES_FILE=$(pwd)/shm-only.xml
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r __ns:=/${ROBOT_NAMESPACE}

# run teleop_twist_keybaord (inside rviz2 container)
run-teleop-docker:
    #!/bin/bash
    docker compose -f compose.pc.yaml exec rviz /bin/bash -c "/ros_entrypoint.sh ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r __ns:=/${ROBOT_NAMESPACE}"

# constantly synchronizes changes from host to rosbot
sync hostname="${ROBOT_NAMESPACE}" password="husarion":  _install-rsync
    #!/bin/bash
    mkdir -m 775 -p maps
    sshpass -p "{{password}}" rsync -vRr --exclude='.git/' --exclude='maps/' --exclude='.docs' --delete ./ husarion@{{hostname}}:/home/husarion/${PWD##*/}
    while inotifywait -r -e modify,create,delete,move ./ --exclude='.git/' --exclude='maps/' --exclude='.docs' ; do
        sshpass -p "{{password}}" rsync -vRr --exclude='.git/' --exclude='maps/' --exclude='.docs' --delete ./ husarion@{{hostname}}:/home/husarion/${PWD##*/}
    done

# copy repo to device and run autonomy on rosbot
start-autonomy hostname="${ROBOT_NAMESPACE}" password="husarion": _install-rsync
    #!/bin/bash
    if ping -c 1 -W 3 {{hostname}} > /dev/null; then
        path=/home/husarion/${PWD##*/}
        RUN_COMMAND="cd $path && /bin/bash -c 'docker compose pull && docker compose up'"
        REMOVE_COMMAND="cd $path && /bin/bash -c 'docker compose down'"

        sshpass -p {{password}} rsync -vRr --delete ./ husarion@{{hostname}}:$path > /dev/null
        sshpass -p {{password}} ssh husarion@{{hostname}} -t "$RUN_COMMAND"
        sshpass -p {{password}} ssh husarion@{{hostname}} -t "$REMOVE_COMMAND" > /dev/null 2>&1

    else
        echo -e "\e[93mUnable to reach the device or encountering a network issue. Verify the availability of your device in the Husarnet Network at https://app.husarnet.com/.\e[0m"; \
    fi
