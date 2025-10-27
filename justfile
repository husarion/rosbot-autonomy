set dotenv-load # to read ROBOT_NAMESPACE from .env file

[private]
alias hw := start-navigation
[private]
alias sim := start-simulation
[private]
alias vis := start-visualization

[private]
default:
    @just --list --unsorted

# Validate if the ROS version matches the snap version and reinstall if necessary
[private]
ros-snap-distro-validation snap:
    #!/bin/bash
    if ! snap list {{snap}} &> /dev/null; then
        echo "{{snap}} is not installed."
        read -p "Do you want to install {{snap}} for ROS_DISTRO=$ROS_DISTRO? (Y/n): " choice
        case "$choice" in
            [nN]|[nN][oO])
                echo "Installation aborted."
                exit 0
                ;;
            *)
                sudo snap install {{snap}} --channel="$ROS_DISTRO"
                ;;
        esac
    else
        SNAP_DISTRO=$(snap info {{snap}} | awk '/tracking:/ {print $2}' | cut -d'/' -f1)
        if [[ "$SNAP_DISTRO" != "$ROS_DISTRO" ]]; then
            echo "Snap {{snap}} ROS distro ($SNAP_DISTRO) does not match the host ROS distro ($ROS_DISTRO)."
            read -p "Do you want to reinstall {{snap}} snap using the $ROS_DISTRO channel? (Y/n): " choice
            case "$choice" in
                [nN]|[nN][oO])
                    echo "Reinstallation aborted."
                    exit 0
                    ;;
                *)
                    sudo snap remove {{snap}}
                    sudo snap install {{snap}} --channel="$ROS_DISTRO"
                    ;;
            esac
        fi
    fi

# Check if a snap is active
[private]
is_snap_active snap:
    #!/bin/bash
    STATUS=$(snap services "{{snap}}.daemon" 2>/dev/null | awk 'NR==1{for(i=1;i<=NF;i++){if($i=="Current")col=i}} NR>1{print $col}' | uniq)

    if [[ "$STATUS" == "active" ]]; then
        exit 0
    elif [[ -z "$STATUS" ]]; then
        echo "⚠️ {{snap}} is not installed." >&2
        echo "💡 Try: sudo snap install {{snap}} --channel=$ROS_DISTRO" >&2
        exit 1
    else
        echo "⚠️ {{snap}} is not running." >&2
        echo "💡 Try: sudo {{snap}}.start" >&2
        exit 1
    fi

# Compare the DDS configuration between two snaps
[private]
compare-ros-transport source_snap target_snap:
    #!/bin/bash
    SOURCE_DDS=$(sudo snap get {{source_snap}} ros.transport 2>/dev/null)
    TARGET_DDS=$(sudo snap get {{target_snap}} ros.transport 2>/dev/null)

    # Check if values were retrieved
    if [[ -z "$SOURCE_DDS" || -z "$TARGET_DDS" ]]; then
        echo "❌ Failed to get 'ros.transport' value(s):"
        [[ -z "$SOURCE_DDS" ]] && echo "   - from {{source_snap}} snap"
        [[ -z "$TARGET_DDS" ]] && echo "   - from {{target_snap}} snap"
        exit 1
    fi

    # Compare values
    if [[ "$TARGET_DDS" != "$SOURCE_DDS" ]]; then
        echo "⚠️ Warning: 'ros.transport' values differ between snaps!"
        echo "   - {{source_snap}}: $SOURCE_DDS"
        echo "   - {{target_snap}}: $TARGET_DDS"
        echo "💡 Recommended action: Set both snaps to use the same DDS configuration to avoid communication issues."
        echo
        read -p "Do you want to continue anyway? (y/N): " CONFIRM
        if [[ "$CONFIRM" != "y" && "$CONFIRM" != "Y" ]]; then
            echo "❌ Aborted by user."
            exit 1
        fi
    fi

[private]
install-sync-dependencies:
    #!/bin/bash
    if ! command -v rsync &> /dev/null || ! command -v sshpass &> /dev/null || ! command -v inotifywait &> /dev/null; then
        sudo apt-get install -y rsync sshpass inotify-tools
    fi

# Start ROSbot autonomy container
start-navigation:
    #!/bin/bash
    set -e

    just --quiet ros-snap-distro-validation "rosbot"
    just --quiet ros-snap-distro-validation "husarion-rplidar"

    show_warn=false
    just --quiet is_snap_active "rosbot" || show_warn=true
    just --quiet is_snap_active "husarion-rplidar" || show_warn=true
    if [[ "$show_warn" == true ]]; then
        echo
        read -rp "🚨 Some snaps are missing or inactive. Do you still want to start navigation? [y/N]: " CONFIRM
        case "$CONFIRM" in
            [yY]|[yY][eE][sS])
                ;;
            *)
                exit 0
                ;;
        esac
    fi
    just --quiet compare-ros-transport "rosbot" "husarion-rplidar"

    if grep -q "Intel(R) Atom(TM) x5-Z8350" /proc/cpuinfo; then
        echo -e "\e[1;33mWarning: MPPI controller does NOT work on ROSbot 2 PRO (Atom x5-Z8350).\e[0m\n"
        read -p "Do you want to use RPP controller instead? [Y/n]: " choice
        case "$choice" in
            [nN]|[nN][oO])
                echo "Aborted. Please select a compatible controller manually."
                exit 1
                ;;
            *)
                export CONTROLLER=RPP
                echo "Using RPP controller."
                ;;
        esac
    fi

    docker compose -f demo/compose.yaml down
    docker compose -f demo/compose.yaml pull
    docker compose -f demo/compose.yaml up -d

    just start-visualization

# Start Gazebo simulator with autonomy
start-simulation:
    #!/bin/bash
    set -e

    xhost +local:docker
    docker compose -f demo/compose.sim.yaml down
    docker compose -f demo/compose.sim.yaml pull
    docker compose -f demo/compose.sim.yaml up -d

    just start-visualization "true"

# Start Husarion WebUI for visualization
start-visualization sim="false":
    #!/bin/bash
    set -e
    just --quiet ros-snap-distro-validation "husarion-webui"
    if [[ "{{sim}}" == "false" ]]; then
        just --quiet compare-ros-transport "rosbot" "husarion-webui"
    fi

    sudo cp demo/foxglove.json /var/snap/husarion-webui/common/foxglove-rosbot-navigation.json
    sudo snap set husarion-webui webui.layout=rosbot-navigation
    sudo husarion-webui.start

    local_ip=$(ip -o -4 addr show | awk '/wlan|wl/ {print $4}' | cut -d/ -f1 | head -n1)
    hostname=$(hostname)
    echo "Access the web interface at:"
    echo "  • Localhost:        http://localhost:8080/ui"
    echo "  • Local network:    http://$local_ip:8080/ui"
    echo "  • Husarnet network: http://$hostname:8080/ui (if Husarnet is set up)"

# Stop running containers and Husarion WebUI
stop:
    #!/bin/bash
    docker compose -f demo/compose.sim.yaml down > /dev/null 2>&1 || true
    docker compose -f demo/compose.yaml down > /dev/null 2>&1 || true
    sudo husarion-webui.stop

# Show logs from navigation container
logs:
    #!/bin/bash
    docker logs demo-rosbot_navigation-1 -f

# Blocking action which constantly synchronizes changes from host to robot
sync hostname="husarion" password="husarion": install-sync-dependencies
    #!/bin/bash
    EXCLUDES="--exclude=.git --exclude=maps --exclude=.docs"
    sshpass -p "{{password}}" rsync -vRr $EXCLUDES --delete ./ husarion@{{hostname}}:/home/husarion/${PWD##*/}
    while inotifywait -r -e modify,create,delete,move ./ --excludei '\.git|maps|\.docs'; do
        sshpass -p "{{password}}" rsync -vRr $EXCLUDES --delete ./ husarion@{{hostname}}:/home/husarion/${PWD##*/}
    done
