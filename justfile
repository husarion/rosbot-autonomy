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
    if ! command -v snap &> /dev/null; then
        echo "Snap is not installed. Please install Snap first and try again."
        echo "sudo apt install snapd"
        exit 1
    fi

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
        INSTALLED_CHANNEL=$(snap info {{snap}} | awk '/tracking:/ {print $2}' | cut -d'/' -f1)
        if [[ "$INSTALLED_CHANNEL" != "$ROS_DISTRO" ]]; then
            echo "Installed channel ($INSTALLED_CHANNEL) does not match ROS_DISTRO ($ROS_DISTRO)."
            read -p "Do you want to reinstall {{snap}} using the $ROS_DISTRO channel? (Y/n): " choice
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

# Compare the DDS configuration between two snaps
[private]
check-ros-transport source_snap target_snap:
    #!/bin/bash
    SOURCE_TRANSPORT=$(sudo snap get {{source_snap}} ros.transport 2>/dev/null)
    TARGET_TRANSPORT=$(sudo snap get {{target_snap}} ros.transport 2>/dev/null)

    # Check if values were retrieved
    if [[ -z "$SOURCE_TRANSPORT" || -z "$TARGET_TRANSPORT" ]]; then
        echo "❌ Failed to get 'ros.transport' value(s):"
        [[ -z "$SOURCE_TRANSPORT" ]] && echo "   - from {{source_snap}} snap"
        [[ -z "$TARGET_TRANSPORT" ]] && echo "   - from {{target_snap}} snap"
        exit 1
    fi

    # Compare values
    if [[ "$TARGET_TRANSPORT" != "$SOURCE_TRANSPORT" ]]; then
        echo "⚠️  Warning: 'ros.transport' values differ between snaps!"
        echo "   - {{source_snap}}: $SOURCE_TRANSPORT"
        echo "   - {{target_snap}}: $TARGET_TRANSPORT"
        echo
        echo "💡 Recommended action: set both snaps to use the same DDS configuration to avoid communication issues."
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
    SNAPS=("husarion-rplidar" "rosbot")

    missing_or_inactive=false

    for SNAP in "${SNAPS[@]}"; do
        STATUS=$(snap services "$SNAP.daemon" 2>/dev/null | awk 'NR>1 {print $3}' | uniq)

        if [[ "$STATUS" == "active" ]]; then
            echo "✅ $SNAP is running."
        elif [[ -z "$STATUS" ]]; then
            echo "⚠️  $SNAP is not installed." >&2
            missing_or_inactive=true
        else
            echo "❌ $SNAP is not running." >&2
            echo "💡 Try: sudo $SNAP.start" >&2
            missing_or_inactive=true
        fi
    done

    if [[ "$missing_or_inactive" == true ]]; then
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
    docker compose -f demo/compose.yaml up

# Start Gazebo simulator with autonomy
start-simulation:
    #!/bin/bash
    xhost +local:docker
    docker compose -f demo/compose.sim.yaml down
    docker compose -f demo/compose.sim.yaml pull
    docker compose -f demo/compose.sim.yaml up

# Start Husarion WebUI for visualization
start-visualization:
    #!/bin/bash
    set -e
    just --quiet ros-snap-distro-validation "husarion-webui"
    just --quiet check-ros-transport "rosbot" "husarion-webui"

    sudo cp demo/foxglove.json /var/snap/husarion-webui/common/foxglove-rosbot-navigation.json
    sudo snap set husarion-webui webui.layout=rosbot-navigation
    sudo husarion-webui.start

    local_ip=$(ip -o -4 addr show | awk '/wlan|wl/ {print $4}' | cut -d/ -f1 | head -n1)
    hostname=$(hostname)
    echo "Access the web interface at:"
    echo "  • Localhost:        http://localhost:8080/ui"
    echo "  • Local network:    http://$local_ip:8080/ui"
    echo "  • Husarnet network: http://$hostname:8080/ui (if Husarnet is set up)"

# Copy repo content to remote host and keep synced
sync hostname="husarion" password="husarion": install-sync-dependencies
    #!/bin/bash
    EXCLUDES="--exclude=.git --exclude=maps --exclude=.docs"
    sshpass -p "{{password}}" rsync -vRr $EXCLUDES --delete ./ husarion@{{hostname}}:/home/husarion/${PWD##*/}
    while inotifywait -r -e modify,create,delete,move ./ --excludei '\.git|maps|\.docs'; do
        sshpass -p "{{password}}" rsync -vRr $EXCLUDES --delete ./ husarion@{{hostname}}:/home/husarion/${PWD##*/}
    done
