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

[private]
check-husarion-webui:
    #!/bin/bash
    if ! command -v snap &> /dev/null; then
        echo "Snap is not installed. Please install Snap first and try again."
        echo "sudo apt install snapd"
        exit 1
    fi

    if ! snap list husarion-webui &> /dev/null; then
        echo "husarion-webui is not installed."
        read -p "Do you want to install husarion-webui for ROS_DISTRO=$ROS_DISTRO? (y/n): " choice
        case "$choice" in
            y|Y )
                sudo snap install husarion-webui --channel="$ROS_DISTRO"
                ;;
            n|N )
                echo "Installation aborted."
                exit 0
                ;;
            * )
                echo "Invalid input. Please respond with 'y' or 'n'."
                exit 1
                ;;
        esac
    else
        INSTALLED_CHANNEL=$(snap info husarion-webui | awk '/tracking:/ {print $2}' | cut -d'/' -f1)
        echo "husarion-webui is installed (channel: $INSTALLED_CHANNEL)."

        if [[ "$INSTALLED_CHANNEL" != "$ROS_DISTRO" ]]; then
            echo "Installed channel ($INSTALLED_CHANNEL) does not match ROS_DISTRO ($ROS_DISTRO)."
            read -p "Do you want to reinstall husarion-webui with channel=$ROS_DISTRO? (y/n): " choice
            case "$choice" in
                y|Y )
                    sudo snap remove husarion-webui
                    sudo snap install husarion-webui --channel="$ROS_DISTRO"
                    ;;
                n|N )
                    echo "Reinstallation aborted."
                    exit 0
                    ;;
                * )
                    echo "Invalid input. Please respond with 'y' or 'n'."
                    exit 1
                    ;;
            esac
        else
            echo "husarion-webui is up to date and matches ROS_DISTRO=$ROS_DISTRO."
        fi
    fi

_install-rsync:
    #!/bin/bash
    if ! command -v rsync &> /dev/null || ! command -v sshpass &> /dev/null || ! command -v inotifywait &> /dev/null; then
        if [ "$EUID" -ne 0 ]; then
            echo -e "\e[1;33mPlease run as root to install dependencies\e[0m"
            exit
        fi
        sudo apt-get install -y rsync sshpass inotify-tools
    fi

# start ROSbot autonomy container
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

# start Gazebo simulator with autonomy
start-simulation:
    #!/bin/bash
    xhost +local:docker
    docker compose -f demo/compose.sim.yaml down
    docker compose -f demo/compose.sim.yaml pull
    docker compose -f demo/compose.sim.yaml up

start-visualization: check-husarion-webui
    #!/bin/bash
    sudo cp demo/foxglove.json /var/snap/husarion-webui/common/foxglove-rosbot-navigation.json
    sudo snap set husarion-webui webui.layout=rosbot-navigation
    sudo husarion-webui.start

    local_ip=$(ip -o -4 addr show | awk '/wlan|wl/ {print $4}' | cut -d/ -f1 | head -n1)
    hostname=$(hostname)
    echo "Access the web interface at:"
    echo "  • Localhost:        http://localhost:8080/ui"
    echo "  • Local network:    http://$local_ip:8080/ui"
    echo "  • Husarnet network: http://$hostname:8080/ui"

# copy repo content to remote host with 'rsync' and watch for changes
sync hostname="${ROBOT_NAMESPACE}" password="husarion": _install-rsync
    #!/bin/bash
    sshpass -p "{{password}}" rsync -vRr --exclude='.git/' --exclude='maps/' --exclude='.docs' --delete ./ husarion@{{hostname}}:/home/husarion/${PWD##*/}
    while inotifywait -r -e modify,create,delete,move ./ --exclude='.git/' --exclude='maps/' --exclude='.docs' ; do
        sshpass -p "{{password}}" rsync -vRr --exclude='.git/' --exclude='maps/' --exclude='.docs' --delete ./ husarion@{{hostname}}:/home/husarion/${PWD##*/}
    done
