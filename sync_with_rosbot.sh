#!/bin/bash

# Check if sshpass is installed
if ! command -v sshpass &> /dev/null; then
    echo "sshpass is not installed. Installing it..."
    sudo apt-get install -y sshpass || { echo "Failed to install sshpass. Exiting."; exit 1; }
fi

# Check if rsync is installed
if ! command -v rsync &> /dev/null; then
    echo "rsync is not installed. Installing it..."
    sudo apt-get install -y rsync || { echo "Failed to install rsync. Exiting."; exit 1; }
fi

# If your ROSbot's IP address is provided as an argument, execute synchronization
if [ $# -eq 1 ]; then
    echo "Syncing with ROSbot at IP: $1"
    sshpass -p "husarion" rsync -vRr --delete ./ husarion@$1:/home/husarion/${PWD##*/}

    while inotifywait -r -e modify,create,delete,move ./ ; do
        sshpass -p "husarion" rsync -vRr --delete ./ husarion@$1:/home/husarion/${PWD##*/}
    done
else
    echo "Please provide the ROSbot's Husarnet DDS device name or IP address as an argument."
fi
