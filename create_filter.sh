#!/bin/bash

# Exit if no arguments are provided
if [ "$#" -lt 1 ]; then
    echo "Usage: $0 <topic_name1> <topic_name2> ... <topic_nameN>"
    exit 1
fi

echo "allowlist:"

# Loop over each topic provided as argument
for TOPIC in "$@"; do
    # Fetch details using 'ros2 topic info'
    INFO=$(ros2 topic info -v $TOPIC)

    # Extract type of the message
    TYPE=$(echo "$INFO" | grep "Type:" | awk '{print $2}')

    # Convert ROS2 message type to DDS type dynamically
    PACKAGE_NAME=$(echo $TYPE | cut -d'/' -f1)
    MSG_NAME=$(echo $TYPE | cut -d'/' -f3)

    # Construct DDS_TYPE using the parsed package and message names
    DDS_TYPE="${PACKAGE_NAME}::msg::dds_::${MSG_NAME}_"

    # Check if conversion is successful
    if [ -z "$DDS_TYPE" ]; then
        echo "Error during type conversion for topic $TOPIC."
        continue
    fi

    # Extract QoS settings
    RELIABILITY=$(echo "$INFO" | grep "Reliability:" | awk '{print $2}')
    DURABILITY=$(echo "$INFO" | grep "Durability:" | awk '{print $2}')

    # Convert QoS settings to the desired format
    RELIABILITY_BOOL="false"
    [ "$RELIABILITY" == "RELIABLE" ] && RELIABILITY_BOOL="true"
    DURABILITY_BOOL="false"
    [ "$DURABILITY" == "TRANSIENT_LOCAL" ] && DURABILITY_BOOL="true"

    # Print entry for current topic with QoS settings
    echo "  - name: \"rt$TOPIC\""
    echo "    type: \"$DDS_TYPE\""
    echo "    qos:"
    echo "      reliability: $RELIABILITY_BOOL"
    echo "      durability: $DURABILITY_BOOL"

done

echo "blocklist: []"
echo "builtin-topics: []"