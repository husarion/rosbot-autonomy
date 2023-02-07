# rosbot-slam

A step-by-step guide for the ROSbot 2R/PRO to map an unknown environment and navigate autonomously within it.

## Clonning the repo

This repository contains the Docker Compose setup for both PC and ROSbot. You can clone it to both PC and ROSbot, or use the `./sync_with_rosbot.sh` script to clone it to your PC and keep it synchronized with the robot

```bash
git clone https://github.com/husarion/rosbot-slam
cd rosbot-slam 
export ROSBOT_ADDR=10.5.10.123 # Replace with your own ROSbot's IP or Husarnet hostname
./sync_with_rosbot.sh $ROSBOT_ADDR
```

## Flashing the ROSbot Firmware

Execute in the ROSbot's shell:

```bash
docker stop rosbot microros || true && docker run \
--rm -it --privileged \
husarion/rosbot:humble \
/flash-firmware.py /root/firmware.bin
```

## Choosing the Network (DDS) Config

Edit `net.env` file and uncomment on of the configs:

```bash
# =======================================
# Network config options (uncomment one)
# =======================================

# 1. Fast DDS + LAN
RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# 2. Cyclone DDS + LAN
# RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 3. Fast DDS + VPN
# RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# FASTRTPS_DEFAULT_PROFILES_FILE=/husarnet-fastdds.xml

# 4. Cyclone DDS + VPN
# RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# CYCLONEDDS_URI=file:///husarnet-cyclonedds.xml
```

If you choose to use the VPN option, both your ROSbot and laptop must be connected to the same Husarnet network. Follow the guide [here](https://husarion.com/manuals/rosbot/remote-access/).

## Verifying Hardware Configuration

To ensure proper hardware configuration, review the content of the `.env` file and select the appropriate LIDAR baudrate and serial port. The hardware configuration is defined as follows:

```bash
# =======================================
# Hardware config
# =======================================
LIDAR_SERIAL=/dev/ttyUSB0

# for RPLIDAR A2M8 (red circle around the sensor):
# LIDAR_BAUDRATE=115200
# for RPLIDAR A2M12 and A3 (violet circle around the sensor):
LIDAR_BAUDRATE=256000
```

The default options should be suitable.

## Switching Between **SLAM** and **Localization Only** Mode

Pull the Docker images defined in `compose.yaml`:

```bash
docker compose pull
```

### SLAM Mode

To launch the mapping mode with manual control on the ROSbot, run:

```bash
SLAM_MODE=slam docker compose up -d
```

### Localization & navigation

To allow the ROSbot to navigate autonomously on a previously created map, run:

```bash
SLAM_MODE=localization docker compose up -d
```

> **Note:** You do not need to stop the containers to switch between modes.

## PC

To visualize the map and control the rebot, launch this part on your PC:

```bash
xhost +local:docker && \
docker compose -f compose.pc.yaml up
```

## Simulation (webots)

> **Prerequisites**
>
> The `compose.sim.webots.yaml` file uses NVIDIA Container Runtime. Make sure you have NVIDIA GPU and the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) installed.

Start the containers in a new terminal:

```bash
xhost +local:docker && \
docker compose -f compose.sim.webots.yaml up
```

And in the second terminal start `telop-twist-keyboard` for manual ROSbot 2R control:

```bash
docker exec -it rviz bash
```

And inside the running container shell execute:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```