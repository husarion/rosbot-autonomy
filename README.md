# rosbot-slam

Create a map of the unknow environment with ROSbot 2 PRO or ROSbot 2R controlled in the LAN network or [over the Internet](https://husarion.com/manuals/rosbot/remote-access/).

### PC

Clone this repository:

```
git clone https://github.com/husarion/rosbot-slam.git
```

**Connect a [gamepad](https://husarion.com/tutorials/other-tutorials/rosbot-gamepad/) to USB port of your PC/laptop** (the steering without the gamepad will also be described as an alternative).

Check your **hardware configs** in the `.env` file:

```bash
# =======================================
# Hardware config
# =======================================
LIDAR_SERIAL=/dev/ttyUSB0

# for RPLIDAR A2M8 (red circle around the sensor):
# LIDAR_BAUDRATE=115200
# for RPLIDAR A2M12 and A3 (violet circle around the sensor):
LIDAR_BAUDRATE=256000

# =======================================
# SLAM config (select one)
# =======================================
SLAM_MODE=slam
# SLAM_MODE=localization
```

**Notes:**
- Usually RPLIDAR is listed under `/dev/ttyUSB0`, but verify it with `ls -la /dev/ttyUSB*` command.
- If you have RPLIDAR A3 or A2M12 (with violet border around the lenses) set: `LIDAR_BAUDRATE=256000`. Otherwise (for older A2 LIDARs): `LIDAR_BAUDRATE=115200`.

Select your **network configuration** in the `net.env` file:

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

**Notes:**
- Husarion's docker images utilize the [husarnet-dds binary](https://github.com/husarnet/husarnet-dds) to create ROS 2 DDS configuration files for Husarnet VPN.

To sync workspace with ROSbot (works with the newest [OS images](https://husarion.com/manuals/rosbot/operating-system-reinstallation/)) execute (in `rosbot-mapping` directory):

```bash
./sync_with_rosbot.sh <ROSbot_ip>
```

Open a new terminal on PC and run RViz depending on whether you [have](https://github.com/husarion/rosbot-mapping#option-1-with-the-gamepad-connected-to-pc) a gamepad or [not](https://github.com/husarion/rosbot-mapping#option-2-without-the-gamepad). Then you will be able to [control the ROSbot](https://husarion.com/tutorials/other-tutorials/rosbot-gamepad/) and create map of the environment. The map is being saved automatically in the `rosbot-mapping/maps` folder.

#### Option 1: With the gamepad connected to PC

```bash
xhost +local:docker && \
docker compose -f compose.pc.yaml up
```

#### Option 2: Without the gamepad

```bash
xhost +local:docker && \
docker compose -f compose.pc.yaml up -d map-saver rviz
```

Then enter the running `rviz` container:

```bash
docker exec -it rosbot-mapping-rviz-1 bash
```

Now, to teleoperate the ROSbot with your keyboard, execute:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## ROSbot

> **Firmware version**
>
> Before running the project, make sure you have the correct version of a firmware flashed on your robot.
>
> Firmware flashing command (run in the ROSbot's terminal)
>
> ```
> docker stop rosbot microros || true && docker run \
> --rm -it --privileged \
> husarion/rosbot:humble \
> /flash-firmware.py /root/firmware.bin
> ```

In the ROSbot's terminal execute (in `/home/husarion/rosbot-slam` directory):

```bash
docker compose -f compose.rosbot.yaml up
```

## Quick Start (Webots simulation)

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
