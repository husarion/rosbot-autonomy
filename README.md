# rosbot-autonomy

A step-by-step guide for the ROSbot 2R/PRO to map an unknown environment and navigate autonomously within it from RViz. Works over the Internet thanks to Husarnet VPN

You can test the robot's autonomy on two branches:

- [**ros2router**](https://github.com/husarion/rosbot-autonomy/) (rviz2)
- [**foxglove**](https://github.com/husarion/rosbot-autonomy/tree/foxglove)

## Quick start

> [!NOTE]
> To simplify the execution of this project, we are utilizing [just](https://github.com/casey/just).
>
> Install it with:
>
> ```bash
> wget -qO - 'https://proget.makedeb.org/debian-feeds/prebuilt-mpr.pub' | gpg --dearmor | sudo tee /usr/share/keyrings/prebuilt-mpr-archive-keyring.gpg 1> /dev/null
> echo "deb [arch=all,$(dpkg --print-architecture) signed-by=/usr/share/keyrings/prebuilt-mpr-archive-keyring.gpg] https://proget.makedeb.org prebuilt-mpr $(lsb_release -cs)" | sudo tee /etc/apt/sources.list.d/prebuilt-mpr.list
> sudo apt update
> sudo apt install just
> ```

To see all available commands just run `just`:

```bash
husarion@rosbot2r:~/rosbot-telepresence$ just
Available recipes:
    connect-husarnet joincode hostname # connect to Husarnet VPN network
    flash-firmware    # flash the proper firmware for STM32 microcontroller in ROSbot 2R / 2 PRO
    start-rosbot      # start ROSbot 2R / 2 PRO autonomy containers
    start-gazebo-sim  # start Gazebo simulator with autonomy
    start-webots-sim  # start Webots simulator with autonomy
    run-teleop        # run teleop_twist_keybaord (host)
    run-teleop-docker # run teleop_twist_keybaord (inside rviz2 container)
    sync hostname password="husarion" # copy repo content to remote host with 'rsync' and watch for changes
```

### ⬇️ Step 1: Clone repository

Go to the directory in which you want to save the project and execute the following commands:

```bash
git clone https://github.com/husarion/rosbot-autonomy
cd rosbot-autonomy
```

### 🌎 Step 2: Connecting ROSbot and Laptop over VPN

Ensure that both ROSbot 2R and your laptop linked to the same Husarnet VPN network. If they are not follow these steps:

1. Setup a free account at [app.husarnet.com](https://app.husarnet.com/), create a new Husarnet network, click the **[Add element]** button and copy the code from the **Join Code** tab.
2. Connect your laptop to the Husarnet network.

   ```bash
   export JOINCODE=<paste-join-code-here>
   just connect-husarnet $JOINCODE my-laptop
   ```

3. Connect your ROSbot and add ROSbot to the Husarnet network.

   ```bash
   export JOINCODE=<paste-join-code-here>
   just connect-husarnet $JOINCODE rosbot2r
   ```

> [!NOTE]
> `rosbot2r` is the robot name that will be used to connect to the robot. This name is related to the robot's namespace.

### 📡 Step 3: Sync

This repository contains the Docker Compose setup for both PC and ROSbot 2, 2R and 2 PRO. You can clone it to both PC and ROSbot 2, 2R and 2 PRO, or use the `just sync` script to clone it to your PC and keep it synchronized with the robot

```bash
just sync rosbot2r
```

> [!NOTE]
> This `just sync` script locks the terminal and synchronizes online all changes made locally on the robot. `rosbot2r` is the name of device set in Husarnet.

### 🔧 Step 4: Verifying User Configuration

To ensure proper user configuration, review the content of the `.env` file and select the appropriate configuration (the default options should be suitable).

- **`LIDAR_BAUDRATE`** - depend on mounted LiDAR
- **`MECANUM`** - wheel type
- **`SLAM`** - choose between mapping and localization modes
- **`SAVE_MAP_PERIOD`** - period of time for autosave map (set `0` to disable)
- **`CONTROLLER`** - choose the navigation controller type
- **`ROBOT_NAMESPACE`** - type your ROSbot device name the same as in Husarnet.

> [!IMPORTANT]
> The value of the `ROBOT_NAMESPACE` parameter in the `.env` file should be the same as the name of the Husarnet device.

### 🤖 Step 5: Running Autonomy

Below are three options for starting autonomy:

- on a physical ROSbot
- in the Gazebo simulation
- in Webots simulation

> [!IMPORTANT]
> To run `Gazebo` or `Webots` Simulators you have to use computer with NVIDIA GPU and the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) installed.

---

#### I. ROSbot 2R and 2 PRO

To enable autonomy on the robot, it is necessary:

- starting autonomy on ROSbot
- launching visualization on PC

##### ROSbot Actions

1. Connect to the ROSbot.

   ```bash
   ssh husarion@rosbot2r
   cd rosbot-autonomy
   ```

   > [!NOTE]
   > `rosbot2r` is the name of device set in Husarnet.

2. Flashing the ROSbot's Firmware.

   To flash the Micro-ROS based firmware for STM32F4 microcontroller responsible for low-level functionalities of ROSbot 2, 2R and 2 PRO, execute in the ROSbot's shell:

   ```bash
   just flash-firmware
   ```

3. Running autonomy on ROSbot.

   ```bash
   just start-rosbot
   ```

##### PC

Open the **Google Chrome** browser on your laptop and navigate to:

http://rosbot2r:8080/ui

---

#### II. Gazebo Simulation

1. To start Gazebo simulator run:

   ```bash
   just start-gazebo-sim
   ```

2. Then open the **Google Chrome** browser on your laptop and navigate to:

   http://localhost:8080/ui

---

#### III. Webots Simulation

1. To start Webots simulator run:

   ```bash
   just start-webots-sim
   ```

2. Then open the **Google Chrome** browser on your laptop and navigate to:

   http://localhost:8080/ui

---

### Result

To instruct the robot to autonomously explore new areas and create a map (in "slam" mode) of **[2D Goal Pose]** in RViz. When `SLAM` is off, you can indicate the robot's current position by **[2D Pose Estimate]** button.

![autonomy-result](.docs/autonomy-result.gif)

> [!NOTE]
> Due to efficiency and official manufacturer support, it is recommended to use `foxglove-websocket`. When using `rosbridge-websocket`, it is necessary to edit `Custom Layers` to visualize the robot mesh.
