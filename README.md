# rosbot-autonomy

Autonomous navigation & mapping for ROSbot 2R / 2 PRO with RViz interface running on remote PC. Works over the Internet thanks to Husarnet VPN

![autonomy-result](https://github-readme-figures.s3.eu-central-1.amazonaws.com/rosbot/rosbot-autonomy/autonomy-result-rviz.gif)

> [!NOTE]
> There are two setups on two separate branchers available
> | branch name | description |
> | - | - |
> | [**ros2router**](https://github.com/husarion/rosbot-autonomy/) | Running ROS 2 containers on ROSbot and on PC with the interface in RViz |
> | [**foxglove**](https://github.com/husarion/rosbot-autonomy/tree/foxglove) | Running ROS 2 containers only on ROSbot with a web user interface powered by Foxglove |

## 🛍️ Necessary Hardware

For the execution of this project **[ROSbot 2R or ROSbot 2 PRO](https://husarion.com/manuals/rosbot/)** is required. 

You can find it at [our online store](https://store.husarion.com/collections/robots/products/rosbot).

## Quick start (Physical ROSbot)

> [!NOTE]
> To simplify the execution of this project, we are utilizing [just](https://github.com/casey/just).
>
> Install it with:
>
> ```bash
> curl --proto '=https' --tlsv1.2 -sSf https://just.systems/install.sh | sudo bash -s -- --to /usr/bin
> ```

To see all available commands just run `just`:

```bash
husarion@rosbot2r:~/rosbot-autonomy$ just
Available recipes:
    connect-husarnet joincode hostname # connect to Husarnet VPN network
    flash-firmware    # flash the proper firmware for STM32 microcontroller in ROSbot 2R / 2 PRO
    start-rosbot      # start ROSbot 2R / 2 PRO autonomy containers
    start-pc          # start RViz visualization on PC
    restart-nav2      # restart the navigation stack (and SLAM)
    start-gazebo-sim  # start Gazebo simulator with autonomy
    start-webots-sim  # start Webots simulator with autonomy
    run-teleop        # run teleop_twist_keybaord (host)
    run-teleop-docker # run teleop_twist_keybaord (inside rviz2 container)
    sync hostname="${ROBOT_NAMESPACE}" password="husarion" # copy repo content to remote host with 'rsync' and watch for changes
```

### 🌎 Step 1: Connecting ROSbot and Laptop over VPN

Ensure that both ROSbot 2R (or ROSbot 2 PRO) and your laptop are linked to the same Husarnet VPN network. If they are not follow these steps:

1. Setup a free account at [app.husarnet.com](https://app.husarnet.com/), create a new Husarnet network, click the **[Add element]** button and copy the code from the **Join Code** tab.
2. Run in the linux terminal on your PC:
   ```bash
   cd rosbot-telepresence/ # remember to run all "just" commands in the repo root folder
   export JOINCODE=<PASTE_YOUR_JOIN_CODE_HERE>
   just connect-husarnet $JOINCODE my-laptop
   ```
3. Run in the linux terminal of your ROSbot:
   ```bash
   export JOINCODE=<PASTE_YOUR_JOIN_CODE_HERE>
   sudo husarnet join $JOINCODE rosbot2r
   ```
   > [!IMPORTANT]
   > note that `rosbot2r` is a default ROSbot hostname used in this project. If you want to change it, edit the `.env` file and change the line:
   > ```bash
   > ROBOT_NAMESPACE=rosbot2r
   > ```

### 📡 Step 2: Sync

Copy the local changes (on PC) to the remote ROSbot

```bash
just sync rosbot2r # or a different ROSbot hostname you used in Step 1 p.3
```

> [!NOTE]
> This `just sync` script locks the terminal and synchronizes online all changes made locally on the robot. `rosbot2r` is the name of device set in Husarnet.

### 🔧 Step 3: Verifying User Configuration

To ensure proper user configuration, review the content of the `.env` file and select the appropriate configuration (the default options should be suitable).

- **`LIDAR_BAUDRATE`** - depend on mounted LiDAR,
- **`MECANUM`** - wheel type,
- **`SLAM`** - choose between mapping and localization modes,
- **`SAVE_MAP_PERIOD`** - period of time for autosave map (set `0` to disable),
- **`CONTROLLER`** - choose the navigation controller type,
- **`ROBOT_NAMESPACE`** - type your ROSbot device name the same as in Husarnet.

> [!IMPORTANT]
> The value of the `ROBOT_NAMESPACE` parameter in the `.env` file should be the same as the name of the Husarnet device.

### 🤖 Step 4: Running Navigation & Mapping

To enable autonomy on the robot, it is necessary:

- starting autonomy on ROSbot
- launching visualization on PC

#### ROSbot

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

#### PC

To initiate a user interface and navigation stack based on RViz, execute below command on your PC:

```bash
just start-pc
```

### 🚗 Step 5: Control the ROSbot from RViz

To instruct the robot to autonomously explore new areas and create a map (in "slam" mode) of **[2D Goal Pose]** in RViz. When `SLAM` is off, you can indicate the robot's current position by **[2D Pose Estimate]** button.

![autonomy-result](https://github-readme-figures.s3.eu-central-1.amazonaws.com/rosbot/rosbot-autonomy/autonomy-result-rviz.gif)

------

## Simulation

> [!IMPORTANT]
> To run `Gazebo` or `Webots` Simulators you have to use computer with NVIDIA GPU and the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) installed.

If you don't have a physical ROSbot 2R / 2 PRO you can run this project in a simulation.

![Gazebo](https://github-readme-figures.s3.eu-central-1.amazonaws.com/rosbot/rosbot-autonomy/gazebo-rviz.png)

### Gazebo

To start Gazebo simulator run:

```bash
just start-gazebo-sim
```

### Webots

To start Webots simulator run:

```bash
just start-webots-sim
```
