# rosbot-autonomy

Autonomous navigation & mapping for ROSbot 2R / 2 PRO with a web user interface powered by Foxglove. Works over the Internet thanks to Husarnet VPN

![autonomy-result](https://github-readme-figures.s3.eu-central-1.amazonaws.com/rosbot/rosbot-autonomy/autonomy-result-foxglove.gif)

> [!NOTE]
> There are two setups on two separate branchers available
> | branch name | description |
> | - | - |
> | [**ros2router**](https://github.com/husarion/rosbot-autonomy/) | Running ROS 2 containers on ROSbot and on PC with the interface in RViz |
> | [**foxglove**](https://github.com/husarion/rosbot-autonomy/tree/foxglove) | Running ROS 2 containers only on ROSbot with a web user interface powered by Foxglove |

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
    flash-firmware     # flash the proper firmware for STM32 microcontroller in ROSbot 2R / 2 PRO
    start-rosbot       # start ROSbot 2R / 2 PRO autonomy containers
    start-gazebo-sim   # start the Gazebo simulation
    start-webots-sim   # start the Webots simulation
    restart-navigation # Restart the Nav2 container
    sync hostname password="husarion" # Copy repo content to remote host with 'rsync' and watch for changes
```

### 🌎 Step 1: Connecting ROSbot and Laptop over VPN

Ensure that both ROSbot 2R (or ROSbot 2 PRO) and your laptop are linked to the same Husarnet VPN network. If they are not follow these steps:

1. Setup a free account at [app.husarnet.com](https://app.husarnet.com/), create a new Husarnet network, click the **[Add element]** button and copy the code from the **Join Code** tab.
2. Run in the linux terminal on your PC:

   ```bash
   cd rosbot-autonomy/ # remember to run all "just" commands in the repo root folder
   export JOINCODE=<PASTE_YOUR_JOIN_CODE_HERE>
   just connect-husarnet $JOINCODE my-laptop
   ```

3. Run in the linux terminal of your ROSbot:

   ```bash
   export JOINCODE=<PASTE_YOUR_JOIN_CODE_HERE>
   sudo husarnet join $JOINCODE rosbot2r
   ```

   > note that `rosbot2r` is a default ROSbot hostname used in this project

### 📡 Step 2: Sync

This repository contains the Docker Compose setup for ROSbot 2R and 2 PRO. You can clone it to both PC and ROSbot, or use the `just sync` script to clone it to your PC and keep it synchronized with the robot

```bash
just sync rosbot2r
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

### 🤖 Step 4: Running Navigation & Mapping

1. Connect to the ROSbot.

   ```bash
   ssh husarion@rosbot2r
   cd rosbot-autonomy
   ```

> [!NOTE]
> `rosbot2r` is the name of device set in Husarnet.

2. Flashing the ROSbot's firmware.

   To flash the Micro-ROS based firmware for STM32F4 microcontroller responsible for low-level functionalities of ROSbot 2, 2R and 2 PRO, execute in the ROSbot's shell:

   ```bash
   just flash-firmware
   ```

3. Running autonomy on ROSbot.

   ```bash
   just start-rosbot
   ```

### 🚗 Step 5: Control the ROSbot from a Web Browser

Open the **Google Chrome** browser on your laptop and navigate to:

http://rosbot2r:8080/ui

> [!NOTE]
> `rosbot2r` is the name of device set in Husarnet.

---

## Simulation

> [!IMPORTANT]
> To run `Gazebo` or `Webots` Simulators you have to use computer with NVIDIA GPU and the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) installed.

If you don't have a physical ROSbot 2R / 2 PRO you can run this project in a simulation environment.

### Gazebo

1. To start Gazebo simulation run:

   ```bash
   just start-gazebo-sim
   ```

2. Then open the **Google Chrome** browser on your laptop and navigate to: http://localhost:8080/ui

### Webots

1. To start Webots simulation run:

   ```bash
   just start-webots-sim
   ```

2. Then open the **Google Chrome** browser on your laptop and navigate to: http://localhost:8080/ui

---

> [!NOTE]
> Due to efficiency and official manufacturer support, it is recommended to use `foxglove-websocket`. When using `rosbridge-websocket`, it is necessary to edit `Custom Layers` to visualize the robot mesh.
