# rosbot-autonomy

Autonomous navigation & mapping for **ROSbot Series** (ROSbot XL, ROSbot 3 / 3 PRO, ROSbot 2R / 2 PRO) with a web user interface powered by Foxglove. Works over the Internet thanks to Husarnet VPN

![autonomy-result](https://github-readme-figures.s3.eu-central-1.amazonaws.com/rosbot/rosbot-autonomy/rosbot-autonomy.webp)

## 🚀 Demo

### 📋 Requirements

1. **ROSbot Platform & ROS Driver**

    This demo is prepared for the **ROSbot Series**. This version is prepared to work with [rosbot](https://snapcraft.io/rosbot) ROS driver snap. To install snap follow the information in snapcraft.

2. **Robot Configuration**

    The demo assumes that the `/scan` topic (`LaserScan` message type) is available.

3. **Just**

    To simplify running commands, we use [just](https://github.com/casey/just). Install it with:

    ```bash
    sudo snap install just
    ```

### 🧭 Navigation

#### Step 1: Environment configuration

Setup environment:

```bash
export ROBOT_MODEL=rosbot # if you want to use ROSbot XL change to 'rosbot_xl'
export SLAM=True # if you have a map you can run navigation without SLAM
```

> [!NOTE]
> Additional arguments are detailed in the [Launch Arguments](#launch-arguments) section.
> MPPI controller is not compatible with **ROSbot 2 PRO**. Please use DWB or RPP controller.

#### Step 2: Run navigation

Run navigation on the **physical robot**:

```bash
just start-navigation
```

Run navigation in **Gazebo simulation**:

```bash
just start-simulation
```

#### Step 3: Control the robot from a Web Browser

1. Install and run husarion-webui

    ```bash
    just start-visualization
    ```

2. Open the your browser on your laptop and navigate to:

    - http://{ip_address}:8080/ui (devices in the same LAN)
    - http://{hostname}:8080/ui (devices in the same Husarnet Network)
