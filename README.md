# rosbot-autonomy

Autonomous navigation & mapping for ROSbot 3 / 3 PRO and ROSbot XL with a web user interface powered by Foxglove. This project runs [husarion/rosbot-autonomy](https://hub.docker.com/r/husarion/rosbot-autonomy) Docker image that integrates Nav2, SLAM toolbox and ready to use configs for ROSbot 3 / 3 PRO and ROSbot XL.

![autonomy-result](https://github-readme-figures.s3.eu-central-1.amazonaws.com/rosbot/rosbot-autonomy/rosbot-autonomy.webp)

This demo can also work over the internet with Husarnet VPN. Simply add your devices to the same Husarnet network.

## 🚀 Demo

> [!NOTE]
> This repo provides only quick demo guideline. The source code can be found in [rosbot_autonomy_ros](https://github.com/husarion/rosbot_autonomy_ros).

### 📋 Requirements

1. **ROSbot Platform & ROS Driver**

    - This demo is prepared for the **ROSbot Series**. This version is prepared to work with [rosbot](https://snapcraft.io/rosbot) ROS driver snap. To install snap follow the information in snapcraft.
    - The demo assumes that the `/scan` topic (`LaserScan` message type) is available. By default Rosbot use RPlidar device with [husarion-rplidar](https://snapcraft.io/husarion-rplidar) ROS driver

2. **Just**

    To simplify running commands, we use [just](https://github.com/casey/just). Install it with:

    ```bash
    sudo snap install just
    ```

### 🧭 Navigation

#### Step 1: Environment configuration

Setup environment variable in `demo/.env` for physical robot or `demo/sim.env` for simulation.

#### Step 2: Run navigation

Run navigation on the **physical robot**:

```bash
just start-navigation
```

> [!TIP]
> Scripts in `justfile` automatically, loads DDS configuration from [rosbot](https://snapcraft.io/rosbot) snap.

Run navigation in **Gazebo simulation**:

```bash
just start-simulation
```

> Stop demonstration using:
>
> ```bash
> just stop
> ```
