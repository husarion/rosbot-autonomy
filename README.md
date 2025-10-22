# rosbot-autonomy

Autonomous navigation & mapping for **ROSbot Series** (ROSbot XL, ROSbot 3 / 3 PRO, ROSbot 2R / 2 PRO) with a web user interface powered by Foxglove. Works over the Internet thanks to Husarnet VPN

![autonomy-result](https://github-readme-figures.s3.eu-central-1.amazonaws.com/rosbot/rosbot-autonomy/rosbot-autonomy.webp)

## 🚀 Demo

> [!NOTE]
> This repo provides only quick demo guideline. The source code can be found in [rosbot_autonomy_ros](https://github.com/husarion/rosbot_autonomy_ros).

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

4. **DDS**

    The default configuration starts [FastDDS - UDP](demo/dds-config-udp.xml) configuration. All snap should share the same DDS configuration.

### 🧭 Navigation

#### Step 1: Environment configuration

Setup environment variable in `demo/.env`.

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
