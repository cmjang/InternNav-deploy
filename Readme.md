# Unitree Go2 / Go2W / B2 — Edge Deployment Guide

![ROS 2](https://img.shields.io/badge/ROS2-Supported-blue)
![Python-3.8+](https://img.shields.io/badge/Python-3.8%2B-yellowgreen)

简体中文: [点此查看](./Readme_zh.md)

## Overview

This guide documents the practical steps to deploy perception and motion-control components on Unitree robots (Go2 / Go2W / B2) using the InternNav project (https://github.com/InternRobotics/InternNav). The focus is on edge-side (robot-side) deployment and debugging. For server-side setup and model serving, consult the InternNav documentation.

Compatibility and scope:

- Target hardware: Unitree Go2 (onboard compute box) and compatible edge devices
- Recommended OS: Ubuntu 20.04 LTS (tested with ROS 2 Foxy)
- Python: 3.8+
- Camera: Intel RealSense series (recommended D455)

---

## 1. Hardware Preparation

1. Update the robot firmware to the latest version to ensure compatibility and stability.
2. Set the robot's motion mode to **AI mode** via the Unitree mobile app — this reduces torso motion and improves visual stability.
3. Connection options:
   - Onboard compute box: uses the robot's internal network (no additional hardware required).
   - External compute (NUC / Jetson): connect via Ethernet. Default robot subnet: `192.168.123.x`. See Unitree SDK2 docs for interface setup.

### 1.1 Camera mounting

- Recommended model: Intel RealSense D455 (global shutter reduces motion blur).
- Use the provided 3D-print mounts for better forward ground view: https://github.com/InternRobotics/InternNav/tree/main/assets/3d_printing_files
- Use a USB 3.2 Gen 1 (or better) data cable and plug into the front-panel Type-C port of the compute box to avoid bandwidth issues.

---

## 2. Software Environment

### 2.1 Install Unitree SDK2 (Python)

```bash
cd ~
sudo apt update
sudo apt install -y python3-pip git
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd unitree_sdk2_python
pip3 install -e .
```

### 2.2 Intel RealSense

Install the librealsense system packages and the Python bindings. See the official docs for Jetson-specific steps.

```bash
# Add RealSense key and repo (example)
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
sudo apt-get install -y librealsense2-utils librealsense2-dev

# Python binding
pip3 install pyrealsense2
```

### 2.3 InternNav client environment

```bash
git clone https://github.com/InternRobotics/InternNav.git
cd InternNav
pip3 install numpy requests Pillow casadi
```

### 2.4 CycloneDDS (if required)

If you encounter "Could not locate cyclonedds" when installing SDKs, compile CycloneDDS and set `CYCLONEDDS_HOME`:

```bash
cd ~
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x
cd cyclonedds
mkdir build install && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../install
cmake --build . --target install

# Use absolute path on export
export CYCLONEDDS_HOME="/home/<user>/cyclonedds/install"
cd ~/unitree_sdk2_python
pip3 install -e .
```

---

## 3. Repository Layout (onboard)

Key modules in `InternNav/onboard`:

- `start_robot.sh` — single script to start driver, ROS 2 bridge, and camera node; includes simple supervision and configurable network interface.
- `unitree_driver.py` — low-level UDP driver (port 8899) wrapping Unitree SDK calls.
- `ros_bridge.py` — ROS 2 bridge publishing `/odom_bridge` and forwarding `/cmd_vel_bridge` to the driver.
- `realsense_publisher.py` — publishes RGB and aligned depth (640×480 @30fps) and implements auto-reconnect logic.

---

## 4. Start-up & Quick Start

1. Make the launcher executable:

```bash
cd InternNav/onboard
chmod +x start_robot.sh
```

2. Edit network interface in `start_robot.sh` if your interface is not `eth0`:

```bash
ROBOT_IF="eth0"  # change to enp3s0, wlan0, etc. as needed
```

3. Start the system:

```bash
./start_robot.sh
```

4. Start the navigation client (example):

Terminal 1 (robot):

```bash
cd InternNav/onboard
./start_robot.sh
```

Terminal 2 (onboard or remote client):

```bash
cd InternNav
python3 scripts/realworld/http_internvla_client.py
```

If running a local inference server:

```bash
python3 scripts/realworld/http_internvla_server.py
```

---

## 5. ROS 2 Topics & Visualization

Subscribe with RViz2 to validate topics:

| Topic | Type | Description |
|---|---:|---|
| `/camera/camera/color/image_raw` | `sensor_msgs/Image` | RGB image |
| `/camera/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | Aligned depth image |
| `/odom_bridge` | `nav_msgs/Odometry` | Robot odometry |

---

## 6. Troubleshooting

- Camera not detected: check USB cable (USB 3.2 Gen1), ensure front Type-C port, run `rs-enumerate-devices`.
- Network issues: verify interface with `ifconfig` / `ip addr`, `ping 192.168.123.161`, and ensure UDP 8899 is reachable.
- ROS 2 no topics: check `echo $ROS_DOMAIN_ID` and ensure same `ROS_DOMAIN_ID` across devices; `ros2 topic list` to inspect.

---

## 7. References

- InternNav docs: https://internrobotics.github.io/user_guide/internnav/
- InternNav repo: https://github.com/InternRobotics/InternNav/
- Unitree SDK2: https://github.com/unitreerobotics/unitree_sdk2
- Intel RealSense: https://github.com/IntelRealSense/librealsense
