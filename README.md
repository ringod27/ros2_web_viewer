# ROS2 Web Viewer

## Prerequisites

- [ROS 2 Humble](https://docs.ros.org/en/humble/index.html) or [RoboStack](https://robostack.github.io/GettingStarted.html)
- Python (recommended: use a virtual environment)
- Node.js (for the frontend UI)

## Setup Instructions

### 1. Install ROS or RoboStack

- Follow the official installation guides:
  - [ROS 2 Humble](https://docs.ros.org/en/humble/index.html)
  - [RoboStack (alternative)](https://robostack.github.io/GettingStarted.html)

### 2. If using RoboStack: Activate the ROS environment

- Activate your ROS 2 or RoboStack environment, then install the MCAP storage plugin:

```bash
mamba install ros-humble-rosbag2-storage-mcap
```

## Launching the WebSocket

- Follow these steps to build and launch the WebSocket bridge node:

### 1. Activate env and go to websocket_bridge:

```bash
cd ~/websocket_bridge
```

### 2. Run scripts below:

```bash
colcon build --packages-select websocket_bridge

source install/setup.zsh

ros2 launch websocket_bridge bridge.launch.py
```

### 3. If error encountered, manually install pip dependencies

```bash
pip install -r requirements.txt 
```

## Install and run web application

### 1. Go to ros2_web_viewer_ui:

```bash
cd ~/ros2_web_viewer_ui
```

### 2. Install node packages:

```bash
npm install
```

### 3. Run the application and browse http://localhost:5173/ or http://127.0.0.1:5173/

```bash
npm run dev
```


## Data Format 

### 1. ros2 / mcap file

### 2. expected topics:

- /sensing/lidar/hesai/pandar → Lidar data (Hesai)
- /ublox_gps_node/fix→ GNSS data (RTK)
- /tf → Static transform tree for spatial alignment




