
# Motion Capture to PX4 Odometry

## Overview
This ROS2 package enables VICON motion capture data streaming for use with PX4 visual odometry. It connects to a VICON server, retrieves motion capture data, and publishes it to a ROS2 topic. The data is then remapped to a format compatible with PX4, making it suitable for drone control via visual odometry.

## Functionality
The provided launch file performs the following operations:
1. Streams motion capture data from a VICON system using the `mocap_vicon_client` package.
2. Remaps and publishes the data to a PX4-compatible topic using the `mocap_to_px4` package.

## Dependencies

- [mocap_vicon_client](https://github.com/atarbabgei/mocap_vicon_client)
- [px4_msgs](https://github.com/atarbabgei/px4_msgs) (a fork known to be compatible at the time of development)

## Installation

### Ensure ROS2 is Installed

Make sure you have ROS2 installed on your system. This package is tested on Ubuntu 22.04 and ROS2 Humble. You can follow the installation guide [here](https://docs.ros.org/en/humble/Installation.html).


### Install the Vicon DataStream SDK

To install the Vicon DataStream SDK, run the following command:

```bash
wget -qO- https://github.com/atarbabgei/miscellaneous/raw/main/mocap_vicon/scripts/install_vicon_datastream_sdk.sh | sudo bash
```

### Install Dependencies

Ensure that the necessary dependencies are installed. 

```bash
sudo apt update
sudo apt install libboost-all-dev python3-colcon-common-extensions cmake
```
### Build and Source Workspace
To set up the required packages, clone the repositories into your ROS2 workspace and build them:

```bash
cd ~/your_workspace/src
# Clone the mocap VICON client package
git clone https://github.com/atarbabgei/mocap_vicon_client.git
# Clone the forked PX4 message definitions
git clone https://github.com/atarbabgei/px4_msgs.git
# Clone the mocap-to-PX4 remapping package
git clone https://github.com/atarbabgei/mocap_to_px4.git

# Build the workspace
cd ~/your_workspace
colcon build

# Source the workspace
source install/setup.bash
```


## Example Usage

To launch the VICON data streaming system with the default parameters:

```bash
ros2 launch mocap_to_px4 mocap_to_px4.launch.py server:=192.168.0.100 uav_topic:='/mocap/drone/drone'
```

### Launch Parameters

The launch file provides several configurable parameters for flexibility:

- **`server`**: IP address of the VICON server (default: `192.168.0.100`).
- **`buffer_size`**: Buffer size for the VICON client (default: `256`).
- **`topic_namespace`**: Namespace for the published VICON topics (default: `mocap`).
- **`uav_topic`**: ROS topic for UAV motion capture data (default: `/mocap/drone/drone`).
- **`px4_visual_odometry_topic`**: PX4 topic for publishing visual odometry data (default: `/fmu/in/vehicle_visual_odometry`).

## Nodes Overview

1. **VICON Streaming Node**:
   - **Package**: `mocap_vicon_client`
   - **Executable**: `vicon_client`
   - **Description**: Streams motion capture data from the VICON server and publishes it to ROS topics.

2. **PX4 Remapping Node**:
   - **Package**: `mocap_to_px4`
   - **Executable**: `mocap_remap_to_px4`
   - **Description**: Remaps motion capture data and publishes it in a format compatible with PX4 visual odometry.
