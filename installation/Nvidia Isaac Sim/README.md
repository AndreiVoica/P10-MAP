# Nvidia Isaac Sim

Nvidia Isaac Sim is a simulation software developed by Nvidia

## Installing Nvidia Isaac Sim on Ubuntu 20.04

Follow the [Workstation Installation](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/install_workstation.html) tutorial.

# ROS 2 Foxy

Nvidia Isaac Sim is currently compatible with ROS 2 Foxy (Ubuntu 20.04) due to ROS 2 bridge.

Follow the [ROS2 Foxy installation method](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html).

# ROS 2 Humble in Docker

ROS 2 Humble is used only for Moveit2 assistant, since it is not available on Foxy.

1. Install Docker following [this tutorial](https://docs.docker.com/engine/install/ubuntu/);
2. Open a terminal and run 
'''docker pull ubuntu:jammy'''
3. 