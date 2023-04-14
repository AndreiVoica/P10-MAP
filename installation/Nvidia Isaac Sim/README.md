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
```docker pull ubuntu:jammy```
3. In the terminal run ```docker images``` and copy the IMAGE ID
3a. If you encounter an error related to QT5 not being able to find a display (this will be needed if you are going to use any application with a GUI in docker) run ```xhost local:root``` in the terminal and after run the docker image be using 
```
docker run -it --rm \
    --network host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix/:/tmp/.X11-unix \
    <IMAGE ID>
```
4. Run the docker image using ```docker run -it <IMAGE ID>```
5. 