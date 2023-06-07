# Nvidia Isaac Sim

Nvidia Isaac Sim is a simulation software developed by Nvidia. In the first instance, the project was intended to be developed using ROS2, these steps may be useful for implementation with ROS2.

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
4. Run the docker image using ```docker run -it IMAGE ID```
5. Start installing ROS 2 Humble following [this tutorial](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
6. Install moveit2 setup assistant ```sudo apt install ros-humble-moveit```
7. Create a colcon workspace ```source /opt/ros/humble/setup.bash```
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```
8. Clone the repo (TBD)
9. 
```
cd ..
rosdep install -i --from-path src --rosdistro humble -y
```
# Converting ROS1 packages to ROS2 
To be able to build using colcon, the packages have to be converted to support it.

# Using Moveit2

# Connecting Moveit2 to Isaac using a custom robot configuration

# Troubleshooting
## QT unable to find display in while trying to launch a GUI application (such as rviz2 or moveit setup assistant)
If you encounter an error related to QT5 not being able to find a display (this will be needed if you are going to use any application with a GUI in docker) run ```xhost local:root``` in the terminal and after run the docker image by using the command below and skip step 4. Recommended to add ```xhost local:root``` in your .bashrc.
```
docker run -it --rm \
    --network host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix/:/tmp/.X11-unix \
    IMAGE ID
```

## URDF files require double
If you have for example velocity = 2.0, try velocity = 2.00001
