# Use nvidia/cuda:11.8.0-cudnn8-devel-ubuntu20.04 as base image
FROM nvidia/cuda:11.8.0-cudnn8-devel-ubuntu20.04

# Set working directory
WORKDIR /workspace

# Copy the ROOT directory contents into the container's workspace (../)
COPY ../ /workspace

# Ensure the X11 socket is available for GUI applications
VOLUME ["/tmp/.X11-unix"]

# ROS noetic installation

# Add keys for ROS Noetic

RUN tee /usr/share/keyrings/ros.gpg > /dev/null

# Install necessary tools
RUN apt-get update && apt-get install -y curl gnupg2 sudo

RUN sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'

# Uncomment the following lines to use USTC mirror
# RUN sudo sh -c 'echo "deb https://mirrors.ustc.edu.cn/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'



# Install ROS Noetic
RUN apt-get update

RUN apt-get install -y ros-noetic-desktop-full

# Initialize rosdep
RUN rosdep init && rosdep update

# Set up environment variables for ROS
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Install some dependencies and tools for ROS development
RUN apt-get install -y \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential

# Install any other tools or dependencies as needed
RUN apt-get update && apt-get install -y \
    python3-catkin-tools

# Set default command to launch bash
CMD ["/bin/bash"]