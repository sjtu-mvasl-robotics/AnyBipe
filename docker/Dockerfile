# Use nvidia/cuda:11.8.0-cudnn8-devel-ubuntu20.04 as base image
FROM nvidia/cuda:11.8.0-cudnn8-devel-ubuntu20.04

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# Set working directory
WORKDIR /workspace

# Copy the ROOT directory contents into the container's workspace (../)
COPY ../ /workspace

# Ensure the X11 socket is available for GUI applications
VOLUME ["/tmp/.X11-unix"]

# Register mirrors for apt-get
RUN sed -i 's/archive.ubuntu.com/mirrors.aliyun.com/g' /etc/apt/sources.list

# Add keys for ROS Noetic

RUN tee /usr/share/keyrings/ros.gpg > /dev/null

# Install necessary tools
RUN apt-get update && apt-get install -y curl gnupg2 sudo

# Set timezone to Asia/Shanghai
ENV TZ=Asia/Shanghai

RUN apt-get update && \
    echo "tzdata tzdata/Areas select Asia" | debconf-set-selections && \
    echo "tzdata tzdata/Zones/Asia select Shanghai" | debconf-set-selections && \
    apt-get install -y tzdata

RUN sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'

# Install ROS Noetic
RUN apt-get update

RUN n=0; until [ $n -ge 5 ]; do sudo apt-get install -y ros-noetic-desktop-full && sudo apt-get update --fix-missing && break; n=$((n+1)); echo "Attempt $n failed, retrying..."; sleep 5; done

# Install ROS Noetic dependencies
RUN apt-get install -y \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential
# Initialize rosdep
RUN rosdep init && rosdep update

# Set up environment variables for ROS
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Install any other tools or dependencies as needed
RUN apt-get update && apt-get install -y \
    python3-catkin-tools

# Set default command to launch bash
CMD ["/bin/bash"]