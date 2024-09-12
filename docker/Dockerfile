# Use nvidia/cuda:11.8.0-cudnn8-devel-ubuntu20.04 as base image
FROM nvidia/cuda:11.8.0-cudnn8-devel-ubuntu20.04

# Set working directory
WORKDIR /workspace

# Copy the current directory contents into the container's workspace
COPY . /workspace

# Set environment variables
ENV DISPLAY=$DISPLAY \
    QT_X11_NO_MITSHM=1 \
    NVIDIA_DRIVER_CAPABILITIES=all

# Make sure the container is run with the correct group and device permissions
RUN apt-get update && apt-get install -y \
    sudo \
    && sudo groupadd video \
    && sudo usermod -aG video $USER \
    && sudo chown -R $USER:$USER /dev/dri

# Ensure the X11 socket is available for GUI applications
VOLUME ["/tmp/.X11-unix"]

# ROS noetic installation
# Add the ROS Noetic repository
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Add the ROS key
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# Install ROS Noetic
RUN apt-get update && apt-get install -y \
    ros-noetic-desktop-full

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