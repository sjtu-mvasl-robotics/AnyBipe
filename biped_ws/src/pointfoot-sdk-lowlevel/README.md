# LimX SDK 使用说明

## 1. 搭建开发环境

在算法开发者自己的电脑中，我们推荐在 Ubuntu 20.04 操作系统上建立基于 ROS Noetic 的算法开发环境。ROS 提供了一系列工具和库，如核心库、通信库和仿真工具（如 Gazebo），极大地便利了机器人算法的开发、测试和部署。这些资源为用户提供了一个丰富而完整的算法开发环境。

当然，即使没有 ROS，您也可以选择在其他环境中开发自己的运动控制算法。我们提供的运动控制开发接口，是一个基于标准 C++11 和 Python 的无依赖 SDK。它支持跨操作系统和平台调用开发，为开发者提供了更灵活的选择。

ROS Noetic 安装请参考文档：https://wiki.ros.org/noetic/Installation/Ubuntu ，选择“ros-noetic-desktop-full”进行安装。ROS Noetic 安装完成后，Bash 终端输入以下 Shell 命令，安装开发环境所依赖的库：

```
sudo apt-get update
sudo apt install ros-noetic-urdf \
                 ros-noetic-kdl-parser \
                 ros-noetic-urdf-parser-plugin \
                 ros-noetic-hardware-interface \
                 ros-noetic-controller-manager \
                 ros-noetic-controller-interface \
                 ros-noetic-controller-manager-msgs \
                 ros-noetic-control-msgs \
                 ros-noetic-ros-control \
                 ros-noetic-gazebo-* \
                 ros-noetic-rqt-gui \
                 ros-noetic-rqt-controller-manager \
                 ros-noetic-plotjuggler* \
                 cmake build-essential libpcl-dev libeigen3-dev libopencv-dev libmatio-dev \
                 python3-pip libboost-all-dev libtbb-dev liburdfdom-dev liborocos-kdl-dev -y
```

## 2. 创建工作空间

可以按照以下步骤，创建一个算法开发工作空间：

- 打开一个 Bash 终端。

- 创建一个新目录来存放工作空间。例如，可以在用户的主目录下创建一个名为“limx_ws”的目录：

  ```
  mkdir -p ~/limx_ws/src
  ```

- 下载运动控制开发接口：

  ```
  cd ~/limx_ws/src
  git clone https://github.com/limxdynamics/pointfoot-sdk-lowlevel.git
  ```

- 下载 Gazebo 仿真器：

  ```
  cd ~/limx_ws/src
  git clone https://github.com/limxdynamics/pointfoot-gazebo-ros.git
  ```

- 下载机器人模型描述文件

  ```
  cd ~/limx_ws/src
  git clone https://github.com/limxdynamics/robot-description.git
  ```

- 下载可视化调试工具

  ```
  cd ~/limx_ws/src
  git clone https://github.com/limxdynamics/robot-visualization.git
  ```

- 编译工程：

  ```
  cd ~/limx_ws
  catkin_make install
  ```


## 3. Python 运动控制开发接口

### 3.1 概述

提供与 C++相同功能的[Python 运动算法开发接口](https://github.com/limxdynamics/pointfoot-sdk-lowlevel/tree/master/python3)，使得不熟悉 C++编程语言的开发者能够使用 Python 进行运动控制算法的开发。Python 语言易于学习，具有简洁清晰的语法和丰富的第三方库，使开发者能够更快速地上手并迅速实现算法。通过 Python 接口，开发者可以利用 Python 的动态特性进行快速原型设计和实验验证，加速算法的迭代和优化过程。同时，Python 的跨平台性和强大的生态系统支持，使得运动算法能够更广泛地应用于不同平台和环境。此外，RL（强化学习）模型的快速部署到仿真和真机环境中也得益于 Python 的灵活性，开发者可以使用 Python 轻松地将 RL 模型集成到各种仿真平台和真实硬件中，实现快速迭代和验证算法的性能。

### 3.2 环境配置

通过设置 PYTHONPATH 环境变量，可以将 Python 的运动控制开发库配置到 Python 环境中。这样做可以让 Python 解释器在搜索模块时包含指定的路径，确保可以找到并使用运动控制库中的模块和功能。

- Linux x86_64 环境

  请确保将"/your/actual/path/pointfoot-sdk-lowlevel"替换为实际路径，这样 Python 解释器才能正确地找到并使用运动控制开发库中的模块和功能。

  ```Bash
  export PYTHONPATH="/your/actual/path/pointfoot-sdk-lowlevel/python3/amd64:$PYTHONPATH"
  ```

- Linux aarch64 环境

  请确保将"/your/actual/path/pointfoot-sdk-lowlevel"替换为实际路径，这样 Python 解释器才能正确地找到并使用运动控制开发库中的模块和功能。

  ```Bash
  export PYTHONPATH="/your/actual/path/pointfoot-sdk-lowlevel/python3/aarch64:$PYTHONPATH"
  ```

- Windows 环境

  请确保将"D:\your\actual\path\pointfoot-sdk-lowlevel"替换为实际路径，这样 Python 解释器才能正确地找到并使用运动控制开发库中的模块和功能。

  ```bash
  set PYTHONPATH=D:\your\actual\path\pointfoot-sdk-lowlevel\python3\win;%PYTHONPATH%
  ```

### 3.3 参考例程

Python 接口参考例程: https://github.com/limxdynamics/pointfoot-sdk-lowlevel/blob/master/python3/amd64/example.py
