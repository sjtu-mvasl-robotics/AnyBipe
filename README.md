# AnyBipe: An End-to-End Framework for Training and Deploying Bipedal Robots Guided by Language Models

<div align="center">

[[Website]](https://anybipe.github.io/)
[[Paper]](URL)
[[Videos]](URL)

Reserved for authors.

Reserved for author institutions.

[![Python Version](https://img.shields.io/badge/Python-3.8-blue.svg)](https://github.com/Humanoid-Robots/AnyBipe.git)
[<img src="https://img.shields.io/badge/Framework-PyTorch-red.svg"/>](https://pytorch.org/)
[![ROS Noetic](https://img.shields.io/badge/ROS-Noetic-blue)](http://wiki.ros.org/noetic)
[![Issacgym](https://img.shields.io/badge/Isaacgym-Preview%204-gree)](https://developer.nvidia.com/isaac-gym)
[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://github.com/Humanoid-Robots/AnyBipe/blob/main/LICENSE)


______________________________________________________________________

</div>

Reserved for abstract.

## Important Notice
The `AnyBot Integrated` project is still under development, and this project is the prime version for a more completed framework. Our general framework is almost ready, but we are still working on coding migration (from `Issacgym` to `Issac Sim`), documentation, further experiments, and more. This repository might be merged to the main framework repository in the future.

This repository is still under maintenance, it might be updated frequently. If you want to use this repository for your own robot, please checkout to stable release branches.

## Pre-requisites
To configure and run this project for your own robot, you have to make sure that following pre-requisites are satisfied:

| Pre-requisite | Description |
| --- | --- |
| Reinforcement Learning Platform | Our work supports ANYmal like training environments based on Issacgym [here](https://github.com/leggedrobotics/legged_gym.git). Your training code should be designed accordingly. |
| LLM Access | Our work is configured to use openAI's api service. You should have access to the API. |
| ROS Noetic | Our work is specialized for robots that are controlled by ROS (currently Noetic, but can support ROS2). |
| Python / C++ SDK | Your robot should have a Python / C++ SDK for controlling and receiving sensor data. |
| Teacher Model (Optional) | You can use your own teacher model for training, no matter what the model is (traditional control, RL, etc.). |

## Installation

This repository contains code for training and deploying bipedal robots using `anybipe.py`. The code contains function to generate reward functions using GPT, train the robot using Issacgym environments, deploy the training model via ROS, convert generated reward functions to real-world measurable metrics, automatically collect feedback from both Gazebo simulation and real-world experiments, and more. The robot model and sdk are provided by [limx dynamics](https://www.limxdynamics.com/en) and adjusted for our work. Training dependencies `legged_gym` and `rsl_rl` are also modified to fit our work.

Since our environment requires `ROS` platform, we recommend using `Ubuntu 20.04` for the best experience. You can also use `Docker` for running the simulation environment. The following steps will guide you to install the required dependencies:

### Step 1: Install ROS Noetic

You can install ROS Noetic by following the instructions [here](http://wiki.ros.org/noetic/Installation/Ubuntu). We recommend installing the `Desktop-Full` version. Or you can use the following command to create a docker container with ROS Noetic:

```bash
docker run -it --rm --name ros-noetic -v $(pwd):/workspace -w /workspace osrf/ros:noetic-desktop-full
```

### Step 2: Create a Conda Environment (Optional)
```bash
conda create -n anybipe python=3.8
conda activate anybipe
```

### Step 3: Install Issacgym
Download and install Issacgym from [here](https://developer.nvidia.com/isaac-gym) and unzip the file, then run the following commands:

```bash
cd isaacgym/python
pip install -e .
```

### Step 4: Install 