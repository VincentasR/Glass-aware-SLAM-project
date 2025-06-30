# Glass-aware-SLAM-project
 This repository contains the implementation of a glass-aware SLAM system designed for indoor mobile robots. The system integrates 2D LiDAR-based SLAM (using slam_toolbox) with real-time glass detection from RGB and stereo depth data using an OAK-D Pro camera and a custom YOLOv8 model.


## 🛠 Requirements on PC side

- Ubuntu 24.04 (or compatible Linux distribution)
- ROS 2 Jazzy installed and sourced
- Python 3.8 or newer
- GPU drivers and CUDA (optional, for faster YOLO inference)

## ⚙️ Install ROS 2 Jazzy

Follow the official installation guide:  
[ROS 2 Jazzy installation instructions](https://docs.ros.org/en/jazzy/Installation.html)

After installation, source ROS 2 in every new terminal before running any ROS-related Python scripts:

```bash
source /opt/ros/jazzy/setup.bash

## 📦 Python Dependencies

Install all required Python packages using:

```bash
pip install -r requirements_pc.txt

##🛠 Requirements on Turtlebot4 side





