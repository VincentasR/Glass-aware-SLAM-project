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
```
## 📦 Python Dependencies

Install all required Python packages using:

```bash
pip install -r requirements_pc.txt
```
## 🛠 Requirements on Turtlebot4 side

Clone your own https://github.com/turtlebot/turtlebot4_robot.git then:
Go the turtlebot4_bringup/launch/standard.launch.py and disable the camera (by commenting out every line of code that mentions oakd)
or just copy the standard.launch.py from this repo. If you do not do this step it will not be possible to stream depth and RGB, because the camera will already be streaming via ROS2 topic.

Now, if you are already running a standard.launch.py file, stop it and run this new modified one, or if it automatically starts on launch disable it with 
sudo systemctl disable turtlebot4.service
then reboot it and run the new standard.launch.py whent it boots up.

## Launching the whole system
1. On the turtlebot:
Start the SLAM (only tested with synchronous slam, but should work with asynchronous as well). Install SLAM with 
```bash
sudo apt install ros-jazzy-turtlebot4-navigation
```
Run SLAM with
```bash
ros2 launch turtlebot4_navigation slam.launch.py
```
For more info about it visit https://turtlebot.github.io/turtlebot4-user-manual/tutorials/generate_map.html

2. On PC:
start the glass_aware_slam.py, wait for it to say "TF transform available"

3. On Turtlebot4:
start the rgb_and_depth.py

After doing this you should start seeing detections on your pc terminal (the one where glass_aware_slam.py was launched).

An optional visualization step could be done after step 3:
```bash
ros2 launch turtlebot4_viz view_navigation.launch.py
```
use this command to launch the RViz2 SLAM visualization and then add another MarkerArray, then type in /glass_markers. After this, you should be able to see visualized glass lines.

4. On PC:
```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "name:
  data: 'map_name'"
```
to save the map .pgm and .yaml files.

To get glass detection csv file just close the glass_aware_slam.py

## Postprocessing

To make a new map with glass obstacles marked as proper walls, first run the filter.py and then draw_filtered_lines.py. For these to work without any tinkering it is assumed that you have saved your map as "my_map" and that the files "my_map.yaml", "my_map.pgm" and "glass_detections.csv" are in the postprocessing folder.
I pasted the results of one of my experimental runs, just remove the "23_" part of the names of the files and launch both scripts in a way that was described above to see how it works.
That is it :)


