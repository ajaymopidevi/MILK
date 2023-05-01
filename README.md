# MILK
Advanced Robotics Project

## Visualize depth images
rosrun rs_depth exportImage.py

Edit the directories in the script

## Mapping:
1. Convert depthImage to laserscan
    sudo apt install ros-noetic-depthimage-to-laserscan
    roslaunch rs_depth depthimage_laser.launch
2. Mapping
    sudo apt install ros-noetic-hector-mapping
    roslaunch rs_depth hector_test.launch base_frame:=camera_depth_frame odom_frame:=camera_depth_frame

Combined script for mapping:
    roslaunch rs_depth hecto_map.launch
