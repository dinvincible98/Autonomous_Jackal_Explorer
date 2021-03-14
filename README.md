# Autonomous Jackal Explorer
### This is an individual winter quarter project at Northwestern University
### Project Goal

The goal of the project is to build a mobile robot system equipped with perception, localization, mapping and navigation abilities in both simulation and real world. The robot is capable of exploring an unknown environment autonomously and mapping the world. It can also detect pedestrians and classify different objects with lidar and camera. Based on the changing the environment, the robot should give responses accordingly.

### Demo



### Dependencies
#### Hardware:
* Clearpath Jackal Robot
* Velodyne-16 lidar
* Intel Realsense Camera D435i
#### Software:
* ROS Noetic
* Slam Toolbox
* Costmap2D
* hdl people tracking
* pyrealsense2
* NumPy
* OpenCV(4.5.1)
* YOLOv4-tiny
### Setup Jackal on ROS Noetic From Scratch
Please refer to my detailed setup instruction [here](https://github.com/dinvincible98/Jackal_ROS_Noetic_Bringup). To run this project, user need to clone both the bringup repo and this repo.
### Structure of The Project
#### Part 1: SLAM
SLAM(Simultaneous Localization and Mapping) technology allows the robot to map an unknown environment while keeps tracking its own location. To run SLAM on real Jackal:

    1. source setup



