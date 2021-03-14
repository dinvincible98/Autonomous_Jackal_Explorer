# Autonomous Jackal Explorer
### This is an individual winter quarter project at Northwestern University
### Project Goal

The goal of the project is to build a mobile robot system equipped with perception, localization, mapping and navigation abilities in both simulation and real world. The robot is capable of exploring an unknown environment autonomously and mapping the world. It can also detect pedestrians and classify different objects with lidar and camera. Based on the changing the environment, the robot should give responses accordingly.

### Full Demo



### Dependencies
#### Hardware:
* Clearpath Jackal Robot
* Velodyne-16 lidar
* Intel Realsense Camera D435i
#### Software:
* ROS Noetic, Rviz, Gazebo
* Slam Toolbox, Costmap2D
* hdl people tracking, YOLOv4-tiny
* pyrealsense2, NumPy, OpenCV(4.5.1)
### Setup Jackal on ROS Noetic From Scratch
Please refer to my detailed setup instruction [here](https://github.com/dinvincible98/Jackal_ROS_Noetic_Bringup) and carefully read each command line. To run this project, user need to clone both the bringup repo and this repo.

#### Note: After git clone this repo into ws/src, please change the folder name to jackal_slam, then build the package with catkin_make. 
### Structure of The Project
#### Part 1: SLAM
SLAM(Simultaneous Localization and Mapping) technology allows the robot to map an unknown environment while keeps tracking its own location. I used slam toolbox package to accomplish the task. The basic layout is shown below:


To run SLAM on real Jackal:
On Jackal:
    
    1. source setup_jackal.bash 
    2. roslaunch jackal_noetic_bringup velodyne.launch
    3. roslaunch jackal_noetic_bringup slam_toolbox_jackal.launch

On Remote PC:
    
    1. source setup_laptop.bash
    2. roslaunch jackal_slam slam_toolbox_pc.launch

Then, user can use nav goal in Rviz to publish goal point and map the environment. 
#### Note: Remember to source the bash file every time when open a new terminal

#### Part 2: Frontier Exploration
Frontier exploration is a fast and advanced algorithm for robot to explore in an unknown environment and map the world. A detailed explanation can be found [here](https://www.cs.cmu.edu/~motionplanning/papers/sbp_papers/integrated1/yamauchi_frontiers.pdf). To implement this algorithm on Jackal, I used Costmap2D package and divided the task into three sections:

1. costmap.cpp: A customized tool library for handling function from Costmap2D package. It provides information of the map and robot pose.
 
2. frontier_search.cpp: A library contains my implementation of frontier exploration algorithm  

3. navigation.cpp: The main exploration code used functions from costmap and frontier_search library. 

Implementation explaination: 


To run the frontier exploration in simulation(Gazebo):
        
        1. roslaunch jackal_slam jackal_test.launch
        2. roslaunch jackal_slam jackal_explore.launch 

To run the frontier exploration on real Jackal(Make sure the SLAM node is running first):

On remote PC:

        1. source setup_laptop.bash
        2. roslaunch jackal_slam jackal_explore.launch

        

        

