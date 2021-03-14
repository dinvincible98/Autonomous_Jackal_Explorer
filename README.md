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
* pyrealsense2, NumPy, OpenCV(4.5.1),cv_bridge
* C++, python3(3.8.1)
### Setup Jackal on ROS Noetic From Scratch
Please refer to my detailed setup instruction [here](https://github.com/dinvincible98/Jackal_ROS_Noetic_Bringup) and carefully read each command line. To run this project, user need to clone both the bringup repo and this repo. In addition to all packages listed in the bringup repo, user need to install [hdl_people_tracking](https://github.com/koide3/hdl_people_tracking), hdl_localization and hdl_global_localization packages into the jackal_ws. 

#### Note: After git clone this repo into your own workspace, please change the folder name to jackal_slam, then build the package with catkin_make. 

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
##### Demo
![jackal_slam](https://user-images.githubusercontent.com/70287453/111059530-f3140d00-845b-11eb-8aa3-7b2310e49384.gif)

#### Note: Remember to source the bash file every time when open a new terminal

#### Part 2: Frontier Exploration
Frontier exploration is a fast and advanced algorithm for robot to explore in an unknown environment and map the world. A detailed explanation can be found [here](https://www.cs.cmu.edu/~motionplanning/papers/sbp_papers/integrated1/yamauchi_frontiers.pdf). To implement this algorithm on Jackal, I used Costmap2D package and divided the task into three sections:

1. costmap.cpp: A customized tool library for handling function from Costmap2D package. It provides information of the map and robot pose.
 
2. frontier_search.cpp: A library contains my implementation of frontier exploration algorithm  

3. navigation.cpp: The main exploration code used functions from costmap and frontier_search library. 

##### ROS Node Flowchart

![Screenshot from 2021-03-13 22-24-06](https://user-images.githubusercontent.com/70287453/111057174-df13df80-844a-11eb-8548-9372ff116ee8.png)

##### Implementation explaination: 
When the robot is initiated, the first step is to get the robot pose in map frame by subscribing to the OccupancyGrid/map topic. After getting the pose, the second step is to search frontier using two bfs(breadth first seach). The outer bfs iterates over 4 connected neighborhoods and check if each neighbrhood is free and unvisited. If the neighborhood is free and unvisited, then added to a queue. If a cell is a new frontier cell, then the second bfs comes up. The second bfs is used to build a new frontier searched from the frontier cell and its given pose.  It checked 8 connected-neighborhoods and added each one to queue if it is new frontier cell. After two bfs seach, all frontiers are obtained. The third step is to store those frontier into a list and sort them based on the cost(distance to the centroid of the frontier plus some gains). The fourth step is choose a non blacklist frontier point to go by publishing message to move_base topic. If the robot does not move to the target point in a specified time limit, the target frontier will be added to the blacklist and the robot will move to a new frontier point. To visualize frontiers, I use blue color for available frontiers and red color for blaklisted frontiers. While the robot is moving to a target point, if the environment is mapped before it reached the point then the robot will immediately move to the next frontier point. Once there is no frontier in an environment, then the mapping is down. 

To run the frontier exploration in simulation(Gazebo):
        
        1. roslaunch jackal_slam jackal_test.launch
        2. roslaunch jackal_slam jackal_explore.launch 

##### Demo
![sim](https://user-images.githubusercontent.com/70287453/111059533-f4453a00-845b-11eb-9a82-8ed0f2c2b920.gif)

To run the frontier exploration on real Jackal(Make sure the SLAM node is running first):

On remote PC:

        1. source setup_laptop.bash
        2. roslaunch jackal_slam jackal_explore.launch
##### Demo
![real](https://user-images.githubusercontent.com/70287453/111059806-e8f30e00-845d-11eb-8db0-c0366c25a32b.gif)

#### Part 3: People Tracking
For people tracking feature, I used hdl_people tracking package which can detect walking pedestrains by extracting information from the pointcloud data. 

To run the people tracking, open a new terminal on remote PC:

        1. source setup_laptop.bash
        2. roslaunch jackal_slam hdl_people_tracking.launch

##### Demo
![people](https://user-images.githubusercontent.com/70287453/111059814-eee8ef00-845d-11eb-96fa-cdfd59554b52.gif)

#### Part 4: Real-time Object Detection 
This part of the project is also my final project for [COMP_ENG 395: Connected and Autonomous Vehicles: Challenges and Design](https://www.mccormick.northwestern.edu/electrical-computer/academics/courses/descriptions/395-495-autonomous-vehicles.html) which I thorougly evaluated the performance of YOLOv3, YOLOv4 and YOLOv4-tiny and tested on the Jackal. Based on my test result, I found YOLOv4-tiny can reach approximately 30FPS in ROS whereas the other two models only have 3FPS. Even though YOLOv4-tiny has less accuracy compared with the other two models, it is still good enough to classify most objects. Besides, FPS is extreamly important in real-time detection for autonomous vehicle so I decided to use YOLOv4-tiny on Jackal. The full analysis paper can be found [here](https://github.com/dinvincible98/Autonomous_Jackal_Explorer/blob/master/final%20project.pdf). 

Before running the object detection node, user need to change the [path](https://github.com/dinvincible98/Autonomous_Jackal_Explorer/blob/master/src/detection) at line 29-30 to their own YOLO path since the module uses absolute path here. 

To run the object detection, open a new terminal on remote PC:    

        1. source setup_laptop.bash
        2. roslaunch jackal_slam detection.launch

##### Demo
![detection](https://user-images.githubusercontent.com/70287453/111059811-eb556800-845d-11eb-906a-2048baa3e898.gif)

### Future Work
* Add a boundary feature to the automapping so I can specify an area for the robot to explore instead of trying to map the whole environment(Not realistic if given a very large environment)
* Lidar - Camera fusion so the detected objects can be shown in the map. 
        

