
# 🤖 Autonomous Navigation Using Traditional and AI-Based Approaches

Welcome to the **Autonomous Navigation** project! This repository demonstrates and compares traditional and AI-enhanced navigation methods for robotic systems in simulated environments using **ROS Noetic** and **Gazebo**. 🚀

![Traditional AI Visualization](https://raw.githubusercontent.com/AdharshKan42/AI_Based_Auton_Nav/main/images/traditionalvai.png)

---

## Demo Clips :

### Traditional Navigation Demo :

<p align="center">
  <img src="https://raw.githubusercontent.com/AdharshKan42/AI_Based_Auton_Nav/main/images/navigationdemo.gif" />
</p>

### AI Based Approach Demo :

![AI Based Approach Demo](https://raw.githubusercontent.com/AdharshKan42/AI_Based_Auton_Nav/main/images/segmentdemo.gif)

---

## 📝 Table of Contents
1. [Introduction](#introduction)
2. [Setup Instructions](#setup-instructions)
3. [Usage](#usage)
4. [Results](#results)
5. [Acknowledgements](#acknowledgements)
6. [Contributions](#contributions)

---

## 🧐 Introduction
This project explores two navigation paradigms:
- **Traditional Navigation**: Using SLAM and A* for pathfinding.
- **AI-Based Navigation**: Leveraging semantic segmentation with YOLOv8 to classify terrain into safe (green), challenging (yellow), and restricted (red) zones.

The **Jackal Robot** was employed in a simulated Gazebo environment, equipped with:
- 🛠 **Sensors**: LiDAR, Intel RealSense D455, and IMU.
- 🌐 **Simulation Environment**: Clearpath Robotics' *Inspection World*.

#### The environment in Gazebo :

![Simulation environment Demo](https://raw.githubusercontent.com/AdharshKan42/AI_Based_Auton_Nav/main/images/simenv.gif)

---

## 🛠 Setup Instructions

### Prerequisites
Ensure the following are installed:
- ROS Noetic
- Gazebo
- RVIZ
- Python 3.9
- [Dependencies listed here](https://wiki.ros.org/ROS/Installation)

### Clone the Repository
```bash
git clone https://github.com/AdharshKan42/AI_Based_Auton_Nav
cd AI_Based_Auton_Nav
```

### Install Dependencies
```bash
rosdep update
rosdep install --from-paths src --ignore-src -r
```

### Build the Workspace
```bash
catkin_make
source devel/setup.bash
```

---

## 🚀 Usage

### 1️⃣ Launching the Simulation and Moving around
To start the **Gazebo** environment with the Jackal robot:
```bash
roslaunch cpr_inspection_gazebo inspection_world.launch platform:=jackal
```
#### Launching RVIZ
```bash
roslaunch jackal_viz view_robot.launch
```

#### To move the Bot around
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/cmd_vel
```

#### To Launch the GUI Interface controller :
```bash
rosrun rqt_gui rqt_gui -s rqt_robot_steering
```

#### To View Camera Output
```bash
rosrun rqt_image_view rqt_image_view
```
#### RGB Camera Output :

![Raw Camera output](https://raw.githubusercontent.com/AdharshKan42/AI_Based_Auton_Nav/main/images/rawcameraop.png)

#### Depth Camera Output :

![Depth Camera output](https://raw.githubusercontent.com/AdharshKan42/AI_Based_Auton_Nav/main/images/depthcameraop.png)

### 2️⃣ Traditional Navigation Approach
#### Launching Gazebo
```bash
roslaunch cpr_inspection_gazebo inspection_world.launch platform:=jackal
```

#### Path Planning and Navigation (Launching gmapping with custom scan topic:)
```bash
roslaunch jackal_navigation gmapping_demo.launch scan_topic:=lidar/scan
```
```bash
roslaunch jackal_viz view_robot.launch config:=gmapping
```

#### Gmapping output :

![Mapping Screenshots](https://raw.githubusercontent.com/AdharshKan42/AI_Based_Auton_Nav/main/images/map-being-generated.png)

#### Saving the map :
```bash
rosrun map_server map_saver -f mymap
```
#### To localize with AMCL :
```bash
roslaunch jackal_navigation amcl_demo.launch map_file:=/path/to/my/map.yaml
```

#### To start navigation :
```bash
roslaunch jackal_viz view_robot.launch config:=localization
```

#### Camera Output
- View camera: `rosrun rqt_image_view rqt_image_view`


### 3️⃣ AI-Based Navigation
Create a goal waypoint based on the robot's current position.
```bash
rosrun utm_to_robot waypoint_creation.py
```

Start YOLOv8 Inference on the Realsense Camera
```bash
rosrun utm_to_robot yolo_inference.py
```
#### Inference Ouput on the Realsense Camera  :
![AI Based approach depth camera op](https://raw.githubusercontent.com/AdharshKan42/AI_Based_Auton_Nav/main/images/aibasedappcameraop.png)

Run the navigation algorithm to move towards the goal position
```bash
rosrun utm_to_robot mock_movebase_combined.py
```
---

## 📊 Results

| Metric                | Traditional (A*) | AI-Based (YOLOv8) |
|-----------------------|------------------|-------------------|
| Path Length (meters)  | 18.6            | 17.3             |
| Traversal Time (secs) | 45.2            | 39.8             |
| Collision Rate        | 1               | 0                |
| Computational Load    | 100% CPU        | 65% CPU, 30% GPU |
| Adaptability          | Low             | High             |

### Key Insights
- **Traditional Approach**: Reliable for static environments, struggles with dynamic changes.
- **AI-Based Approach**: Adapts well to unstructured terrains with efficient navigation.
---


---

## 🤝 Contributions
This project was a collaborative effort by:
- @AdharshKan42 Adharsh Kandula: Environment setup and AI model integration.
- @lowwhit Lohith Venkat Chamakura: Implementation of Traditional navigation algorithms.
- @nishitpopat Nishit Popat: Mapping algorithms Pathfinding optimizations.
- @rrrraghav Raghav Mathur: Testing of multiple models and finetuning.
- @SwordAndTea Wei Xiang: Data collection and preprocessing, AI model training and evaluation. 

---

**Feel free to contribute!** Submit issues or pull requests to improve this repository. 🎉

---
