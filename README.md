# 🤖 Autonomous Navigation Using Traditional and AI-Based Approaches

Welcome to the **Autonomous Navigation** project! This repository demonstrates and compares traditional and AI-enhanced navigation methods for robotic systems in simulated environments using **ROS Noetic** and **Gazebo**. 🚀

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

![Placeholder for Gazebo Simulation Screenshot](#) <!-- Replace with actual image link -->

---

## 🛠 Setup Instructions

### Prerequisites
Ensure the following are installed:
- ROS Noetic
- Gazebo
- Python 3.x
- [Dependencies listed here](https://wiki.ros.org/ROS/Installation)

### Clone the Repository
```bash
git clone https://github.com/AdharshKan42/AI_Based_Auton_Nav
cd AI_Based_Auton_Nav
