<div id="top"></div>

<h1 align="center">UCSDrive! Autonomous Campus Rideshare</h1>

<!-- PROJECT LOGO -->
<br />
<div align="center">
  <a href="https://jacobsschool.ucsd.edu/">
    <img src="https://github.com/UCSD-ECEMAE-148/winter-2024-final-project-team-4/blob/main/images/UCSDLogo_JSOE_BlueGold.png" alt="Logo" width="400" height="100">
  </a>
<h3>MAE148 Final Project</h3>
<p>
Team 1 Fall 2024
</p>

![image](images/7C66CA64-C422-4535-9721-F523EA8FAC5B.jpeg)
</div>




<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li><a href="#team-members">Team Members</a></li>
    <li><a href="#final-project">Final Project</a></li>
      <ul>
        <li><a href="#original-goals">Original Goals</a></li>
          <ul>
            <li><a href="#goals-we-met">Goals We Met</a></li>
            <li><a href="#our-hopes-and-dreams">Our Hopes and Dreams</a></li>
              <ul>
                <li><a href="#stretch-goal-1">Stretch Goal 1</a></li>
                <li><a href="#stretch-goal-2">Stretch Goal 2</a></li>
              </ul>
          </ul>
        <li><a href="#final-project-documentation">Final Project Documentation</a></li>
      </ul>
    <li><a href="#robot-design">Robot Design </a></li>
      <ul>
        <li><a href="#cad-parts">CAD Parts</a></li>
          <ul>
            <li><a href="#final-assembly">Final Assembly</a></li>
            <li><a href="#custom-designed-parts">Custom Designed Parts</a></li>
            <li><a href="#open-source-parts">Open Source Parts</a></li>
          </ul>
        <li><a href="#electronic-hardware">Electronic Hardware</a></li>
        <li><a href="#software">Software</a></li>
          <ul>
            <li><a href="#embedded-systems">Embedded Systems</a></li>
            <li><a href="#ros2">ROS2</a></li>
            <li><a href="#donkeycar-ai">DonkeyCar AI</a></li>
          </ul>
      </ul>
    <li><a href="#acknowledgments">Acknowledgments</a></li>
    <li><a href="#authors">Authors</a></li>
    <li><a href="#contact">Contact</a></li>
  </ol>
</details>

# Autonomous Penalty Kick Goalie

## Overview

This project focuses on developing a robot car capable of acting as an autonomous penalty kick goalie. Leveraging computer vision (CV) and a DepthAI framework, the robot detects and tracks a soccer ball, calculates its spatial coordinates, and responds to intercept the ball. By combining a PID controller with ROS2 nodes, the system outputs steering and throttle commands to the vehicle's VESC (Variable Electronic Speed Controller) for precise control.

### **Key Features**
- **Ball Tracking:** Detects and tracks the ball in real time using DepthAI and OpenCV algorithms.
- **Depth and Angle Estimation:** Computes the ball's depth (distance) and horizontal angle relative to the robot.
- **ROS2 Framework:** Publishes and subscribes to relevant data across custom ROS2 nodes for modular functionality.
- **PID Control:** Implements a proportional-integral-derivative controller for accurate steering adjustments.
- **Goalkeeper Behavior:** Waits for ball movement before executing intercept maneuvers, mimicking real penalty-kick rules.

---

## Team Members

| Name              | Major                      | Class       |
|-------------------|----------------------------|-------------|
| Abhi Sachdeva     | Electrical Engineering     | Class of 2025 |
| Charles Lahey     | Mechanical Engineering     | Class of 2025 |
| Evan Gibson       | Mechanical Engineering, Ctrls & Robotics | Class of 2025 |
| Gautam Ganesh     | Mechanical Engineering     | Class of 2026 |

---

## **Project Goals**

### **Core Objectives**
1. **Ball Detection and Tracking:**
   - Implement a tracking node that uses DepthAI to:
     - Detect a circular object (soccer ball) in the frame.
     - Publish:
       - `Ball Depth`: Average distance between the ball and the robot in millimeters.
       - `Ball Angle`: Horizontal offset angle from the robot's center.
   - Display bounding boxes for visualization.

2. **Reactive Control:**
   - Implement a controller node that:
     - Monitors the ball's movement (change in depth).
     - Executes PID-based steering adjustments to keep the ball centered in the frame.
     - Outputs throttle and steering commands to the VESC.

3. **Goalkeeper Rules:**
   - Ensure the robot remains stationary until the ball begins moving (mimicking real penalty-kick rules).
   - React swiftly to block the ball once it starts moving.

### **Nice-to-Have Features**
- **Path Prediction:**
  - Use ball position and velocity data to predict the trajectory and intercept the ball optimally.
  - Incorporate robot dynamics to determine the ideal intercept point.
  
---

## **System Architecture**

The project leverages a modular architecture, where each node in the ROS2 framework is responsible for specific tasks. 

### **Node Descriptions**

1. **Track Node**
   - **Inputs:** Camera feed from OAK-D Lite.
   - **Outputs:** 
     - `Ball Depth` (distance in mm).
     - `Ball Angle` (horizontal position in radians or degrees).

2. **Controller Node**
   - **Inputs:** 
     - `Ball Depth` and `Ball Angle` from the Track Node.
   - **Outputs:**
     - `Throttle`: Sends a signal to accelerate the vehicle when ball movement is detected.
     - `Servo Angle`: Provides steering adjustments to align the robot with the ball.

3. **VESC Node**
   - **Inputs:** 
     - `Throttle` and `Servo Angle` from the Controller Node.
   - **Outputs:**
     - Commands to the VESC for motor control.

---

## **Technologies Used**

- **DepthAI:** For object detection, depth estimation, and spatial tracking of the soccer ball.
- **OpenCV:** To process image frames and detect circular objects.
- **ROS2:** Middleware framework for data publishing and subscribing between nodes.
- **PID Controller:** Ensures smooth steering adjustments for ball tracking.
- **VESC:** Controls the robot's drivetrain, providing precise throttle and steering.

---

## **How to Run**

### **Prerequisites**
- Install ROS2 (Humble or Foxy recommended).
- Set up the DepthAI SDK.
- Ensure the VESC is configured and calibrated.

### **Steps**
1. Clone the repository:
   ```bash
   git clone https://github.com/your-repo/penalty-kick-goalie.git
   cd penalty-kick-goalie
2. Build the ROS2 workspace:
   ```bash
   colcon build
3. Launch the system:
   ```bash
   source install/setup.bash
   ros2 launch goalie_system.launch.py
   
## **Future Improvements**

- Integrate a path prediction algorithm for smarter ball interception.
- Enhance the PID controller for faster and smoother responses.
- Explore deep learning-based ball detection for improved accuracy in varying lighting conditions.


### Final Project Documentation

* [Final Project Proposal](https://docs.google.com/presentation/d/199oVWJiOSEHAjcmizN8rejuzU7rHNCNl4qY55uGqgxQ/edit?usp=sharing)
* [Progress Update 2/29](https://github.com/kiers-neely/ucsd-mae-148-team-4/files/14469441/mae148-slides-update.pdf)
* [Progress Update 3/7](https://github.com/kiers-neely/ucsd-mae-148-team-4/files/14547470/mae148-slides-update.2.pdf)

<!-- Early Quarter -->
## Robot Design

### CAD Parts
#### Final Assembly
<img src="https://github.com/kiers-neely/ucsd-mae-148-team-4/assets/161119406/aa99560c-a7ff-4ca0-b913-24ac75bb6eec" width="700" height="500" />

#### Custom Designed Parts
| Part | CAD Model | Designer |
|------|--------------|------------|
| Front Camera and LiDAR Mount | <img src="https://github.com/kiers-neely/ucsd-mae-148-team-4/assets/161119406/03902430-3625-4b19-ae1d-3ddaa344aa6a" width="300" height="300" /> | Kiersten
| Side Camera and GNSS Puck Mount | <img src="https://github.com/kiers-neely/ucsd-mae-148-team-4/assets/161119406/ce443b16-9706-402e-be97-a78447cd391f" width="300" height="400" /> | Kiersten
| Acrylic Base | <img src="https://github.com/kiers-neely/ucsd-mae-148-team-4/assets/161119406/2b4e5f76-f76d-4184-8922-512b867e38bc" width="300" height="300" /> | Damien
| Side Paneling | <img src="https://github.com/kiers-neely/ucsd-mae-148-team-4/assets/161119406/d4d178f0-1912-44ac-8c8f-8a4d6e4bb17f" width="300" height="300" /> | Damien


#### Open Source Parts
| Part | CAD Model | Source |
|------|--------|-----------|
| Jetson Nano Case | <img src="https://github.com/kiers-neely/ucsd-mae-148-team-4/assets/161119406/6770d099-0e2e-4f8d-8072-991f1b72971f" width="400" height="300" /> | [Thingiverse](https://www.thingiverse.com/thing:3778338) |
| Oak-D Lite Case | <img src="https://github.com/kiers-neely/ucsd-mae-148-team-4/assets/161119406/bcc64c60-d67c-47af-b0cb-f46ac7b8a4c1" width="400" height="300" /> | [Thingiverse](https://www.thingiverse.com/thing:533649) |


### Electronic Hardware
Below is a circuit diagram of the electronic hardware setup for the car.

<img src="https://github.com/kiers-neely/ucsd-mae-148-team-4/assets/161119406/6f7501ee-382a-4590-9c0a-f8ce738efec3" width="800" height="400" />


### Software
#### Embedded Systems
To program the Jetson Nano, we accessed the Jetson Nano through remote SSH connection to an embedded Linux system onboard and ran a docker container with all the necessary dependencies to run our packages. This allowed us to eliminate any incompatibility issues and to maximize resource efficiency on the Jetson. We used a variation of virtualization softwares including VMWare and WSL2 to build, test and launch our programs. 

#### ROS2
The base image pulled from Docker Hub for our project development contained the UCSD Robocar module ran on a Linux OS (Ubuntu 20.04). The Robocar module, consisting of several submodules using ROS/ROS2, was originally developed by Dominic Nightingale, a UC San Diego graduate student. His framework was built for use with a wide variety of sensors and actuation methods on scale autonomous vehicles, providing the ability to easily control a car-like robot while enabling the robot to simultaneously perform autonomous tasks.

#### DonkeyCar AI
For our early quarter course deliverables we used DonkeyCar to train a car in driving autonomous laps around a track in a simulated environment. We used Deep Learning to record visual data of driving on a simulated track and trained the car with the data to then race on a remote server. This helped us to prepare for training our physical car on an outdoor track with computer vision.

<!-- Authors -->
## Authors
  - [@kiers-neely](https://github.com/kiers-neely)  

<!-- Badges -->
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

<!-- ACKNOWLEDGMENTS -->
## Acknowledgments
*Thank you to my teammates, Professor Jack Silberman, and our incredible TA Arjun Naageshwaran for an amazing Winter 2024 class!*

<!-- CONTACT -->
## Contact

* Kiersten | kneely@ucsd.edu
* Jacob | jacoberobison@gmail.com 
* Joe | hjjeong@ucsd.edu
* Damien | dcuara@ucsd.edu
