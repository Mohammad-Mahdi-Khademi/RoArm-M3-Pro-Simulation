# RoArm-M3-Pro Simulation

A realistic **Webots** simulation of the **Waveshare RoArm-M3-Pro** — a 5+1 DOF high-torque desktop robotic arm based on ESP32 and serial bus servos.

<p align="center">
  <img src="Media\17.png" alt="RoArm-M3-Pro in Webots" width="70%"/>
  <br><em>Simulation of Waveshare RoArm-M3-Pro robotic arm</em>
</p>

## Table of Contents

- [RoArm-M3-Pro Simulation](#roarm-m3-pro-simulation)
  - [Table of Contents](#table-of-contents)
  - [Overview](#overview)
  - [Features](#features)
  - [Robot Specifications](#robot-specifications)
  - [Technologies](#technologies)
  - [Repository Structure](#repository-structure)
  - [📸 Simulation Results](#-simulation-results)
    - [Robot View](#robot-view)
    - [Main Menu / Controls](#main-menu--controls)
    - [Move Joints](#move-joints)
    - [Move End-Effector / Tracking](#move-end-effector--tracking)
    - [Scan For Objects](#scan-for-objects)
  - [Authors](#authors)

## Overview

This project provides a detailed **Webots** simulation model of the **RoArm-M3-Pro**, a compact 5+1 degrees of freedom intelligent robotic arm developed by Waveshare.  

The simulation is ideal for:

- Robotics education and learning
- Kinematics, inverse kinematics and trajectory planning experiments
- Controller development and testing
- Visualization of desktop robotic arm behavior
- Prototyping AI, imitation learning or ROS2-related applications

## Features

- Accurate 5+1 DOF kinematic model
- Realistic joint limits and servo behavior
- High-quality textures and visual appearance
- Ready-to-use Webots world files
- Python controller examples
- Support for joint control, end-effector positioning, trajectory following (circle, square, rectangle), imitation modes, etc.
- URDF model included for potential ROS / cross-platform use

## Robot Specifications

| Parameter              | Value                          |
|------------------------|--------------------------------|
| Model                  | Waveshare RoArm-M3-Pro         |
| Degrees of Freedom     | 5 + 1 (with 2-DOF wrist)       |
| Payload                | 200 g @ 0.5 m                  |
| Reach / Workspace      | ≈ 1 m                          |
| Base Rotation          | 360° omnidirectional           |
| Servos                 | All-metal ST3235 bus servos    |
| Control Board          | ESP32 (simulated behavior)     |
| Weight (arm)           | ≈ 1020 g                       |
| Mounting               | Desktop / Table edge clamp     |

> Source: official Waveshare documentation

## Technologies

- **Webots** (recommended: R2025a or newer)
- Python (for controllers)
- URDF model support
- Textures & 3D model preparation tools

## Repository Structure
RoArm-M3-Pro-Simulation/
├── Media/              # Pictures and Videos of simulation

├── URDF Project2/      # URDF model files

├── controllers/        # Python controller scripts

├── protos/             # Webots PROTO files (robot nodes)

├── textures/           # Texture files used in the model

├── worlds/             # Webots simulation scenes (.wbt files)

└── README.md


## 📸 Simulation Results

### Robot View

<p align="center">
  <img src="Media\17.png" alt="Robot View" width="70%"/>
</p>

### Main Menu / Controls

<p align="center">
  <img src="Media\18.png" alt="Main Menu" width="70%"/>
</p>

### Move Joints

<p align="center">
  <img src="Media\19.png" alt="Move Joints" width="70%"/>
</p>


### Move End-Effector / Tracking

<p align="center">
  <img src="Media\20.png" alt="Move End-Effector" width="70%"/>
</p>


### Scan For Objects

<p align="center">
  <img src="Media\21.png" alt="Draw Rectangle" width="70%"/>
</p>

---

## Authors

- **Mohammad Mahdi Khademi**  
- **Negar Naghavian**  

Supervised by: **Dr. Seyed Hassan Zabihifar**

---

Feel free to open issues or contribute!  