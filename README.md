# Research & Education - Regolith Advanced Surface Systems Operations Robot (RE-RASSOR)

**Description:**  
This repositry contains the source code for the University of Adelaide's 2024 iteration of the RE-RASSOR project. The RE-RASSOR is a small-scale version of a lunar rover, that uses a variety of interchanging tools for excavation and construction.

## Table of Contents
1. [Introduction](#introduction)
2. [Features](#features)
3. [System Requirements](#system-requirements)
4. [Raspberry Pi Configuration](#raspberry-pi-configuration)
5. [Installation](#installation)
    - [Dependencies](#dependencies)
    - [Building the Project](#building-the-project)
    - [Setting Up Services](#setting-up-services)
6. [Usage](#usage)
    - [Running Services](#running-services)
    - [Stopping Services](#stopping-services)
    - [Operation Guide](#operation-guide)
7. [Troubleshooting](#troubleshooting)
8. [License](#license)
9. [Contributing](#contributing)
10. [Contact](#contact)

## Introduction
Established by the Florida Space Institute, the RE-RASSOR program involves international collaboration between universities, aiming to constantly bring new research and improvements to the platform. This project builds upon the work of the University of Adelaide’s 2022 and 2023 teams, and aims to improve the rover's design, functionality and performance. The 2024 project saw significant changes implemented to the actuator control systems of the rover, and hence significant changes to the software. The aims of the software component of the project were to develop enhanced software systems for user control and feedback, through developing a modular system with a physical control interface and a graphical user interface, with integrated sensor feedback. The software uses [ROS]([URL]https://www.ros.org/), a set of software libraries and tools for building modular robot applications. The code is designed to be used in conjunction with the graphical user interace, which can be found [here]([URL](https://automate.dronedeploy.com/project/re-rassor-426007/robots/re-rassor/dashboard/247aca40-efda-11ee-a929-eb3f2ba3f8ad). Contact Teresa Kelly or the project supervisor for the login details.

## Features
- List the main features of your project.

## System Requirements
- **Hardware:** Specify the Raspberry Pi model and other hardware components.
- **Software:** Mention required OS, software versions, libraries, etc.

## Raspberry Pi Configuration
### Required Configuration
1. **OS Setup:** Provide details on how to set up the operating system, including any configurations for Ubuntu on Raspberry Pi.
2. **Networking:** Explain how to configure networking (WiFi, Ethernet, SSH, etc.).
3. **Hardware Connections:** Describe the connections (e.g., sensors, motor drivers, etc.) and any settings required on the Raspberry Pi.
4. **Enabling Interfaces:** Provide instructions on enabling necessary interfaces like I2C, SPI, GPIO, etc.

## Installation
### Dependencies
1. **System Packages:** List and provide installation commands for required system packages.
   ```bash
   sudo apt-get update
   sudo apt-get install [packages]
