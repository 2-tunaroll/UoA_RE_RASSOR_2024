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
Established by the Florida Space Institute, the RE-RASSOR program involves international collaboration between universities, aiming to constantly bring new research and improvements to the platform. This project builds upon the work of the University of Adelaide’s 2022 and 2023 teams, and aims to improve the rover's design, functionality and performance. The 2024 project saw significant changes implemented to the actuator control systems of the rover, and hence significant changes to the software. The aims of the software component of the project were to develop enhanced software systems for user control and feedback, through developing a modular system with a physical control interface and a graphical user interface, with integrated sensor feedback. The software uses [ROS]([URL]https://www.ros.org/), a set of libraries and tools for building modular robot applications. The code is designed to be used in conjunction with the graphical user interace, which can be found [here]([URL]https://automate.dronedeploy.com/project/re-rassor-426007/robots/re-rassor/dashboard/247aca40-efda-11ee-a929-eb3f2ba3f8ad). Contact Teresa Kelly or the project supervisor for the login details.

## Features
- List the main features of your project.

  

## System Requirements
- **Hardware:** Raspberry Pi 5, with the required hardware and battery attached and configuration mentioned below, PS4 controller, external laptop client.
- **Software:** Ubuntu 24.04, ROS 2 Jazzy Jalisco, Python 3.12 (on Raspberry Pi and external laptop).

## Raspberry Pi Configuration
The following configuration of the Raspbery Pi 5 is required for the software to run and connect to the DroneDeploy user interface. Note that while all of these items must be completed, they don't need to be implemented in the listed order.
1. Install Ubuntu 24.04: [URL]https://ubuntu.com/download/raspberry-pi
2. Edit the EEPROM configuration: 'sudo -E rpi-eeprom-config -edit'. Add the following line to enable 5A current draw from the power supply: 'PSU_MAX_CURRENT = 5000'
3. Consult the following instructions to configure the Raspberry Pi serial port: https://forums.raspberrypi.com/viewtopic.php?t=36282z. https://github.com/nasa-jpl/osr-rover-code/blob/foxy-devel/setup/rpi.md/#5-setting-up-serial-communication-on-the-rpi. This involves adding the following line to '/boot/firmware/config.txt': 'dtparam=uart0'
4. Use the following instructions to change the 'raspi-config' to enable 1-wire interface and I2C: https://www.raspberrypi.com/documentation/computers/configuration.html
5. Ensure that the connected I2C devices are detected on the system by reviewing the output from 'i2cdetect -y 1'. It should match this:
![Expected output of 'i2cdetect -y 1'](images/i2cdetect.png)
Note that the item at address 70 is to be ignored; it is the "all call" address for the controller chips on the Adafruit HATs.

\begin{tabularx}{\textwidth}{|p{0.4\textwidth} p{0.2\textwidth}|}
\caption{I2C devices and their addresses.}
\label{tab:i2c-addresses} \\
    \hline
    \rowcolor{gray!50}
    \textbf{Component} & \textbf{I2C Address} \\ \hline
    DFRobot Motor HAT & 0x10 \\ \hline
    Accelerometer & 0x19 \\ \hline
    Current \& power sensor & 0x40 \\ \hline
    Internal temperature sensor & 0x48 \\ \hline
    Adafruit Motor HAT (Tool interchange) & 0x60 \\ \hline
    Adafruit Motor HAT (T-joints) & 0x61 \\ \hline
    \hline
\end{tabularx}


\item Install ROS2 Jazzy Jalisco: \url{ https://docs.ros.org/en/jazzy/Installation.html}
Note: If this is the first time using ROS, it is recommended to take some time to read through the documentation and complete some basic tutorials. It is especially important to understand how to use the build tool, \texttt{colcon}. The official ROS 2 Jazzy tutorial are here: \url{ https://docs.ros.org/en/jazzy/Tutorials.html}
\item Ensure that package versions are compliant with the dependency requirements for Jazzy Jalisco (especially note \texttt{numpy==1.26.4}): \\ \url{ https://www.ros.org/reps/rep-2000.html#jazzy-jalisco-may-2024-may-2029}
\item Create a ROS 2 workspace named \texttt{ros2\_ws} and clone the RE-RASSOR repository into it: \url{https://github.com/2-tunaroll/UoA_RE_RASSOR_2024}. The root directory of the workspace is where the packages will be built from. Consult the README for further information.
\item Install the repository’s dependencies: \url{https://github.com/2-tunaroll/UoA_RE_RASSOR_2024/blob/main/requirements.txt}
\item Add the following lines to \texttt{\textasciitilde/.bashrc}. This enables sourcing of the ROS 2 installation and newly built packages any time a new terminal is opened, or by running \texttt{source \textasciitilde/.bashrc}. Note: Change the file paths if they are different to what is specified here.

    \texttt{source /opt/ros/jazzy/setup.bash} \\
    \texttt{source \textasciitilde/ros2\_ws/install/setup.bash}\\
    \texttt {export LD\_LIBRARY\_PATH=\$LD\_LIBRARY\_PATH:/opt/ros/jazzy/lib} \\
    \texttt{export RMW\_IMPLEMENTATION=rmw\_cyclonedds\_cpp}

\item Install the rocos-agent for DroneDeploy and enable it as a service using the following instructions: \url{ https://docs-automate.dronedeploy.com/robotics-toolkit/getting-started/connect-your-own-robot/ubuntu}. Note: Create an unstable build if the plugin for ROS 2 Jazzy is not yet available.
\item Install the ROS2 plugin for DroneDeploy: \url{https://docs-automate.dronedeploy.com/robotics-toolkit/agent-plugins/ros2}. Contact DroneDeploy to retrieve a custom build for ROS 2 Jazzy if not yet available.
\item Install GStreamer to enable camera streaming to DroneDeploy: \url{ https://gstreamer.freedesktop.org/download/#linux}
\item Once it is verified that all nodes run as expected, systemd service files can be created and enabled to run the launch files on startup. 


1. **OS Setup:** Provide details on how to set up the operating system, including any configurations for Ubuntu on Raspberry Pi.
2. **Networking:** Explain how to configure networking (WiFi, Ethernet, SSH, etc.).
3. **Hardware Connections:** See the project's Final Report for information regarding connection of the motor controllers and sensors to the Raspberry Pi..
4. **Enabling Interfaces:** Provide instructions on enabling necessary interfaces like I2C, SPI, GPIO, etc.

## Installation
### Dependencies
1. **System Packages:** List and provide installation commands for required system packages.
   ```bash
   sudo apt-get update
   sudo apt-get install [packages]
