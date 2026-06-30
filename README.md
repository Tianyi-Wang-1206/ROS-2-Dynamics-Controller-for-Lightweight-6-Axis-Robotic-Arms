# ROS 2 Dynamics Controller for Lightweight 6-Axis Robotic Arms

[![ROS 2 Humble](https://img.shields.io/badge/ROS_2-Humble-3498db.svg)](https://docs.ros.org/en/humble/)
[![C++17](https://img.shields.io/badge/C++-17-blue.svg)](https://en.wikipedia.org/wiki/C%2B%2B17)
[![Python 3.10](https://img.shields.io/badge/Python-3.10-yellow.svg)](https://www.python.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)

> **A hard-real-time, Computed Torque Control (CTC) architecture demonstrating industrial standards in open-source robotics. Features headless MuJoCo effort-mode physics, Controller Chaining, Pilz Industrial Motion Planning, and Automated System Identification.**

> ⚠️ **Notice:** The robot model used in this simulation is the **UFactory Lite6**, sourced from the official [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie). This repository is an independent open-source project designed for research and educational purposes.

*(📸 **Visual Teaser:** Insert a high-quality GIF here showing the PyQt5 GUI on the left and the robot smoothly moving in RViz on the right).*


## 📑 Table of Contents
1. [System Architecture](#system-architecture)
2. [Core Features & Demonstrations](#core-features--demonstrations)
3. [Technical Highlights](#technical-highlights)
4. [Installation & Setup](#installation--setup)
5. [Usage](#usage)
6. [Roadmap & Future Work](#roadmap--future-work)
7. [Contributing & License](#contributing--license)


## 🧠 System Architecture
*(📈 **Flowchart:** Insert a block diagram here (Draw.io or Mermaid.js). It should clearly show the data flow and the **operating frequencies** of each block.)*

*   **UI Layer (Asynchronous):** PyQt5 HMI safely threaded via signals/slots.
*   **Planning Layer (~100Hz):** MoveIt2 + **TRAC-IK** (Inverse Kinematics) + **Pilz Industrial Motion Planner** & Default **PTP Planner**.
*   **Controller Chaining (1000Hz Hard Real-Time):**
    *   `JointTrajectoryController` (JTC): Demoted to act purely as a high-order spline interpolator.
    *   `Lite6CTCController` (CTC): Custom `ChainableControllerInterface` utilizing **Pinocchio** for Rigid Body Dynamics ($M\ddot{q} + C\dot{q} + G$).
*   **Hardware Interface / Physics (1000Hz):** **Headless MuJoCo** running in pure **Effort Mode** (bypassing default PID), paired with **RViz** for visualization. Simulates realistic **Dry/Viscous Friction** and **Motor Armature** (Rotor Inertia).

## 🎥 Core Features & Demonstrations

### 1. Industrial Motion Planning (PTP, MoveL, MoveC)
*(📸 **GIF:** Show the user selecting different modes in the GUI. Show the robot drawing a straight line (MoveL) and a perfect circle (MoveC) in RViz.)*
*   **PTP (Point-to-Point):** Joint space planning to specific angles (e.g., Home position) or Cartesian poses (MoveP) via TRAC-IK.
*   **MoveL:** Deterministic linear Cartesian interpolation via the Pilz planner.
*   **MoveC:** Circular Cartesian interpolation using an auxiliary midpoint frame.

### 2. Automated System Identification
*(📸 **GIF:** Show the robot performing the Fourier excitation trajectory, followed by the YAML parameters updating in the GUI terminal.)*
*   Executes bounded Fourier excitation trajectories.
*   Records $q, \dot{q}, \tau$ and utilizes **Least Squares Optimization** to extract exact **Armature, Viscous Friction, and Coulomb Friction** matrices.

### 3. The "Shadow Robot" Debugging Twin
*(📸 **GIF:** Show the robot moving. The cyan "Shadow Robot" should lead the movement, and the real robot tracks it tightly).*
*   A collision-free, cyan-colored digital twin runs alongside the main robot. It perfectly reflects the unperturbed JTC reference trajectory, allowing instant visual verification of tracking error and dynamic deviations.

### 4. Kinematic E-Stop and Fault Recovery
*(📸 **GIF:** Click the red E-STOP button during a fast movement. Show the robot braking smoothly without jerking, followed by the recovery sequence).*
*   Hardware-level emergency stop overriding MoveIt.
*   Generates a safe deceleration profile based on physical kinematic limits rather than blindly setting torque to zero (which causes free-fall).

## 🔬 Technical Highlights

*   **Kalman Filter State Observation:** 
    Instead of relying on noisy raw velocity differentiation, the CTC employs a 1D Kalman Filter per joint. It merges raw encoder positions with predicted accelerations to output zero-lag, noiseless velocity ($\dot{q}$) and position ($q$) estimations.
*   **Advanced Friction Compensation:** 
    Directly addresses Stiction and Coulomb friction zero-crossing instability. Implements a smooth `std::tanh` transition curve to prevent chattering when joint velocities approach zero.
*   **Safe GUI Threading & Precision Syncing:** 
    The HMI runs a strict isolation pattern. The ROS 2 Executor spins in a background `QThread`, pushing data to the GUI via PyQt Signals. 
    *   **Precision Logic:** The backend strictly stores 64-bit precision floats for target calculations, while the UI displays user-friendly 2-decimal strings. A bidirectional sync mechanism ensures exact precision is preserved during planning without UI clutter.
*   **Global Overrides:** Real-time scaling (0-100%) of maximum velocity and acceleration bounds directly from the HMI.

## 🚀 Quick Start & Reproduction Guide

This project is fully containerized using Docker to eliminate OS and dependency conflicts. You do not need to install ROS 2 or MuJoCo on your host machine.

### 📋 Prerequisites
Before starting, ensure your system has the following installed:
1. [**Docker Engine**](https://docs.docker.com/engine/install/).
2. [**NVIDIA Container Toolkit**](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html): Required for GUI rendering and GPU acceleration.

### 📂 Step 1: Prepare the Workspace
Create a workspace folder and place the project files exactly in this structure:

```
~/lite6_ws/
├── mujoco-3.9.0/          # MuJoCo binaries
├── src/                   # Source code (lite6_bringup, lite6_controllers, etc.)
├── Dockerfile             # Docker configuration
├── run_docker.sh          # Container boot script
├── Readme.md
└── Theory.md
```

### 🐳 Step 2: Start the Docker Environment
Open a terminal on your host machine, navigate to the workspace, and execute the startup script.

```
cd ~/lite6_ws
chmod +x ./run_docker.sh
./run_docker.sh
```

### 🏗️ Step 3: Build and Launch (Inside Docker)
Inside the Docker terminal, build the ROS 2 packages and launch the system:

```
colcon build --symlink-install

source install/setup.bash

ros2 launch lite6_bringup system_bringup.launch.py
```

*(Note: If you reboot your computer, you can repeat Step 2 and Step 3 to launch the robot again.  In addition, since the `run_docker.sh` script uses a "shared folder" feature, the `src` folder on your computer is directly linked to the inside of Docker. You **do not** need to rebuild the Docker image every time you change the code.).*

## 🔮 Roadmap & Future Work
This framework is actively evolving. Upcoming features include:
- [ ] **End-Effector Integration:** Adding URDF and controller support for parallel jaw grippers.
- [ ] **Impedance / Admittance Control:** Transitioning from strict position tracking to compliant Cartesian control for physical human-robot interaction and assembly tasks.
- [ ] **Hardware Deployment:** Swapping the MuJoCo `SystemInterface` with a real EtherCAT driver to control a physical arm.

## 🙏 Acknowledgements

This project stands on the shoulders of giants. I would like to sincerely thank the creators and maintainers of the following open-source projects and organizations:

*   **[MuJoCo (DeepMind)](https://mujoco.org/):** For providing the world's most stable and fastest contact physics engine, and the MuJoCo Menagerie for the high-quality robot models.
*   **[ROS 2 & ros2_control](https://control.ros.org/):** For the incredible real-time hardware abstraction and controller chaining architecture.
*   **[Pinocchio (LAAS-CNRS)](https://gepettoweb.laas.fr/articles/pinocchio.html):** For the lightning-fast C++ rigid body dynamics algorithms that make 1000Hz CTC possible.
*   **[MoveIt 2 & Pilz Planner](https://moveit.ros.org/):** For the robust motion planning and industrial trajectory generation.
*   **[UFactory](https://www.ufactory.cc/):** For creating the Lite6 robotic arm, which serves as the physical inspiration and digital twin for this control architecture.

## ⚖️ Disclaimer

This is an independent, open-source project created by a college student. I am not affiliated with, sponsored by, or endorsed by any commercial entities mentioned in this repository. All trademarks and registered trademarks are the property of their respective owners. The software is provided "as is", without warranty of any kind.