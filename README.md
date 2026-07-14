# Physical-AI-Interactive-Biped-Humanoid-Robot

This repository contains the complete software architecture, kinematics math, and control systems required to operate a dynamic bipedal robot. The system is built on **ROS 2 Foxy** and features a decentralized "Sense-Plan-Act" architecture running on a Jetson Nano (Edge Compute) bridged to an STM32 Nucleo (Real-Time Controller).

## 🧠 System Architecture

Our software ecosystem is designed for high-frequency, deterministic execution. Heavy computations (SLAM, Vision, AI) are isolated from the critical 100Hz real-time balancing loop to ensure physical stability.

### ROS 2 Packages Overview

* **`biped_bringup`**
  The launch center of the robot. Contains the core launch files (e.g., `robot_core.launch.py`) and YAML parameter configurations required to spin up the entire system safely and initialize the network (Cyclone DDS) for remote RViz visualization.

* **`biped_description`**
  The Digital Twin. Contains the URDF (Unified Robot Description Format) XML files and 3D meshes (.stl/.dae). This allows us to visualize joint limits, inertia tensors, and kinematics in a risk-free RViz simulation before sending power to the physical motors.

* **`biped_kinematics`**
  The mathematical core of the robot. This package independently calculates complex inverse and forward kinematics to isolate heavy math from the main walking engine.
  * **Hip IK Node:** Solves the non-linear parallel kinematics for the 3-DOF UPR hip mechanisms (Actuonix L16P linear actuators).
  * **Ankle IK Node:** Solves the Law of Cosines geometry for the Series Elastic Actuator (SEA) ankle mechanisms (RMD X6-7).
  * **Upper Body Controller:** Manages the neck, elbow servos, and DC gripper manipulations.

* **`biped_walking_controller`**
  The dynamic balance engine. Houses the ZMP (Zero Moment Point) controller and the LQR balancing algorithms. It intercepts `cmd_vel` from Nav2 and translates continuous velocity into discrete swing-foot trajectories and stride lengths.

* **`biped_gateway`**
  The custom middleware bridge. Instead of multiple nodes fighting for serial port access, this Command Aggregator synchronizes the complex arrays from the kinematics nodes into a single, dense data payload. It guarantees the STM32 receives deterministic, microsecond-level updates.

* **`biped_ai_agent`**
  Handles high-level reasoning and manipulation tasks. To optimize compute resources, this node is conditionally executed only during double-support (`gait_phase = 0`) when the robot reaches a secure waypoint.

* **`biped_perception` & `biped_odometry`**
  The sensor fusion pipeline. Processes high-speed data streams from the RPLIDAR A1M8, depth camera, and IMU to feed the Nav2 local costmaps and LQR balancing loops.

---

## 📊 MATLAB & Simscape Modeling

Before deploying C++ code to the physical hardware, core mechanical concepts and control algorithms were rigorously modeled and validated using MATLAB, Simulink, and Simscape Multibody.

* **Bipedal Kinematics & Control Validation:** Used to verify the analytical Jacobian matrices for the 3-UPR hip mechanism, simulate the compliance of the SEA ankles, and tune the LQR weight matrices for optimal balance recovery.
* **Independent 4-DOF Robotic Arm:** *Note: This repository also contains an independent, separate project detailing the design and simulation of a 4-Degree-of-Freedom robotic arm modeled entirely using MATLAB, Simulink, Simscape Multibody, and SolidWorks.*

---

## 🚀 Quick Start & Launch Instructions

### Prerequisites
* Ubuntu 20.04
* ROS 2 Foxy Fitzroy
* Cyclone DDS (`ros-foxy-rmw-cyclonedds-cpp`)

### Building the Workspace
```bash
cd ~/ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
