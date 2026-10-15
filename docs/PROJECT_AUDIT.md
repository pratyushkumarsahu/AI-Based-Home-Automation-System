# Project Audit Report

**Date:** 2026-10-09  
**System:** Ubuntu 24.04.4 LTS (Noble Numbat) x86_64  
**ROS 2 Distro:** Jazzy Jalisco  
**Gazebo Version:** Gazebo Sim 8.11.0 (Harmonic) via `ros_gz_sim`, `ros_gz_bridge`, `ros_gz_image`  
**Vision Libraries:** OpenCV 4.6.0 (`libopencv-dev`), `ros-jazzy-cv-bridge`  

---

## 1. Executive Summary

An audit of the initial workspace `~/home_monitor_ws` reveals a collection of early prototype ROS 2 packages (`home_monitor_*`). While the workspace successfully builds (`colcon build`), the existing codebase consists almost entirely of placeholder mock implementations that do not fulfill the autonomous robotics, C++ state machine, computer vision, or sensor simulation requirements specified in `PROMPT.md`.

---

## 2. Environment Inspection

| Component | Status | Details |
|---|---|---|
| **OS** | OK | Ubuntu 24.04.4 LTS (Linux 7.0.0-29-generic) |
| **ROS 2** | OK | ROS 2 Jazzy Jalisco installed in `/opt/ros/jazzy` |
| **Gazebo** | OK | Gazebo Sim 8 (Harmonic) installed; gz command at `/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz` |
| **ROS-GZ Bridge** | OK | `ros-jazzy-ros-gz-sim`, `ros-jazzy-ros-gz-bridge`, `ros-jazzy-ros-gz-image` installed |
| **OpenCV** | OK | OpenCV 4.6.0 (C++ dev headers and Python bindings) |
| **cv_bridge** | OK | `ros-jazzy-cv-bridge` installed |
| **Nav2 / SLAM** | Absent | `ros-jazzy-navigation2` is not installed; sudo requires a password. A custom C++ waypoint follower with reactive LiDAR avoidance will be implemented as authorized in `PROMPT.md`. |
| **Colcon** | OK | `colcon` build tools installed |

---

## 3. Package-by-Package Audit

### 3.1 `home_monitor_gazebo`
- **What Works:** `worlds/house.sdf` contains a detailed architectural model of a 6-room house (living room, dining room, kitchen, bedroom 1, bedroom 2, pooja room, common toilet/bathroom, entrance verandah, exterior walls, doors, windows).
- **What is Broken:** In `house.sdf`, a `patrol_robot` model was placed after the closing `</world></sdf>` tags, causing the SDFormat parser to ignore it. The diff-drive plugin configured in that model used non-standard topic names (`/cmd_vel/linear`, `/cmd_vel/angular`). Furthermore, no camera or LiDAR sensors were defined on the robot in `house.sdf`.
- **What is Missing:** Intruder human model/actor, dynamic obstacle models (box), world launch configuration for scenarios (`patrol`, `intruder`, `obstacle`).

### 3.2 `home_monitor_robot`
- **What Works:** Basic URDF geometry (`patrol_robot.urdf`) with a differential drive chassis, caster, and links.
- **What is Broken:** URDF lacked Gazebo Sim sensor tags (camera, GPU lidar, IMU) and Gazebo Harmonic plugins (`gz-sim-diff-drive-system`, `gz-sim-sensors-system`, `gz-sim-joint-state-publisher-system`).
- **What is Broken:** `patrol_robot.py` was an open-loop script running alternating 5-second forward/turn commands and publishing randomized numbers for gas and dummy person alerts.

### 3.3 `home_monitor_sensors`
- **What Works:** None of the nodes interfaced with Gazebo.
- **What is Broken:** `camera_node.py` generated random uint8 numpy arrays (`np.random.randint`) instead of reading Gazebo camera feeds. `motion_sensor_node.py` generated random boolean triggers.

### 3.4 `home_monitor_perception`
- **What Works:** None.
- **What is Broken:** `detection_node.py` used `random.random() < 0.3` to simulate detections. No neural network, OpenCV DNN, or image processing was implemented.

### 3.5 `home_monitor_core`
- **What Works:** Basic subscriber/publisher skeletons.
- **What is Broken:** `monitoring_node.py` and `notification_node.py` were Python scripts without state machines, zone rules, snapshot capture, or persistence.

### 3.6 `home_monitor_description`
- **What Works:** Skeleton package.
- **What is Broken:** No robot description files or xacros installed.

### 3.7 `home_monitor_bringup`
- **What Works:** Basic launch files executing `gz sim` and launching the mock python nodes.
- **What is Broken:** Lacked arguments (`headless`, `use_nav2`, `scenario`), ros_gz bridges, and complete node orchestration.

---

## 4. Gap Analysis vs. PROMPT.md Requirements

| Requirement | Current State | Required Target |
|---|---|---|
| **Programming Language** | Mostly Python mock scripts | C++ (`rclcpp`) for core nodes; OpenCV DNN in C++ |
| **Robot Sensors & Sim** | No actual Gazebo sensors | URDF/SDF with real RGB camera (`/camera/image_raw`), 2D LiDAR (`/scan`), IMU (`/imu`), Odometry (`/odom`), TF tree |
| **Custom Interfaces** | None (used `std_msgs/String`) | `smart_home_interfaces` with `Detection.msg`, `DetectionArray.msg`, `Alert.msg` |
| **Navigation & Avoidance** | Open-loop forward/turn | C++ Waypoint Follower with reactive LiDAR obstacle avoidance and stuck recovery |
| **AI Detection** | Random string generator | C++ OpenCV DNN (ONNX/Caffe/Darknet) + automated weights downloader + color/motion fallback detector |
| **Suspicious Activity Rules** | None | Configurable YAML rules: restricted zone, restricted hours/away mode, loitering, unknown objects, path blocked |
| **Decision State Machine** | None | C++ 6-state machine (`PATROL`, `INVESTIGATE`, `ALERT`, `AVOID_OBSTACLE`, `RETURN_TO_PATROL`, `IDLE`) |
| **Alerts & Logging** | Console print only | `/alerts` topic, structured console, snapshot `.jpg` with annotations, CSV and JSON event logging |
| **Unified Launch** | Partial launch file | `ros2 launch smart_home_bringup bringup.launch.py` with `headless:=...`, `use_nav2:=...`, `scenario:=...` |
| **Unit & Integration Tests** | Empty test scaffolds | GTest unit tests for decision logic/rules + scenario verification |

---

## 5. Execution Strategy

1. **Retain Sound Elements:** Preserve the room layout and dimensions from `home_monitor_gazebo/worlds/house.sdf`.
2. **Implement Core Packages (`smart_home_*`):**
   - `smart_home_interfaces`: Custom ROS 2 message definitions.
   - `smart_home_description`: Complete URDF with camera, LiDAR, IMU, diff-drive plugins.
   - `smart_home_gazebo`: Updated house world, actor/intruder models, dynamic obstacle box, ros_gz bridge parameter config.
   - `smart_home_navigation`: C++ reactive waypoint navigator with LiDAR collision avoidance and stuck recovery.
   - `smart_home_detection`: C++ AI detector using OpenCV DNN with fallback detection and model downloader script.
   - `smart_home_decision`: C++ workflow state machine implementing rules and robot actions.
   - `smart_home_alerts`: C++ alert manager with image snapshot storage, CSV/JSON logging, and console alerts.
   - `smart_home_bringup`: Master launch system supporting all required arguments and scenarios.
3. **Verification:** Run all 5 mandatory scenarios, capture test logs, and produce documentation.
