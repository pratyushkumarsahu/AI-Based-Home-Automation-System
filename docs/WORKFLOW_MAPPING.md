# Workflow Flowchart Implementation Mapping

This document provides a 1-to-1 traceability matrix mapping each block of the project's **Workflow Flowchart** and **Block Diagram** directly to the implementing C++ ROS 2 nodes, topics, source files, and Gazebo plugins.

---

## 1. Flowchart Traceability Matrix

| Flowchart Step | Block Name | Description | Implementing Node(s) | Topics Involved | Source File(s) |
|---|---|---|---|---|---|
| **START** | System Initialization | Launch simulation environment, spawn robot, start ROS-GZ bridges | `bringup.launch.py`, `ros_gz_sim`, `ros_gz_bridge` | `/clock`, `/parameter_events` | `smart_home_bringup/launch/bringup.launch.py`<br>`smart_home_gazebo/worlds/house.sdf` |
| **1** | **Home Monitoring** | System monitors house environment across all 7 rooms (Living, Kitchen, Bedroom 1 [restricted], Bedroom 2, Entrance Foyer, Dining, Hallway) | `patrol_navigator`, `decision_node` | `/current_zone`<br>`/navigation/status`<br>`/status` | `smart_home_navigation/src/patrol_navigator.cpp`<br>`smart_home_bringup/config/zones.yaml` |
| **2** | **Data Collection** | Camera, 2D LiDAR, Odometry, and Gas Sensor observe surroundings and stream raw data | Gazebo Harmonic Sensor plugins, `gas_sensor_node`, `parameter_bridge` | `/camera/image_raw`<br>`/scan`<br>`/odom`<br>`/gas_level` | `smart_home_description/urdf/patrol_robot.urdf.xacro`<br>`smart_home_gazebo/src/gas_sensor_node.cpp`<br>`smart_home_gazebo/config/bridge_params.yaml` |
| **3** | **Data Processing** | Visual frames, laser scans, and gas ppm readings are converted (cv_bridge) and filtered | `detection_node`, `decision_node`, `patrol_navigator` | `/camera/image_raw`<br>`/scan`<br>`/gas_level` | `smart_home_detection/src/detection_node.cpp`<br>`smart_home_navigation/src/patrol_navigator.cpp` |
| **4** | **AI Detection & Recognition** | Neural network (YOLOv5n ONNX) or adaptive fallback detects people and classifies FAMILY (identity badges) vs OUTSIDERS | `detection_node` | `/detections`<br>`/camera/annotated_image` | `smart_home_detection/src/detection_node.cpp`<br>`smart_home_interfaces/msg/Detection.msg`<br>`smart_home_bringup/config/family.yaml` |
| **5** | **Decision Branch: Unusual Activity?** | C++ state machine evaluates detections, gas level, and LiDAR against rules with strict priority and 30s cooldown | `decision_node`, `smart_home::RuleEngine` | Evaluates `/detections`, `/gas_level`, `/scan`, `/current_zone` | `smart_home_decision/src/decision_node.cpp`<br>`smart_home_decision/include/smart_home_decision/rule_engine.hpp` |
| **5A** | **YES: Anomaly Detected** | Gas danger leak, outsider intruder, corridor blockage, or restricted zone breach | `decision_node` | Transitions to `ALERT`, `INVESTIGATE`, `AVOID_OBSTACLE`, or `GAS_RESPONSE` | `smart_home_decision/src/decision_node.cpp` |
| **5B** | **NO: Normal Monitoring** | Path clear, authorized family member in allowed zone, safe gas level; continue surveillance | `decision_node` | Stays in `PATROL` state | `smart_home_decision/src/decision_node.cpp` |
| **6A** | **Alert Action** | Publish alert event, display console warning, record snapshot, write CSV/JSON logs, notify file, RViz markers | `alert_manager_node` | `/alerts`<br>`/alert_markers`<br>`/zone_markers` | `smart_home_alerts/src/alert_manager_node.cpp`<br>`smart_home_interfaces/msg/Alert.msg` |
| **6B** | **Normal Action** | Log status (`🟢 [NORMAL MONITORING] Area: <zone> \| CLEAR`), log telemetry data on `/status` | `decision_node`, `alert_manager_node` | `/status`, console stdout | `smart_home_decision/src/decision_node.cpp`<br>`smart_home_interfaces/msg/SystemStatus.msg` |
| **7** | **Robot Control** | Convert decision into robot actions (stop, reverse, steer, sweep, gradient follow) | `decision_node`, `patrol_navigator` | `/decision/nav_cmd`<br>`/cmd_vel` | `smart_home_navigation/src/patrol_navigator.cpp`<br>`smart_home_decision/src/decision_node.cpp` |
| **8** | **Gazebo Simulation** | Execute differential drive velocity commands in simulated physics safely | Gazebo Harmonic `DiffDrive` plugin | `/cmd_vel` (ROS) -> `/cmd_vel` (GZ) | `smart_home_description/urdf/patrol_robot.urdf.xacro`<br>`smart_home_gazebo/worlds/house.sdf` |
| **9** | **Continuous Loop** | Robot completes reaction, returns to patrol, inspects continuously without supervision | Full pipeline orchestrated loop | Looping waypoint patrol & periodic timers | `smart_home_navigation/src/patrol_navigator.cpp`<br>`smart_home_decision/src/decision_node.cpp` |

---

## 2. Block Diagram to Package Architecture Mapping

```mermaid
flowchart LR
    B1["1 Home Environment<br/>(smart_home_gazebo)"] --> B2["2 Sensors & Camera<br/>(smart_home_description)"]
    B2 --> B3["3 Data Collection<br/>(ros_gz_bridge)"]
    B3 --> B4["4 AI Detection & Family Rec<br/>(smart_home_detection)"]
    B4 --> B5["5 Decision Making<br/>(smart_home_decision)"]
    B5 --> B6["6 ROS 2 Communication<br/>(smart_home_interfaces)"]
    B6 --> B7["7 Robot Controller<br/>(smart_home_navigation)"]
    B7 --> B8["8 Gazebo Simulation<br/>(gz::sim::DiffDrive)"]
    B5 --> B9["8 Monitoring & Alerts<br/>(smart_home_alerts)"]
    B8 -.-> B1
```

---

## 3. Decision Maker State Machine Mapping

```mermaid
stateDiagram-v2
    [*] --> PATROL: System Start
    
    PATROL --> GAS_RESPONSE: Gas Danger (>= 100 ppm) [CRITICAL]
    PATROL --> ALERT: Outsider Intrusion / Gas Warning (>= 50 ppm)
    PATROL --> INVESTIGATE: Family in Restricted Curfew / Loitering
    PATROL --> AVOID_OBSTACLE: Path blocked (<0.45m)
    
    GAS_RESPONSE --> RETURN_TO_PATROL: Gas concentration falls < 50 ppm (Alarm Cleared)
    ALERT --> INVESTIGATE: Alert recorded (3s timeout) -> 360 sweep & monitor
    INVESTIGATE --> RETURN_TO_PATROL: Area verified clear
    AVOID_OBSTACLE --> RETURN_TO_PATROL: Obstacle cleared or avoidance executed
    
    RETURN_TO_PATROL --> PATROL: Resume waypoint patrol
    PATROL --> IDLE: Safe standby
    IDLE --> PATROL: Resume patrol
```
