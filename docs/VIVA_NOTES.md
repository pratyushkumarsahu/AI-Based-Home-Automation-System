# Smart Home Monitoring System - Viva Examination Notes (15 Questions & Answers)

**Project Title:** AI-Based Smart Home Monitoring and Suspicious Activity Detection System using ROS 2 and Gazebo  
**Framework:** ROS 2 Jazzy, Gazebo Sim Harmonic, C++ rclcpp, OpenCV DNN  
**Intended Use:** Comprehensive reference for viva examination, oral defense, and guide reviews.

---

### Q1: What is the high-level architecture of your system, and how do the nodes interact?
**Answer:** The architecture follows a modular sense-plan-act pipeline built in C++ (`rclcpp`):
1. **Simulation & Environment (`smart_home_gazebo`):** Houses the 7-room house world in Gazebo Harmonic, diff-drive robot, stove gas plume source, and person models.
2. **Perception (`smart_home_detection`):** Subscribes to `/camera/image_raw`, performs YOLO ONNX / contour inference, segments chest identity badges mapped to `family.yaml`, and publishes `/detections`.
3. **Navigation (`smart_home_navigation`):** Drives across 27 room waypoints using `/odom`, processes 360° LiDAR on `/scan`, publishes `/obstacle_status`, and handles stuck recovery.
4. **Gas Sensing (`gas_sensor_node`):** Simulates kitchen stove gas dispersion ($ppm = C_{\text{peak}} / (1 + k \cdot d^2)$) and publishes `/gas_level`.
5. **Decision Maker (`smart_home_decision`):** Finite state machine evaluating strict priority rules: $\text{GAS DANGER} > \text{OUTSIDER} > \text{OBSTACLE} > \text{FAMILY/NORMAL}$, publishing commands to `/decision/nav_cmd` and alerts to `/alerts`.
6. **Alert Manager (`smart_home_alerts`):** Subscribes to `/alerts`, draws bounding box overlays on camera frames, saves annotated JPEGs in `snapshots/`, logs CSV and JSON records in `logs/`, and publishes `/status` telemetry and RViz markers.

---

### Q2: Why did you implement the core nodes in C++ (`rclcpp`) rather than Python (`rclpy`)?
**Answer:**
1. **Deterministic Execution:** Real-time perception and 10 Hz obstacle avoidance require low, predictable latency without Python Global Interpreter Lock (GIL) stalls or garbage collection pauses.
2. **OpenCV & DNN Performance:** Native C++ OpenCV avoids image marshaling overhead when copying raw frames into DNN tensors.
3. **Robotics Industry Standard:** Real embedded deployment on robotic compute platforms (such as NVIDIA Jetson or Raspberry Pi) favors C++ for minimal memory and CPU footprints.

---

### Q3: How is room/zone detection implemented, and why is timer-based zone guessing unacceptable?
**Answer:** `patrol_navigator` continuously cross-references the robot's real-time odometry coordinates `(robot_x_, robot_y_)` from `/odom` against 2D bounding boxes defined in `config/zones.yaml` (e.g., Living Room: $[2.7, 8.0] \times [-6.7, -0.5]$). When coordinates enter a polygon, it publishes the actual room name to `/current_zone`. Timer-based zone transitions fail if the robot encounters an obstacle or slows down; coordinate-based zone detection guarantees spatial ground truth.

---

### Q4: How did you solve the ROS 2 QoS (Quality of Service) mismatch between Gazebo sensors and subscribers?
**Answer:** Gazebo Harmonic's `ros_gz_bridge` publishes high-frequency sensor streams (`/scan`, `/odom`, `/camera/image_raw`) using `BEST_EFFORT` reliability and `VOLATILE` durability (`qos_profile_sensor_data`). A default `RELIABLE` subscriber is incompatible with a `BEST_EFFORT` publisher. We configured all sensor subscribers with `qos_profile_sensor_data`, ensuring universal compatibility and zero dropped connections.

---

### Q5: How does the Decision Maker prioritize simultaneous hazards?
**Answer:** The C++ state machine evaluates hazards in a strict descending hierarchy on every control cycle:
1. **Gas Danger ($\ge 100\text{ ppm}$):** Critical life hazard; robot switches to `GAS_RESPONSE`, stops patrol, and navigates toward the stove.
2. **Outsider Intrusion:** Security threat; robot triggers `ALERT`, pauses, records snapshot, and performs an `INVESTIGATE` sweep.
3. **Obstacle on Patrol Route:** Navigation blockage; robot engages `AVOID_OBSTACLE` or recovery turn.
4. **Authorized Family Member:** Normal activity; logged as `INFO` without raising security alerts.

---

### Q6: How does family member vs outsider recognition work in simulation, and how does this relate to real-world deployment?
**Answer:** In Gazebo simulation, photorealistic facial geometry is not modeled on low-polygon actors. We implemented identity badges: each authorized family member model (`family_pratyush`, `family_arpita`) wears a chest badge with calibrated chromatic properties mapped to `family.yaml` (ID 1: Green badge = Pratyush, ID 2: Blue badge = Arpita). A person without a valid badge is classified as an `OUTSIDER`. In a real-world physical deployment, the visual badge module would be replaced with a deep facial recognition model (e.g., FaceNet, ArcFace), leaving the downstream state machine and alerting logic identical.

---

### Q7: How is the gas leak simulated without physical gas dynamics in Gazebo?
**Answer:** The `gas_sensor_node` acts as an active simulation agent:
$$ppm(d) = \frac{C_{\text{peak}}}{1 + k \cdot d^2} + \mathcal{N}(0, \sigma^2)$$
where $d$ is Euclidean distance from the robot's `/odom` position to the stove coordinates `(-4.5, -2.5)`. When the leak is toggled via `/gas_leak/trigger`, concentration rises as the robot approaches the kitchen, triggering `Rule_Gas_Warning` ($50\text{ ppm}$) and `Rule_Gas_Danger` ($100\text{ ppm}$). Toggling the trigger off returns concentration to safe ambient level ($5\text{ ppm}$), clearing the alarm.

---

### Q8: What is alert cooldown, and why is severity escalation an exception?
**Answer:** To prevent alert spamming (e.g., generating 30 alerts per second when looking at an intruder), the rule engine enforces a 30-second cooldown per alert type and zone. However, if a condition escalates in severity (e.g., gas concentration rises from `WARNING / MEDIUM` to `DANGER / CRITICAL`), the higher-severity event immediately overrides the cooldown to ensure emergency safety.

---

### Q9: How does obstacle avoidance work, and how does the robot handle being completely stuck?
**Answer:** `patrol_navigator` partitions LiDAR into three angular sectors: Front cone `[-22°, +22°]`, Left `[+22°, +80°]`, and Right `[-80°, -22°]`.
- If an obstacle appears at $< 0.60\text{m}$, the robot slows and steers toward the clearer flank.
- If an obstacle is $< 0.40\text{m}$, forward velocity is halted.
- If blocked for $> 3.0\text{s}$, a recovery counter increments. The robot performs an in-place turn away from the obstacle. If blocked for $> 3$ consecutive attempts, it marks the waypoint as obstructed, logs a warning, skips to the next waypoint, and resumes patrol.

---

### Q10: What information is recorded when an alert is triggered?
**Answer:**
1. **Snapshot Image:** Saved as JPEG in `snapshots/alert_<ID>_<timestamp>.jpg` with bounding box, identity label, and header banner.
2. **CSV Log (`logs/security_alerts.csv`):** Formatted row: `timestamp, alert_id, alert_type, severity, zone, identity, ppm, confidence, description, snapshot_path`.
3. **JSON Log (`logs/security_alerts.json`):** Structured JSON object for web/cloud dashboard integration.
4. **Local Alerting:** Appends to `logs/notifications.txt`, rings terminal bell (`\a`), and publishes RViz visual markers.

---

### Q11: How does the system handle missing deep learning model weights?
**Answer:** `detection_node` features a graceful fallback architecture: upon startup, if `model_path` cannot be loaded, it does not crash or throw unhandled exceptions. Instead, it logs `Engaging Fallback Detector` and switches to a 2nd-stage color segmentation and morphological contour pipeline that tracks clothing and identity markers.

---

### Q12: What ROS 2 interfaces (messages) were custom-designed for this project?
**Answer:** In `smart_home_interfaces`:
- `Detection.msg`: Bounding box `[xmin, ymin, xmax, ymax]`, `class_name`, `confidence`, `string identity`, `bool is_family`, `zone`.
- `Alert.msg`: `alert_id`, `alert_type`, `severity`, `description`, `snapshot_path`, `string identity`, `float32 gas_ppm`.
- `ObstacleStatus.msg`: `is_blocked`, `min_distance`, `blocked_duration_sec`, `description`.
- `GasLevel.msg`: `float32 ppm`, `string zone`, `bool leak_active`.
- `SystemStatus.msg`: Telemetry on `/status` (`state`, `zone`, `uptime_sec`, `last_alert_id`, `gas_ppm`).

---

### Q13: What happens during a 5-minute headless stability run?
**Answer:** The bringup launch runs all 6 core nodes concurrently with Gazebo physics. We verified zero `[ERROR]` lines from ROS nodes, zero segmentation faults, clean node shutdowns, and passing unit tests (33 tests in `colcon test` with 0 failures).

---

### Q14: What are the main limitations of the current implementation?
**Answer:**
1. Visual identity badges are used as a simulation substitute for facial recognition.
2. Obstacle avoidance uses reactive LiDAR vector steering rather than global costmap path replanning (Nav2).
3. Notifications are local (file, sound, RViz) rather than cloud push (Telegram/Twilio).

---

### Q15: What are the primary future enhancements for this project?
**Answer:**
1. **Nav2 Integration:** Integrating the Nav2 NavFn planner and DWBB local planner for complete SLAM-based costmap navigation.
2. **Deep Face Recognition:** Deploying MobileFaceNet or InsightFace on an actual onboard camera feed.
3. **Cloud & Mobile Alerts:** Implementing an MQTT / Webhook bridge for instant mobile app push notifications with live snapshot previews.
