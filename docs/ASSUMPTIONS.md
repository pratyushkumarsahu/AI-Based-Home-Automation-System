# System Assumptions, Design Decisions, and Architectural Notes

**Project:** AI-Based Smart Home Monitoring and Suspicious Activity Detection System  
**Framework:** ROS 2 Jazzy Jalisco | Gazebo Sim 8.11.0 (Harmonic) | OpenCV 4.6.0  
**Workspace:** `~/home_monitor_ws`

---

## 1. Environment & Architecture Decisions

1. **Autonomous Reactive Navigation vs. Full Nav2 SLAM:**
   - *Context:* `ros-jazzy-navigation2` is not installed on the system image, and passwordless root access is unavailable.
   - *Design Choice:* As authorized by the specifications, a high-reliability C++ Waypoint Navigator (`smart_home_navigation/src/patrol_navigator.cpp`) is implemented. It provides:
     - Closed-loop waypoint following using real odometry (`/odom`).
     - LiDAR sector scanning with reactive stop, crawl, and avoidance steering.
     - Multi-stage recovery behaviors (reverse crawl + clear-space rotation).
     - Consecutive retry tracking with automatic waypoint skipping after 3 failed attempts.
     - Telemetry status published on `/obstacle_status`.

2. **Family Recognition in Simulation:**
   - *Context:* Realistic 3D face recognition on simple polygon mesh faces in virtual simulation is unreliable and computationally prohibitive without high-resolution human face textures.
   - *Design Choice:* In accordance with the prompt guidance ("Add family recognition with a method that works in simulation: every family member model carries a visible identity marker... mapped to a name in config/family.yaml"), family members carry visible identity badges on their chest (Green = Pratyush, Blue = Arpita, Cyan = Rohit).
   - *Future Work:* In a physical real-world deployment, this badge classifier would be replaced with a deep facial recognition pipeline (e.g., FaceNet, InsightFace, or MobileFaceNet).

3. **Gas Leak Physics Simulation (R4):**
   - *Context:* Gazebo Sim 8 lacks computational fluid dynamics and gas diffusion physics.
   - *Design Choice:* A realistic C++ gas dispersion node (`gas_sensor_node`) computes spatial gas concentration from the robot's Euclidean distance to the active kitchen stove source:
     $$\text{ppm} = \frac{C_{\text{peak}}}{1.0 + k \cdot d^2} + \mathcal{N}(0, \sigma)$$
     When the leak is inactive, ambient safe concentration (~5 ppm) is reported. When activated, concentration rises above 50 ppm (Warning) and 100 ppm (Danger), triggering robot gradient response and alert clearance upon source shutoff.

4. **Additional Hazard Simplifications (R5):**
   - In accordance with R5 ("Skip if time-consuming, and record it in ASSUMPTIONS.md"), additional hazards like smoke/fire visual markers and door/window contact switches are documented as extensions and deferred in favor of full verification of R1–R4.

5. **Decision Hierarchy & Alert Cooldown:**
   - State Machine: `PATROL`, `INVESTIGATE`, `ALERT`, `AVOID_OBSTACLE`, `GAS_RESPONSE`, `RETURN_TO_PATROL`, `IDLE`.
   - Priority Hierarchy: `GAS DANGER` (Critical) > `Outsider Intruder` (High/Critical) > `Corridor Obstacle` (Warning/Medium) > `Authorized Family` (Normal / Info).
   - Cooldown: 30-second suppression per `(rule, zone)` pair, with immediate override if an alert escalates to higher severity.

6. **Camera FOV Geometry & Spawn Waypoints (P2):**
   - *Context:* The robot camera has an ~80° horizontal field-of-view ($\pm 40^\circ$) pointing along its forward heading ($+X$). When spawned at $(0, 0)$ facing forward, entities located outside this arc or behind walls cannot be detected until the robot patrols into their sector.
   - *Design Choice:* To ensure immediate, reliable identification across scenarios and acceptance tests without requiring prolonged patrol cycles:
     - Pratyush is spawned at $(2.5, -0.5)$ ($-11.3^\circ$), approaching the living room.
     - Arpita is spawned at $(1.5, -0.7)$ ($-25.0^\circ$), at the dining/living room threshold.
     - Rohit is spawned at $(1.8, 0.8)$ ($+24.0^\circ$), approaching the corridor.
     - Outsider intruder is spawned at $(2.0, 0.0)$ ($0.0^\circ$), directly down the central hallway.
   - All four entities are within the forward FOV at start and have dynamic routes crossing through the camera view.

7. **Loitering Rule and Authorized Family Exemption (P2):**
   - *Context:* Rule 3 in the security specification triggers an alert if an entity remains stationary or loiters in a zone for $>10\text{ s}$.
   - *Design Choice:* In a domestic setting, family members regularly remain stationary in allowed living areas (living room, hallway, kitchen). Triggering intruder/security alerts for family members simply resting or conversing would produce frequent false alarms. Rule 3 was calibrated to explicitly enforce `!det.is_family` (or presence in restricted zones), ensuring authorized family members in permitted rooms generate zero false security alerts.

8. **Dual-Layer Gas Plume Visualization (P3):**
   - *Context:* Gazebo Sim Harmonic supports particle emitters via the `gz-sim-particle-emitter-system` plugin. However, software-rendered OpenGL environments (such as VirtualBox or headless containers using `llvmpipe`) frequently omit particle emitter primitives or fail shader compilations.
   - *Design Choice:* A robust dual-layer visualizer was implemented:
     - **Native Particle Emitter:** Added to the kitchen gas stove in `house.sdf` emitting yellow/grey smoke particles at $15\text{ particles/sec}$ with lifetime $2.0\text{ s}$.
     - **Dynamic Fallback Model:** A visual spherical gas cloud model (`gas_cloud`) is dynamically positioned at the stove coordinates $(2.0, 3.2, 0.95)$ when `leak_active == true` via `gz-transport` entity pose services, and translated subterranean ($z = -10.0\text{ m}$) when inactive.
     - **RViz Marker Array:** Translucent colored spheres (yellow for warning, red for danger) and dynamic floating text labels ($[PPM]$ level) published on `/gas_leak_marker`.

9. **Software OpenGL Rendering Support & View Presets (P1):**
   - *Context:* Running Gazebo inside virtual machines or remote cloud containers often relies on CPU software rasterization (`LIBGL_ALWAYS_SOFTWARE=1`). Low-contrast robot models blend into gray world floorboards or suffer poor visibility.
   - *Design Choice:*
     - Robot chassis recolored in high-contrast Safety Orange (`#FF6F00`), augmented with a glowing Cyan active beacon cylinder (`#00E5FF`) on the top mast and black treads.
     - Added launch presets `gui_view:=topdown` (overview camera centered at $(2.0, 1.0, 8.5)$ looking straight down) and `gui_view:=follow` (third-person chase camera tracking $2.0\text{ m}$ behind the robot).

10. **Data Telemetry Logging & Asynchronous Processing (P4):**
    - *Context:* High-rate sensors (`/odom`, `/scan` at 10–20 Hz) combined with synchronous disk I/O can introduce latency in ROS 2 executor threads.
    - *Design Choice:* `scripts/record_data.py` uses `qos_profile_sensor_data` (volatile, best-effort) compatible with `ros_gz_bridge`. Files are flushed at clean intervals and gracefully closed upon receiving `SIGINT` or `SIGTERM`. Plots and summary metrics are generated offline by `scripts/plot_results.py` to prevent any simulation slowdown during patrol operations.

