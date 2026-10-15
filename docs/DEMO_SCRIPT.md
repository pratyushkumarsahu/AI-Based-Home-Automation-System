# Smart Home Monitoring System - 10-Minute Demonstration Script

**Project Title:** AI-Based Smart Home Monitoring and Suspicious Activity Detection System using ROS 2 and Gazebo  
**Framework:** ROS 2 Jazzy, Gazebo Sim Harmonic, C++ rclcpp, OpenCV  
**Target Audience:** Project Guide, External Examiner, Viva Evaluation Panel  
**Estimated Duration:** 10 minutes  

---

## 1. Quick Setup & Environment Check (1 Minute)

Open a terminal and source the workspace:

```bash
cd ~/home_monitor_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

To run the automated real verification test suite (all 9 tests) at any time:
```bash
python3 scripts/verify_real_acceptance_tests.py ALL
```
*Expected Result:* All 9 acceptance tests pass (`🏆 ALL ACCEPTANCE TESTS PASSED WITH REAL SIMULATION DATA!`).

### Graphical Visualization & Software Rendering Notes:
If running inside a virtual machine or software OpenGL environment:
```bash
# Force software rasterization if hardware acceleration fails:
export LIBGL_ALWAYS_SOFTWARE=1

# Launch with Top-Down Overview camera:
ros2 launch smart_home_bringup bringup.launch.py gui_view:=topdown

# Or launch with Third-Person Follow camera tracking the robot:
ros2 launch smart_home_bringup bringup.launch.py gui_view:=follow
```

---

## 2. Demonstration 1: Autonomous Room-to-Room Patrol (2 Minutes)
**Scenario Argument:** `scenario:=patrol`

### Command to Execute:
```bash
# Headless run:
timeout 60s ros2 launch smart_home_bringup bringup.launch.py headless:=true scenario:=patrol

# Or graphical mode with RViz and Top-Down view:
ros2 launch smart_home_bringup bringup.launch.py headless:=false use_rviz:=true gui_view:=topdown scenario:=patrol
```

### What to Show to the Guide:
1. **High-Contrast Robot Visuals (P1):** Safety Orange (`#FF6F00`) chassis, active glowing Cyan (`#00E5FF`) beacon mast, and black treads clearly visible in any lighting.
2. **House Floor Plan:** Point out the 7 rooms: Entrance, Dining Room, Hallway, Living Room, Kitchen, Bedroom 1 (Restricted Zone), and Bedroom 2.
3. **Autonomous Multi-Room Navigation:** Robot starts at `(0, 0)` in the hallway and follows 27 patrol waypoints using real `/odom`:
   - `hallway` -> `living_room` -> `dining_room` -> `entrance` -> `kitchen` -> `bedroom1` -> `bedroom2` -> loop.
4. **Telemetry & Real-Time Zone Tracking:** Show terminal logging:
   ```text
   [patrol_navigator]: Robot entered zone: [hallway] at (-0.00, 0.00)
   [patrol_navigator]: Waypoint [hallway_center] REACHED! Advancing to next room.
   [decision_maker_node]: 🟢 [NORMAL MONITORING] Area: hallway | Status: CLEAR | Decision: Continue monitoring.
   ```
5. **Live Topics:**
   ```bash
   ros2 topic echo /odom --once
   ros2 topic echo /current_zone --once
   ros2 topic echo /status --once
   ```

---

## 3. Demonstration 2: Obstacle Detection & Avoidance (2 Minutes)
**Scenario Argument:** `scenario:=obstacle`

### Command to Execute:
```bash
timeout 45s ros2 launch smart_home_bringup bringup.launch.py headless:=true scenario:=obstacle
```

### What to Show to the Guide:
1. **Obstacle Placement:** An obstacle box is spawned in the hallway directly along the robot's route.
2. **LiDAR Telemetry (`/obstacle_status`):**
   - At `< 0.70m`: Robot detects obstacle on LiDAR.
   - At `< 0.40m`: Forward speed is halted (`cmd_vel.linear.x = 0.0`).
   - Steering avoidance turn is initiated (`cmd_vel.angular.z != 0`).
   ```bash
   ros2 topic echo /obstacle_status --once
   ```
3. **Persistent Blockage Handling:** If blocked for > 3.0s, the navigator emits an `UNKNOWN_OBJECT_ON_ROUTE` event to the decision maker, which transitions to `AVOID_OBSTACLE`.

---

## 4. Demonstration 3: Family vs Outsider Recognition & Walking Routes (P2) (2 Minutes)
**Scenario Arguments:** `scenario:=family` vs `scenario:=intruder`

### Step A: All 3 Authorized Family Members (`scenario:=family`)
```bash
timeout 35s ros2 launch smart_home_bringup bringup.launch.py headless:=true scenario:=family
```
- **What happens:** 
  - `family_pratyush` (Green chest badge, walking along living room route)
  - `family_arpita` (Blue chest badge, dining/living patrol)
  - `family_rohit` (Cyan chest badge, north hallway patrol)
- **Detector Output:** The camera classifies persons as `FAMILY: Pratyush`, `FAMILY: Arpita`, and `FAMILY: Rohit` via chest badge segmentation calibrated in `config/family.yaml`.
- **Decision:** Allowed zones -> **NO unauthorized security alerts raised** (0 false alerts). Loitering rules apply only to unauthorized persons.

### Step B: Outsider Intruder Detection (`scenario:=intruder`)
```bash
timeout 40s ros2 launch smart_home_bringup bringup.launch.py headless:=true scenario:=intruder
```
- **What happens:** An outsider (no valid family badge) enters the hallway route.
- **Alert Trigger:** Decision maker immediately raises `ALT-1` (`HIGH` severity, `Rule_Outsider_Intrusion`).
- **Show Alert Output:**
  ```text
  🚨 [SECURITY ALERT] ID: ALT-1 | Severity: HIGH
  📍 Location/Zone: hallway (Rule_Outsider_Intrusion)
  👤 Identity:      OUTSIDER
  📷 Snapshot:      /home/vboxuser/home_monitor_ws/snapshots/alert_ALT-1_xxx.jpg
  ```
- **Show Saved Evidence:**
  ```bash
  ls -lh ~/home_monitor_ws/snapshots/
  tail -n 2 ~/home_monitor_ws/logs/security_alerts.csv
  tail -n 12 ~/home_monitor_ws/logs/security_alerts.json
  ```

---

## 5. Demonstration 4: Kitchen Gas Leak Simulation & Visual Plumes (P3) (2 Minutes)
**Scenario Argument:** `scenario:=gas_leak`

### Command to Execute:
```bash
timeout 40s ros2 launch smart_home_bringup bringup.launch.py headless:=true scenario:=gas_leak
```

### What to Show to the Guide:
1. **Gas Plume Visuals (Gazebo & RViz):**
   - **Gazebo Particle Emitter:** Yellow/grey gas particles billowing from the stove burners.
   - **RViz Gas Marker (`/gas_leak_marker`):** Translucent colored sphere (yellow for warning, red for danger) and dynamic floating text label showing live PPM.
2. **Threshold Escalation:**
   - At $\ge 50\text{ ppm}$: `Rule_Gas_Warning` triggered (`MEDIUM` severity alert).
   - At $\ge 100\text{ ppm}$: `Rule_Gas_Danger` triggered (`CRITICAL` severity). Robot transitions to `GAS_RESPONSE`, stops patrol, and navigates toward the stove gradient.
3. **Alarm Clearance:** Show that publishing `{data: false}` to `/gas_leak/trigger` causes concentration to drop to safe baseline ($\sim 5\text{ ppm}$), shuts off the particle plume, and clears the alarm.

---

## 6. Demonstration 5: Multi-Event Telemetry Recording & Plot Generation (P4) (1 Minute)
**Scenario Argument:** `scenario:=full`

### Command to Execute:
```bash
# Run 3-minute full scenario with automatic telemetry logging:
timeout 180s ros2 launch smart_home_bringup bringup.launch.py headless:=true scenario:=full log_data:=true
```

### What to Show to the Guide:
1. **Automatic Data Logging:** Telemetry is written directly to `results/full_<timestamp>/`:
   - `odom.csv`, `scan.csv`, `status.csv`, `gas.csv`, `alerts.csv`, `detections.csv`, `events.jsonl`.
2. **Generate Telemetry Graphs & Dashboard:**
   ```bash
   # Find latest result directory:
   LATEST_DIR=$(ls -td ~/home_monitor_ws/results/full_* | head -n 1)
   
   # Generate all 8 plots, combined dashboard, and summary report:
   python3 scripts/plot_results.py $LATEST_DIR
   ```
3. **Inspect Generated Deliverables:**
   ```bash
   cat $LATEST_DIR/summary.txt
   ls -lh $LATEST_DIR/*.png
   ```
   Show `dashboard.png` (9-panel visualization) and individual plots (trajectory, LiDAR clearance, gas concentration, alerts timeline, detections, velocity).

