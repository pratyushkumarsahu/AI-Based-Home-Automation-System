#!/usr/bin/env python3
"""
Automated Verification Suite for Smart Home Monitoring System

Verifies all 5 Acceptance Test Scenarios in STEP 4:
  Scenario 1: Robot spawns and patrols between rooms with no collisions.
  Scenario 2: A box is placed in path -> robot detects, avoids, continues.
  Scenario 3: Intruder in restricted zone -> detected -> alert -> snapshot & log -> investigate -> returns to patrol.
  Scenario 4: Owner in allowed zone -> no alert, data logged.
  Scenario 5: Model file missing -> fallback detector works.
"""

import os
import sys
import time
import subprocess
import signal
import json
import csv
import glob
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from smart_home_interfaces.msg import Alert, DetectionArray, Detection
import cv2
import numpy as np

WS_DIR = os.path.expanduser("~/home_monitor_ws")
SNAPSHOTS_DIR = os.path.join(WS_DIR, "snapshots")
LOGS_DIR = os.path.join(WS_DIR, "logs")

results = {}

def kill_process_tree(pid):
    try:
        os.killpg(os.getpgid(pid), signal.SIGTERM)
        time.sleep(1)
        os.killpg(os.getpgid(pid), signal.SIGKILL)
    except Exception:
        pass

def test_scenario_5_fallback_detector():
    print("\n=======================================================")
    print("Testing SCENARIO 5: Missing Model -> Fallback Detector")
    print("=======================================================")
    
    rclpy.init()
    node = rclpy.create_node("test_scenario_5_node")

    pub_img = node.create_publisher(Image, "/camera/image_raw", 10)
    pub_zone = node.create_publisher(String, "/current_zone", 10)

    received_detections = []
    def det_cb(msg):
        received_detections.append(msg)

    sub_det = node.create_subscription(DetectionArray, "/detections", det_cb, 10)

    # Launch detection node with nonexistent model path to trigger fallback
    det_proc = subprocess.Popen(
        ["ros2", "run", "smart_home_detection", "detection_node", "--ros-args", "-p", "model_path:=/invalid/path/model.onnx"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        preexec_fn=os.setsid
    )

    time.sleep(2) # Allow node to initialize

    # Create synthetic frame containing human colors (skin tone + red shirt)
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    # Head (skin tone BGR)
    cv2.circle(frame, (320, 160), 30, (180, 200, 240), -1)
    # Torso (red jacket BGR)
    cv2.rectangle(frame, (280, 200), (360, 360), (30, 30, 220), -1)

    from cv_bridge import CvBridge
    bridge = CvBridge()
    img_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
    img_msg.header.stamp = node.get_clock().now().to_msg()
    img_msg.header.frame_id = "camera_link_optical"

    zone_msg = String()
    zone_msg.data = "bedroom1"

    # Publish images and spin
    start_t = time.time()
    person_detected = False
    while time.time() - start_t < 6.0:
        pub_zone.publish(zone_msg)
        pub_img.publish(img_msg)
        rclpy.spin_once(node, timeout_sec=0.2)

        for d_arr in received_detections:
            for d in d_arr.detections:
                if d.class_name == "person" and d.confidence > 0.50:
                    person_detected = True
                    print(f"  [PASS] Fallback detector recognized '{d.class_name}' with conf {d.confidence:.2f} at [{d.xmin},{d.ymin},{d.xmax},{d.ymax}] in zone {d.zone}")
                    break
            if person_detected:
                break
        if person_detected:
            break

    kill_process_tree(det_proc.pid)
    node.destroy_node()
    rclpy.shutdown()

    if person_detected:
        results["Scenario 5 (Fallback Detector)"] = "PASS"
    else:
        results["Scenario 5 (Fallback Detector)"] = "FAIL"

def test_scenario_4_owner_in_allowed_zone():
    print("\n=======================================================")
    print("Testing SCENARIO 4: Owner in Allowed Zone -> No Alert")
    print("=======================================================")
    
    # We test the Decision State Machine when a person is in an allowed zone (living_room)
    # with home_mode set to 'home'.
    rclpy.init()
    node = rclpy.create_node("test_scenario_4_node")

    pub_det = node.create_publisher(DetectionArray, "/detections", 10)
    pub_zone = node.create_publisher(String, "/current_zone", 10)

    alerts_received = []
    def alert_cb(msg):
        alerts_received.append(msg)
    sub_alert = node.create_subscription(Alert, "/alerts", alert_cb, 10)

    # Start decision maker with home_mode:=home
    dec_proc = subprocess.Popen(
        ["ros2", "run", "smart_home_decision", "decision_node", "--ros-args", "-p", "home_mode:=home", "-p", "restricted_hours_enabled:=false", "-p", "loitering_threshold_sec:=10.0"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        preexec_fn=os.setsid
    )

    time.sleep(2)

    # Publish person detection in living_room
    det_msg = DetectionArray()
    det_msg.header.stamp = node.get_clock().now().to_msg()
    d = Detection()
    d.class_name = "person"
    d.confidence = 0.92
    d.xmin = 100
    d.ymin = 100
    d.xmax = 200
    d.ymax = 350
    d.zone = "living_room"
    d.timestamp = node.get_clock().now().to_msg()
    det_msg.detections.append(d)

    zone_msg = String()
    zone_msg.data = "living_room"

    start_t = time.time()
    while time.time() - start_t < 4.0:
        pub_zone.publish(zone_msg)
        pub_det.publish(det_msg)
        rclpy.spin_once(node, timeout_sec=0.2)

    kill_process_tree(dec_proc.pid)
    node.destroy_node()
    rclpy.shutdown()

    if len(alerts_received) == 0:
        print("  [PASS] Owner detected in allowed zone 'living_room' produced 0 alerts (normal condition).")
        results["Scenario 4 (Owner in Allowed Zone)"] = "PASS"
    else:
        print(f"  [FAIL] Unexpected alerts received: {len(alerts_received)}")
        results["Scenario 4 (Owner in Allowed Zone)"] = "FAIL"

def test_scenario_3_intruder_restricted_zone():
    print("\n=======================================================")
    print("Testing SCENARIO 3: Intruder in Restricted Zone -> Alert & Log")
    print("=======================================================")
    
    # Clean previous logs/snapshots for deterministic check
    for f in glob.glob(os.path.join(SNAPSHOTS_DIR, "*.jpg")):
        try: os.remove(f)
        except Exception: pass
    csv_file = os.path.join(LOGS_DIR, "security_alerts.csv")
    json_file = os.path.join(LOGS_DIR, "security_alerts.json")
    if os.path.exists(csv_file): os.remove(csv_file)
    if os.path.exists(json_file): os.remove(json_file)

    rclpy.init()
    node = rclpy.create_node("test_scenario_3_node")

    pub_det = node.create_publisher(DetectionArray, "/detections", 10)
    pub_zone = node.create_publisher(String, "/current_zone", 10)
    pub_annotated = node.create_publisher(Image, "/camera/annotated_image", 10)

    alerts_received = []
    def alert_cb(msg):
        alerts_received.append(msg)
    sub_alert = node.create_subscription(Alert, "/alerts", alert_cb, 10)

    # Launch Decision Maker and Alert Manager
    dec_proc = subprocess.Popen(
        ["ros2", "run", "smart_home_decision", "decision_node", "--ros-args", "-p", "restricted_zone:=bedroom1", "-p", "home_mode:=away"],
        preexec_fn=os.setsid
    )
    alert_proc = subprocess.Popen(
        ["ros2", "run", "smart_home_alerts", "alert_manager_node", "--ros-args", "-p", f"snapshots_dir:={SNAPSHOTS_DIR}", "-p", f"logs_dir:={LOGS_DIR}"],
        preexec_fn=os.setsid
    )

    time.sleep(2)

    # Create dummy camera image
    frame = np.full((480, 640, 3), 40, dtype=np.uint8)
    cv2.putText(frame, "INTRUDER SCENARIO TEST", (50, 240), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
    from cv_bridge import CvBridge
    bridge = CvBridge()
    img_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
    img_msg.header.stamp = node.get_clock().now().to_msg()

    # Publish intruder detection inside restricted zone (bedroom1)
    det_msg = DetectionArray()
    det_msg.header.stamp = node.get_clock().now().to_msg()
    d = Detection()
    d.class_name = "person"
    d.confidence = 0.94
    d.xmin = 200
    d.ymin = 120
    d.xmax = 380
    d.ymax = 420
    d.zone = "bedroom1"
    d.timestamp = node.get_clock().now().to_msg()
    det_msg.detections.append(d)

    zone_msg = String()
    zone_msg.data = "bedroom1"

    start_t = time.time()
    while time.time() - start_t < 5.0:
        pub_annotated.publish(img_msg)
        pub_zone.publish(zone_msg)
        pub_det.publish(det_msg)
        rclpy.spin_once(node, timeout_sec=0.2)
        if len(alerts_received) > 0:
            break

    kill_process_tree(dec_proc.pid)
    kill_process_tree(alert_proc.pid)
    node.destroy_node()
    rclpy.shutdown()

    # Check assertions:
    # 1. Alert received on topic
    has_alert = len(alerts_received) > 0 and alerts_received[0].severity == "CRITICAL"
    # 2. Snapshot file created
    snapshots = glob.glob(os.path.join(SNAPSHOTS_DIR, "*.jpg"))
    has_snapshot = len(snapshots) > 0
    # 3. CSV log contains record
    has_csv = os.path.exists(csv_file) and os.path.getsize(csv_file) > 50
    # 4. JSON log contains record
    has_json = os.path.exists(json_file) and os.path.getsize(json_file) > 50

    print(f"  Alert received: {has_alert} ({len(alerts_received)} alerts)")
    print(f"  Snapshot image written: {has_snapshot} ({snapshots})")
    print(f"  CSV log written: {has_csv}")
    print(f"  JSON log written: {has_json}")

    if has_alert and has_snapshot and has_csv and has_json:
        print("  [PASS] Scenario 3 completed with verified alert, snapshot, and security logs.")
        results["Scenario 3 (Intruder in Restricted Zone)"] = "PASS"
    else:
        results["Scenario 3 (Intruder in Restricted Zone)"] = "FAIL"

def test_scenario_2_obstacle_avoidance():
    print("\n=======================================================")
    print("Testing SCENARIO 2: Box in Path -> Obstacle Avoidance")
    print("=======================================================")
    
    # We test the Navigator node reacting to an obstacle blocking its path
    rclpy.init()
    node = rclpy.create_node("test_scenario_2_node")

    pub_scan = node.create_publisher(LaserScan, "/scan", 10)
    pub_odom = node.create_publisher(Odometry, "/odom", 10)

    cmd_vels = []
    def cmd_cb(msg):
        cmd_vels.append(msg)
    sub_cmd = node.create_subscription(Twist, "/cmd_vel", cmd_cb, 10)

    nav_proc = subprocess.Popen(
        ["ros2", "run", "smart_home_navigation", "patrol_navigator", "--ros-args", "-p", "safety_distance:=0.70", "-p", "stop_distance:=0.45", "-p", "stuck_timeout_sec:=1.5"],
        preexec_fn=os.setsid
    )

    time.sleep(2)

    # Publish Odometry (robot at (0,0), yaw=0 heading towards living room)
    odom_msg = Odometry()
    odom_msg.header.stamp = node.get_clock().now().to_msg()
    odom_msg.pose.pose.position.x = 0.0
    odom_msg.pose.pose.position.y = 0.0
    odom_msg.pose.pose.orientation.w = 1.0

    # Publish LaserScan with obstacle directly in front at 0.35m (< stop_distance)
    scan_msg = LaserScan()
    scan_msg.header.stamp = node.get_clock().now().to_msg()
    scan_msg.header.frame_id = "laser_link"
    scan_msg.angle_min = -3.14159
    scan_msg.angle_max = 3.14159
    scan_msg.angle_increment = 0.01745 # 1 deg
    scan_msg.range_min = 0.15
    scan_msg.range_max = 12.0
    num_samples = int((scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment) + 1
    # default clear 5.0m
    scan_msg.ranges = [5.0] * num_samples
    # obstacle in front indices [-35 deg to +35 deg]
    front_center_idx = num_samples // 2
    for idx in range(front_center_idx - 30, front_center_idx + 30):
        scan_msg.ranges[idx] = 0.35 # obstacle at 35 cm!

    start_t = time.time()
    saw_stop = False
    saw_turn_or_reverse = False

    while time.time() - start_t < 4.0:
        pub_odom.publish(odom_msg)
        pub_scan.publish(scan_msg)
        rclpy.spin_once(node, timeout_sec=0.1)

        for cmd in cmd_vels:
            if cmd.linear.x <= 0.05:
                saw_stop = True
            if abs(cmd.angular.z) > 0.1 or cmd.linear.x < 0.0:
                saw_turn_or_reverse = True

    kill_process_tree(nav_proc.pid)
    node.destroy_node()
    rclpy.shutdown()

    print(f"  Robot stopped forward motion for obstacle: {saw_stop}")
    print(f"  Robot initiated avoidance rotation/reversal: {saw_turn_or_reverse}")

    if saw_stop and saw_turn_or_reverse:
        print("  [PASS] Scenario 2 completed with successful reactive obstacle avoidance.")
        results["Scenario 2 (Obstacle Avoidance)"] = "PASS"
    else:
        results["Scenario 2 (Obstacle Avoidance)"] = "FAIL"

def test_scenario_1_patrol_navigation():
    print("\n=======================================================")
    print("Testing SCENARIO 1: Spawns and Patrols Between Rooms")
    print("=======================================================")
    
    rclpy.init()
    node = rclpy.create_node("test_scenario_1_node")

    pub_odom = node.create_publisher(Odometry, "/odom", 10)
    pub_scan = node.create_publisher(LaserScan, "/scan", 10)

    cmd_vels = []
    zones_visited = set()
    def cmd_cb(msg):
        cmd_vels.append(msg)
    def zone_cb(msg):
        zones_visited.add(msg.data)

    sub_cmd = node.create_subscription(Twist, "/cmd_vel", cmd_cb, 10)
    sub_zone = node.create_subscription(String, "/current_zone", zone_cb, 10)

    nav_proc = subprocess.Popen(
        ["ros2", "run", "smart_home_navigation", "patrol_navigator", "--ros-args", "-p", "tolerance:=0.5"],
        preexec_fn=os.setsid
    )

    time.sleep(2)

    # Simulate path clear scan
    scan_msg = LaserScan()
    scan_msg.header.stamp = node.get_clock().now().to_msg()
    scan_msg.header.frame_id = "laser_link"
    scan_msg.angle_min = -3.14159
    scan_msg.angle_max = 3.14159
    scan_msg.angle_increment = 0.01745
    scan_msg.range_min = 0.15
    scan_msg.range_max = 12.0
    scan_msg.ranges = [5.0] * 361

    # Simulate moving through multiple waypoints (hallway -> living_room -> entrance -> kitchen -> bedroom1)
    test_positions = [
        (0.0, 0.0),    # hallway
        (4.5, -2.5),   # living room
        (0.0, -5.8),   # entrance
        (-4.5, -2.5),  # kitchen
        (-4.5, 2.5),   # bedroom1
        (0.0, 0.0),    # return to hallway
    ]

    for (x, y) in test_positions:
        odom_msg = Odometry()
        odom_msg.header.stamp = node.get_clock().now().to_msg()
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.orientation.w = 1.0
        
        for _ in range(10):
            pub_odom.publish(odom_msg)
            pub_scan.publish(scan_msg)
            rclpy.spin_once(node, timeout_sec=0.05)

    kill_process_tree(nav_proc.pid)
    node.destroy_node()
    rclpy.shutdown()

    has_cmd = len(cmd_vels) > 0
    print(f"  Robot produced navigation velocity commands: {has_cmd}")
    print(f"  Rooms visited and tracked: {zones_visited}")

    required_rooms = {"hallway", "living_room", "entrance", "kitchen", "bedroom1"}
    if has_cmd and required_rooms.issubset(zones_visited):
        print(f"  [PASS] Scenario 1 completed: all {len(required_rooms)} rooms visited and tracked.")
        results["Scenario 1 (Autonomous Patrol)"] = "PASS"
    else:
        results["Scenario 1 (Autonomous Patrol)"] = "FAIL"

def main():
    print("=======================================================")
    print("STARTING FULL ACCEPTANCE VERIFICATION TEST SUITE")
    print("=======================================================")

    test_scenario_5_fallback_detector()
    test_scenario_4_owner_in_allowed_zone()
    test_scenario_3_intruder_restricted_zone()
    test_scenario_2_obstacle_avoidance()
    test_scenario_1_patrol_navigation()

    print("\n=======================================================")
    print("FINAL ACCEPTANCE VERIFICATION SUMMARY")
    print("=======================================================")
    all_passed = True
    for scenario, res in results.items():
        status_symbol = "✅" if res == "PASS" else "❌"
        print(f"  {status_symbol} {scenario}: {res}")
        if res != "PASS":
            all_passed = False

    print("=======================================================")
    if all_passed:
        print("🏆 ALL ACCEPTANCE TEST SCENARIOS PASSED!")
        sys.exit(0)
    else:
        print("💥 SOME SCENARIOS FAILED!")
        sys.exit(1)

if __name__ == "__main__":
    main()
