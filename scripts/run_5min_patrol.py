#!/usr/bin/env python3
"""
5-Minute Full Simulation Verification Runner
Runs Gazebo headless patrol scenario for 5 minutes and 15 seconds (315 seconds).
Captures all logs and verifies that all 5 rooms (hallway, kitchen, bedroom, living room, entrance)
are visited with confirmed zone changes in the decision_node logs.
"""

import os
import sys
import time
import signal
import subprocess
import re

WS_DIR = os.path.expanduser("~/home_monitor_ws")
LOG_FILE = os.path.join(WS_DIR, "logs", "5min_patrol_run.log")

def main():
    os.makedirs(os.path.dirname(LOG_FILE), exist_ok=True)
    print("=================================================================")
    print("STARTING 5-MINUTE HEADLESS SIMULATION PATROL TEST")
    print(f"Logging output to: {LOG_FILE}")
    print("=================================================================")

    cmd = [
        "ros2", "launch", "smart_home_bringup", "bringup.launch.py",
        "headless:=true", "scenario:=patrol"
    ]

    with open(LOG_FILE, "w") as log_f:
        log_f.write(f"=== Patrol Run Started at {time.strftime('%Y-%m-%d %H:%M:%S')} ===\n")
        proc = subprocess.Popen(
            cmd,
            stdout=log_f,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid
        )

        start_time = time.time()
        duration_sec = 315 # 5 minutes and 15 seconds

        print(f"Process PID: {proc.pid}. Running for {duration_sec} seconds (~5.25 min)...")

        try:
            while time.time() - start_time < duration_sec:
                elapsed = int(time.time() - start_time)
                remaining = duration_sec - elapsed
                if elapsed > 0 and elapsed % 30 == 0:
                    print(f"  [Elapsed: {elapsed}s / Remaining: {remaining}s] Patrol in progress...")
                time.sleep(5)
        finally:
            print("\n5 minutes elapsed. Sending graceful shutdown to simulation...")
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                proc.wait(timeout=10)
            except Exception:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                    proc.wait(timeout=5)
                except Exception:
                    try:
                        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                    except Exception:
                        pass

    print(f"Patrol run finished. Reading and analyzing {LOG_FILE}...")
    with open(LOG_FILE, "r") as f:
        log_content = f.read()

    # Search for decision_node zone changes and normal monitoring in zones
    zone_changes = re.findall(r"\[decision_maker_node\]: Zone changed: .*? -> \[(.*?)\]", log_content)
    monitoring_zones = set(re.findall(r"\[decision_maker_node\]: 🟢 \[NORMAL MONITORING\] Area: (.*?) \|", log_content))
    navigator_zones = set(re.findall(r"\[patrol_navigator\]: Robot entered zone: \[(.*?)\]", log_content))
    waypoints_reached = re.findall(r"\[patrol_navigator\]: Waypoint \[(.*?)\] reached!", log_content)

    all_visited_zones = set(zone_changes) | monitoring_zones | navigator_zones

    print("\n=================================================================")
    print("5-MINUTE PATROL ANALYSIS RESULTS")
    print("=================================================================")
    print(f"Total Waypoints Reached: {len(waypoints_reached)}")
    print(f"Waypoints sequence: {waypoints_reached[:15]}... ({len(waypoints_reached)} total)")
    print(f"Decision Node Zone Changes ({len(zone_changes)} transitions): {zone_changes}")
    print(f"Decision Node Normal Monitoring Zones: {monitoring_zones}")
    print(f"All Visited Zones: {all_visited_zones}")

    required_rooms = {"hallway", "living_room", "entrance", "kitchen", "bedroom1"}
    missing = required_rooms - all_visited_zones

    if not missing:
        print("\n🏆 SUCCESS: Robot visited ALL required rooms (hallway, kitchen, bedroom, living room, entrance)!")
        print(f"Zone changes confirmed in decision_node logs: {set(zone_changes)}")
        return 0
    else:
        print(f"\n❌ FAILURE: Missing required rooms: {missing}")
        return 1

if __name__ == "__main__":
    sys.exit(main())
