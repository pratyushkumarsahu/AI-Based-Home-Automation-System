#!/usr/bin/env python3
"""
Data Logging Node for Smart Home Monitoring & Suspicious Activity Detection
Writes real-time run telemetry to results/<scenario>_<timestamp>/

Generated files:
  - odom.csv        (t, x, y, yaw, speed)
  - scan.csv        (t, min obstacle distance)
  - status.csv      (t, state, zone)
  - gas.csv         (t, ppm, level)
  - alerts.csv      (t, type, severity, zone, identity, confidence, snapshot path)
  - detections.csv  (t, identity, is_family, confidence, zone)
  - events.jsonl    (structured raw JSON lines of events)

Usage:
  python3 scripts/record_data.py [scenario] [--out-dir <path>] [--auto-plot]
"""

import argparse
import csv
import json
import math
import os
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from smart_home_interfaces.msg import Alert, DetectionArray, GasLevel, SystemStatus


class DataRecorderNode(Node):
    def __init__(self, scenario_name, out_dir, auto_plot=True):
        super().__init__('data_recorder_node')
        self.scenario_name = scenario_name
        self.out_dir = out_dir
        self.auto_plot = auto_plot
        self.t0 = time.time()
        self.last_flush_t = self.t0

        os.makedirs(self.out_dir, exist_ok=True)
        self.get_logger().info(f"Logging run data for scenario [{self.scenario_name}] to: {self.out_dir}")

        # CSV File Handles & Writers
        # 1. odom.csv (t, x, y, yaw, speed)
        self.odom_f = open(os.path.join(self.out_dir, 'odom.csv'), 'w', newline='')
        self.odom_w = csv.writer(self.odom_f)
        self.odom_w.writerow(['t', 'x', 'y', 'yaw', 'speed'])

        # 2. scan.csv (t, min obstacle distance)
        self.scan_f = open(os.path.join(self.out_dir, 'scan.csv'), 'w', newline='')
        self.scan_w = csv.writer(self.scan_f)
        self.scan_w.writerow(['t', 'min_distance'])

        # 3. status.csv (t, state, zone)
        self.status_f = open(os.path.join(self.out_dir, 'status.csv'), 'w', newline='')
        self.status_w = csv.writer(self.status_f)
        self.status_w.writerow(['t', 'state', 'zone'])

        # 4. gas.csv (t, ppm, level)
        self.gas_f = open(os.path.join(self.out_dir, 'gas.csv'), 'w', newline='')
        self.gas_w = csv.writer(self.gas_f)
        self.gas_w.writerow(['t', 'ppm', 'level'])

        # 5. alerts.csv (t, type, severity, zone, identity, confidence, snapshot path)
        self.alerts_f = open(os.path.join(self.out_dir, 'alerts.csv'), 'w', newline='')
        self.alerts_w = csv.writer(self.alerts_f)
        self.alerts_w.writerow(['t', 'type', 'severity', 'zone', 'identity', 'confidence', 'snapshot_path'])

        # 6. detections.csv (t, identity, is_family, confidence, zone)
        self.det_f = open(os.path.join(self.out_dir, 'detections.csv'), 'w', newline='')
        self.det_w = csv.writer(self.det_f)
        self.det_w.writerow(['t', 'identity', 'is_family', 'confidence', 'zone'])

        # 7. events.jsonl
        self.events_f = open(os.path.join(self.out_dir, 'events.jsonl'), 'w')

        self.all_files = [
            self.odom_f, self.scan_f, self.status_f,
            self.gas_f, self.alerts_f, self.det_f, self.events_f
        ]

        # Rate throttle bookkeeping
        self.last_record_t = {}
        self.current_zone = 'unknown'
        self.current_state = 'PATROL'

        # Subscriptions
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(SystemStatus, '/status', self.on_system_status, 10)
        self.create_subscription(String, '/decision/state', self.on_decision_state, 10)
        self.create_subscription(String, '/current_zone', self.on_zone, 10)
        self.create_subscription(GasLevel, '/gas_level', self.on_gas_level, 10)
        self.create_subscription(Alert, '/alerts', self.on_alert, 10)
        self.create_subscription(DetectionArray, '/detections', self.on_detections, 10)

        # Periodic flush timer (every 3 seconds)
        self.create_timer(3.0, self.flush_files)

    def elapsed_time(self):
        return round(time.time() - self.t0, 3)

    def is_throttled(self, key, min_interval):
        now = time.time()
        if now - self.last_record_t.get(key, 0.0) < min_interval:
            return True
        self.last_record_t[key] = now
        return False

    def log_event(self, event_type, data):
        record = {
            't': self.elapsed_time(),
            'event': event_type,
            'data': data
        }
        self.events_f.write(json.dumps(record, default=str) + '\n')

    def on_odom(self, msg: Odometry):
        if self.is_throttled('odom', 0.10):  # 10 Hz max
            return
        t = self.elapsed_time()
        x = round(msg.pose.pose.position.x, 3)
        y = round(msg.pose.pose.position.y, 3)
        q = msg.pose.pose.orientation
        yaw = round(math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)), 3)
        speed = round(msg.twist.twist.linear.x, 3)
        self.odom_w.writerow([t, x, y, yaw, speed])

    def on_scan(self, msg: LaserScan):
        if self.is_throttled('scan', 0.10):  # 10 Hz max
            return
        t = self.elapsed_time()
        valid_ranges = [r for r in msg.ranges if math.isfinite(r) and r >= msg.range_min and r <= msg.range_max]
        min_dist = round(min(valid_ranges), 3) if valid_ranges else 10.0
        self.scan_w.writerow([t, min_dist])

    def on_zone(self, msg: String):
        self.current_zone = msg.data

    def on_decision_state(self, msg: String):
        self.current_state = msg.data
        t = self.elapsed_time()
        self.status_w.writerow([t, self.current_state, self.current_zone])
        self.log_event('state_change', {'state': self.current_state, 'zone': self.current_zone})

    def on_system_status(self, msg: SystemStatus):
        if self.is_throttled('status', 0.50):  # 2 Hz
            return
        t = self.elapsed_time()
        self.current_state = msg.state
        self.current_zone = msg.zone
        self.status_w.writerow([t, self.current_state, self.current_zone])
        self.log_event('system_status', {
            'state': msg.state,
            'zone': msg.zone,
            'gas_ppm': round(msg.gas_ppm, 1),
            'last_alert': msg.last_alert,
            'uptime_sec': round(msg.uptime_sec, 1)
        })

    def on_gas_level(self, msg: GasLevel):
        if self.is_throttled('gas', 0.25):  # 4 Hz
            return
        t = self.elapsed_time()
        ppm = round(msg.ppm, 1)
        level = "CRITICAL" if ppm >= 100.0 else ("WARNING" if ppm >= 50.0 else "NORMAL")
        self.gas_w.writerow([t, ppm, level])
        self.log_event('gas_level', {'ppm': ppm, 'level': level, 'zone': msg.zone, 'leak_active': msg.leak_active})

    def on_alert(self, msg: Alert):
        t = self.elapsed_time()
        self.alerts_w.writerow([
            t, msg.alert_type, msg.severity, msg.zone,
            msg.identity, round(msg.confidence, 2), msg.snapshot_path
        ])
        self.log_event('alert', {
            'alert_id': msg.alert_id,
            'type': msg.alert_type,
            'severity': msg.severity,
            'zone': msg.zone,
            'identity': msg.identity,
            'confidence': round(msg.confidence, 2),
            'description': msg.description,
            'snapshot_path': msg.snapshot_path
        })
        self.flush_files()

    def on_detections(self, msg: DetectionArray):
        t = self.elapsed_time()
        for det in msg.detections:
            self.det_w.writerow([
                t, det.identity, det.is_family, round(det.confidence, 2), det.zone
            ])
            self.log_event('detection', {
                'identity': det.identity,
                'class_name': det.class_name,
                'is_family': det.is_family,
                'confidence': round(det.confidence, 2),
                'zone': det.zone
            })

    def flush_files(self):
        for f in self.all_files:
            try:
                f.flush()
            except Exception:
                pass

    def close(self):
        self.flush_files()
        for f in self.all_files:
            try:
                f.close()
            except Exception:
                pass
        self.get_logger().info(f"All data files saved successfully in {self.out_dir}")

        if self.auto_plot:
            plot_script = os.path.expanduser('~/home_monitor_ws/scripts/plot_results.py')
            if os.path.exists(plot_script):
                print(f"\n[AUTO-PLOT] Generating presentation graphs from {self.out_dir}...")
                try:
                    subprocess.run([sys.executable, plot_script, self.out_dir], check=False)
                except Exception as e:
                    print(f"Auto-plot invocation failed: {e}")


def main():
    parser = argparse.ArgumentParser(description="AI Home Monitor Data Recorder")
    parser.add_argument('scenario', nargs='?', default='run', help='Scenario name')
    parser.add_argument('--scenario', dest='scenario_opt', default=None, help='Scenario name via option')
    parser.add_argument('--out-dir', default=None, help='Target output directory')
    parser.add_argument('--auto-plot', action='store_true', default=True, help='Generate graphs on shutdown')
    args, unknown = parser.parse_known_args()

    scenario_name = args.scenario_opt if args.scenario_opt else args.scenario
    if not args.out_dir:
        ts = time.strftime('%Y%m%d_%H%M%S')
        out_dir = os.path.expanduser(f"~/home_monitor_ws/results/{scenario_name}_{ts}")
    else:
        out_dir = os.path.expanduser(args.out_dir)

    rclpy.init()
    node = DataRecorderNode(scenario_name, out_dir, auto_plot=args.auto_plot)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except BaseException as e:
        print(f"[DATA RECORDER EXCEPTION]: {type(e).__name__}: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()
    finally:
        node.close()
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
