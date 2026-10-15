#!/usr/bin/env python3
"""
Integration Launch Test for Smart Home Monitoring Pipeline
Tests launch orchestration, multi-node communication, zone updates, and alert triggering.
"""

import os
import unittest
import time
import pytest
import rclpy
from rclpy.node import Node
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from smart_home_interfaces.msg import Detection, DetectionArray, Alert


@pytest.mark.launch_test
def generate_test_description():
    bringup_pkg = get_package_share_directory('smart_home_bringup')
    params_file = os.path.join(bringup_pkg, 'config', 'params.yaml')

    nav_node = launch_ros.actions.Node(
        package='smart_home_navigation',
        executable='patrol_navigator',
        output='screen',
        parameters=[params_file, {'use_sim_time': False}]
    )

    decision_node = launch_ros.actions.Node(
        package='smart_home_decision',
        executable='decision_node',
        output='screen',
        parameters=[params_file, {'use_sim_time': False}]
    )

    alert_node = launch_ros.actions.Node(
        package='smart_home_alerts',
        executable='alert_manager_node',
        output='screen',
        parameters=[params_file, {'use_sim_time': False}]
    )

    return launch.LaunchDescription([
        nav_node,
        decision_node,
        alert_node,
        launch_testing.actions.ReadyToTest(),
    ]), {
        'nav_node': nav_node,
        'decision_node': decision_node,
        'alert_node': alert_node,
    }


class TestPipelineLaunch(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_pipeline_launch_harness')

    def tearDown(self):
        self.node.destroy_node()

    def test_zone_tracking_and_alert_pipeline(self):
        received_zones = []
        received_alerts = []
        received_states = []

        def zone_cb(msg):
            received_zones.append(msg.data)

        def alert_cb(msg):
            received_alerts.append(msg)

        def state_cb(msg):
            received_states.append(msg.data)

        self.node.create_subscription(String, '/current_zone', zone_cb, 10)
        self.node.create_subscription(Alert, '/alerts', alert_cb, 10)
        self.node.create_subscription(String, '/decision/state', state_cb, 10)

        odom_pub = self.node.create_publisher(Odometry, '/odom', 10)
        det_pub = self.node.create_publisher(DetectionArray, '/detections', 10)

        # 1. Test Odometry -> Zone tracking for all rooms
        test_positions = [
            (0.0, -5.5, "entrance"),
            (-4.5, -2.5, "kitchen"),
            (4.5, -2.5, "living_room"),
            (-4.5, 2.5, "bedroom1"),
        ]

        for x, y, expected_zone in test_positions:
            odom = Odometry()
            odom.header.stamp = self.node.get_clock().now().to_msg()
            odom.header.frame_id = "odom"
            odom.pose.pose.position.x = x
            odom.pose.pose.position.y = y
            odom.pose.pose.orientation.w = 1.0

            start_t = time.time()
            matched = False
            while time.time() - start_t < 2.0:
                odom_pub.publish(odom)
                rclpy.spin_once(self.node, timeout_sec=0.1)
                if expected_zone in received_zones:
                    matched = True
                    break
            self.assertTrue(matched, f"Expected zone '{expected_zone}' not detected in {received_zones}")

        # 2. Test Intruder Detection -> Rule 1 Alert Triggering
        det_array = DetectionArray()
        det_array.header.stamp = self.node.get_clock().now().to_msg()
        det = Detection()
        det.class_name = "person"
        det.confidence = 0.95
        det.zone = "bedroom1"
        det.timestamp = det_array.header.stamp
        det_array.detections.append(det)

        start_t = time.time()
        while time.time() - start_t < 4.0:
            det_pub.publish(det_array)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if len(received_alerts) > 0:
                break

        self.assertGreater(len(received_alerts), 0, "No alerts received on /alerts topic")
        alert = received_alerts[0]
        self.assertEqual(alert.zone, "bedroom1")
        self.assertEqual(alert.severity, "CRITICAL")
        self.assertEqual(alert.alert_type, "Rule_1_Restricted_Zone")
