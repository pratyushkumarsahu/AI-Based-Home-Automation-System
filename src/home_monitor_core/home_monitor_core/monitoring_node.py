#!/usr/bin/env python3
"""Core home monitoring system node"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String
from sensor_msgs.msg import Image

class HomeMonitoringSystem(Node):
    def __init__(self):
        super().__init__('home_monitoring_system')
        
        # Subscribers
        self.motion_sub = self.create_subscription(
            Bool, '/sensors/motion_detected', self.motion_callback, 10)
        
        self.distance_sub = self.create_subscription(
            Float32, '/sensors/distance', self.distance_callback, 10)
        
        self.image_sub = self.create_subscription(
            Image, '/camera/rgb/image_raw', self.image_callback, 10)
        
        # Publisher
        self.alert_pub = self.create_publisher(String, '/monitoring/alerts', 10)
        
        # State
        self.motion_detected = False
        self.alert_count = 0
        
        self.get_logger().info('Home Monitoring System Started')
        self.get_logger().info('Listening to sensors...')
    
    def motion_callback(self, msg):
        """Handle motion sensor data"""
        self.motion_detected = msg.data
        if msg.data:
            self.publish_alert("MOTION_DETECTED")
    
    def distance_callback(self, msg):
        """Handle distance sensor data"""
        if msg.data < 1.0:  # Less than 1 meter
            self.publish_alert(f"OBJECT_TOO_CLOSE:{msg.data:.2f}m")
    
    def image_callback(self, msg):
        """Handle camera data (placeholder for AI processing)"""
        pass
    
    def publish_alert(self, alert_type):
        """Publish alert message"""
        self.alert_count += 1
        msg = String()
        msg.data = f"[ALERT #{self.alert_count}] {alert_type}"
        self.alert_pub.publish(msg)
        self.get_logger().warn(msg.data)

def main(args=None):
    rclpy.init(args=args)
    monitor = HomeMonitoringSystem()
    rclpy.spin(monitor)
    monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
