#!/usr/bin/env python3
"""User notification system"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from datetime import datetime

class NotificationSystem(Node):
    def __init__(self):
        super().__init__('notification_system')
        
        # Subscribers
        self.robot_alert_sub = self.create_subscription(
            String, '/robot/alerts', self.handle_robot_alert, 10)
        
        self.person_alert_sub = self.create_subscription(
            String, '/robot/person_alert', self.handle_person_alert, 10)
        
        self.monitoring_alert_sub = self.create_subscription(
            String, '/monitoring/alerts', self.handle_monitoring_alert, 10)
        
        # Publisher
        self.notification_pub = self.create_publisher(
            String, '/notifications/user', 10)
        
        self.get_logger().info('📱 Notification System Started')

    def handle_robot_alert(self, msg):
        """Handle robot patrol alerts"""
        notification = String()
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        notification.data = f"[{timestamp}] 🤖 ROBOT: {msg.data}"
        self.notification_pub.publish(notification)
        self.get_logger().info(f"📤 User Notified: {notification.data}")

    def handle_person_alert(self, msg):
        """Handle person detection alerts"""
        notification = String()
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        notification.data = f"[{timestamp}] 👁️ CAMERA: {msg.data}"
        self.notification_pub.publish(notification)
        self.get_logger().warn(f"📤 URGENT: {notification.data}")

    def handle_monitoring_alert(self, msg):
        """Handle gas/object alerts"""
        if "GAS" in msg.data or "CLOSE" in msg.data:
            notification = String()
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            notification.data = f"[{timestamp}] ⚠️ SENSOR: {msg.data}"
            self.notification_pub.publish(notification)
            self.get_logger().warn(f"📤 ALERT: {notification.data}")

def main(args=None):
    rclpy.init(args=args)
    notif = NotificationSystem()
    rclpy.spin(notif)
    notif.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
