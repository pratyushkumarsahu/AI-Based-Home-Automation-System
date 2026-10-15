#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image
import random

class PatrolRobot(Node):
    def __init__(self):
        super().__init__('patrol_robot')
        
        # Publishers for Gazebo topics
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.alert_pub = self.create_publisher(String, '/robot/alerts', 10)
        self.gas_pub = self.create_publisher(Float32, '/robot/gas_level', 10)
        self.person_alert_pub = self.create_publisher(String, '/robot/person_alert', 10)
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/rgb/image_raw', self.process_camera, 10)
        
        self.current_room = "living_room"
        self.rooms = ["living_room", "bedroom1", "bedroom2", "kitchen", "bathroom1", "pooja"]
        self.current_room_idx = 0
        
        self.patrol_timer = self.create_timer(2.0, self.patrol_logic)
        self.gas_timer = self.create_timer(1.0, self.check_gas)
        self.frame_count = 0
        
        self.get_logger().info('🤖 Patrol Robot Started - Beginning House Inspection')
        self.get_logger().info(f'Moving in house - Publishing to /cmd_vel')

    def patrol_logic(self):
        """Move robot in house"""
        twist = Twist()
        
        # Alternate between forward and rotation
        timestamp = int(self.get_clock().now().nanoseconds / 1e9)
        
        if timestamp % 10 < 5:
            # Move forward
            twist.linear.x = 0.3  # 0.3 m/s forward
            twist.angular.z = 0.0
            self.get_logger().info('🤖 Moving forward...')
        else:
            # Rotate
            twist.linear.x = 0.0
            twist.angular.z = 0.5  # 0.5 rad/s rotation
            self.get_logger().info('🤖 Rotating...')
        
        self.cmd_vel_pub.publish(twist)
        
        # Change room every 20 seconds
        if timestamp % 20 == 0:
            self.current_room_idx = (self.current_room_idx + 1) % len(self.rooms)
            self.current_room = self.rooms[self.current_room_idx]
            
            alert = String()
            alert.data = f"🔍 [PATROL] Entering {self.current_room}"
            self.alert_pub.publish(alert)
            self.get_logger().info(alert.data)

    def check_gas(self):
        """Check gas levels"""
        gas_level = random.uniform(0, 100)
        gas_msg = Float32()
        gas_msg.data = gas_level
        self.gas_pub.publish(gas_msg)
        
        if gas_level > 50:
            alert = String()
            alert.data = f"⚠️ [GAS ALERT] Gas detected: {gas_level:.1f} ppm in {self.current_room}!"
            self.alert_pub.publish(alert)
            self.get_logger().warn(alert.data)

    def process_camera(self, msg):
        """Process camera frames"""
        self.frame_count += 1
        
        if self.frame_count % 30 == 0:
            if random.random() < 0.3:
                confidence = random.uniform(0.7, 0.99)
                
                if random.random() < 0.2:
                    alert = String()
                    alert.data = f"🚨 [UNKNOWN PERSON] Detected in {self.current_room} (Confidence: {confidence:.2f})"
                    self.person_alert_pub.publish(alert)
                    self.get_logger().warn(alert.data)

def main(args=None):
    rclpy.init(args=args)
    robot = PatrolRobot()
    rclpy.spin(robot)
    robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
