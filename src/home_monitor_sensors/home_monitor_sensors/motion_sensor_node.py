#!/usr/bin/env python3
"""Motion sensor simulation node"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
import random

class MotionSensor(Node):
    def __init__(self):
        super().__init__('motion_sensor')
        
        # Publishers
        self.motion_pub = self.create_publisher(Bool, '/sensors/motion_detected', 10)
        self.distance_pub = self.create_publisher(Float32, '/sensors/distance', 10)
        
        # Timer at 2Hz
        self.timer = self.create_timer(0.5, self.publish_sensor_data)
        
        self.motion_simulation_on = True
        self.get_logger().info('Motion Sensor Node Started')
    
    def publish_sensor_data(self):
        """Simulate motion and distance sensor"""
        # Simulate random motion (20% chance)
        motion = Bool()
        motion.data = random.random() < 0.2
        
        # Simulate distance (0.5 to 10 meters)
        distance = Float32()
        distance.data = random.uniform(0.5, 10.0)
        
        self.motion_pub.publish(motion)
        self.distance_pub.publish(distance)
        
        if motion.data:
            self.get_logger().warn(f'MOTION DETECTED! Distance: {distance.data:.2f}m')

def main(args=None):
    rclpy.init(args=args)
    sensor = MotionSensor()
    rclpy.spin(sensor)
    sensor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
