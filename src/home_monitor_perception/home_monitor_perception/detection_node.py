#!/usr/bin/env python3
"""AI-based object detection node"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection')
        
        # Subscriber
        self.image_sub = self.create_subscription(
            Image, '/camera/rgb/image_raw', self.detect_callback, 10)
        
        # Publisher
        self.detection_pub = self.create_publisher(
            String, '/perception/detections', 10)
        
        self.frame_count = 0
        
        self.get_logger().info('Object Detection Node Started')
        self.get_logger().info('Waiting for camera feed...')
    
    def detect_callback(self, msg):
        """Process image and detect objects"""
        try:
            self.frame_count += 1
            
            # Simulate detection (random)
            import random
            if random.random() < 0.3:
                detection = String()
                detection.data = f"Potential object detected (frame {self.frame_count})"
                self.detection_pub.publish(detection)
                
                if self.frame_count % 10 == 0:
                    self.get_logger().info(detection.data)
        
        except Exception as e:
            self.get_logger().error(f'Detection error: {e}')

def main(args=None):
    rclpy.init(args=args)
    detector = ObjectDetectionNode()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
