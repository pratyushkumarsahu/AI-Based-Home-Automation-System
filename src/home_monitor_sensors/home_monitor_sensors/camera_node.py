#!/usr/bin/env python3
"""Camera sensor simulation node"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import numpy as np

class CameraSimulator(Node):
    def __init__(self):
        super().__init__('camera_simulator')
        
        # Publisher
        self.image_pub = self.create_publisher(Image, '/camera/rgb/image_raw', 10)
        
        # Timer for publishing at 30Hz
        self.timer = self.create_timer(0.033, self.publish_frame)
        self.frame_count = 0
        
        self.get_logger().info('Camera Simulator Node Started')
    
    def publish_frame(self):
        """Simulate camera frame"""
        # Create a dummy frame (480x640x3)
        frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        
        # Create ROS Image message
        msg = Image()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        msg.height = 480
        msg.width = 640
        msg.encoding = 'rgb8'
        msg.is_bigendian = False
        msg.step = 640 * 3
        msg.data = frame.tobytes()
        
        self.image_pub.publish(msg)
        self.frame_count += 1
        
        if self.frame_count % 30 == 0:
            self.get_logger().info(f'Published {self.frame_count} frames')

def main(args=None):
    rclpy.init(args=args)
    camera = CameraSimulator()
    rclpy.spin(camera)
    camera.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
