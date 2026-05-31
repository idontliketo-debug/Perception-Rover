import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import numpy as np
import json

class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.bbox_sub = self.create_subscription(String, '/waste_bbox', self.bbox_callback, 10)
        self.target_pub = self.create_publisher(String, '/target', 10)
        self.latest_scan = None
        self.camera_fov = 1.047
        self.image_width = 640
        self.get_logger().info('LiDAR Node started')

    def scan_callback(self, msg):
        self.latest_scan = msg

    def bbox_callback(self, msg):
        if self.latest_scan is None:
            return
        bbox = json.loads(msg.data)
        u = bbox['u']
        scan = self.latest_scan
        num_samples = len(scan.ranges)
        pixels_from_center = u - (self.image_width / 2)
        indices_per_radian = num_samples / (2 * np.pi)
        index_offset = int(pixels_from_center / self.image_width * self.camera_fov * indices_per_radian)
        forward_index = int((0.0 - scan.angle_min) / scan.angle_increment) % num_samples
        center_idx = (forward_index + index_offset) % num_samples
        window = 15
        indices = [(center_idx + i) % num_samples for i in range(-window, window)]
        crop = np.array([scan.ranges[i] for i in indices])
        valid = crop[np.isfinite(crop) & (crop > 0.5)]
        if len(valid) == 0:
            self.get_logger().warn('No valid LiDAR returns in bbox angle')
            return
        distance = float(np.min(valid))
        target_angle = scan.angle_min + center_idx * scan.angle_increment
        x = distance * np.cos(target_angle)
        y = distance * np.sin(target_angle)
        target_data = {'x': round(x, 3), 'y': round(y, 3), 'z': 0.0,
                      'distance': round(distance, 3), 'angle': round(target_angle, 3)}
        msg_out = String()
        msg_out.data = json.dumps(target_data)
        self.target_pub.publish(msg_out)
        self.get_logger().info(f'Target at x={x:.2f}m y={y:.2f}m distance={distance:.2f}m')

def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
