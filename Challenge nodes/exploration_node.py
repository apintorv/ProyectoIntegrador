import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import random

class ExplorerNode(Node):
    def __init__(self):
        super().__init__('lidar_based_explorer')
        self.get_logger().info("🚀 LIDAR-based Explorer Node Started")

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.twist = Twist()

        # Safe thresholds
        self.front_thresh = 0.20   # 20 cm for front
        self.side_thresh = 0.10     # 15 cm for sides

        self.linear_speed = 0.1     # Safe forward speed
        self.angular_speed = 0.4    # Turn speed

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)

        if len(ranges) != len(angles):
            angles = angles[:len(ranges)]

        # Define sectors in radians
        front = self.get_sector(ranges, angles, -0.35, 0.35)           # ±20°
        left = self.get_sector(ranges, angles, np.pi/2 - 0.3, np.pi/2 + 0.3)   # ~60° left
        right = self.get_sector(ranges, angles, -np.pi/2 - 0.3, -np.pi/2 + 0.3) # ~60° right

        min_front = np.min(front) if len(front) > 0 else float('inf')
        min_left = np.min(left) if len(left) > 0 else float('inf')
        min_right = np.min(right) if len(right) > 0 else float('inf')

        self.get_logger().info(f"Front: {min_front:.2f} m | Left: {min_left:.2f} m | Right: {min_right:.2f} m")

        if min_front < self.front_thresh:
            # Obstacle ahead — decide turning direction
            if min_left > self.side_thresh and min_right > self.side_thresh:
                turn_dir = random.choice(['left', 'right'])
            elif min_left > self.side_thresh:
                turn_dir = 'left'
            elif min_right > self.side_thresh:
                turn_dir = 'right'
            else:
                turn_dir = 'left'  # fallback if both sides blocked

            self.twist.linear.x = 0.0
            self.twist.angular.z = self.angular_speed if turn_dir == 'left' else -self.angular_speed
            self.get_logger().info(f"Obstacle ahead → Turning {turn_dir}")
        else:
            # Clear front, side doesn't matter — go forward
            self.twist.linear.x = self.linear_speed
            self.twist.angular.z = 0.0
            self.get_logger().info("Path clear → Moving forward")

        self.cmd_pub.publish(self.twist)

    def get_sector(self, ranges, angles, start_angle, end_angle):
        """Extract valid (finite) ranges within a specific angle sector."""
        indices = np.where((angles >= start_angle) & (angles <= end_angle))[0]
        sector = ranges[indices]
        return sector[np.isfinite(sector)]

def main(args=None):
    rclpy.init(args=args)
    node = ExplorerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
