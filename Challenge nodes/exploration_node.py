import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import random
import numpy as np

class RandomWalkWithAvoidance(Node):
    def __init__(self):
        super().__init__('random_walk_avoidance_node')
        self.get_logger().info("Random Walk with Obstacle Avoidance Started")

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.twist = Twist()
        self.obstacle_detected = False

        self.timer = self.create_timer(0.1, self.timer_callback)

    def scan_callback(self, msg: LaserScan):
        # Focus on a 60-degree window in front (±30° from center)
        ranges = np.array(msg.ranges)
        angle_range = int(30 / (180 / np.pi) / msg.angle_increment)  # convert 30 degrees to index range
        center_index = len(ranges) // 2
        front_ranges = ranges[center_index - angle_range : center_index + angle_range]

        # Filter out invalid (inf or nan) readings
        valid_front = front_ranges[np.isfinite(front_ranges)]

        if len(valid_front) == 0:
            self.obstacle_detected = False
            return

        min_dist = valid_front.min()
        self.get_logger().info(f"Front min distance: {min_dist:.2f} m")

        # Check if anything is too close (within 0.4m)
        self.obstacle_detected = min_dist < 0.8

    def timer_callback(self):
        if self.obstacle_detected:
            # Stop and rotate in place
            self.twist.linear.x = 0.0
            self.twist.angular.z = random.choice([-1.0, 1.0])  # turn left or right
            self.get_logger().info("Obstacle detected - Turning")
        else:
            # Move forward with small turn
            self.twist.linear.x = random.uniform(0.1, 0.25)
            self.twist.angular.z = random.uniform(-0.5, 0.5)
            self.get_logger().info("Moving forward")

        self.cmd_vel_pub.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    node = RandomWalkWithAvoidance()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
