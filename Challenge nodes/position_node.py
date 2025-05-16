import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Twist  # Changed import
from std_msgs.msg import Bool
import numpy as np
import math

class Position_Node(Node):
    def __init__(self):
        super().__init__('Position_Node')
        self.get_logger().info("Start position node")

        self.subscriber = self.create_subscription(Twist, "/vel_raw", self.callback, 1)
        self.posePublisher = self.create_publisher(Pose2D, "/pose", 1)  # Changed publisher type

        self.q = np.array([[0, 0, 0]]).T  # State vector q = [x, y, theta]
        self.h = 0.05
        self.pose = Pose2D()  # Changed to Pose2D
        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(0.001, self.timer_callback)

    def timer_callback(self):
        x = float(self.q[0][0])
        y = float(self.q[1][0])
        theta = float(self.q[2][0])

        # Set Pose2D values
        self.pose.x = x
        self.pose.y = y
        self.pose.theta = theta
        self.posePublisher.publish(self.pose)

        self.get_logger().info(
            f"Pose → x: {x:.2f}, y: {y:.2f}, θ: {theta:.2f}"
        )

    def callback(self, msg):
        # Compute time delta
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        v = msg.linear.x
        omega = msg.angular.z
        theta = self.q[2][0]
        theta = (theta + math.pi) % (2 * math.pi) - math.pi
        
        u = np.array([[v, omega]]).T
        gx = np.array([
            [np.cos(theta), -self.h * np.sin(theta)],
            [np.sin(theta), self.h * np.cos(theta)],
            [0, 1]
        ])
        
        self.q = self.q + dt * (gx @ u)

def main(args=None):
    rclpy.init(args=args)
    position = Position_Node()
    rclpy.spin(position)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
