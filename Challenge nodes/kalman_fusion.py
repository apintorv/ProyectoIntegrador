import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D, PoseStamped
import numpy as np
import math
from tf_transformations import euler_from_quaternion  # pip install tf-transformations

class Kalman_Node(Node):
    def __init__(self):
        super().__init__('Kalman_Node')
        self.get_logger().info("Start Kalman Node")

        # Subscriptions
        self.subscriber_slam = self.create_subscription(PoseStamped, "/pose_stamped", self.callback_slam, 1)
        self.subscriber_control = self.create_subscription(Twist, "/cmd_vel", self.callback_control, 1)

        # Publisher (Pose2D)
        self.posePublisher = self.create_publisher(Pose2D, "/pose_kalman", 1)

        # Kalman filter state
        self.x_hat = np.array([[0.0], [0.0], [0.0]])  # [x, y, theta]
        self.P = np.eye(3) * 0.01                     # Covariance
        self.Q = np.eye(3) * 0.01                     # Process noise
        self.R = np.diag([0.05, 0.05, 0.01])          # Measurement noise
        self.H = np.eye(3)                            # Observation model

        # Control input [v, omega]
        self.u = np.array([[0.0], [0.0]])

        # SLAM Measurement placeholder
        self.Z = np.zeros((3, 1))
        self.received_slam = False

        self.pose_msg = Pose2D()
        self.last_time = self.get_clock().now()

        # Timer
        self.timer = self.create_timer(0.01, self.timer_callback)

    def callback_slam(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        q = msg.pose.orientation
        quat = [q.x, q.y, q.z, q.w]
        _, _, yaw = euler_from_quaternion(quat)

        self.Z = np.array([[x], [y], [yaw]])
        self.received_slam = True

    def callback_control(self, msg: Twist):
        self.u = np.array([[msg.linear.x], [msg.angular.z]])

    def timer_callback(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        if dt == 0.0:
            return
        self.last_time = current_time

        # === Prediction ===
        theta = self.x_hat[2, 0]
        v = self.u[0, 0]
        w = self.u[1, 0]

        # Motion model
        dx = v * math.cos(theta) * dt
        dy = v * math.sin(theta) * dt
        dtheta = w * dt

        self.x_hat[0, 0] += dx
        self.x_hat[1, 0] += dy
        self.x_hat[2, 0] += dtheta

        # Jacobian of motion model
        F = np.array([
            [1.0, 0.0, -v * math.sin(theta) * dt],
            [0.0, 1.0,  v * math.cos(theta) * dt],
            [0.0, 0.0, 1.0]
        ])

        self.P = F @ self.P @ F.T + self.Q

        # === Correction ===
        if self.received_slam:
            y = self.Z - self.H @ self.x_hat  # Innovation
            S = self.H @ self.P @ self.H.T + self.R
            K = self.P @ self.H.T @ np.linalg.inv(S)  # Kalman gain

            self.x_hat = self.x_hat + K @ y
            self.P = (np.eye(3) - K @ self.H) @ self.P
            self.received_slam = False

        # === Publish filtered pose ===
        self.pose_msg.x = self.x_hat[0, 0]
        self.pose_msg.y = self.x_hat[1, 0]
        self.pose_msg.theta = self.x_hat[2, 0]
        self.posePublisher.publish(self.pose_msg)

        self.get_logger().info(
            f"[Kalman] x: {self.pose_msg.x:.2f}, y: {self.pose_msg.y:.2f}, θ: {math.degrees(self.pose_msg.theta):.1f}°"
        )

def main(args=None):
    rclpy.init(args=args)
    node = Kalman_Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
