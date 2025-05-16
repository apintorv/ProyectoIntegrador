##MUST CHANGE THIS CODE SO IT INCLUDES THE LOCALIZATION FROM SLAM
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
import numpy as np
import math

class Kalman_Node(Node):
    def __init__(self):
        super().__init__('Kalman_Node')
        self.get_logger().info("Start Kalman Node")

        # Subscriptions
        self.subscriber_position = self.create_subscription(Pose2D, "/pose", self.callback_pose, 1)
        self.subscriber_control = self.create_subscription(Twist, "/cmd_vel", self.callback_control, 1)

        # Publisher (Pose2D)
        self.posePublisher = self.create_publisher(Pose2D, "/pose_kalman", 1)

        # Kalman filter variables
        self.A = np.zeros((3, 3))
        self.B = np.array([
            [0.0, 0.0],
            [0.0, 0.0],
            [0.0, 1.0]
        ])
        self.u = np.array([[0.0], [0.0]])
        self.x_hat = np.array([[0.0], [0.0], [0.0]])
        self.P = np.zeros((3, 3))
        self.H = np.eye(3)
        self.R = np.diag([0.001, 0.001, 0.001])
        self.Q = np.eye(3) * 0.01
        self.Z = np.array([[0.0], [0.0], [0.0]])

        self.pose_msg = Pose2D()
        self.last_time = self.get_clock().now()
        self.received_pose = False

        self.timer = self.create_timer(0.01, self.timer_callback)

    def callback_pose(self, msg: Pose2D):
        self.Z = np.array([[msg.x], [msg.y], [msg.theta]])
        self.received_pose = True

    def callback_control(self, msg: Twist):
        self.u = np.array([[msg.linear.x], [msg.angular.z]])

    def timer_callback(self):
        if not self.received_pose:
            return

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        if dt == 0.0:
            return
        self.last_time = current_time

        # Kalman filter
        x_hat_dot = self.A @ self.x_hat + self.B @ self.u + self.P @ self.H.T @ np.linalg.inv(self.R) @ (self.Z - self.H @ self.x_hat)
        P_dot = self.A @ self.P + self.P @ self.A.T + self.Q - self.P @ self.H.T @ np.linalg.inv(self.R) @ self.H @ self.P

        self.x_hat += dt * x_hat_dot
        self.P += dt * P_dot

        # Set and publish Pose2D
        self.pose_msg.x = self.x_hat[0, 0]
        self.pose_msg.y = self.x_hat[1, 0]
        self.pose_msg.theta = self.x_hat[2, 0]
        self.posePublisher.publish(self.pose_msg)
        
        self.get_logger().info(
            f"Estimated Pose → x: {self.pose_msg.x:.2f}, y: {self.pose_msg.y:.2f}, θ: {math.degrees(self.pose_msg.theta):.2f}°"
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
