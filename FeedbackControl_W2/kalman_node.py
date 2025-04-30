import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np
import math
import csv
import os

class Kalman_Node(Node):
    def __init__(self):
        super().__init__('Kalman_Node')
        self.get_logger().info("Start Kalman Node")

        # Subscriptions
        self.subscriber_position = self.create_subscription(Twist, "/pose", self.callback_pose, 1)
        self.subscriber_control = self.create_subscription(Twist, "/cmd_vel", self.callback_control, 1)

        # Publisher
        self.posePublisher = self.create_publisher(Twist, "/pose_kalman", 1)

        # Kalman filter matrices
        self.A = np.zeros((3, 3))
        self.B = np.array([[0.0, 0.0], [0.0, 0.0], [0.0, 1.0]])
        self.u = np.array([[0.0], [0.0]])    # Control input (v, w)
        self.x_hat = np.array([[0.0], [0.0], [0.0]])  # Estado estimado
        self.P = np.zeros((3, 3))             # Covarianza inicial
        self.H = np.eye(3)                    # Medición directa
        self.R = np.array([[1, 0, 0],
                           [0, 1, 0],
                           [0, 0, 1]]) # modificar
        self.Q = np.eye(3) * 0.01             # Ruido de proceso más pequeño
        self.Z = np.array([[0.0], [0.0], [0.0]])  # Medición inicial

        # Mensaje de salida
        self.pose_msg = Twist()

        # Timer
        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(0.01, self.timer_callback)  # 100 Hz

        # CSV Logging setup
        self.log_file_path = os.path.expanduser("~/kalman_log.csv")
        with open(self.log_file_path, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Time', 'Measured_x', 'Measured_y', 'Measured_theta',
                             'Estimated_x', 'Estimated_y', 'Estimated_theta'])

    def timer_callback(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        if dt == 0.0:
            return
        self.last_time = current_time

        # Ecuaciones de Kalman continuo
        x_hat_dot = self.A @ self.x_hat + self.B @ self.u + self.P @ self.H.T @ np.linalg.inv(self.R) @ (self.Z - self.H @ self.x_hat)
        P_dot = self.A @ self.P + self.P @ self.A.T + self.Q - self.P @ self.H.T @ np.linalg.inv(self.R) @ self.H @ self.P

        self.x_hat += dt * x_hat_dot
        self.P += dt * P_dot

        # Publish estimated state
        self.pose_msg.linear.x = self.x_hat[0, 0]
        self.pose_msg.linear.y = self.x_hat[1, 0]
        self.pose_msg.angular.z = self.x_hat[2, 0]
        self.posePublisher.publish(self.pose_msg)

        # Logging
        timestamp = current_time.nanoseconds * 1e-9
        with open(self.log_file_path, mode='a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                f"{timestamp:.4f}",
                self.Z[0, 0], self.Z[1, 0], self.Z[2, 0],
                self.x_hat[0, 0], self.x_hat[1, 0], self.x_hat[2, 0]
            ])

        # Log to console
        self.get_logger().info(
            f"Pose → x: {self.pose_msg.linear.x:.2f}, y: {self.pose_msg.linear.y:.2f}, θ: {math.degrees(self.pose_msg.angular.z):.2f}°"
        )

    def callback_pose(self, msg):
        self.Z = np.array([
            [msg.linear.x],
            [msg.linear.y],
            [msg.angular.z]
        ])

    def callback_control(self, msg):
        self.u = np.array([
            [msg.linear.x],
            [msg.angular.z]
        ])

def main(args=None):
    rclpy.init(args=args)
    node = Kalman_Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
