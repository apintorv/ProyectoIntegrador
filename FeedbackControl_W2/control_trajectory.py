import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class Control_Trajectory(Node):
    def __init__(self):
        super().__init__('Control_trajectory')
        self.get_logger().info("Start control node")
        
        qos_profile = QoSProfile(
            reliability = QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history = QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth = 1
        )

        self.twist = Twist()
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 1)
        self.create_subscription(Twist, '/pose', self.position_callback, qos_profile)

        # Puntos que forman un círculo (aproximado)
        self.qd_list = [
            np.array([[1.0, 0.0]]).T,
            np.array([[0.0, 1.0]]).T,
            np.array([[-1.0, 0.0]]).T,
            np.array([[0.0, -1.0]]).T
        ]
        self.current_target_index = 0
        self.qd = self.qd_list[self.current_target_index]

        self.q0 = np.array([[0.0, 0.0]]).T
        self.thetha = 0.0

        # Parámetros del control
        self.k = 0.2
        self.h = 0.05
        self.threshold = 0.0  # Umbral para considerar que se ha llegado a un punto

        self.timer = self.create_timer(0.01, self.timer_callback)
        
    def position_callback(self, msg):
        self.q0 = np.array([[msg.linear.x, msg.linear.y]]).T
        self.thetha = msg.angular.z
        
    def timer_callback(self):
        matrix_D = np.array([
            [np.cos(self.thetha), -self.h * np.sin(self.thetha)],
            [np.sin(self.thetha),  self.h * np.cos(self.thetha)]
        ])
        
        e = self.q0 - self.qd
        dist_to_target = np.linalg.norm(e)

        self.get_logger().info(f'Qd: {self.qd.T}')
        self.get_logger().info(f'Q0: {self.q0.T}')
        self.get_logger().info(f'Distancia al objetivo: {dist_to_target}')


        # Si llegó al punto, pasar al siguiente
        if dist_to_target < self.threshold:
            self.current_target_index = (self.current_target_index + 1) % len(self.qd_list)
            self.qd = self.qd_list[self.current_target_index]
            self.get_logger().info(f"Cambiando al siguiente punto: {self.qd.T}")

        aux = -self.k * e

        if np.linalg.det(matrix_D) != 0:
            U = np.linalg.inv(matrix_D) @ aux
        else:
            U = np.array([[0.0], [0.0]])

        self.twist.linear.x = float(U[0])
        self.twist.angular.z = float(U[1])

        #self.get_logger().info(f'Twist x: {self.twist.linear.x }, Twist z: {self.twist.angular.z}')
        #self.get_logger().info(f'Error: {e.T}')
        
        self.publisher.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    control = Control_Trajectory()
    rclpy.spin(control)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
