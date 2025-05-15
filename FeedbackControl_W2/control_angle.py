import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3, Pose2D
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class Control_Angle(Node):
    def __init__(self):
        super().__init__('control_angle')
        self.get_logger().info("Start control node")

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        self.twist = Twist()

        self.publisher = self.create_publisher(Twist, "/cmd_vel", 1)
        self.create_subscription(Pose2D, '/pose_kalman', self.position_callback, qos_profile)
        self.create_subscription(Vector3, '/qd', self.desired_position_callback, qos_profile)

        self.h = 0.05
        self.k_pos = 0.1
        self.k_th = 0.1

        self.qd = np.array([[0.0, 0.0]]).T
        self.q0 = np.array([[0.0, 0.0, 0.0]]).T
        self.th_d = np.array([[np.pi / 2]])

        self.u = np.array([[0.0, 0.0]]).T
        self.flag_ctrl = 0

        self.e_pos = 0.0
        self.e_th = 0.0

        self.received_pose = False
        self.received_qd = False

        self.timer = self.create_timer(0.01, self.timer_callback)

    def position_callback(self, msg):
        self.q0 = np.array([[msg.x, msg.y, msg.theta]]).T
        self.received_pose = True

    def desired_position_callback(self, msg):
        self.qd = np.array([[msg.x, msg.y]]).T
        self.received_qd = True

    def timer_callback(self):
        if not (self.received_pose and self.received_qd):
            self.get_logger().info("Esperando datos de /pose_kalman y /qd...")
            return

        B = np.array([
            [np.cos(self.q0[2][0]), -self.h * np.sin(self.q0[2][0])],
            [np.sin(self.q0[2][0]), self.h * np.cos(self.q0[2][0])],
            [0, 1]
        ])

        D = np.array([B[0], B[1]])
        phi = np.array([B[2]])

        if self.flag_ctrl == 0:
            self.get_logger().info('Enter position')
            self.e_pos = self.q0[0:2] - self.qd
            self.get_logger().info(f'Error position: {self.e_pos}')
            self.u = np.linalg.inv(D) @ (-self.k_pos * self.e_pos)

        elif self.flag_ctrl == 1:
            self.get_logger().info('Enter angle')
            self.e_th = self.q0[2][0] - self.th_d
            self.get_logger().info(f'Error angle: {self.e_th}')
            self.u  = phi.T @ (-self.k_th * self.e_th)
        else:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.publisher.publish(self.twist)
            self.get_logger().info("Objetivo alcanzado. Robot detenido.")
            return  # Salir del callback

        v_min = 0.05
        w_min = 0.05

        self.twist.linear.x = float(np.sign(self.u[0]) * max(abs(self.u[0]), v_min))
        self.twist.angular.z = float(np.sign(self.u[1]) * max(abs(self.u[1]), w_min))

        self.publisher.publish(self.twist)

        if np.linalg.norm(self.e_pos) < 0.001 and self.flag_ctrl == 0:
            self.flag_ctrl = 1
        elif np.abs(self.e_th) < 0.001 and self.flag_ctrl == 1:
            self.flag_ctrl = 2

def main(args=None):
    rclpy.init(args=args)
    control = Control_Angle()
    rclpy.spin(control)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
