import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np
import math
# from openpyxl import Workbook
# import atexit

class Kalman_Node(Node):
    def __init__(self):
        super().__init__('Kalman_Node')
        self.get_logger().info("Start Kalman Node")

        # Suscripciones
        self.subscriber_position = self.create_subscription(Twist, "/pose", self.callback_pose, 1)
        self.subscriber_control = self.create_subscription(Twist, "/cmd_vel", self.callback_control, 1)

        # Publicador
        self.posePublisher = self.create_publisher(Twist, "/pose_kalman", 1)

        # Variables del sistema
        self.A = np.zeros((3, 3))  # Se actualizará dinámicamente si es necesario
        self.B = np.array([
            [0.0, 0.0],
            [0.0, 0.0],
            [0.0, 1.0]
        ])
        self.u = np.array([[0.0], [0.0]])
        self.x_hat = np.array([[0.0], [0.0], [0.0]])
        self.P = np.zeros((3, 3))
        self.H = np.eye(3)
        self.R = np.diag([10, 5, 1])
        self.Q = np.eye(3) * 0.01
        self.Z = np.array([[0.0], [0.0], [0.0]])

        self.pose_msg = Twist()
        self.last_time = self.get_clock().now()
        self.received_pose = False

        # Crear archivo Excel
        # self.workbook = Workbook()
        # self.sheet = self.workbook.active
        # self.sheet.title = "Kalman Data"
        # self.sheet.append(["Time (s)", "Pose x", "Pose y", "Pose θ", "Kalman x", "Kalman y", "Kalman θ"])

        # # Cerrar el archivo al salir
        # atexit.register(self.save_excel)

        # Timer
        self.timer = self.create_timer(0.01, self.timer_callback)

    def callback_pose(self, msg):
        self.get_logger().info(
            f"Received"
        )
        self.Z = np.array([[msg.linear.x], [msg.linear.y], [msg.angular.z]])
        self.received_pose = True

    def callback_control(self, msg):
        self.u = np.array([[msg.linear.x], [msg.angular.z]])

    def timer_callback(self):
        if not self.received_pose:
            return

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        if dt == 0.0:
            return
        self.last_time = current_time

        # Filtro de Kalman continuo
        x_hat_dot = self.A @ self.x_hat + self.B @ self.u + self.P @ self.H.T @ np.linalg.inv(self.R) @ (self.Z - self.H @ self.x_hat)
        P_dot = self.A @ self.P + self.P @ self.A.T + self.Q - self.P @ self.H.T @ np.linalg.inv(self.R) @ self.H @ self.P

        self.x_hat += dt * x_hat_dot
        self.P += dt * P_dot

        # Publicar el estado estimado
        self.pose_msg.linear.x = self.x_hat[0, 0]
        self.pose_msg.linear.y = self.x_hat[1, 0]
        self.pose_msg.angular.z = self.x_hat[2, 0]
        self.posePublisher.publish(self.pose_msg)

        # Guardar en Excel
        # self.sheet.append([
        #     current_time.nanoseconds * 1e-9,
        #     self.Z[0, 0], self.Z[1, 0], self.Z[2, 0],
        #     self.x_hat[0, 0], self.x_hat[1, 0], self.x_hat[2, 0]
        # ])

        # Log opcional
        # self.get_logger().info(
        #     f"Pose → x: {self.pose_msg.linear.x:.2f}, y: {self.pose_msg.linear.y:.2f}, θ: {math.degrees(self.pose_msg.angular.z):.2f}°"
        # )

    # def save_excel(self):
    #     self.get_logger().info("Guardando archivo Excel...")
    #     self.workbook.save("pose_kalman_data.xlsx")
    #     self.get_logger().info("Archivo guardado como 'pose_kalman_data.xlsx'.")

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
