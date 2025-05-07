import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class Control_Trajectory(Node):
    def __init__(self):
        super().__init__('Control_trajectory')
        self.get_logger().info("Start control node")
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        self.twist = Twist()
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 1)
        self.create_subscription(Twist, '/pose', self.position_callback, qos_profile)
        self.create_subscription(Float32MultiArray, '/qd_coordinates', self.qd_callback, qos_profile)  # Nueva suscripción
        
        self.qd_list = []  # Lista vacía que se llenará con los puntos recibidos
        self.current_target_index = 0
        self.qd = np.array([[0.0, 0.0]]).T  # Inicializamos con un valor por defecto

        self.q0 = np.array([[0.0, 0.0]]).T
        self.thetha = 0.0

        # Parámetros del control
        self.k = 0.2
        self.h = 0.05
        self.threshold = 0.1  # Umbral para considerar que se ha llegado a un punto

        self.timer = self.create_timer(0.001, self.timer_callback)
        
    def position_callback(self, msg):
        # Actualiza la posición actual del robot
        self.q0 = np.array([[msg.linear.x, msg.linear.y]]).T
        self.thetha = msg.angular.z
        
    def qd_callback(self, msg):
        # Recibe las coordenadas de qd en un Float32MultiArray
        flattened_data = msg.data #[(1,0),(1,1)] -> [1,0,1,1]
        self.get_logger().info(f"data: {flattened_data}")
        

        # Asegurarse de que el número de elementos sea par (para formar pares de coordenadas)
        if len(flattened_data) % 2 != 0:
            self.get_logger().warning("El número de elementos en el mensaje qd no es par.")
            return

        # Reconstruir las coordenadas de qd (tuplas de 2 elementos)
        self.qd_list = [np.array([[flattened_data[i], flattened_data[i + 1]]]).T for i in range(0, len(flattened_data), 2)]
        self.qd = self.qd_list[self.current_target_index]  # Inicializar con el primer objetivo

        self.get_logger().info(f"Nuevo conjunto de puntos qd recibido: {self.qd_list}")

    def timer_callback(self):
        # Calculamos el controlador
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

        self.publisher.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    control = Control_Trajectory()
    rclpy.spin(control)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
