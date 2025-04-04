import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np
import math

class FeedbackControl(Node):
    def __init__(self):
        super().__init__('feedback_control')
        self.get_logger().info("Start control node")

        self.velPublisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.timer = self.create_timer(0.01, self.timer_callback)
        
        #self.car = SunriseRobot()
        #self.car.set_car_type(6)
        #self.car.create_receive_threading()
                
        # Estado del robot
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Referencias deseadas
        self.xd = 1.0
        self.yd = 1.0
        self.thetad = 0.0
        
        # Parámetros del control
        self.k = 5.0   # Ganancia del controlador
        self.h = 2.0   # Parámetro de transformación (debe ser diferente de 0)
        self.angle_threshold = 0.1  # Umbral para considerar el ángulo alineado
        
        # Control del tiempo para integración
        self.last_time = self.get_clock().now()
        
    def control_function(self, ex, ey, etheta):
        # Priorizar el control de orientación
        if abs(etheta) > self.angle_threshold:
            return 0.0, etheta * self.k  # Solo girar
        
        # Matriz de transformación
        D = np.array([
            [np.cos(self.theta), -self.h * np.sin(self.theta)],
            [np.sin(self.theta),  self.h * np.cos(self.theta)]
        ])
        
        # Control de posición
        q_dot = np.array([-self.k * ex, -self.k * ey])
        
        try:
            # Verificar si la matriz es invertible
            if np.linalg.det(D) != 0:
                U = np.linalg.inv(D) @ q_dot
                return U[0], etheta * self.k  # Retorna v y w
            else:
                raise np.linalg.LinAlgError  # Forzar la excepción si la matriz no es válida
        except np.linalg.LinAlgError:
            self.get_logger().warn("Matriz D no es invertible. Ajustando control.")
            return 0.0, 0.0
        
    def timer_callback(self):

        twist = Twist()
        # Obtener el tiempo actual y calcular el tiempo transcurrido
        #current_time = time.time()
        #dt = current_time - self.last_time
        #self.last_time = current_time

        # Capturar velocidades enviadas
        #vx = self.x
        #angular = self.theta
        #self.car.set_car_motion(vx, 0, angular)

        # Obtener datos reales del robot
        #vx, vy, angular = self.car.get_motion_data()

        # Actualizar la posición del robot usando integración simple
        #self.x += vx * np.cos(self.theta) * dt
        #self.y += vx * np.sin(self.theta) * dt
        #self.theta += angular * dt
        
        # Calcular errores de posición y orientación
        #ex = self.xd - self.x
        #ey = self.yd - self.y
        #etheta = self.thetad - self.theta
        
        # Calcular U usando la función de control
        #v, w = self.control_function(ex, ey, etheta)
        
        # Asignar valores a Twist
        twist.linear.x = 5.0 #
        twist.angular.z = 0.2 #
        
        self.get_logger().info(f'x: {twist.linear.x }, z: {twist.angular.z}')
        
        # Publicar nueva velocidad corregida
        self.velPublisher.publish(twist)
        
def main(args=None):
    rclpy.init(args=args)
    control = FeedbackControl()
    rclpy.spin(control)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
