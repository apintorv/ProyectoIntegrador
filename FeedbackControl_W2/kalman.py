import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np
import math

class Kalman_Node(Node):
    def __init__(self):
        super().__init__('Kalman_Node')
        self.get_logger().info("Start position node")

        self.subscriber_position = self.create_subscription(Twist, "/pose", self.callback, 1)
        self.subscriber_control = self.create_subscription(Twist, "/cmd_vel", self.callback_control, 1)
        self.velPublisher = self.create_publisher(Twist, "/pose_kalman", 1)
        
        
        self.A = np.array([[0, 0],
                           [0, 0]])

        self.B = np.array([[0,0],
                           [0,0],
                           [0,1]])
        self.u = 0

        # Estado inicial del carrito
        self.x = np.array([[0, 0, 0]]).T  # Vector de estados q = [x,y,theta]

        # Estado estimado inicial
        self.x_hat = np.array([[0, 0, 0]]).T

        # Matriz de covarianza inicial
        self.P = np.array([[0.0, 0.0, 0.0],
                           [0.0, 0.0, 0.0],
                           [0.0, 0.0, 0.0]])

        # Matrices de ruido
        self.H = np.array([[1, 0, 0],
                           [0, 1, 0],
                           [0, 0, 1]])

        # Covarianza del ruido
        self.R = np.array([[1, 0, 0],
                           [0, 1, 0],
                           [0, 0, 1]]) # modificar

        # Matriz de covarianza del proceso
        self.Q = np.array([[1, 0, 0],
                           [0, 1, 0],
                           [0, 0, 1]])
 
        # Medición inicial
        self.Z = np.array([[0, 0, 0]]).T
         
        self.pose = Twist()
        
        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(0.001, self.timer_callback)

    def timer_callback(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        # Kalman update
        x_hat_dot = self.A @ self.x_hat + self.B * self.u + self.P @ self.H.T * np.linalg.inv(self.R) @ (self.Z - self.H @ self.x_hat)
        P_dot = self.A @ self.P + self.P @ self.A.T + self.Q - self.P @ self.H.T * np.linalg.inv(self.R) @ self.H @ self.P
        

        # Integración de Euler
        x_hat = x_hat + dt * x_hat_dot
        P = P + dt * P_dot
        
        self.pose.linear.x = x_hat[0][0]
        self.pose.linear.y = x_hat[1][0]
        self.pose.angular.z = x_hat[2][0]
        
        self.velPublisher.publish(self.pose)
        
        

        
    def callback(self, msg):
        self.x = np.array([[msg.linear.x, msg.linear.y, msg.angular.z]]).T
    
    def callback_control(self, msg):
        self.u = np.array([[msg.linear.x, msg.angular.z]]).T
        
def main(args=None):
    rclpy.init(args=args)
    position = Kalman_Node()
    rclpy.spin(position)
    rclpy.shutdown()

if __name__ == '__main__':
    main()