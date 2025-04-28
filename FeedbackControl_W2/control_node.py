import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
import numpy as np
from msgs_circle.msg import Circle
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

class Control_Node(Node):
    def __init__(self):
        super().__init__('Control_Node')
        self.get_logger().info("Start control node")
        
        qos_profile = QoSProfile(
            reliability = QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history = QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth = 1
        )

        self.twist = Twist()
        
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 1)
        self.create_subscription(Twist, '/pose_kalman', self.position_callback, qos_profile)
        self.create_subscription(Vector3, '/qd', self.desired_position_callback, qos_profile)
        #self.create_subscription(Circle, '/qd_qddot', self.circle_callback, qos_profile)

           
        self.qd = np.array([[0.0, 0.0]]).T
        self.qd_dot = np.array([[0.0, 0.0]]).T
        # Referencias deseadas
        self.q0 = np.array([[0.0, 0.0]]).T
        self.thetha = 0.0
        
        # Parámetros del control
        self.k = 0.2   # Ganancia del controlador
        self.h = 0.05   # Parámetro de transformación (debe ser diferente de 0)
        self.timer = self.create_timer(0.01, self.timer_callback)
        
    #def circle_callback(self, msg):
    #    self.qd = np.array([[msg.x, msg.y]]).T 
    #    self.qd_dot = np.array([[msg.x_dot, msg.y_dot]]).T 
        
    def position_callback(self, msg):
        #self.get_logger().info(f'Actual Position: x:{msg.linear.x}, y:{msg.linear.y}, z:{msg.angular.z}')
        self.q0 = np.array([[msg.linear.x, msg.linear.y]]).T
        self.thetha = msg.angular.z
        
    def desired_position_callback(self, msg):
        #self.get_logger().info(f'Desired Position: x:{msg.x}, y:{msg.y}')
        self.qd = np.array([[msg.x, msg.y]]).T          
        
    def timer_callback(self):
        matrix_D = np.array([
            [np.cos(self.thetha), -self.h * np.sin(self.thetha)],
            [np.sin(self.thetha),  self.h * np.cos(self.thetha)]
        ])
        
        e = self.q0 - self.qd 
        aux = -self.k * e
        #aux = self.qd_dot -self.k * e  ##used for qd_dot
        
        if np.linalg.det(matrix_D) != 0:
            U = np.linalg.inv(matrix_D) @ aux

        self.twist.linear.x = U[0][0]
        self.twist.angular.z = U[1][0]
        
        self.get_logger().info(f'Twist x: {self.twist.linear.x }, Twist z: {self.twist.angular.z}')
        self.get_logger().info(f'Error:{e}')
        
        self.publisher.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    control = Control_Node()
    rclpy.spin(control)
    rclpy.shutdown()

if __name__ == '__main__':
    main()