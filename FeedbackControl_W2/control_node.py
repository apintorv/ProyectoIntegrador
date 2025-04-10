import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, Vector3
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

class Control_Node(Node):
    def __init__(self):
        super().__init__('Control_Node')
        self.get_logger().info("Start control node")
        
        qos_profile = QoSProfile(
            reliability = QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history = QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth = 10
        )

        self.twist = Twist()
        
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Pose, '/pose', self.position_callback, qos_profile)
        self.create_subscription(Vector3, '/qd', self.desired_position_callback, qos_profile)
           
        # Referencias deseadas
        self.q0 = np.array([[0.1, 0.0]]).T
        self.thetha = 0.0
        
        # Parámetros del control
        self.k = 0.1   # Ganancia del controlador
        self.h = 0.05   # Parámetro de transformación (debe ser diferente de 0)
        
        self.timer = self.create_timer(0.001, self.timer_callback)
        
    def position_callback(self, msg):
        self.get_logger().info(f'Actual Position: x:{msg.position.x}, y:{msg.position.y}, z:{msg.orientation.w}')
        self.q0 = np.array([[msg.position.x, msg.position.y]]).T
        self.thetha = msg.orientation.w
        
    def desired_position_callback(self, msg):
        self.get_logger().info(f'Desired Position: x:{msg.x}, y:{msg.y}, z:{msg.w}')
        self.qd = np.array([[msg.x, msg.y]]).T        
        
    def timer_callback(self):
        matrix_D = np.array([
            [np.cos(self.thetha), -self.h * np.sin(self.thetha)],
            [np.sin(self.thetha),  self.h * np.cos(self.thetha)]
        ])
        
        e = self.q0 - self.qd 
        aux = -self.k * e
        
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