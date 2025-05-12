import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np
from std_msgs.msg import Float32, Bool
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

class Control_Angle(Node):
    def __init__(self):
        super().__init__('Control_Angle')
        self.get_logger().info("Start control node")
        
        qos_profile = QoSProfile(
            reliability = QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history = QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth = 1
        )

        self.twist = Twist()
        
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 1)
        self.create_subscription(Twist, '/theta', self.angle_callback, qos_profile)
        self.create_subscription(Float32, '/thetad', self.desired_angle_callback, qos_profile)
        self.create_subscription(Bool, '/decision', self.decision_callback, qos_profile)

        # self.qd = np.array([[0.0, 0.0]]).T
        # self.qd_dot = np.array([[0.0, 0.0]]).T
        # # Referencias deseadas
        # self.q0 = np.array([[0.0, 0.0]]).T
        self.theta = 0.0
        self.thetad = 0.0
        self.decision = False
        self.thetad_dot = 0.0
        
        # Parámetros del control
        self.k = 0.2   # Ganancia del controlador
        self.timer = self.create_timer(0.01, self.timer_callback)
        
    def angle_callback(self, msg):
        #self.get_logger().info(f'Actual Position: x:{msg.linear.x}, y:{msg.linear.y}, z:{msg.angular.z}')
        self.theta = msg.angular.z
        
    def desired_position_callback(self, msg):
        #self.get_logger().info(f'Desired Position: x:{msg.x}, y:{msg.y}')
        self.thetad = msg.data
        
    def decision_callback(self, msg):
        self.decision = msg.data       
        
    def timer_callback(self):
        
        # Ajuste de ángulo primero
        angle_error = self.theta - self.thetad
        if self.decision == True: 
            self.phi =  np.array([[0.0, 1.0]])
            U = self.phi * (-self.k*angle_error + self.thetad_dot)
           
            self.twist.linear.x = U[0][0] 
            self.twist.angular.z = U[1][0] 
            
        
        self.get_logger().info(f'Twist x: {self.twist.linear.x }, Twist z: {self.twist.angular.z}')
        self.get_logger().info(f' Angle Error:{angle_error}')
        
        self.publisher.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    control = Control_Angle()
    rclpy.spin(control)
    rclpy.shutdown()

if __name__ == '__main__':
    main()