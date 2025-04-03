import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool, String
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
# from yolo_msg.msg import InferenceResult
# from yolo_msg.msg import Yolov8Inference

class Control_Listener(Node):
    def __init__(self):
        super().__init__('Control_Listener')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('kp', 0.4),
                ('vmax', 0.1),
                ('w', 0.1),
                ('vmax_emergencia', 0.05),
                ('w_emergencia', 0.08),
                ('type', 1)
            ]
        )
        
        qos_profile = QoSProfile(
            reliability = QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history = QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth = 10
        )
        
        # Start twist
        self.twist = Twist()
        
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber_error = self.create_subscription(Float32, '/error', self.error_callback, qos_profile)
        self.subscriber_contour_flag = self.create_subscription(Bool, '/flag', self.contour_callback, qos_profile)
        self.subscriber_class = self.create_subscription(String, '/Yolov8_Inference', self.class_callback, qos_profile)  #CHECAR 
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Listener node initialized')
        
        
        
    def class_callback(self, msg):
        self.type = msg.data
        
    def error_callback(self, msg):
        self.error = msg.data

    def contour_callback(self, msg):
        self.contour = msg.data
        
    def timer_callback(self):
        self.control()
        
    def control(self): 

        # Get parameters
        self.kp = self.get_parameter('kp').get_parameter_value().double_value
        self.vmax = self.get_parameter('vmax').get_parameter_value().double_value
        self.w_emergencia = self.get_parameter('w_emergencia').get_parameter_value().double_value
        self.vmax_emergencia = self.get_parameter('vmax_emergencia').get_parameter_value().double_value
        
        # Control
        if self.contour:  
            if self.error > 3 and self.error > -3:
                w = - self.kp * self.error 
                self.twist.angular.z = np.clip(w, -self.vmax, self.vmax)
                
                #CONDICIONES YOLO
                if self.type == 'Stop':
                    self.get_logger().info(f'Stop')
                # if self.type == amarillo and self.type == construccion :
                    self.twist.linear.x = self.vmax/2
                # elif self.type == verde and self.type == flechas:
                else:
                    self.twist.linear.x = self.vmax
                    
                # self.get_logger().info(f'w: {self.twist.angular.z}')
                self.publisher_.publish(self.twist)            
        else:
            self.twist.angular.z = self.w_emergencia
            self.twist.linear.x =  self.vmax_emergencia # Velocidad constante hacia adelante
        
def main(args = None):
    rclpy.init(args=args)
    m_c_l = Control_Listener()
    rclpy.spin(m_c_l)
    m_c_l.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()