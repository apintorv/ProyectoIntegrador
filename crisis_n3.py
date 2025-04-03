import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool
import math
import numpy as np
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

class OpenLoopControl(Node):
    def __init__(self):
        super().__init__('open_loop_control')
        
        # Define QoS profile
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=5
        ) 
        
        self.twist = Twist()
                
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.subscriber_flag_yolo = self.create_subscription(Bool, "/Detection_Flag", self.callback_flag_yolo, qos_profile)
        self.subscriber_x = self.create_subscription(Float32, 'linea', self.callback_x_line, qos_profile)
        self.subscriber_z = self.create_subscription(Float32, 'angular', self.callback_z_line, qos_profile)
        self.subscriber_x_yolo = self.create_subscription(Float32, 'linea_yolo', self.callback_x_yolo, qos_profile)
        self.subscriber_z_yolo = self.create_subscription(Float32, 'angular_yolo', self.callback_z_yolo, qos_profile)

        self.timer = self.create_timer(0.01, self.move)
        self.get_logger().info('Talker node successfully initialized')

        self.last_time = self.get_clock().now()
        
        self.traffic_sign = False

        self.x_line = Float32() # x-position (m)
        self.z_line = Float32() # y-position (m)
        self.x_yolo = Float32() # x-position (m)
        self.z_yolo = Float32() # y-position (m)

    def move(self):
        
        if self.traffic_sign:
            self.twist.angular.z = self.z_yolo.data
            self.twist.linear.x = self.x_yolo.data
            self.get_logger().info(f'Pub {self.x_yolo}, {self.z_yolo}')
        else:
            self.twist.angular.z = self.z_line.data
            self.twist.linear.x = self.x_line.data
            self.get_logger().info(f'Pub {self.x_line}, {self.z_line}')

        self.publisher_.publish(self.twist)

    def callback_flag_yolo(self, msg):
        self.traffic_sign = msg.data

    def callback_x_line(self,msg):
        self.x_line.data = msg.data

    def callback_z_line(self,msg):
        self.z_line.data = msg.data
        
    def callback_x_yolo(self,msg):
       self.x_yolo.data = msg.data
        
    def callback_z_yolo(self,msg):
        self.z_yolo.data = msg.data

def main(args=None):
    rclpy.init(args=args)
    open_loop_control = OpenLoopControl()
    rclpy.spin(open_loop_control)
    open_loop_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()