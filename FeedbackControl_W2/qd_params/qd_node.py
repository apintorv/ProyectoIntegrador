import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

class qd_Node(Node):
    def __init__(self):
        super().__init__('Desired_Position_Node')
        self.get_logger().info("Start QD node")
                
        self.publisher = self.create_publisher(Vector3, "/qd", 1)
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.qd = Vector3()
        self.declare_parameters(
            namespace='',
            parameters=[
                ('desired_x', 0.0),
                ('desired_y', 0.0),
                ('desired_z', 0.0)
            ]
        )
        
    def timer_callback(self):
        # Obtener los parámetros deseados
        self.qd_x = self.get_parameter('desired_x').get_parameter_value().double_value
        self.qd_y = self.get_parameter('desired_y').get_parameter_value().double_value
        self.qd_z  = self.get_parameter('desired_z').get_parameter_value().double_value

        self.qd.x = self.qd_x
        self.qd.y = self.qd_y
        self.qd.z = self.qd_z
        
        self.get_logger().info(f'qd x={self.qd_x}, y={self.qd_y}, z={self.qd_z}')        
        self.publisher.publish(self.qd)

def main(args=None):
    rclpy.init(args=args)
    control = qd_Node()
    rclpy.spin(control)
    rclpy.shutdown()

if __name__ == '__main__':
    main()